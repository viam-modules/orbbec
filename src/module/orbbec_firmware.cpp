// Copyright 2025 Viam Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "orbbec_firmware.hpp"
#include "orbbec.hpp"

#include <curl/curl.h>
#include <zip.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace orbbec {
namespace firmware {

size_t writeFileCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    auto* buffer = static_cast<std::vector<char>*>(userp);
    size_t totalSize = size * nmemb;
    buffer->insert(buffer->end(), static_cast<char*>(contents), static_cast<char*>(contents) + totalSize);
    return totalSize;
}

template <auto cleanup_fp>
struct Cleanup {
    using pointer_type = std::tuple_element_t<0, boost::callable_traits::args_t<decltype(cleanup_fp)>>;
    using value_type = std::remove_pointer_t<pointer_type>;

    void operator()(pointer_type p) {
        if (p != nullptr) {
            cleanup_fp(p);
        }
    }
};

template <auto cleanup_fp>
using CleanupPtr = std::unique_ptr<typename Cleanup<cleanup_fp>::value_type, Cleanup<cleanup_fp>>;

void updateFirmware(std::unique_ptr<ViamOBDevice>& my_dev,
                    std::shared_ptr<ob::Context> ctx,
                    const std::string& firmwareUrl,
                    viam::sdk::LogSource& logger) {
// On linux, orbbec recommends to set libuvc backend for firmware update
#if defined(__linux__)
    ctx->setUvcBackendType(OB_UVC_BACKEND_TYPE_LIBUVC);
#endif

    CleanupPtr<curl_easy_cleanup> curl(curl_easy_init());
    if (!curl) {
        throw std::invalid_argument("curl easy init failed");
    }

    // Download the firmware and write it to a buffer
    std::vector<char> zipBuffer;
    curl_easy_setopt(curl.get(), CURLOPT_URL, firmwareUrl.c_str());
    curl_easy_setopt(curl.get(), CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, writeFileCallback);
    curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &zipBuffer);
    CURLcode res = curl_easy_perform(curl.get());
    if (res != CURLE_OK) {
        std::ostringstream buffer;
        buffer << "curl easy perform failed: " << curl_easy_strerror(res);
        throw std::invalid_argument(buffer.str());
    }

    std::vector<uint8_t> binData;
    zip_error_t ziperror;
    zip_error_init(&ziperror);

    zip_source_t* src = zip_source_buffer_create(zipBuffer.data(), zipBuffer.size(), 0, &ziperror);
    if (!src) {
        std::ostringstream buffer;
        buffer << "failed to create zip buffer: " << zip_error_strerror(&ziperror);
        throw std::runtime_error(buffer.str());
    }

    // Ensure src cleanup if zip_open fails
    CleanupPtr<zip_source_free> srcCleanup(src);

    // If this succeeds, zip takes ownership of src, so src will be freed when zip_close is called.
    zip_t* zip = zip_open_from_source(src, 0, &ziperror);
    if (!zip) {
        std::ostringstream buffer;
        buffer << "failed to open zip from source: " << zip_error_strerror(&ziperror);
        throw std::runtime_error(buffer.str());
    }

    srcCleanup.release();
    CleanupPtr<zip_close> zipCleanup(zip);

    if (zip_get_num_entries(zip, 0) != 1) {
        throw std::runtime_error("unexpected number of files in firmware zip");
    }

    const char* fileName = zip_get_name(zip, 0, 0);
    if (!fileName) {
        throw std::runtime_error("couldn't get bin file name");
    }

    CleanupPtr<zip_fclose> binFile(zip_fopen(zip, fileName, 0));
    if (!binFile) {
        throw std::runtime_error("failed to open the firmware bin file");
    }

    zip_stat_t stats;
    zip_stat_init(&stats);
    if (zip_stat(zip, fileName, 0, &stats) != 0) {
        throw std::invalid_argument("failed to stat file");
    }

    binData.resize(stats.size);
    zip_int64_t bytesRead = zip_fread(binFile.get(), binData.data(), stats.size);
    if (bytesRead == -1) {
        zip_error_t* err = zip_file_get_error(binFile.get());
        std::ostringstream buffer;
        buffer << "failed to read bin: " << zip_error_strerror(err);
        throw std::runtime_error(buffer.str());
    }

    if (bytesRead != stats.size) {
        std::ostringstream buffer;
        buffer << "failed to fully read binary file, file size: " << stats.size << "bytes read: " << bytesRead;
        throw std::runtime_error(buffer.str());
    }

    auto firmwareUpdateCallback = [](OBFwUpdateState state, const char* message, uint8_t percent) {
        switch (state) {
            case STAT_VERIFY_SUCCESS:
                std::cout << "Image file verification success\n";
                break;
            case STAT_FILE_TRANSFER:
                std::cout << "File transfer in progress\n";
                break;
            case STAT_DONE:
                std::cout << "Update completed\n";
                break;
            case STAT_IN_PROGRESS:
                std::cout << "Upgrade in progress\n";
                break;
            case STAT_START:
                std::cout << "Starting the upgrade\n";
                break;
            case STAT_VERIFY_IMAGE:
                std::cout << "Verifying image file\n";
                break;
            default:
                std::cerr << "Unknown status or error\n";
                break;
        }
        std::cout << "Firmware update in progress: " << message << " upgrade " << static_cast<int>(percent) << "% complete\n";
    };

    bool executeAsync = false;
    try {
        my_dev->device->updateFirmwareFromData(binData.data(), binData.size(), std::move(firmwareUpdateCallback), executeAsync);
        VIAM_SDK_LOG_IMPL(logger, info) << "firmware update successful!";
    } catch (...) {
        VIAM_SDK_LOG_IMPL(logger, error) << "firmware update failed!";
        // Reset UVC backend type before re-throwing
#if defined(__linux__)
        ctx->setUvcBackendType(OB_UVC_BACKEND_TYPE_AUTO);
#endif
        throw;
    }

    // Reset UVC backend type on success
#if defined(__linux__)
    ctx->setUvcBackendType(OB_UVC_BACKEND_TYPE_AUTO);
#endif
}
}  // namespace firmware
}  // namespace orbbec
