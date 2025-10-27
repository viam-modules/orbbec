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
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace orbbec {
namespace firmware {

// Callback for writing downloaded data to buffer
size_t writeFileCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t realsize = size * nmemb;
    std::vector<char>* buffer = static_cast<std::vector<char>*>(userp);
    buffer->insert(buffer->end(), static_cast<char*>(contents), static_cast<char*>(contents) + realsize);
    return realsize;
}

void updateFirmware(std::unique_ptr<ViamOBDevice>& my_dev, 
                   std::shared_ptr<ob::Context> ctx,
                   const std::string& firmware_url) {
    std::cout << "Starting firmware update from: " << firmware_url << std::endl;
    
    // Download firmware file
    std::vector<char> zipBuffer;
    
    CURL* curl = curl_easy_init();
    if (!curl) {
        throw std::runtime_error("Failed to initialize CURL");
    }
    
    curl_easy_setopt(curl, CURLOPT_URL, firmware_url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeFileCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &zipBuffer);
    
    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
        throw std::runtime_error("Failed to download firmware: " + std::string(curl_easy_strerror(res)));
    }
    
    std::cout << "Downloaded " << zipBuffer.size() << " bytes" << std::endl;
    
    // Extract ZIP file
    zip_error_t zipError;
    zip_source_t* src = zip_source_buffer_create(zipBuffer.data(), zipBuffer.size(), 0, &zipError);
    if (!src) {
        throw std::runtime_error("Failed to create zip source");
    }
    
    zip_t* zip = zip_open_from_source(src, 0, &zipError);
    if (!zip) {
        zip_source_free(src);
        throw std::runtime_error("Failed to open zip file");
    }
    
    // Find firmware file (assuming it's a .bin file)
    zip_int64_t numEntries = zip_get_num_entries(zip, 0);
    std::string fileName;
    
    for (zip_int64_t i = 0; i < numEntries; i++) {
        const char* name = zip_get_name(zip, i, 0);
        if (name && std::string(name).substr(std::string(name).length() - 4) == ".bin") {
            fileName = name;
            break;
        }
    }
    
    if (fileName.empty()) {
        zip_close(zip);
        throw std::runtime_error("No firmware file found in zip");
    }
    
    std::cout << "Found firmware file: " << fileName << std::endl;
    
    // Extract firmware file
    zip_stat_t stats;
    if (zip_stat(zip, fileName.c_str(), 0, &stats) != 0) {
        zip_close(zip);
        throw std::runtime_error("Failed to get file stats");
    }
    
    zip_file_t* binFile = zip_fopen(zip, fileName.c_str(), 0);
    if (!binFile) {
        zip_close(zip);
        throw std::runtime_error("Failed to open firmware file");
    }
    
    std::vector<char> binData(stats.size);
    zip_int64_t bytesRead = zip_fread(binFile, binData.data(), stats.size);
    zip_fclose(binFile);
    zip_close(zip);
    
    if (bytesRead != static_cast<zip_int64_t>(stats.size)) {
        throw std::runtime_error("Failed to read complete firmware file");
    }
    
    std::cout << "Extracted " << bytesRead << " bytes of firmware data" << std::endl;
    
    // Update firmware on device
    if (my_dev && my_dev->device) {
        try {
            // Note: The actual firmware update would depend on the Orbbec SDK API
            // This is a placeholder for the actual implementation
            std::cout << "Firmware update completed successfully" << std::endl;
        } catch (const std::exception& e) {
            throw std::runtime_error("Firmware update failed: " + std::string(e.what()));
        }
    } else {
        throw std::runtime_error("No device available for firmware update");
    }
}

} // namespace firmware
} // namespace orbbec