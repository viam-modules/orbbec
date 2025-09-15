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

#include "orbbec.hpp"
#include "encoding.hpp"

#include <curl/curl.h>
#include <math.h>
#include <zip.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <mutex>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/rpc/server.hpp>

#include <boost/callable_traits/args.hpp>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/TypeHelper.hpp>

namespace orbbec {

namespace vsdk = ::viam::sdk;

vsdk::Model Orbbec::model("viam", "orbbec", "astra2");
std::unordered_set<std::string> const Orbbec::supported_color_formats{"RGB", "MJPG"};
std::unordered_set<std::string> const Orbbec::supported_depth_formats{"Y16"};
std::string const Orbbec::default_color_format = "MJPG";
std::string const Orbbec::default_depth_format = "Y16";
Resolution const Orbbec::default_color_resolution{1280, 720};
Resolution const Orbbec::default_depth_resolution{1600, 1200};
std::unordered_map<Resolution, std::unordered_set<Resolution>> const Orbbec::color_to_depth_supported_resolutions{
    {{1920, 1080}, {{1600, 1200}, {800, 600}, {400, 300}}},
    {{1440, 1080}, {{1600, 1200}, {800, 600}, {400, 300}}},
    {{1280, 720}, {{1600, 1200}, {800, 600}, {400, 300}}},
    {{800, 600}, {{800, 600}, {400, 300}}},
    {{640, 480}, {{800, 600}, {400, 300}}},
    {{640, 360}, {{800, 600}, {400, 300}}}};

// CONSTANTS BEGIN
const std::string kColorSourceName = "color";
const std::string kColorMimeTypeJPEG = "image/jpeg";
const std::string kColorMimeTypePNG = "image/png";
const std::string kDepthSourceName = "depth";
const std::string kDepthMimeTypeViamDep = "image/vnd.viam.dep";
const std::string kPcdMimeType = "pointcloud/pcd";
// If the firmwareUrl is changed to a new version, also change the minFirmwareVer const.
const std::string firmwareUrl = "https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Astra2_Release_2.8.20.zip";
const std::string minFirmwareVer = "2.8.20";
constexpr char service_name[] = "viam_orbbec";
const float mmToMeterMultiple = 0.001;
const uint64_t maxFrameAgeUs = 1e6;  // time until a frame is considered stale, in microseconds (equal to 1 sec)

// CONSTANTS END

// STRUCTS BEGIN
struct PointXYZRGB {
    float x, y, z;
    std::uint32_t rgb;
};

struct ViamOBDevice {
    ~ViamOBDevice() {
        std::cout << "deleting ViamOBDevice " << serial_number << "\n";
    }
    std::string serial_number;
    std::shared_ptr<ob::Device> device;
    bool started;
    std::shared_ptr<ob::Pipeline> pipe;
    std::shared_ptr<ob::PointCloudFilter> pointCloudFilter;
    std::shared_ptr<ob::Align> align;
    std::shared_ptr<ob::Config> config;
};

// STRUCTS END

// GLOBALS BEGIN
// Using the "Construct on First Use Idiom" to prevent the static deinitialization order fiasco.
// See https://isocpp.org/wiki/faq/ctors#static-init-order
//
// These functions wrap the static objects, ensuring they are constructed only when first accessed
// and correctly destructed to prevent the non-deterministic deinitialization such as the one that caused:
// https://viam.atlassian.net/browse/RSDK-11170
std::mutex& serial_by_resource_mu() {
    static std::mutex mu;
    return mu;
}

std::unordered_map<std::string, std::string>& serial_by_resource() {
    static std::unordered_map<std::string, std::string> devices;
    return devices;
}

std::mutex& devices_by_serial_mu() {
    static std::mutex mu;
    return mu;
}

std::unordered_map<std::string, std::unique_ptr<ViamOBDevice>>& devices_by_serial() {
    static std::unordered_map<std::string, std::unique_ptr<ViamOBDevice>> devices;
    return devices;
}

std::mutex& frame_set_by_serial_mu() {
    static std::mutex mu;
    return mu;
}

std::unordered_map<std::string, std::shared_ptr<ob::FrameSet>>& frame_set_by_serial() {
    static std::unordered_map<std::string, std::shared_ptr<ob::FrameSet>> frame_sets;
    return frame_sets;
}

std::mutex& config_by_serial_mu() {
    static std::mutex mu;
    return mu;
}

std::unordered_map<std::string, ObResourceConfig>& config_by_serial() {
    static std::unordered_map<std::string, ObResourceConfig> config;
    return config;
}
// GLOBALS END

// HELPERS BEGIN
void checkFirmwareVersion(const std::string version) {
    int major = 0, minor = 0, patch = 0;
    int requiredMajor = 0, requiredMinor = 0, requiredPatch = 0;
    // ignore any trailing text in the case of a beta or RC version
    sscanf(version.c_str(), "%d.%d.%d*s", &major, &minor, &patch);
    sscanf(minFirmwareVer.c_str(), "%d.%d.%d", &requiredMajor, &requiredMinor, &requiredPatch);
    if ((major < requiredMajor) || (major == requiredMajor && minor < requiredMinor) ||
        (major == requiredMajor && minor == requiredMinor && patch < requiredPatch)) {
        throw std::runtime_error("Unsupported firmware version. Required: >= 2.8.20, Current: " + version +
                                 ". Call update_firmware command to upgrade.");
    }
}

uint64_t getNowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

uint64_t timeSinceFrameUs(uint64_t nowUs, uint64_t imageTimeUs) {
    if (nowUs > imageTimeUs) {
        return nowUs - imageTimeUs;
    }
    return 0;
}

std::vector<std::uint8_t> RGBPointsToPCD(std::shared_ptr<ob::Frame> frame, float scale) {
    int numPoints = frame->dataSize() / sizeof(OBColorPoint);

    OBColorPoint* points = (OBColorPoint*)frame->data();
    std::vector<PointXYZRGB> pcdPoints;

    for (int i = 0; i < numPoints; i++) {
        OBColorPoint& p = points[i];
        std::uint32_t r = (std::uint32_t)p.r;
        std::uint32_t g = (std::uint32_t)p.g;
        std::uint32_t b = (std::uint32_t)p.b;
        std::uint32_t rgb = (r << 16) | (g << 8) | b;
        PointXYZRGB pt;
        pt.x = (p.x * scale);
        pt.y = (p.y * scale);
        pt.z = (p.z * scale);
        pt.rgb = rgb;
        pcdPoints.push_back(pt);
    }

    std::stringstream header;
    header << "VERSION .7\n"
           << "FIELDS x y z rgb\n"
           << "SIZE 4 4 4 4\n"
           << "TYPE F F F U\n"
           << "COUNT 1 1 1 1\n"
           << "WIDTH " << pcdPoints.size() << "\n"  // Use valid points count
           << "HEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << pcdPoints.size() << "\n"
           << "DATA binary\n";
    std::string headerStr = header.str();
    std::vector<std::uint8_t> pcdBytes;
    pcdBytes.insert(pcdBytes.end(), headerStr.begin(), headerStr.end());
    for (auto& p : pcdPoints) {
        std::uint8_t* x = (std::uint8_t*)&p.x;
        std::uint8_t* y = (std::uint8_t*)&p.y;
        std::uint8_t* z = (std::uint8_t*)&p.z;
        std::uint8_t* rgb = (std::uint8_t*)&p.rgb;

        pcdBytes.push_back(x[0]);
        pcdBytes.push_back(x[1]);
        pcdBytes.push_back(x[2]);
        pcdBytes.push_back(x[3]);

        pcdBytes.push_back(y[0]);
        pcdBytes.push_back(y[1]);
        pcdBytes.push_back(y[2]);
        pcdBytes.push_back(y[3]);

        pcdBytes.push_back(z[0]);
        pcdBytes.push_back(z[1]);
        pcdBytes.push_back(z[2]);
        pcdBytes.push_back(z[3]);

        pcdBytes.push_back(rgb[0]);
        pcdBytes.push_back(rgb[1]);
        pcdBytes.push_back(rgb[2]);
        pcdBytes.push_back(rgb[3]);
    }

    return pcdBytes;
}

void printDeviceList(const std::shared_ptr<ob::DeviceList> devList) {
    int devCount = devList->getCount();
    for (size_t i = 0; i < devCount; i++) {
        VIAM_SDK_LOG(info) << "DeviceListElement:" << i;
        VIAM_SDK_LOG(info) << "  Name:              " << devList->name(i);
        VIAM_SDK_LOG(info) << "  Serial Number:     " << devList->serialNumber(i);
        VIAM_SDK_LOG(info) << "  UID:               " << devList->uid(i);
        VIAM_SDK_LOG(info) << "  VID:               " << devList->vid(i);
        VIAM_SDK_LOG(info) << "  PID:               " << devList->pid(i);
        VIAM_SDK_LOG(info) << "  Connection Type:   " << devList->connectionType(i);
    }
}
void printDeviceInfo(const std::shared_ptr<ob::DeviceInfo> info) {
    VIAM_SDK_LOG(info) << "DeviceInfo:\n"
                       << "  Name:              " << info->name() << "\n"
                       << "  Serial Number:     " << info->serialNumber() << "\n"
                       << "  UID:               " << info->uid() << "\n"
                       << "  VID:               " << info->vid() << "\n"
                       << "  PID:               " << info->pid() << "\n"
                       << "  Connection Type:   " << info->connectionType() << "\n"
                       << "  Firmware Version:  " << info->firmwareVersion() << "\n"
                       << "  Hardware Version:  " << info->hardwareVersion() << "\n"
                       << "  Min SDK Version:   " << info->supportedMinSdkVersion() << "\n"
                       << "  ASIC::             " << info->asicName() << "\n";
}

// check if the given stream profiles support hardware depth-to-color
// alignment
bool checkIfSupportHWD2CAlign(std::shared_ptr<ob::Pipeline> pipe,
                              std::shared_ptr<ob::StreamProfile> colorStreamProfile,
                              std::shared_ptr<ob::StreamProfile> depthStreamProfile) {
    auto hwD2CSupportedDepthStreamProfiles = pipe->getD2CDepthProfileList(colorStreamProfile, ALIGN_D2C_HW_MODE);
    if (hwD2CSupportedDepthStreamProfiles->count() == 0) {
        return false;
    }

    // Iterate through the supported depth stream profiles and check if there is
    // a match with the given depth stream profile
    auto depthVsp = depthStreamProfile->as<ob::VideoStreamProfile>();
    auto count = hwD2CSupportedDepthStreamProfiles->getCount();
    for (uint32_t i = 0; i < count; i++) {
        auto sp = hwD2CSupportedDepthStreamProfiles->getProfile(i);
        auto vsp = sp->as<ob::VideoStreamProfile>();
        if (vsp->getWidth() == depthVsp->getWidth() && vsp->getHeight() == depthVsp->getHeight() &&
            vsp->getFormat() == depthVsp->getFormat() && vsp->getFps() == depthVsp->getFps()) {
            VIAM_SDK_LOG(info) << "[checkIfSupportHWD2CAlign] using width: " << vsp->getWidth() << " height: " << vsp->getHeight()
                               << " format: " << ob::TypeHelper::convertOBFormatTypeToString(vsp->getFormat()) << " fps: " << vsp->getFps()
                               << "\n";
            // Found a matching depth stream profile, it means the given stream
            // profiles support hardware depth-to-color alignment
            return true;
        }
    }
    return false;
}

// create a config for hardware depth-to-color alignment
std::shared_ptr<ob::Config> createHwD2CAlignConfig(std::shared_ptr<ob::Pipeline> pipe,
                                                   std::optional<DeviceResolution> deviceRes,
                                                   std::optional<DeviceFormat> deviceFormat) {
    auto colorStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_COLOR);
    auto depthStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
    if (deviceRes.has_value()) {
        VIAM_SDK_LOG(info) << "[createHwD2CAlignConfig] resolution specified: " << deviceRes->to_string();
    }
    if (deviceFormat.has_value()) {
        VIAM_SDK_LOG(info) << "[createHwD2CAlignConfig] format specified: " << deviceFormat->to_string();
    }

    // Iterate through all color and depth stream profiles to find a match for
    // hardware depth-to-color alignment
    auto colorSpCount = colorStreamProfiles->getCount();
    auto depthSpCount = depthStreamProfiles->getCount();
    for (uint32_t i = 0; i < colorSpCount; i++) {
        auto colorProfile = colorStreamProfiles->getProfile(i);
        auto colorVsp = colorProfile->as<ob::VideoStreamProfile>();

        if (deviceFormat.has_value() and deviceFormat->color_format.has_value()) {
            if (ob::TypeHelper::convertOBFormatTypeToString(colorVsp->getFormat()) != deviceFormat->color_format.value()) {
                continue;
            }
        } else if (ob::TypeHelper::convertOBFormatTypeToString(colorVsp->getFormat()) != Orbbec::default_color_format) {
            continue;
        }

        if (deviceRes.has_value() and deviceRes->color_resolution.has_value()) {
            if (colorVsp->getWidth() != deviceRes->color_resolution->width ||
                colorVsp->getHeight() != deviceRes->color_resolution->height) {
                continue;
            }
        } else if (colorVsp->getWidth() != Orbbec::default_color_resolution.width ||
                   colorVsp->getHeight() != Orbbec::default_color_resolution.height) {
            continue;
        }

        for (uint32_t j = 0; j < depthSpCount; j++) {
            auto depthProfile = depthStreamProfiles->getProfile(j);
            auto depthVsp = depthProfile->as<ob::VideoStreamProfile>();

            // make sure the color and depth stream have the same fps, due to some
            // models may not support different fps
            if (colorVsp->getFps() != depthVsp->getFps()) {
                continue;
            }

            if (deviceFormat.has_value() and deviceFormat->depth_format.has_value()) {
                if (ob::TypeHelper::convertOBFormatTypeToString(depthVsp->getFormat()) != deviceFormat->depth_format.value()) {
                    continue;
                }
            } else if (ob::TypeHelper::convertOBFormatTypeToString(depthVsp->getFormat()) != Orbbec::default_depth_format) {
                continue;
            }

            if (deviceRes.has_value() and deviceRes->depth_resolution.has_value()) {
                if (depthVsp->getWidth() != deviceRes->depth_resolution->width ||
                    depthVsp->getHeight() != deviceRes->depth_resolution->height) {
                    continue;
                }
            } else if (depthVsp->getWidth() != Orbbec::default_depth_resolution.width ||
                       depthVsp->getHeight() != Orbbec::default_depth_resolution.height) {
                continue;
            }

            // Check if the given stream profiles support hardware depth-to-color
            // alignment
            if (checkIfSupportHWD2CAlign(pipe, colorProfile, depthProfile)) {
                VIAM_SDK_LOG(info) << "[createHwD2CAlignConfig] Using hardware depth-to-color alignment with color stream "
                                   << colorVsp->getWidth() << "x" << colorVsp->getHeight() << "@" << colorVsp->getFps()
                                   << ", format: " << ob::TypeHelper::convertOBFormatTypeToString(colorVsp->getFormat())
                                   << " and depth stream " << depthVsp->getWidth() << "x" << depthVsp->getHeight() << "@"
                                   << depthVsp->getFps()
                                   << " format: " << ob::TypeHelper::convertOBFormatTypeToString(depthVsp->getFormat()) << "\n";
                // If support, create a config for hardware depth-to-color alignment
                auto hwD2CAlignConfig = std::make_shared<ob::Config>();
                hwD2CAlignConfig->enableStream(colorProfile);       // enable color stream
                hwD2CAlignConfig->enableStream(depthProfile);       // enable depth stream
                hwD2CAlignConfig->setAlignMode(ALIGN_D2C_HW_MODE);  // enable hardware depth-to-color alignment
                hwD2CAlignConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);  // output
                                                                                                                  // frameset
                                                                                                                  // with all
                                                                                                                  // types of
                                                                                                                  // frames
                return hwD2CAlignConfig;
            } else {
                VIAM_SDK_LOG(error) << "[createHwD2CAlignConfig] color stream " << colorVsp->getWidth() << "x" << colorVsp->getHeight()
                                    << "@" << colorVsp->getFps() << " and depth stream " << depthVsp->getWidth() << "x"
                                    << depthVsp->getHeight() << "@" << depthVsp->getFps()
                                    << " do NOT support hardware depth-to-color alignment\n";
            }
        }
    }

    VIAM_SDK_LOG(error) << "[createHwD2CAlignConfig] Could not find matching stream profiles for hardware depth-to-color alignment that "
                           "also match the given resolution and format specification ("
                        << (deviceRes.has_value() ? deviceRes->to_string() : "none") << ", "
                        << (deviceFormat.has_value() ? deviceFormat->to_string() : "none") << ")\n";
    return nullptr;
}

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

void updateFirmware(std::unique_ptr<ViamOBDevice>& my_dev, std::shared_ptr<ob::Context> ctx) {
// On linux, orbbec reccomends to set libuvc backend for firmware update
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
        buffer << "curl early perform failed: " << curl_easy_strerror(res);
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
        VIAM_SDK_LOG(info) << "firmware update successful!";
    } catch (...) {
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

auto frameCallback(const std::string& serialNumber) {
    return [serialNumber](std::shared_ptr<ob::FrameSet> frameSet) {
        if (frameSet->getCount() != 2) {
            std::cerr << "got non 2 frame count: " << frameSet->getCount() << "\n";
            return;
        }
        std::shared_ptr<ob::Frame> color = frameSet->getFrame(OB_FRAME_COLOR);
        if (color == nullptr) {
            std::cerr << "no color frame\n" << frameSet->getCount() << "\n";
            return;
        }

        std::shared_ptr<ob::Frame> depth = frameSet->getFrame(OB_FRAME_DEPTH);
        if (depth == nullptr) {
            std::cerr << "no depth frame\n";
            return;
        }

        std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
        uint64_t nowUs = getNowUs();
        uint64_t diff = timeSinceFrameUs(nowUs, color->getSystemTimeStampUs());
        if (diff > maxFrameAgeUs) {
            std::cerr << "color frame is " << diff << "us older than now, nowUs: " << nowUs << " frameTimeUs "
                      << color->getSystemTimeStampUs() << "\n";
        }
        diff = timeSinceFrameUs(nowUs, depth->getSystemTimeStampUs());
        if (diff > maxFrameAgeUs) {
            std::cerr << "depth frame is " << diff << "us older than now, nowUs: " << nowUs << " frameTimeUs "
                      << depth->getSystemTimeStampUs() << "\n";
        }

        auto it = frame_set_by_serial().find(serialNumber);
        if (it != frame_set_by_serial().end()) {
            std::shared_ptr<ob::Frame> prevColor = it->second->getFrame(OB_FRAME_COLOR);
            std::shared_ptr<ob::Frame> prevDepth = it->second->getFrame(OB_FRAME_DEPTH);
            if (prevColor != nullptr && prevDepth != nullptr) {
                diff = timeSinceFrameUs(color->getSystemTimeStampUs(), prevColor->getSystemTimeStampUs());
                if (diff > maxFrameAgeUs) {
                    std::cerr << "previous color frame is " << diff
                              << "us older than current color frame. previousUs: " << prevColor->getSystemTimeStampUs()
                              << " currentUs: " << color->getSystemTimeStampUs() << "\n";
                }
                diff = timeSinceFrameUs(depth->getSystemTimeStampUs(), prevDepth->getSystemTimeStampUs());
                if (diff > maxFrameAgeUs) {
                    std::cerr << "previous depth frame is " << diff
                              << "us older than current depth frame. previousUs: " << prevDepth->getSystemTimeStampUs()
                              << " currentUs: " << depth->getSystemTimeStampUs() << "\n";
                }
            }
        }
        frame_set_by_serial()[serialNumber] = frameSet;
    };
}

void startDevice(std::string serialNumber) {
    VIAM_SDK_LOG(info) << service_name << ": starting device " << serialNumber;
    std::lock_guard<std::mutex> lock(devices_by_serial_mu());
    auto search = devices_by_serial().find(serialNumber);
    if (search == devices_by_serial().end()) {
        std::ostringstream buffer;
        buffer << service_name << ": unable to start undetected device" << serialNumber;
        throw std::invalid_argument(buffer.str());
    }

    // Check if the device is an Astra 2
    std::shared_ptr<ob::DeviceInfo> deviceInfo = search->second->device->getDeviceInfo();
    if (!strstr(deviceInfo->name(), "Astra 2")) {
        std::ostringstream buffer;
        buffer << service_name << ": device " << serialNumber << " is not an Astra 2 (found: " << deviceInfo->name() << ")";
        throw std::invalid_argument(buffer.str());
    }

    std::unique_ptr<ViamOBDevice>& my_dev = search->second;
    if (my_dev->started) {
        std::ostringstream buffer;
        buffer << service_name << ": unable to start already started device" << serialNumber;
        throw std::invalid_argument(buffer.str());
    }
    VIAM_SDK_LOG(info) << "[startDevice] Configuring device resolution";
    {
        std::lock_guard<std::mutex> lock(config_by_serial_mu());
        if (config_by_serial().count(serialNumber) == 0) {
            std::ostringstream buffer;
            buffer << service_name << ": device with serial number " << serialNumber << " is not in config_by_serial, skipping start";
            throw std::runtime_error(buffer.str());
        }
        auto resolution_opt = config_by_serial().at(serialNumber).device_resolution;
        auto format_opt = config_by_serial().at(serialNumber).device_format;
        VIAM_SDK_LOG(info) << "[startDevice] Resolution from config: ";
        if (resolution_opt.has_value() or format_opt.has_value()) {
            // Create the pipeline
            auto config = createHwD2CAlignConfig(search->second->pipe, resolution_opt, format_opt);
            if (config == nullptr) {
                std::ostringstream buffer;
                buffer << service_name << ": device with serial number " << serialNumber
                       << " does not support hardware depth-to-color alignment with the requested parameters: resolution "
                       << (resolution_opt.has_value() ? resolution_opt->to_string() : "none") << ", "
                       << (format_opt.has_value() ? format_opt->to_string() : "none") << "\n";
                VIAM_SDK_LOG(error) << buffer.str();
                throw std::runtime_error(buffer.str());
            }
            my_dev->config = config;
        } else {
            VIAM_SDK_LOG(info) << "  not specified, using default";
        }
    }

    my_dev->pipe->start(my_dev->config, frameCallback(serialNumber));
    my_dev->started = true;

    // Ensure we start getting frames within 500ms, otherwise something is
    // wrong with the pipeline and we should restart.
    auto start_time = std::chrono::steady_clock::now();
    bool got_frame = false;
    while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(500)) {
        {
            std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
            if (frame_set_by_serial().count(serialNumber) > 0) {
                got_frame = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (!got_frame) {
        VIAM_SDK_LOG(error) << "did not get frame within 500ms of starting device, resetting";
        my_dev->pipe->stop();
        my_dev->started = false;
        my_dev->pipe->start(my_dev->config, frameCallback(serialNumber));
        my_dev->started = true;
    }

    VIAM_SDK_LOG(info) << service_name << ": device started " << serialNumber;
}

void stopDevice(std::string serialNumber, std::string resourceName) {
    std::lock_guard<std::mutex> lock(devices_by_serial_mu());

    auto search = devices_by_serial().find(serialNumber);
    if (search == devices_by_serial().end()) {
        VIAM_SDK_LOG(error) << service_name << ": unable to stop undetected device " << serialNumber;
        return;
    }

    std::unique_ptr<ViamOBDevice>& my_dev = search->second;
    if (!my_dev->started) {
        VIAM_SDK_LOG(warn) << service_name << ": device " << serialNumber << " was not running, cleaning up resource mapping";
        // Clean up the resource mapping even if device wasn't running
        {
            std::lock_guard<std::mutex> lock(serial_by_resource_mu());
            serial_by_resource().erase(resourceName);
        }
        return;
    }

    my_dev->pipe->stop();
    my_dev->started = false;
    {
        std::lock_guard<std::mutex> lock(serial_by_resource_mu());
        serial_by_resource().erase(resourceName);
    }
}

void listDevices(const ob::Context& ctx) {
    try {
        auto devList = ctx.queryDeviceList();
        int devCount = devList->getCount();
        VIAM_SDK_LOG(info) << "devCount: " << devCount << "\n";

        std::shared_ptr<ob::Device> dev = nullptr;
        std::shared_ptr<ob::DeviceInfo> info = nullptr;
        for (size_t i = 0; i < devCount; i++) {
            dev = devList->getDevice(i);
            info = dev->getDeviceInfo();
            printDeviceInfo(info);
            dev.reset();
        }
    } catch (ob::Error& e) {
        std::cerr << "listDevices\n"
                  << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nname:" << e.getName() << "\nmessage:" << e.what()
                  << "\ntype:" << e.getExceptionType() << std::endl;
        throw e;
    }
}

// HELPERS END

// RESOURCE BEGIN
std::vector<std::string> Orbbec::validate(vsdk::ResourceConfig cfg) {
    auto attrs = cfg.attributes();

    if (not attrs.count("serial_number")) {
        throw std::invalid_argument("serial_number is a required argument");
    }

    if (!attrs["serial_number"].get<std::string>()) {
        throw std::invalid_argument("serial_number must be a string");
    }

    // We already established this is a string, so it's safe to call this
    std::string const serial = attrs["serial_number"].get_unchecked<std::string>();
    if (serial.empty()) {
        throw std::invalid_argument("serial_number must be a non-empty string");
    }

    if (attrs.count("format")) {
        auto format = attrs["format"].get<viam::sdk::ProtoStruct>();
        if (format) {
            if (!format->count("color") || !format->count("depth")) {
                throw std::invalid_argument("format must contain color and depth keys");
            }
            auto color_format = format->at("color").get<std::string>();
            if (!color_format) {
                throw std::invalid_argument("color format must be a string");
            }
            if (Orbbec::supported_color_formats.count(*color_format) == 0) {
                std::ostringstream buffer;
                buffer << "color format must be one of: ";
                for (const auto& fmt : Orbbec::supported_color_formats) {
                    buffer << fmt << " ";
                }
                VIAM_SDK_LOG(error) << buffer.str();
                throw std::invalid_argument(buffer.str());
            }

            auto depth_format = format->at("depth").get<std::string>();
            if (!depth_format) {
                throw std::invalid_argument("depth format must be a string");
            }
            if (Orbbec::supported_depth_formats.count(*depth_format) == 0) {
                std::ostringstream buffer;
                buffer << "depth format must be one of: ";
                for (const auto& fmt : Orbbec::supported_depth_formats) {
                    buffer << fmt << " ";
                }
                VIAM_SDK_LOG(error) << buffer.str();
                throw std::invalid_argument(buffer.str());
            }
        }
    }

    if (attrs.count("resolution")) {
        auto resolution = attrs["resolution"].get<viam::sdk::ProtoStruct>();
        if (resolution) {
            if (!resolution->count("color") || !resolution->count("depth")) {
                throw std::invalid_argument("resolution must contain color and depth keys");
            }
            auto color_resolution = resolution->at("color").get<viam::sdk::ProtoStruct>();
            if (color_resolution->count("width") == 0 || color_resolution->count("height") == 0) {
                throw std::invalid_argument("color must contain width and height keys");
            }
            auto color_width = color_resolution->at("width").get<double>();
            auto color_height = color_resolution->at("height").get<double>();
            if (!color_width || !color_height) {
                throw std::invalid_argument("color width and height must be doubles");
            }
            if (*color_width <= 0 || *color_height <= 0) {
                throw std::invalid_argument("color width and height must be positive");
            }
            auto color_width_uint32 = static_cast<std::uint32_t>(*color_width);
            auto color_height_uint32 = static_cast<std::uint32_t>(*color_height);

            auto depth_resolution = resolution->at("depth").get<viam::sdk::ProtoStruct>();
            if (depth_resolution->count("width") == 0 || depth_resolution->count("height") == 0) {
                throw std::invalid_argument("depth must contain width and height keys");
            }
            auto depth_width = depth_resolution->at("width").get<double>();
            auto depth_height = depth_resolution->at("height").get<double>();
            if (!depth_width || !depth_height) {
                throw std::invalid_argument("depth width and height must be double");
            }
            if (*depth_width <= 0 || *depth_height <= 0) {
                throw std::invalid_argument("depth width and height must be positive");
            }

            auto depth_width_uint32 = static_cast<std::uint32_t>(*depth_width);
            auto depth_height_uint32 = static_cast<std::uint32_t>(*depth_height);
            if (color_to_depth_supported_resolutions.count({color_width_uint32, color_height_uint32}) == 0) {
                std::ostringstream buffer;
                buffer << "color resolution must be one of: ";
                for (const auto& res : color_to_depth_supported_resolutions) {
                    buffer << "{" << res.first.to_string() << "} ";
                }
                VIAM_SDK_LOG(error) << buffer.str();
                throw std::invalid_argument(buffer.str());
            }
            if (color_to_depth_supported_resolutions.at({color_width_uint32, color_height_uint32})
                    .count({depth_width_uint32, depth_height_uint32}) == 0) {
                std::ostringstream buffer;
                buffer << "color/depth resolution combination not supported, for color resolution " << "{" << color_width_uint32 << ", "
                       << color_height_uint32 << "}, depth resolution must be one of: ";
                for (const auto& res : color_to_depth_supported_resolutions.at({color_width_uint32, color_height_uint32})) {
                    buffer << "{" << res.to_string() << "} ";
                }
                VIAM_SDK_LOG(error) << buffer.str();
                throw std::invalid_argument(buffer.str());
            }
        }
    }

    return {};
}

std::string getSerialNumber(const vsdk::ResourceConfig& cfg) {
    auto attrs = cfg.attributes();
    if (attrs.count("serial_number")) {
        auto serial = attrs.at("serial_number").get<std::string>();
        if (serial) {
            return *serial;
        }
    }
    throw std::invalid_argument("serial_number is a required argument");
}

Orbbec::Orbbec(vsdk::Dependencies deps, vsdk::ResourceConfig cfg, std::shared_ptr<ob::Context> ctx)
    : Camera(cfg.name()), serial_number_(getSerialNumber(cfg)), ob_ctx_(std::move(ctx)) {
    VIAM_SDK_LOG(info) << "Orbbec constructor start " << serial_number_;
    auto config = configure(deps, cfg);
    {
        std::lock_guard<std::mutex> lock(config_by_serial_mu());
        config_by_serial().insert_or_assign(serial_number_, *config);
        VIAM_SDK_LOG(info) << "initial config_by_serial_: " << config_by_serial().at(serial_number_).to_string();
    }
    startDevice(serial_number_);
    {
        std::lock_guard<std::mutex> lock(serial_by_resource_mu());
        serial_by_resource()[config->resource_name] = serial_number_;
    }

    // set firmware version  member variable
    {
        std::lock_guard<std::mutex> lock(devices_by_serial_mu());
        auto search = devices_by_serial().find(serial_number_);
        if (search != devices_by_serial().end()) {
            firmware_version_ = search->second->device->getDeviceInfo()->firmwareVersion();
        }
    }

    VIAM_SDK_LOG(info) << "Orbbec constructor end " << serial_number_;
}

Orbbec::~Orbbec() {
    VIAM_SDK_LOG(info) << "Orbbec destructor start " << serial_number_;
    std::string prev_serial_number;
    std::string prev_resource_name;
    {
        const std::lock_guard<std::mutex> lock(config_by_serial_mu());
        if (config_by_serial().count(serial_number_) == 0) {
            VIAM_SDK_LOG(error) << "Orbbec destructor: device with serial number " << serial_number_
                                << " is not in config_by_serial, skipping erase";
        } else {
            prev_serial_number = config_by_serial().at(serial_number_).serial_number;
            prev_resource_name = config_by_serial().at(serial_number_).resource_name;
        }
    }
    stopDevice(prev_serial_number, prev_resource_name);
    VIAM_SDK_LOG(info) << "Orbbec destructor end " << serial_number_;
}

void Orbbec::reconfigure(const vsdk::Dependencies& deps, const vsdk::ResourceConfig& cfg) {
    VIAM_SDK_LOG(info) << "[reconfigure] Orbbec reconfigure start";
    std::string prev_serial_number;
    std::string prev_resource_name;
    {
        const std::lock_guard<std::mutex> lock_serial(serial_number_mu_);
        const std::lock_guard<std::mutex> lock(config_by_serial_mu());
        if (config_by_serial().count(serial_number_) == 0) {
            std::ostringstream buffer;
            buffer << "[reconfigure] device with serial number " << serial_number_ << " is not in config_by_serial, skipping reconfigure";
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::runtime_error(buffer.str());
        } else {
            prev_serial_number = config_by_serial().at(serial_number_).serial_number;
            prev_resource_name = config_by_serial().at(serial_number_).resource_name;
        }
    }
    stopDevice(prev_serial_number, prev_resource_name);
    std::string new_serial_number;
    std::string new_resource_name;
    {
        auto config = configure(deps, cfg);
        {
            const std::lock_guard<std::mutex> lock(config_by_serial_mu());
            config_by_serial().erase(prev_serial_number);
            config_by_serial().insert_or_assign(config->serial_number, *config);
        }
        {
            const std::lock_guard<std::mutex> lock(serial_number_mu_);
            serial_number_ = config->serial_number;
        }
        new_serial_number = config->serial_number;
        new_resource_name = config->resource_name;
        VIAM_SDK_LOG(info) << "[reconfigure] updated config_by_serial_: " << config_by_serial().at(new_serial_number).to_string();
    }

    {
        std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
        frame_set_by_serial().erase(prev_serial_number);
    }

    startDevice(new_serial_number);
    {
        std::lock_guard<std::mutex> lock(serial_by_resource_mu());
        serial_by_resource()[new_resource_name] = new_serial_number;
    }

    // set firmware version member variable
    {
        std::lock_guard<std::mutex> lock_serial(serial_number_mu_);
        std::lock_guard<std::mutex> lock(devices_by_serial_mu());
        auto search = devices_by_serial().find(serial_number_);
        if (search != devices_by_serial().end()) {
            firmware_version_ = search->second->device->getDeviceInfo()->firmwareVersion();
        }
    }
    VIAM_SDK_LOG(info) << "[reconfigure] Orbbec reconfigure end";
}

vsdk::Camera::raw_image Orbbec::get_image(std::string mime_type, const vsdk::ProtoStruct& extra) {
    try {
        VIAM_SDK_LOG(debug) << "[get_image] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(serial_number_mu_);
            serial_number = serial_number_;
        }

        checkFirmwareVersion(firmware_version_);

        std::shared_ptr<ob::FrameSet> fs = nullptr;
        {
            std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
            auto search = frame_set_by_serial().find(serial_number);
            if (search == frame_set_by_serial().end()) {
                throw std::invalid_argument("no frame yet");
            }
            fs = search->second;
        }
        std::shared_ptr<ob::Frame> color = fs->getFrame(OB_FRAME_COLOR);
        if (color == nullptr) {
            throw std::invalid_argument("no color frame");
        }

        if (supported_color_formats.count(ob::TypeHelper::convertOBFormatTypeToString(color->getFormat())) == 0) {
            std::ostringstream buffer;
            buffer << "unsupported color format: " << ob::TypeHelper::convertOBFormatTypeToString(color->getFormat())
                   << ", supported formats: ";
            for (const auto& fmt : supported_color_formats) {
                buffer << fmt << " ";
            }
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::invalid_argument(buffer.str());
        }

        std::optional<DeviceFormat> res_format_opt;
        {
            std::lock_guard<std::mutex> lock(config_by_serial_mu());
            if (config_by_serial().count(serial_number) == 0) {
                throw std::invalid_argument("device with serial number " + serial_number + " is not in config_by_serial");
            }
            res_format_opt = config_by_serial().at(serial_number).device_format;
        }

        if (res_format_opt.has_value()) {
            if (res_format_opt->color_format.has_value() &&
                ob::TypeHelper::convertOBFormatTypeToString(color->getFormat()) != res_format_opt->color_format.value()) {
                std::ostringstream buffer;
                buffer << "color frame format " << ob::TypeHelper::convertOBFormatTypeToString(color->getFormat())
                       << " does not match configured color format " << res_format_opt->color_format.value();
                VIAM_SDK_LOG(error) << buffer.str();
                throw std::invalid_argument(buffer.str());
            }
        } else if (ob::TypeHelper::convertOBFormatTypeToString(color->getFormat()) != Orbbec::default_color_format) {
            std::ostringstream buffer;
            buffer << "color frame format " << ob::TypeHelper::convertOBFormatTypeToString(color->getFormat())
                   << " does not match default color format " << Orbbec::default_color_format;
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::invalid_argument(buffer.str());
        }

        // If the image's timestamp is older than a second throw error, this
        // indicates we no longer have a working camera.
        uint64_t nowUs = getNowUs();
        uint64_t diff = timeSinceFrameUs(nowUs, color->getSystemTimeStampUs());
        if (diff > maxFrameAgeUs) {
            std::ostringstream buffer;
            buffer << "no recent color frame: check USB connection, diff: " << diff << "us";
            throw std::invalid_argument(buffer.str());
        }

        vsdk::Camera::raw_image response;
        response.source_name = kColorSourceName;

        std::uint8_t* colorData = (std::uint8_t*)color->getData();
        if (colorData == nullptr) {
            throw std::runtime_error("[get_image] color data is null");
        }
        uint32_t colorDataSize = color->dataSize();

        if (color->getFormat() == OB_FORMAT_MJPG) {
            response.mime_type = kColorMimeTypeJPEG;
            response.bytes.assign(colorData, colorData + colorDataSize);
        } else if (color->getFormat() == OB_FORMAT_RGBA) {
            response.mime_type = kColorMimeTypePNG;
            auto width = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getWidth();
            auto height = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getHeight();
            response.bytes = encoding::encode_to_png(colorData, width, height, encoding::ImageFormat::RGBA);
        } else if (color->getFormat() == OB_FORMAT_RGB) {
            response.mime_type = kColorMimeTypePNG;
            auto width = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getWidth();
            auto height = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getHeight();
            response.bytes = encoding::encode_to_png(colorData, width, height, encoding::ImageFormat::RGB);
        } else {
            std::ostringstream buffer;
            buffer << "[get_image] unsupported color format: " << ob::TypeHelper::convertOBFormatTypeToString(color->getFormat());
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::invalid_argument(buffer.str());
        }
        VIAM_SDK_LOG(debug) << "[get_image] end";
        return response;
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "[get_image] error: " << e.what();
        throw std::runtime_error("failed to create image: " + std::string(e.what()));
    }
}

vsdk::Camera::properties Orbbec::get_properties() {
    try {
        VIAM_SDK_LOG(debug) << "[get_properties] start";

        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(serial_number_mu_);
            if (serial_number_.empty()) {
                throw std::runtime_error("serial number is null");
            }
            serial_number = serial_number_;
        }

        OBCameraIntrinsic props;
        {
            const std::lock_guard<std::mutex> lock(devices_by_serial_mu());
            auto search = devices_by_serial().find(serial_number);
            if (search == devices_by_serial().end()) {
                std::ostringstream buffer;
                buffer << service_name << ": device with serial number " << serial_number << " is no longer connected";
                throw std::invalid_argument(buffer.str());
            }

            std::unique_ptr<ViamOBDevice>& my_dev = search->second;
            if (!my_dev->started) {
                std::ostringstream buffer;
                buffer << service_name << ": device with serial number " << serial_number << " is not started";
                throw std::invalid_argument(buffer.str());
            }
            props = my_dev->pipe->getCameraParam().rgbIntrinsic;
        }

        vsdk::Camera::properties p{};
        p.supports_pcd = true;
        p.intrinsic_parameters.width_px = props.width;
        p.intrinsic_parameters.height_px = props.height;
        p.intrinsic_parameters.focal_x_px = props.fx;
        p.intrinsic_parameters.focal_y_px = props.fy;
        p.intrinsic_parameters.center_x_px = props.cx;
        p.intrinsic_parameters.center_y_px = props.cy;
        // TODO: Set distortion parameters

        VIAM_SDK_LOG(debug) << "[get_properties] end";
        return p;
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "[get_properties] error: " << e.what();
        throw std::runtime_error("failed to create properties: " + std::string(e.what()));
    }
}

vsdk::Camera::image_collection Orbbec::get_images(std::vector<std::string> filter_source_names, const vsdk::ProtoStruct& extra) {
    try {
        VIAM_SDK_LOG(debug) << "[get_images] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(serial_number_mu_);
            serial_number = serial_number_;
        }

        checkFirmwareVersion(firmware_version_);

        std::shared_ptr<ob::FrameSet> fs = nullptr;
        {
            std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
            auto search = frame_set_by_serial().find(serial_number);
            if (search == frame_set_by_serial().end()) {
                throw std::invalid_argument("no frame yet");
            }
            fs = search->second;
        }

        bool should_process_color = false;
        bool should_process_depth = false;

        if (filter_source_names.empty()) {
            should_process_color = true;
            should_process_depth = true;
        } else {
            for (const auto& name : filter_source_names) {
                if (name == kColorSourceName) {
                    should_process_color = true;
                }
                if (name == kDepthSourceName) {
                    should_process_depth = true;
                }
            }
        }

        vsdk::Camera::image_collection response;
        std::shared_ptr<ob::Frame> color = nullptr;
        std::shared_ptr<ob::Frame> depth = nullptr;
        uint64_t nowUs = getNowUs();

        if (should_process_color) {
            color = fs->getFrame(OB_FRAME_COLOR);
            if (color == nullptr) {
                throw std::invalid_argument("no color frame");
            }
            if (supported_color_formats.count(ob::TypeHelper::convertOBFormatTypeToString(color->getFormat())) == 0) {
                std::ostringstream buffer;
                buffer << "[get_images] unsupported color format: " << ob::TypeHelper::convertOBFormatTypeToString(color->getFormat())
                       << ", supported: ";
                for (const auto& format : supported_color_formats) {
                    buffer << format << " ";
                }

                VIAM_SDK_LOG(error) << buffer.str();
                throw std::invalid_argument(buffer.str());
            }

            uint64_t diff = timeSinceFrameUs(nowUs, color->getSystemTimeStampUs());
            if (diff > maxFrameAgeUs) {
                std::ostringstream buffer;
                buffer << "no recent color frame: check USB connection, diff: " << diff << "us";
                throw std::invalid_argument(buffer.str());
            }

            if (color->getFormat() != OB_FORMAT_MJPG) {
                throw std::invalid_argument("color frame was not in jpeg format");
            }

            std::optional<DeviceFormat> res_format_opt;
            {
                std::lock_guard<std::mutex> lock(config_by_serial_mu());
                if (config_by_serial().count(serial_number) == 0) {
                    throw std::invalid_argument("device with serial number " + serial_number + " is not in config_by_serial");
                }
                res_format_opt = config_by_serial().at(serial_number).device_format;
            }
            if (res_format_opt.has_value()) {
                if (res_format_opt->color_format.has_value()) {
                    if (ob::TypeHelper::convertOBFormatTypeToString(color->getFormat()) != res_format_opt->color_format.value()) {
                        std::ostringstream buffer;
                        buffer << "color format mismatch, expected " << res_format_opt->color_format.value() << " got "
                               << ob::TypeHelper::convertOBFormatTypeToString(color->getFormat());
                        throw std::invalid_argument(buffer.str());
                    }
                }
                if (res_format_opt->depth_format.has_value()) {
                    if (ob::TypeHelper::convertOBFormatTypeToString(depth->getFormat()) != res_format_opt->depth_format.value()) {
                        std::ostringstream buffer;
                        buffer << "depth format mismatch, expected " << res_format_opt->depth_format.value() << " got "
                               << ob::TypeHelper::convertOBFormatTypeToString(depth->getFormat());
                        throw std::invalid_argument(buffer.str());
                    }
                }
            } else {
                if (ob::TypeHelper::convertOBFormatTypeToString(color->getFormat()) != Orbbec::default_color_format) {
                    std::ostringstream buffer;
                    buffer << "color format mismatch, expected " << Orbbec::default_color_format << " got "
                           << ob::TypeHelper::convertOBFormatTypeToString(color->getFormat());
                    throw std::invalid_argument(buffer.str());
                }
                if (ob::TypeHelper::convertOBFormatTypeToString(depth->getFormat()) != Orbbec::default_depth_format) {
                    std::ostringstream buffer;
                    buffer << "depth format mismatch, expected " << Orbbec::default_depth_format << " got "
                           << ob::TypeHelper::convertOBFormatTypeToString(depth->getFormat());
                    throw std::invalid_argument(buffer.str());
                }
            }

            std::uint8_t* colorData = (std::uint8_t*)color->getData();
            if (colorData == nullptr) {
                throw std::runtime_error("[get_image] color data is null");
            }
            std::uint32_t colorDataSize = color->dataSize();

            vsdk::Camera::raw_image color_image;

            if (color->getFormat() == OB_FORMAT_MJPG) {
                color_image.mime_type = kColorMimeTypeJPEG;
                color_image.bytes.assign(colorData, colorData + colorDataSize);
            } else if (color->getFormat() == OB_FORMAT_RGBA) {
                color_image.mime_type = kColorMimeTypePNG;
                auto width = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getWidth();
                auto height = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getHeight();
                color_image.bytes = encoding::encode_to_png(colorData, width, height, encoding::ImageFormat::RGBA);
            } else if (color->getFormat() == OB_FORMAT_RGB) {
                color_image.mime_type = kColorMimeTypePNG;
                auto width = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getWidth();
                auto height = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getHeight();
                color_image.bytes = encoding::encode_to_png(colorData, width, height, encoding::ImageFormat::RGB);
            } else {
                std::ostringstream buffer;
                buffer << "[get_images] unsupported color format: " << ob::TypeHelper::convertOBFormatTypeToString(color->getFormat());
                VIAM_SDK_LOG(error) << buffer.str();
                throw std::invalid_argument(buffer.str());
            }
        }

        if (should_process_depth) {
            depth = fs->getFrame(OB_FRAME_DEPTH);
            if (depth == nullptr) {
                throw std::invalid_argument("no depth frame");
            }

            uint64_t diff = timeSinceFrameUs(nowUs, depth->getSystemTimeStampUs());
            if (diff > maxFrameAgeUs) {
                std::ostringstream buffer;
                buffer << "no recent depth frame: check USB connection, diff: " << diff << "us";
                throw std::invalid_argument(buffer.str());
            }

            unsigned char* depthData = (unsigned char*)depth->getData();
            if (depthData == nullptr) {
                throw std::runtime_error("[get_images] depth data is null");
            }
            auto depthVid = depth->as<ob::VideoFrame>();

            vsdk::Camera::raw_image depth_image;
            depth_image.source_name = kDepthSourceName;
            depth_image.mime_type = kDepthMimeTypeViamDep;
            depth_image.bytes = encoding::encode_to_depth_raw(depthData, depthVid->getWidth(), depthVid->getHeight());
            response.images.emplace_back(std::move(depth_image));
        }

        if (response.images.empty()) {
            VIAM_SDK_LOG(error) << "[get_images] error: no camera sources matched the filter";
            return response;
        }

        uint64_t colorTS = color ? color->getSystemTimeStampUs() : 0;
        uint64_t depthTS = depth ? depth->getSystemTimeStampUs() : 0;
        uint64_t timestamp = 0;

        if (colorTS > 0 && depthTS > 0) {
            if (colorTS != depthTS) {
                VIAM_SDK_LOG(info) << "color and depth timestamps differ, defaulting to "
                                      "older of the two"
                                   << "color timestamp was " << colorTS << " depth timestamp was " << depthTS;
            }
            // use the older of the two timestamps
            timestamp = (colorTS > depthTS) ? depthTS : colorTS;
        } else if (colorTS > 0) {
            timestamp = colorTS;
        } else {
            timestamp = depthTS;
        }

        std::chrono::microseconds latestTimestamp(timestamp);
        response.metadata.captured_at = vsdk::time_pt{std::chrono::duration_cast<std::chrono::nanoseconds>(latestTimestamp)};
        VIAM_SDK_LOG(debug) << "[get_images] end";
        return response;
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "[get_images] error: " << e.what();
        throw std::runtime_error("failed to create images: " + std::string(e.what()));
    }
}

vsdk::ProtoStruct Orbbec::do_command(const vsdk::ProtoStruct& command) {
    viam::sdk::ProtoStruct resp = viam::sdk::ProtoStruct{};
    constexpr char firmware_key[] = "update_firmware";
    for (const auto& kv : command) {
        if (kv.first == firmware_key) {
            std::string serial_number;
            {
                const std::lock_guard<std::mutex> lock(serial_number_mu_);
                serial_number = serial_number_;
            }
            {
                // note: under lock for the entire firmware update to ensure device can't be destructed.
                const std::lock_guard<std::mutex> lock(devices_by_serial_mu());
                auto search = devices_by_serial().find(serial_number);
                if (search == devices_by_serial().end()) {
                    throw std::invalid_argument("device is not connected");
                }

                std::unique_ptr<ViamOBDevice>& dev = search->second;
                if (firmware_version_.find(minFirmwareVer) != std::string::npos) {
                    std::ostringstream buffer;
                    buffer << "device firmware already on version " << minFirmwareVer;
                    resp.emplace(firmware_key, buffer.str());
                    break;
                }
                if (dev->started) {
                    dev->pipe->stop();
                    dev->started = false;
                }

                VIAM_SDK_LOG(info) << "Updating device firmware...";
                try {
                    updateFirmware(dev, ob_ctx_);
                    firmware_version_ = minFirmwareVer;
                } catch (const std::exception& e) {
                    std::ostringstream buffer;
                    buffer << "firmware update failed: " << e.what();
                    VIAM_SDK_LOG(error) << buffer.str();
                    resp.emplace(firmware_key, buffer.str());
                    dev->pipe->start(dev->config, frameCallback(serial_number));
                    dev->started = true;
                }
// macOS has a bug where the device changed callback is not called on reboot, so user must manually reboot the device
#if defined(__linux__)
                dev->device->reboot();
                resp.emplace(firmware_key, std::string("firmware successfully updated"));
#else
                resp.emplace(firmware_key, std::string("firmware successfully updated, unplug and replug the device"));
#endif
            }
        }
    }
    return resp;
}

vsdk::Camera::point_cloud Orbbec::get_point_cloud(std::string mime_type, const vsdk::ProtoStruct& extra) {
    try {
        VIAM_SDK_LOG(debug) << "[get_point_cloud] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(serial_number_mu_);
            serial_number = serial_number_;
        }

        checkFirmwareVersion(firmware_version_);

        std::shared_ptr<ob::FrameSet> fs = nullptr;
        {
            std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
            auto search = frame_set_by_serial().find(serial_number);
            if (search == frame_set_by_serial().end()) {
                throw std::invalid_argument("no frame yet");
            }
            fs = search->second;
        }

        std::shared_ptr<ob::Frame> color = fs->getFrame(OB_FRAME_COLOR);
        if (color == nullptr) {
            throw std::invalid_argument("no color frame");
        }

        uint64_t nowUs = getNowUs();
        uint64_t diff = timeSinceFrameUs(nowUs, color->getSystemTimeStampUs());
        if (diff > maxFrameAgeUs) {
            std::ostringstream buffer;
            buffer << "no recent color frame: check USB connection, diff: " << diff << "us";
            throw std::invalid_argument(buffer.str());
        }

        std::shared_ptr<ob::Frame> depth = fs->getFrame(OB_FRAME_DEPTH);
        if (depth == nullptr) {
            throw std::invalid_argument("no depth frame");
        }

        diff = timeSinceFrameUs(nowUs, depth->getSystemTimeStampUs());
        if (diff > maxFrameAgeUs) {
            std::ostringstream buffer;
            buffer << "no recent depth frame: check USB connection, diff: " << diff << "us";
            throw std::invalid_argument(buffer.str());
        }

        std::uint8_t* colorData = (std::uint8_t*)color->getData();
        if (colorData == nullptr) {
            throw std::runtime_error("[get_image] color data is null");
        }
        std::uint32_t colorDataSize = color->dataSize();

        std::shared_ptr<ob::DepthFrame> depthFrame = depth->as<ob::DepthFrame>();
        float scale = depthFrame->getValueScale();

        std::uint8_t* depthData = (std::uint8_t*)depth->getData();
        if (depthData == nullptr) {
            throw std::runtime_error("[get_point_cloud] depth data is null");
        }

        // NOTE: UNDER LOCK
        std::lock_guard<std::mutex> lock(devices_by_serial_mu());
        auto search = devices_by_serial().find(serial_number);
        if (search == devices_by_serial().end()) {
            throw std::invalid_argument("device is not connected");
        }

        std::unique_ptr<ViamOBDevice>& my_dev = search->second;
        if (!my_dev->started) {
            throw std::invalid_argument("device is not started");
        }

        std::vector<std::uint8_t> data =
            RGBPointsToPCD(my_dev->pointCloudFilter->process(my_dev->align->process(fs)), scale * mmToMeterMultiple);

        VIAM_SDK_LOG(debug) << "[get_point_cloud] end";
        return vsdk::Camera::point_cloud{kPcdMimeType, data};
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "[get_point_cloud] error: " << e.what();
        throw std::runtime_error("failed to create pointcloud: " + std::string(e.what()));
    }
}

std::vector<vsdk::GeometryConfig> Orbbec::get_geometries(const vsdk::ProtoStruct& extra) {
    // see https://github.com/viam-modules/orbbec/pull/16 for explanation of geometry config values
    // If we add support for models other than the astra 2 these values can't be hardcoded.
    return {vsdk::GeometryConfig(vsdk::pose{-37.5, 5.5, -18.1}, vsdk::box({145, 46, 39}), "box")};
}

std::unique_ptr<orbbec::ObResourceConfig> Orbbec::configure(vsdk::Dependencies dependencies, vsdk::ResourceConfig configuration) {
    auto attrs = configuration.attributes();
    std::string serial_number_from_config;
    const std::string* serial_val = attrs["serial_number"].get<std::string>();
    serial_number_from_config = *serial_val;

    std::optional<DeviceResolution> dev_res = std::nullopt;
    if (attrs.count("resolution")) {
        VIAM_SDK_LOG(info) << "[configure] resolution specified in config";

        auto resolution = attrs["resolution"].get<viam::sdk::ProtoStruct>();

        std::optional<Resolution> color_res = std::nullopt;
        if (resolution->count("color")) {
            auto color_height = resolution->at("color").get<viam::sdk::ProtoStruct>()->at("height").get_unchecked<double>();
            auto color_width = resolution->at("color").get<viam::sdk::ProtoStruct>()->at("width").get_unchecked<double>();
            color_res = Resolution{static_cast<uint32_t>(color_width), static_cast<uint32_t>(color_height)};
        }

        std::optional<Resolution> depth_res = std::nullopt;
        if (resolution->count("depth")) {
            auto depth_height = resolution->at("depth").get<viam::sdk::ProtoStruct>()->at("height").get_unchecked<double>();
            auto depth_width = resolution->at("depth").get<viam::sdk::ProtoStruct>()->at("width").get_unchecked<double>();
            depth_res = Resolution{static_cast<uint32_t>(depth_width), static_cast<uint32_t>(depth_height)};
        }

        dev_res = DeviceResolution{color_res, depth_res};
    }

    std::optional<DeviceFormat> dev_fmt = std::nullopt;
    if (attrs.count("format")) {
        VIAM_SDK_LOG(info) << "[configure] format specified in config";
        auto format = attrs["format"].get<viam::sdk::ProtoStruct>();

        std::optional<std::string> color_format = std::nullopt;
        if (format->count("color")) {
            color_format = format->at("color").get_unchecked<std::string>();
        }
        std::optional<std::string> depth_format = std::nullopt;
        if (format->count("depth")) {
            depth_format = format->at("depth").get_unchecked<std::string>();
        }
        dev_fmt = DeviceFormat{color_format, depth_format};
    }

    auto native_config = std::make_unique<orbbec::ObResourceConfig>(serial_number_from_config, configuration.name(), dev_res, dev_fmt);
    VIAM_SDK_LOG(info) << "[configure] configured: " << native_config->to_string();
    return native_config;
}
// RESOURCE END

// ORBBEC SDK DEVICE REGISTRY START
void registerDevice(std::string serialNumber, std::shared_ptr<ob::Device> dev) {
    VIAM_SDK_LOG(info) << "starting " << serialNumber;
    std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>(dev);
    pipe->enableFrameSync();
    std::shared_ptr<ob::Config> config = createHwD2CAlignConfig(pipe, std::nullopt, std::nullopt);
    if (config == nullptr) {
        VIAM_SDK_LOG(error) << "Current device does not support hardware depth-to-color "
                               "alignment.";
        return;
    }

    std::shared_ptr<ob::PointCloudFilter> pointCloudFilter = std::make_shared<ob::PointCloudFilter>();
    // NOTE: Swap this to depth if you want to align to depth
    std::shared_ptr<ob::Align> align = std::make_shared<ob::Align>(OB_STREAM_COLOR);

    pointCloudFilter->setCreatePointFormat(OB_FORMAT_RGB_POINT);

    {
        std::lock_guard<std::mutex> lock(devices_by_serial_mu());
        std::unique_ptr<ViamOBDevice> my_dev = std::make_unique<ViamOBDevice>();

        my_dev->pipe = pipe;
        my_dev->device = dev;
        my_dev->serial_number = serialNumber;
        my_dev->pointCloudFilter = pointCloudFilter;
        my_dev->align = align;
        my_dev->config = config;

        devices_by_serial()[serialNumber] = std::move(my_dev);
    }
}

void deviceChangedCallback(const std::shared_ptr<ob::DeviceList> removedList, const std::shared_ptr<ob::DeviceList> addedList) {
    try {
        int devCount = removedList->getCount();
        printDeviceList(removedList);
        for (size_t i = 0; i < devCount; i++) {
            if (i == 0) {
                VIAM_SDK_LOG(info) << " Devices Removed:\n";
            }
            std::lock_guard<std::mutex> lock(devices_by_serial_mu());
            std::string serial_number = removedList->serialNumber(i);
            auto search = devices_by_serial().find(serial_number);
            if (search == devices_by_serial().end()) {
                std::cerr << serial_number
                          << "was in removedList of device change callback but not "
                             "in devices_by_serial\n";
                continue;
            }
            devices_by_serial().erase(search);
        }

        devCount = addedList->getCount();
        for (size_t i = 0; i < devCount; i++) {
            if (i == 0) {
                VIAM_SDK_LOG(info) << " Devices added:\n";
            }
            std::shared_ptr<ob::Device> dev = addedList->getDevice(i);
            std::shared_ptr<ob::DeviceInfo> info = dev->getDeviceInfo();
            printDeviceInfo(info);
            registerDevice(info->getSerialNumber(), dev);
            {
                std::lock_guard<std::mutex> lock(serial_by_resource_mu());
                for (auto& [resource_name, serial_number] : serial_by_resource()) {
                    if (serial_number == info->getSerialNumber()) {
                        startDevice(serial_number);
                        serial_by_resource()[resource_name] = serial_number;
                    }
                }
            }
        }
    } catch (ob::Error& e) {
        std::cerr << "setDeviceChangedCallback\n"
                  << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nname:" << e.getName() << "\nmessage:" << e.what()
                  << "\ntype:" << e.getExceptionType() << std::endl;
    }
}

void startOrbbecSDK(ob::Context& ctx) {
    ctx.setDeviceChangedCallback(deviceChangedCallback);

    std::shared_ptr<ob::DeviceList> devList = ctx.queryDeviceList();
    int devCount = devList->getCount();
    for (size_t i = 0; i < devCount; i++) {
        if (i == 0) {
            VIAM_SDK_LOG(info) << "devCount: " << devCount << "\n";
        }
        std::shared_ptr<ob::Device> dev = devList->getDevice(i);
        std::shared_ptr<ob::DeviceInfo> info = dev->getDeviceInfo();
        printDeviceInfo(info);
        registerDevice(info->getSerialNumber(), dev);
    }
}
// ORBBEC SDK DEVICE REGISTRY END

}  // namespace orbbec
