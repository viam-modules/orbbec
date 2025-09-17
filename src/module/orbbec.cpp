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
#include "device_control.hpp"
#include "encoding.hpp"
#ifdef _WIN32
#include <windows.h>
#endif

#include <curl/curl.h>
#include <math.h>
#include <zip.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iomanip>

#include <filesystem>  // For path manipulation and absolute paths
#include <fstream>     // <-- Add this line near your other includes
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
std::map<Resolution, std::set<Resolution, std::greater<Resolution>>, std::greater<Resolution>> const
    Orbbec::color_to_depth_supported_resolutions{{{1920, 1080}, {{1600, 1200}, {800, 600}, {400, 300}}},
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
    std::string serial_number{};
    std::shared_ptr<ob::Device> device{};
    bool started{};
    std::shared_ptr<ob::Pipeline> pipe{};
    std::shared_ptr<ob::PointCloudFilter> pointCloudFilter{};
    std::shared_ptr<ob::Align> align{};
    std::shared_ptr<ob::Config> config{};
    std::vector<std::shared_ptr<ob::Filter>> postProcessDepthFilters{};
    bool applyEnabledPostProcessDepthFilters{};
    bool dumpPCLFiles{};
    bool skipAlignment{};
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

// Convert integer to uppercase 4-digit hex string
std::string uint16ToHex(uint16_t value) {
    std::stringstream ss;
    ss << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << value;
    return ss.str();
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
    VIAM_SDK_LOG(info) << "[RGBPointsToPCD] input numPoints: " << numPoints;

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

    VIAM_SDK_LOG(info) << "[RGBPointsToPCD] PCD number of points generated: " << pcdPoints.size();

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

        if (deviceFormat.has_value() && deviceFormat->color_format.has_value()) {
            if (ob::TypeHelper::convertOBFormatTypeToString(colorVsp->getFormat()) != deviceFormat->color_format.value()) {
                continue;
            }
        } else if (ob::TypeHelper::convertOBFormatTypeToString(colorVsp->getFormat()) != Orbbec::default_color_format) {
            continue;
        }

        if (deviceRes.has_value() && deviceRes->color_resolution.has_value()) {
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

            if (deviceFormat.has_value() && deviceFormat->depth_format.has_value()) {
                if (ob::TypeHelper::convertOBFormatTypeToString(depthVsp->getFormat()) != deviceFormat->depth_format.value()) {
                    continue;
                }
            } else if (ob::TypeHelper::convertOBFormatTypeToString(depthVsp->getFormat()) != Orbbec::default_depth_format) {
                continue;
            }

            if (deviceRes.has_value() && deviceRes->depth_resolution.has_value()) {
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
                VIAM_SDK_LOG(info) << "Enabling hardware depth-to-color alignment";
                VIAM_SDK_LOG(info) << "Color stream: width: " << colorVsp->getWidth() << " height: " << colorVsp->getHeight()
                                   << " format: " << ob::TypeHelper::convertOBFormatTypeToString(colorVsp->getFormat())
                                   << " fps: " << colorVsp->getFps();
                VIAM_SDK_LOG(info) << "Depth stream: width: " << depthVsp->getWidth() << " height: " << depthVsp->getHeight()
                                   << " format: " << ob::TypeHelper::convertOBFormatTypeToString(depthVsp->getFormat())
                                   << " fps: " << depthVsp->getFps();

                hwD2CAlignConfig->enableStream(colorProfile);  // enable color stream
                hwD2CAlignConfig->enableStream(depthProfile);  // enable depth stream
                // hwD2CAlignConfig->enableVideoStream(OB_STREAM_DEPTH, 800, 600, 30, OB_FORMAT_Y16);
                // hwD2CAlignConfig->enableVideoStream(OB_STREAM_COLOR, 800, 600, 30, OB_FORMAT_MJPEG);
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

void displayPresets(std::shared_ptr<ob::Device> device) {
    std::shared_ptr<ob::DevicePresetList> presetLists = device->getAvailablePresetList();
    if (presetLists && presetLists->getCount() == 0) {
        VIAM_SDK_LOG(error) << "The current device does not support preset mode" << std::endl;
        return;
    }

    VIAM_SDK_LOG(info) << "Available Presets:" << std::endl;
    for (uint32_t index = 0; index < presetLists->getCount(); index++) {
        // Print available preset name.
        VIAM_SDK_LOG(info) << " - " << index << "." << presetLists->getName(index) << std::endl;
    }

    // Print current preset name.
    VIAM_SDK_LOG(info) << "Current PresetName: " << device->getCurrentPresetName() << std::endl;
}

void setDepthSoftFilter(std::shared_ptr<ob::Device> device, bool enable) {
    try {
        if (device->isPropertySupported(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL, OB_PERMISSION_WRITE)) {
            device->setBoolProperty(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL, enable);
            VIAM_SDK_LOG(info) << "[setDepthSoftFilter] " << (enable ? "enabled" : "disabled") << " depth soft filter" << std::endl;
        } else {
            VIAM_SDK_LOG(error) << "[setDepthSoftFilter] OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL is not supported";
        }
    } catch (ob::Error& e) {
        std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what()
                  << "\ntype:" << e.getExceptionType() << std::endl;
        exit(EXIT_FAILURE);
    }
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
        VIAM_SDK_LOG(info) << "[startDevice] Resolution from config: "
                           << (resolution_opt.has_value() ? resolution_opt->to_string() : "not specified, Format from config: ")
                           << (format_opt.has_value() ? format_opt->to_string() : "not specifed");
        if (resolution_opt.has_value() || format_opt.has_value()) {
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

// HELPERS END

// RESOURCE BEGIN
void Orbbec::validate_sensor(std::pair<std::string, viam::sdk::ProtoValue> const& sensor_pair) {
    auto const& sensor_type = sensor_pair.first;
    auto const& sensor = sensor_pair.second.get<viam::sdk::ProtoStruct>();
    if (!sensor) {
        throw std::invalid_argument("sensor must be a struct");
    }
    if (!sensor->count("width")) {
        throw std::invalid_argument("sensor must contain width key");
    }
    auto width = sensor->at("width").get<double>();
    if (!width) {
        throw std::invalid_argument("sensor width must be a double");
    }
    if (*width <= 0) {
        throw std::invalid_argument("sensor width must be positive");
    }
    if (!sensor->count("height")) {
        throw std::invalid_argument("sensor must contain height key");
    }
    auto height = sensor->at("height").get<double>();
    if (!height) {
        throw std::invalid_argument("sensor height must be a double");
    }
    if (*height <= 0) {
        throw std::invalid_argument("sensor height must be positive");
    }
    auto const format = sensor->at("format").get<std::string>();
    if (!format) {
        throw std::invalid_argument("sensor format must be a string");
    }
    if (sensor_type == "color") {
        if (!supported_color_formats.count(*format)) {
            std::ostringstream buffer;
            buffer << "color sensor format must be one of: ";
            for (const auto& type : supported_color_formats) {
                buffer << type << " ";
            }
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::invalid_argument(buffer.str());
        }

    } else if (sensor_type == "depth") {
        if (!supported_depth_formats.count(*format)) {
            std::ostringstream buffer;
            buffer << "depth sensor format must be one of: ";
            for (const auto& type : supported_depth_formats) {
                buffer << type << " ";
            }
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::invalid_argument(buffer.str());
        }
    } else {
        throw std::invalid_argument("sensor type must be color or depth");
    }
}

std::vector<std::string> Orbbec::validate(vsdk::ResourceConfig cfg) {
    auto attrs = cfg.attributes();

    if (!attrs.count("serial_number")) {
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

    if (attrs.count("sensors")) {
        auto sensors = attrs["sensors"].get<viam::sdk::ProtoStruct>();
        if (!sensors) {
            throw std::invalid_argument("sensors must be a struct");
        }
        if (sensors) {
            for (auto const& sensor_pair : *sensors) {
                validate_sensor(sensor_pair);
            }
        }
        auto color_width_uint32 =
            static_cast<std::uint32_t>(*sensors->at("color").get<viam::sdk::ProtoStruct>()->at("width").get<double>());
        auto color_height_uint32 =
            static_cast<std::uint32_t>(*sensors->at("color").get<viam::sdk::ProtoStruct>()->at("height").get<double>());
        if (color_to_depth_supported_resolutions.count({color_width_uint32, color_height_uint32}) == 0) {
            std::ostringstream buffer;
            buffer << "color resolution must be one of: ";
            for (const auto& res : color_to_depth_supported_resolutions) {
                buffer << "{" << res.first.to_string() << "} ";
            }
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::invalid_argument(buffer.str());
        }
        auto depth_width_uint32 =
            static_cast<std::uint32_t>(*sensors->at("depth").get<viam::sdk::ProtoStruct>()->at("width").get<double>());
        auto depth_height_uint32 =
            static_cast<std::uint32_t>(*sensors->at("depth").get<viam::sdk::ProtoStruct>()->at("height").get<double>());
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
        } else if (color->getFormat() == OB_FORMAT_RGB) {
            response.mime_type = kColorMimeTypePNG;
            auto width = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getWidth();
            auto height = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getHeight();
            response.bytes = encoding::encode_to_png(colorData, width, height);
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
            } else if (color->getFormat() == OB_FORMAT_RGB) {
                color_image.mime_type = kColorMimeTypePNG;
                auto width = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getWidth();
                auto height = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getHeight();
                color_image.bytes = encoding::encode_to_png(colorData, width, height);
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

vsdk::ProtoStruct getCameraParams(std::shared_ptr<ob::Pipeline> pipe) {
    if (pipe == nullptr) {
        return {{"error", "pipe is null"}};
    }
    auto const config = pipe->getConfig();
    if (!config) {
        return {{"error", "failed to get pipeline config"}};
    }

    auto enabledStreamProfileListPtr = config->getEnabledStreamProfileList();

    vsdk::ProtoStruct result;
    if (enabledStreamProfileListPtr) {
        auto const& enabledStreamProfileList = *enabledStreamProfileListPtr;
        auto const count = enabledStreamProfileList.getCount();
        for (int i = 0; i < count; i++) {
            auto sp = enabledStreamProfileList.getProfile(i);
            auto vsp = sp->as<ob::VideoStreamProfile>();
            std::string sensorName = ob::TypeHelper::convertOBStreamTypeToString(sp->getType());
            vsdk::ProtoStruct profile;
            profile["width"] = static_cast<int>(vsp->getWidth());
            profile["height"] = static_cast<int>(vsp->getHeight());
            profile["format"] = ob::TypeHelper::convertOBFormatTypeToString(vsp->getFormat());
            profile["fps"] = static_cast<int>(vsp->getFps());

            vsdk::ProtoStruct intrinsics_struct;
            auto intrinsics = vsp->getIntrinsic();
            intrinsics_struct["fx"] = static_cast<double>(intrinsics.fx);
            intrinsics_struct["fy"] = static_cast<double>(intrinsics.fy);
            intrinsics_struct["cx"] = static_cast<double>(intrinsics.cx);
            intrinsics_struct["cy"] = static_cast<double>(intrinsics.cy);
            intrinsics_struct["width"] = static_cast<double>(intrinsics.width);
            intrinsics_struct["height"] = static_cast<double>(intrinsics.height);
            profile["intrinsics"] = intrinsics_struct;

            vsdk::ProtoStruct distortion_struct;
            auto distortion = vsp->getDistortion();
            distortion_struct["k1"] = static_cast<double>(distortion.k1);
            distortion_struct["k2"] = static_cast<double>(distortion.k2);
            distortion_struct["k3"] = static_cast<double>(distortion.k3);
            distortion_struct["k4"] = static_cast<double>(distortion.k4);
            distortion_struct["k5"] = static_cast<double>(distortion.k5);
            distortion_struct["k6"] = static_cast<double>(distortion.k6);
            distortion_struct["p1"] = static_cast<double>(distortion.p1);
            distortion_struct["p2"] = static_cast<double>(distortion.p2);
            profile["distortion"] = distortion_struct;

            result[sensorName] = profile;
        }
    }

    return result;
}

viam::sdk::ProtoStruct setDepthWorkingMode(std::unique_ptr<ViamOBDevice>& viam_device,
                                           viam::sdk::ProtoValue const& value,
                                           std::string const& serialNumber,
                                           std::string const& command) {
    if (not value.is_a<std::string>()) {
        return {{"error", "Invalid value type for Depth Working Mode. Expected string."}};
    }
    auto device = viam_device->device;
    if (not device) {
        return {{"error", "Device not found."}};
    }
    std::string const mode = value.get_unchecked<std::string>();
    if (mode == "In-scene Calibration") {
        return {{"error", "In-scene Calibration mode is apparently not supported by the SDK."}};
    }
    try {
        if (device->isPropertySupported(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_WRITE)) {
            auto depthModeList = device->getDepthWorkModeList();
            bool modeFound = false;
            for (uint32_t i = 0; i < depthModeList->getCount(); i++) {
                if ((*depthModeList)[i].name == mode) {
                    modeFound = true;
                    break;
                }
            }
            if (!modeFound) {
                std::stringstream error_ss;
                error_ss << "Depth working mode " << mode << " not found. Available modes are: ";
                for (uint32_t i = 0; i < depthModeList->getCount(); i++) {
                    error_ss << (*depthModeList)[i].name << " ";
                }
                return {{"error", error_ss.str()}};
            }
            viam_device->started = false;
            viam_device->pipe->stop();

            device->switchDepthWorkMode(mode.c_str());
            auto resolution_opt = config_by_serial().at(serialNumber).device_resolution;
            auto format_opt = config_by_serial().at(serialNumber).device_format;

            std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>(device);
            std::shared_ptr<ob::Config> config = createHwD2CAlignConfig(pipe, resolution_opt, format_opt);

            pipe->enableFrameSync();
            viam_device->pipe = pipe;
            viam_device->config = config;

            viam_device->pipe->start(config, [serialNumber](std::shared_ptr<ob::FrameSet> frameSet) { frameCallback(serialNumber); });
            viam_device->started = true;
            return device_control::getDepthWorkingMode(device, command);
        } else {
            return {{"error", "Depth working mode property is not supported."}};
        }
    } catch (ob::Error& e) {
        std::stringstream error_ss;
        error_ss << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what()
                 << "\ntype:" << e.getExceptionType() << std::endl;
        return {{"error", error_ss.str()}};
    }
}

bool isPrimaryTypeProperty(OBPropertyItem propertyItem) {
    return propertyItem.type == OB_INT_PROPERTY || propertyItem.type == OB_FLOAT_PROPERTY || propertyItem.type == OB_BOOL_PROPERTY;
}

std::string permissionToString(OBPermissionType permission) {
    switch (permission) {
        case OB_PERMISSION_DENY:
            return "deny";
        case OB_PERMISSION_READ:
            return "read";
        case OB_PERMISSION_WRITE:
            return "write";
        case OB_PERMISSION_READ_WRITE:
            return "read_write";
        case OB_PERMISSION_ANY:
            return "any";
        default:
            return "unknown";
    }
}

std::string propertyTypeToString(OBPropertyType type) {
    switch (type) {
        case OB_BOOL_PROPERTY:
            return "bool";
        case OB_INT_PROPERTY:
            return "int";
        case OB_FLOAT_PROPERTY:
            return "float";
        case OB_STRUCT_PROPERTY:
            return "struct";
        default:
            return "unknown";
    }
}

vsdk::ProtoStruct getDeviceProperty(std::shared_ptr<ob::Device> device, viam::sdk::ProtoValue const& value) {
    if (!value.is_a<std::string>()) {
        return {{"error", "value must be a struct"}};
    }
    std::string property = value.get_unchecked<std::string>();
    uint32_t size = device->getSupportedPropertyCount();
    for (uint32_t i = 0; i < size; i++) {
        OBPropertyItem property_item = device->getSupportedProperty(i);
        if (property_item.name == property) {
            viam::sdk::ProtoStruct property_struct;
            property_struct["name"] = property_item.name;
            property_struct["id"] = property_item.id;
            property_struct["type"] = propertyTypeToString(property_item.type);
            property_struct["permission"] = permissionToString(property_item.permission);
            if (property_item.type == OB_INT_PROPERTY) {
                OBIntPropertyRange valueRange = device->getIntPropertyRange(property_item.id);
                property_struct["current"] = valueRange.cur;
                property_struct["min"] = valueRange.min;
                property_struct["max"] = valueRange.max;
                property_struct["default"] = valueRange.def;
            } else if (property_item.type == OB_FLOAT_PROPERTY) {
                OBFloatPropertyRange valueRange = device->getFloatPropertyRange(property_item.id);
                property_struct["current"] = valueRange.cur;
                property_struct["min"] = valueRange.min;
                property_struct["max"] = valueRange.max;
                property_struct["default"] = valueRange.def;
            } else if (property_item.type == OB_BOOL_PROPERTY) {
                OBBoolPropertyRange valueRange = device->getBoolPropertyRange(property_item.id);
                property_struct["current"] = valueRange.cur;
                property_struct["default"] = valueRange.def;
            } else if (property_item.type == OB_STRUCT_PROPERTY) {
                // For struct properties, we can add more detailed handling if needed
                // uint32_t bufferSize = 65536;  // Choose a size you expect to be sufficient
                // std::vector<uint8_t> buffer(bufferSize);
                // device->getStructuredData(property_item.id, buffer.data(), &bufferSize);
                // VIAM_SDK_LOG(info) << "Retrieved structured data for property " << property_item.id << ", size: " << bufferSize;
                property_struct["current"] = "struct_property";
            } else {
                property_struct["current"] = "unknown_type";
            }
            return property_struct;
        }
    }

    return {};
}
vsdk::ProtoStruct setDeviceProperty(std::shared_ptr<ob::Device> device, const vsdk::ProtoValue& property) {
    if (not property.is_a<vsdk::ProtoStruct>()) {
        return {{"error", "property must be a struct"}};
    }
    auto const& property_map = property.get_unchecked<vsdk::ProtoStruct>();
    if (property_map.size() == 0) {
        return {{"error", "property map is empty"}};
    }
    if (property_map.size() > 1) {
        return {{"error", "property map must contain exactly one entry"}};
    }

    uint32_t size = device->getSupportedPropertyCount();
    for (uint32_t i = 0; i < size; i++) {
        OBPropertyItem property_item = device->getSupportedProperty(i);
        if (property_item.name == property_map.begin()->first) {
            // Set the property value
            if (property_item.type == OB_INT_PROPERTY) {
                if (not property_map.begin()->second.is_a<double>()) {
                    return {{"error", "Invalid type for int property"}};
                }
                int int_value = property_map.begin()->second.get_unchecked<double>();
                device->setIntProperty(property_item.id, int_value);
            } else if (property_item.type == OB_FLOAT_PROPERTY) {
                if (not property_map.begin()->second.is_a<double>()) {
                    return {{"error", "Invalid type for float property"}};
                }
                float float_value = property_map.begin()->second.get_unchecked<double>();
                device->setFloatProperty(property_item.id, float_value);
            } else if (property_item.type == OB_BOOL_PROPERTY) {
                if (not property_map.begin()->second.is_a<bool>()) {
                    return {{"error", "Invalid type for bool property"}};
                }
                bool bool_value = property_map.begin()->second.get_unchecked<bool>();
                device->setBoolProperty(property_item.id, bool_value);
            } else {
                return {{"error", "Unsupported property type"}};
            }
            return getDeviceProperty(device, property_item.name);
        }
    }

    return {};
}

// Get property list
vsdk::ProtoStruct getDeviceProperties(std::shared_ptr<ob::Device> device) {
    vsdk::ProtoStruct properties_list;
    uint32_t size = device->getSupportedPropertyCount();
    for (uint32_t i = 0; i < size; i++) {
        OBPropertyItem property_item = device->getSupportedProperty(i);
        if (/*isPrimaryTypeProperty(property_item) && property_item.permission != OB_PERMISSION_DENY*/ true) {
            properties_list[property_item.name] = getDeviceProperty(device, property_item.name);
        }
    }
    return properties_list;
}

vsdk::ProtoStruct setDeviceProperties(std::shared_ptr<ob::Device> device, const vsdk::ProtoValue& properties) {
    if (not properties.is_a<viam::sdk::ProtoStruct>()) {
        return {{"error", "properties must be a struct"}};
    }
    auto const& properties_map = properties.get_unchecked<viam::sdk::ProtoStruct>();
    int const supportedPropertyCount = device->getSupportedPropertyCount();
    for (int i = 0; i < supportedPropertyCount; i++) {
        OBPropertyItem property_item = device->getSupportedProperty(i);
        if (properties_map.count(property_item.name) > 0) {
            auto const& value = properties_map.at(property_item.name);
            if (property_item.permission == OB_PERMISSION_DENY || property_item.permission == OB_PERMISSION_READ) {
                std::stringstream error_ss;
                error_ss << "Property " << property_item.name << " is not writable, skipping.";
                VIAM_SDK_LOG(warn) << error_ss.str();
                return {{"error", error_ss.str()}};
                continue;
            }
            try {
                if (property_item.type == OB_INT_PROPERTY && value.is_a<double>()) {
                    int int_value = static_cast<int>(value.get_unchecked<double>());
                    device->setIntProperty(property_item.id, int_value);
                    VIAM_SDK_LOG(info) << "Set int property " << property_item.name << " to " << int_value;
                } else if (property_item.type == OB_FLOAT_PROPERTY && value.is_a<double>()) {
                    double float_value = value.get_unchecked<double>();
                    device->setFloatProperty(property_item.id, float_value);
                    VIAM_SDK_LOG(info) << "Set float property " << property_item.name << " to " << float_value;
                } else if (property_item.type == OB_BOOL_PROPERTY && value.is_a<bool>()) {
                    bool bool_value = value.get_unchecked<bool>();
                    device->setBoolProperty(property_item.id, bool_value);
                    VIAM_SDK_LOG(info) << "Set bool property " << property_item.name << " to " << (bool_value ? "true" : "false");
                } else {
                    VIAM_SDK_LOG(warn) << "Type mismatch or unsupported type for property " << property_item.name << ", skipping.";
                    return {{"error", "Type mismatch or unsupported type"}};
                }
            } catch (ob::Error& e) {
                std::stringstream error_ss;
                error_ss << "Failed to set property " << property_item.name << ": " << e.what();
                VIAM_SDK_LOG(error) << error_ss.str();
                return {{"error", error_ss.str()}};
            }
        }
    }
    return getDeviceProperties(device);
}

vsdk::ProtoStruct getCameraPresets(std::shared_ptr<ob::Device> device, std::string const& command) {
    std::shared_ptr<ob::DevicePresetList> presetLists = device->getAvailablePresetList();
    if (presetLists && presetLists->getCount() == 0) {
        VIAM_SDK_LOG(error) << "The current device does not support preset mode" << std::endl;
        return {{"error", "The current device does not support preset mode"}};
    }

    vsdk::ProtoList presets;
    VIAM_SDK_LOG(info) << "Available Presets:" << std::endl;
    for (uint32_t index = 0; index < presetLists->getCount(); index++) {
        // Print available preset name.
        VIAM_SDK_LOG(info) << " - " << index << "." << presetLists->getName(index) << std::endl;
        presets.push_back(presetLists->getName(index));
    }

    // Print current preset name.
    VIAM_SDK_LOG(info) << "Current PresetName: " << device->getCurrentPresetName() << std::endl;

    return {{"presets", presets}, {"current_preset", device->getCurrentPresetName()}};
}

vsdk::ProtoStruct Orbbec::do_command(const vsdk::ProtoStruct& command) {
    try {
        viam::sdk::ProtoStruct resp = viam::sdk::ProtoStruct{};
        constexpr char firmware_key[] = "update_firmware";
        for (auto const& [key, value] : command) {
            if (key == firmware_key) {
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
            } else {
                std::string value_str;
                switch (value.kind()) {
                    case vsdk::ProtoValue::Kind::k_null:
                        break;
                    case vsdk::ProtoValue::Kind::k_bool:
                        value_str = value.get_unchecked<bool>() ? "true" : "false";
                        break;
                    case vsdk::ProtoValue::Kind::k_double:
                        value_str = std::to_string(value.get_unchecked<double>());
                        break;
                    case vsdk::ProtoValue::Kind::k_string:
                        value_str = value.get_unchecked<std::string>();
                        break;
                    default:
                        value_str = "unknown";
                        break;
                }

                auto const serialNumber = serial_number_;
                {
                    const std::lock_guard<std::mutex> lock(devices_by_serial_mu());
                    auto search = devices_by_serial().find(serialNumber);
                    if (search == devices_by_serial().end()) {
                        VIAM_SDK_LOG(error) << service_name << ": unable to stop undetected device " << serialNumber;
                        return vsdk::ProtoStruct{};
                    }
                    std::unique_ptr<ViamOBDevice>& my_dev = search->second;
                    if (!my_dev->started) {
                        VIAM_SDK_LOG(error) << service_name << ": unable to stop device that is not currently running " << serialNumber;
                        return vsdk::ProtoStruct{};
                    }

                    VIAM_SDK_LOG(info) << "[do_command] key: " << key << ", value: " << value_str;
                    if (key == "dump_pcl_files") {
                        if (not value.is_a<bool>()) {
                            VIAM_SDK_LOG(error) << "[do_command] dump_pcl_files: expected bool, got " << value.kind();
                            return vsdk::ProtoStruct{{"error", "expected bool"}};
                        }
                        my_dev->dumpPCLFiles = value.get_unchecked<bool>();
                        return {{"dump_pcl_files", my_dev->dumpPCLFiles}};
                    }
                    if (key == "skip_alignment") {
                        if (not value.is_a<bool>()) {
                            VIAM_SDK_LOG(error) << "[do_command] skip_alignment: expected bool, got " << value.kind();
                            return vsdk::ProtoStruct{{"error", "expected bool"}};
                        }
                        my_dev->skipAlignment = value.get_unchecked<bool>();
                        return {{"skip_alignment", my_dev->skipAlignment}};
                    }
                    if (key == "apply_post_process_depth_filters") {
                        return device_control::applyPostProcessDepthFilters(my_dev, value, key);
                    }
                    if (key == "get_recommended_post_process_depth_filters") {
                        return device_control::getRecommendedPostProcessDepthFilters(my_dev->device);
                    }

                    if (key == "set_recommended_post_process_depth_filters") {
                        return device_control::setRecommendedPostProcessDepthFilters(my_dev);
                    }

                    if (key == "get_post_process_depth_filters") {
                        return device_control::getPostProcessDepthFilters(my_dev->postProcessDepthFilters, key);
                    }

                    if (key == "set_post_process_depth_filters") {
                        return device_control::setPostProcessDepthFilters(my_dev->postProcessDepthFilters, value, key);
                    }

                    if (key == "get_depth_noise_removal_filter") {
                        return device_control::getDepthNoiseRemovalFilter(my_dev->device, key);
                    }

                    if (key == "set_depth_noise_removal_filter") {
                        return device_control::setDepthNoiseRemovalFilter(my_dev->device, value, key);
                    }

                    if (key == "get_depth_gain") {
                        return device_control::getDepthGain(my_dev->device);
                    }

                    if (key == "set_depth_gain") {
                        return device_control::setDepthGain(my_dev->device, value);
                    }

                    if (key == "get_depth_auto_exposure") {
                        return device_control::getDepthAutoExposure(my_dev->device, key);
                    }

                    if (key == "set_depth_auto_exposure") {
                        return device_control::setDepthAutoExposure(my_dev->device, value, key);
                    }

                    if (key == "get_laser") {
                        return device_control::getLaser(my_dev->device, key);
                    }

                    if (key == "set_laser") {
                        return device_control::setLaser(my_dev->device, value, key);
                    }

                    if (key == "get_depth_mirror") {
                        return device_control::getDepthMirror(my_dev->device, key);
                    }

                    if (key == "set_depth_mirror") {
                        return device_control::setDepthMirror(my_dev->device, value, key);
                    }

                    if (key == "get_depth_exposure") {
                        return device_control::getDepthExposure(my_dev->device, key);
                    }

                    if (key == "set_depth_exposure") {
                        return device_control::setDepthExposure(my_dev->device, value, key);
                    }

                    if (key == "get_depth_unit") {
                        return device_control::getDepthUnit(my_dev->device, key);
                    }

                    if (key == "set_depth_unit") {
                        return device_control::setDepthUnit(my_dev->device, value, key);
                    }

                    if (key == "get_depth_working_mode") {
                        return device_control::getDepthWorkingMode(my_dev->device, key);
                    }

                    if (key == "set_depth_working_mode") {
                        return setDepthWorkingMode(my_dev, value, serialNumber, key);
                    }
                    if (key == "get_device_properties") {
                        return device_control::getDeviceProperties(my_dev->device, key);
                    }
                    if (key == "set_device_properties") {
                        return device_control::setDeviceProperties(my_dev->device, value, key);
                    }
                    if (key == "get_device_property") {
                        return device_control::getDeviceProperty(my_dev->device, value, key);
                    }
                    if (key == "set_device_property") {
                        return device_control::setDeviceProperty(my_dev->device, value, key);
                    }
                    if (key == "get_camera_params") {
                        return getCameraParams(my_dev->pipe);
                    }
                    if (key == "get_camera_presets") {
                        return getCameraPresets(my_dev->device, key);
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << service_name << ": exception caught: " << e.what();
    }
    return vsdk::ProtoStruct{};
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

        std::shared_ptr<ob::ColorFrame> colorFrame = color->as<ob::ColorFrame>();

        VIAM_SDK_LOG(info) << "[get_point_cloud] colorFrame width: " << colorFrame->getWidth() << ", height: " << colorFrame->getHeight();
        VIAM_SDK_LOG(info) << "[get_point_cloud] depthFrame width: " << depthFrame->getWidth() << ", height: " << depthFrame->getHeight();

        if (my_dev->applyEnabledPostProcessDepthFilters && my_dev->postProcessDepthFilters.size() > 0) {
            for (auto& filter : my_dev->postProcessDepthFilters) {
                depth = filter->process(depth);
            }
            fs->pushFrame(depth);
        }

        std::vector<unsigned char> data;
        if (my_dev->skipAlignment) {
            VIAM_SDK_LOG(info) << "[get_point_cloud] skipping alignment as per configuration";
            data = RGBPointsToPCD(my_dev->pointCloudFilter->process(fs), scale * mmToMeterMultiple);
        } else {
            data = RGBPointsToPCD(my_dev->pointCloudFilter->process(my_dev->align->process(fs)), scale * mmToMeterMultiple);
        }

        if (my_dev->dumpPCLFiles) {
            auto timestamp = getNowUs();
            std::stringstream outfile_name;
            outfile_name << "pointcloud_" << timestamp << ".pcd";
            std::ofstream outfile(outfile_name.str(), std::ios::out | std::ios::binary);
            outfile.write((const char*)&data[0], data.size());
            std::filesystem::path file_path(outfile_name.str());
            std::filesystem::path absolute_path = std::filesystem::absolute(file_path);
            std::string absolute_path_str = absolute_path.string();
            outfile.close();
            VIAM_SDK_LOG(info) << "[get_point_cloud] wrote PCD to location: " << absolute_path_str;
        }

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
    std::optional<DeviceFormat> dev_fmt = std::nullopt;
    if (attrs.count("sensors")) {
        VIAM_SDK_LOG(info) << "[configure] sensors specified in config";
        auto sensors = attrs["sensors"].get<viam::sdk::ProtoStruct>();

        auto const color_height = sensors->at("color").get<viam::sdk::ProtoStruct>()->at("height").get_unchecked<double>();
        auto const color_width = sensors->at("color").get<viam::sdk::ProtoStruct>()->at("width").get_unchecked<double>();
        auto const color_res = Resolution{static_cast<uint32_t>(color_width), static_cast<uint32_t>(color_height)};
        auto const color_format = sensors->at("color").get<viam::sdk::ProtoStruct>()->at("format").get_unchecked<std::string>();

        auto const depth_height = sensors->at("depth").get<viam::sdk::ProtoStruct>()->at("height").get_unchecked<double>();
        auto const depth_width = sensors->at("depth").get<viam::sdk::ProtoStruct>()->at("width").get_unchecked<double>();
        auto const depth_res = Resolution{static_cast<uint32_t>(depth_width), static_cast<uint32_t>(depth_height)};
        auto const depth_format = sensors->at("depth").get<viam::sdk::ProtoStruct>()->at("format").get_unchecked<std::string>();

        dev_res = DeviceResolution{color_res, depth_res};
        dev_fmt = DeviceFormat{color_format, depth_format};
    } else {
        VIAM_SDK_LOG(info) << "[configure] no sensors specified in config, using defaults";
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

    auto depthSensor = dev->getSensor(OB_SENSOR_DEPTH);
    auto depthFilterList = depthSensor->createRecommendedFilters();

    std::shared_ptr<ob::PointCloudFilter> pointCloudFilter = std::make_shared<ob::PointCloudFilter>();
    // NOTE: Swap this to depth if you want to align to depth
    std::shared_ptr<ob::Align> align = std::make_shared<ob::Align>(OB_STREAM_COLOR);

    pointCloudFilter->setCreatePointFormat(OB_FORMAT_RGB_POINT);

#ifdef _WIN32
    // On windows, we must add a metadata value to the windows device registry for the device to work correctly.
    // Adapted from the orbbec SDK setup script:
    // https://github.com/orbbec/OrbbecSDK_v2/blob/main/scripts/env_setup/obsensor_metadata_win10.ps1
    try {
        const char* command = "powershell -Command \"Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser -Force\"";
        int result = std::system(command);
        if (result != 0) {
            // command failed, try the backup
            command = "powershell -Command \"Set-ExecutionPolicy -ExecutionPolicy Unrestricted -Scope CurrentUser -Force\"";
            int result = std::system(command);
            if (result != 0) {
                VIAM_SDK_LOG(error) << "Could not set execution policy";
            }
        }

        // Base registry paths
        std::vector<std::string> searchTrees = {
            // KSCATEGORY_CAPTURE class,used for video capture devices
            "SYSTEM\\CurrentControlSet\\Control\\DeviceClasses\\{e5323777-f976-4f5b-9b55-b94699c46e44}",
            // KSCATEGORY_RENDER class, used for rendering devices
            "SYSTEM\\CurrentControlSet\\Control\\DeviceClasses\\{65E8773D-8F56-11D0-A3B9-00A0C9223196}"};

        uint16_t vid = dev->getDeviceInfo()->vid();
        uint16_t pid = dev->getDeviceInfo()->pid();
        std::string baseDeviceId = "##?#USB#VID_" + uint16ToHex(vid) + "&PID_" + uint16ToHex(pid);

        for (const auto& subtree : searchTrees) {
            // Open the device registry key
            HKEY hkey;
            if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, subtree.c_str(), 0, KEY_ENUMERATE_SUB_KEYS, &hkey) != ERROR_SUCCESS) {
                VIAM_SDK_LOG(error) << "Could not open device registry key: " << subtree;
                continue;
            }

            char name[512];
            DWORD nameSize;
            DWORD index = 0;
            while (true) {
                // reset before each call
                nameSize = sizeof(name);

                // enumerate all of the keys in the devices folder we previously opened
                LONG result = RegEnumKeyExA(hkey, index, name, &nameSize, NULL, NULL, NULL, NULL);
                if (result == ERROR_NO_MORE_ITEMS) {
                    break;  // normal end of enumeration
                } else if (result != ERROR_SUCCESS) {
                    VIAM_SDK_LOG(error) << "device registry key enumeration failed: " << result;
                    break;
                }
                std::string subKeyName(name);
                // find the enteries for our orbbec device.
                if (subKeyName.find("USB#VID_" + uint16ToHex(vid) + "&PID_" + uint16ToHex(pid)) == std::string::npos) {
                    // not a match for an orbbec device, go to next key
                    ++index;
                    continue;
                }

                std::string deviceParamsKey = subtree + "\\" + subKeyName + "\\#GLOBAL\\Device Parameters";
                std::string valueName = "MetadataBufferSizeInKB0";
                HKEY hDeviceKey;
                // open the orbbec device parameters key
                result = RegOpenKeyExA(HKEY_LOCAL_MACHINE, deviceParamsKey.c_str(), 0, KEY_READ | KEY_SET_VALUE, &hDeviceKey);
                if (result != ERROR_SUCCESS) {
                    VIAM_SDK_LOG(error) << "Couldn't open windows registry device parameters key: " << deviceParamsKey;
                    ++index;
                    continue;
                }
                // check if the metadata value already exists
                DWORD data;
                DWORD dataSize = sizeof(data);
                result = RegQueryValueExA(hDeviceKey, valueName.c_str(), nullptr, nullptr, reinterpret_cast<BYTE*>(&data), &dataSize);
                if (result == ERROR_FILE_NOT_FOUND) {
                    // value does not exist yet, create it.
                    DWORD value = 5;
                    result =
                        RegSetValueExA(hDeviceKey, valueName.c_str(), 0, REG_DWORD, reinterpret_cast<const BYTE*>(&value), sizeof(value));
                    if (result != ERROR_SUCCESS) {
                        VIAM_SDK_LOG(error) << "Couldn't set metadata registry value for key " << deviceParamsKey;
                    } else {
                        VIAM_SDK_LOG(info) << "Created value " << valueName << " = " << value << " for key " << deviceParamsKey;
                    }
                } else if (result == ERROR_SUCCESS) {
                    VIAM_SDK_LOG(info) << "Value already exists on key " << deviceParamsKey << ", skipping.";
                } else {
                    VIAM_SDK_LOG(error) << "Error reading metadata value for key " << deviceParamsKey << ": " << result;
                }
                RegCloseKey(hDeviceKey);
                ++index;
            }
            RegCloseKey(hkey);
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to update windows device registry keys: " + std::string(e.what()));
    }
#endif
    {
        std::lock_guard<std::mutex> lock(devices_by_serial_mu());
        std::unique_ptr<ViamOBDevice> my_dev = std::make_unique<ViamOBDevice>();

        my_dev->pipe = pipe;
        my_dev->device = dev;
        my_dev->serial_number = serialNumber;
        my_dev->pointCloudFilter = pointCloudFilter;
        my_dev->align = align;
        my_dev->config = config;
        my_dev->postProcessDepthFilters = depthFilterList;

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
