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
#include "orbbec_firmware.hpp"
#include "orbbec_windows_registry.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
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

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/TypeHelper.hpp>

namespace orbbec {

namespace vsdk = ::viam::sdk;

vsdk::Model Orbbec::model_astra2("viam", "orbbec", "astra2");
vsdk::Model Orbbec::model_gemini_335le("viam", "orbbec", "gemini_335le");

// CONSTANTS BEGIN
const std::string kColorSourceName = "color";
const std::string kColorMimeTypeJPEG = "image/jpeg";
const std::string kColorMimeTypePNG = "image/png";
const std::string kDepthSourceName = "depth";
const std::string kDepthMimeTypeViamDep = "image/vnd.viam.dep";
const std::string kPcdMimeType = "pointcloud/pcd";
// If the firmwareUrl is changed to a new version, also change the minFirmwareVer const.
constexpr char service_name[] = "viam_orbbec";
const float mmToMeterMultiple = 0.001;
const uint64_t maxFrameAgeUs = 1e6;  // time until a frame is considered stale, in microseconds (equal to 1 sec)

// Model configurations
namespace {
static const OrbbecModelConfig ASTRA2_CONFIG{
    {"Orbbec Astra 2", "Orbbec Astra2"},                                                   // model_names
    "astra2",                                                                              // viam_model_suffix
    {1280, 720},                                                                           // default_color_resolution
    {1600, 1200},                                                                          // default_depth_resolution
    "https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Astra2_Release_2.8.20.zip",  // firmware_url
    "2.8.20",                                                                              // min_firmware_version
    {{{1920, 1080}, {{1600, 1200}, {800, 600}, {400, 300}}},                               // Supported resolutions, 16:9 aspect ratio
     {{1440, 1080}, {{1600, 1200}, {800, 600}, {400, 300}}},                               // Supported resolutions, 16:9 aspect ratio
     {{1280, 720}, {{1600, 1200}, {800, 600}, {400, 300}}},                                // Supported resolutions, 16:9 aspect ratio
     {{800, 600}, {{800, 600}, {400, 300}}},                                               // Supported resolutions, 4:3 aspect ratio
     {{640, 480}, {{800, 600}, {400, 300}}},                                               // Supported resolutions, 4:3 aspect ratio
     {{640, 360}, {{800, 600}, {400, 300}}}},                                              // Supported resolutions, 4:3 aspect ratio
    {"RGB", "MJPG"},                                                                       // supported_color_formats
    {"Y16"},                                                                               // supported_depth_formats
    "MJPG",                                                                                // default_color_format
    "Y16"                                                                                  // default_depth_format
};

static const OrbbecModelConfig GEMINI_335LE_CONFIG{
    {"Orbbec Gemini 335Le"},                                                                  // model_names
    "gemini_335le",                                                                           // viam_model_suffix
    {1280, 800},                                                                              // default_color_resolution
    {1280, 800},                                                                              // default_depth_resolution
    "https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.5.55.zip",  // firmware_url
    "1.5.55",                                                                                 // min_firmware_version
    {{{1280, 800}, {{1280, 800}, {848, 530}, {640, 400}, {424, 266}, {320, 200}}},            // Supported resolutions, 16:10 aspect ratio
     {{848, 530}, {{848, 530}, {640, 400}, {424, 266}, {320, 200}}},                          // 16:10 aspect ratio
     {{640, 400}, {{640, 400}, {424, 266}, {320, 200}}},                                      // 16:10 aspect ratio
     {{640, 480}, {{640, 480}}}},                                                             // 4:3 aspect ratio
    {"MJPG"},                                                                                 // supported_color_formats
    {"Y16"},                                                                                  // supported_depth_formats
    "MJPG",                                                                                   // default_color_format
    "Y16"                                                                                     // default_depth_format
};

static const std::vector<OrbbecModelConfig> all_model_configs = {ASTRA2_CONFIG, GEMINI_335LE_CONFIG};
}  // namespace

std::optional<OrbbecModelConfig> OrbbecModelConfig::forDevice(const std::string& device_name) {
    VIAM_SDK_LOG(debug) << "OrbbecModelConfig::forDevice called with device_name: '" << device_name << "'";

    // Check each model config
    for (const auto& config : all_model_configs) {
        // First, try exact match on any model name in the set
        if (config.model_names.count(device_name) != 0) {
            VIAM_SDK_LOG(debug) << "Found exact match for device_name";
            return config;
        }
    }

    VIAM_SDK_LOG(debug) << "No match found for device_name";
    return std::nullopt;
}

// CONSTANTS END

// STRUCTS BEGIN
struct PointXYZRGB {
    float x, y, z;
    std::uint32_t rgb;
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
void checkFirmwareVersion(const std::string version, const std::string& minFirmwareVer, const std::string& modelName) {
    int major = 0, minor = 0, patch = 0;
    int requiredMajor = 0, requiredMinor = 0, requiredPatch = 0;

    // ignore any trailing text in the case of a beta or RC version
    sscanf(version.c_str(), "%d.%d.%d*s", &major, &minor, &patch);
    sscanf(minFirmwareVer.c_str(), "%d.%d.%d", &requiredMajor, &requiredMinor, &requiredPatch);
    if ((major < requiredMajor) || (major == requiredMajor && minor < requiredMinor) ||
        (major == requiredMajor && minor == requiredMinor && patch < requiredPatch)) {
        throw std::runtime_error("Unsupported firmware version for " + modelName + ". Required: >= " + minFirmwareVer +
                                 ", Current: " + version + ". Call update_firmware command to upgrade.");
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

namespace {

// Helper function to format error messages
template <typename... Args>
std::string formatError(Args&&... args) {
    std::ostringstream buffer;
    (buffer << ... << args);
    return buffer.str();
}

// Get the best available timestamp for a frame (Global > System)
uint64_t getBestTimestampUs(std::shared_ptr<ob::Frame> frame) {
    if (frame == nullptr) {
        return 0;
    }
    uint64_t ts = frame->getGlobalTimeStampUs();
    if (ts > 0) {
        return ts;
    }
    return frame->getSystemTimeStampUs();
}

// Validate color frame format and timestamp
void validateColorFrame(std::shared_ptr<ob::Frame> color,
                        const std::optional<DeviceFormat>& device_format_opt,
                        const OrbbecModelConfig& modelConfig) {
    if (color == nullptr) {
        throw std::runtime_error("no color frame");
    }

    // Format validation
    std::string frameFormat = ob::TypeHelper::convertOBFormatTypeToString(color->getFormat());
    if (modelConfig.supported_color_formats.count(frameFormat) == 0) {
        std::ostringstream buffer;
        buffer << "unsupported color format: " << frameFormat << ", supported: ";
        for (const auto& fmt : modelConfig.supported_color_formats) {
            buffer << fmt << " ";
        }
        throw std::runtime_error(buffer.str());
    }

    // Timestamp validation
    uint64_t nowUs = getNowUs();
    uint64_t diff = timeSinceFrameUs(nowUs, getBestTimestampUs(color));
    if (diff > maxFrameAgeUs) {
        throw std::runtime_error(formatError("no recent color frame: check connection, diff: ", diff, "us"));
    }

    // Config format validation
    if (device_format_opt.has_value() && device_format_opt->color_format.has_value()) {
        if (frameFormat != device_format_opt->color_format.value()) {
            throw std::runtime_error(
                formatError("color format mismatch, expected ", device_format_opt->color_format.value(), " got ", frameFormat));
        }
    } else if (frameFormat != modelConfig.default_color_format) {
        throw std::runtime_error(formatError("color format mismatch, expected ", modelConfig.default_color_format, " got ", frameFormat));
    }
}

// Validate depth frame format and timestamp
void validateDepthFrame(std::shared_ptr<ob::Frame> depth,
                        const std::optional<DeviceFormat>& device_format_opt,
                        const OrbbecModelConfig& modelConfig) {
    if (depth == nullptr) {
        throw std::runtime_error("no depth frame");
    }

    // Format validation
    std::string frameFormat = ob::TypeHelper::convertOBFormatTypeToString(depth->getFormat());
    if (modelConfig.supported_depth_formats.count(frameFormat) == 0) {
        std::ostringstream buffer;
        buffer << "unsupported depth format: " << frameFormat << ", supported: ";
        for (const auto& fmt : modelConfig.supported_depth_formats) {
            buffer << fmt << " ";
        }
        throw std::runtime_error(buffer.str());
    }

    // Timestamp validation
    uint64_t nowUs = getNowUs();
    uint64_t diff = timeSinceFrameUs(nowUs, getBestTimestampUs(depth));
    if (diff > maxFrameAgeUs) {
        throw std::runtime_error(formatError("no recent depth frame: check connection, diff: ", diff, "us"));
    }

    // Config format validation
    if (device_format_opt.has_value() && device_format_opt->depth_format.has_value()) {
        if (frameFormat != device_format_opt->depth_format.value()) {
            throw std::runtime_error(
                formatError("depth format mismatch, expected ", device_format_opt->depth_format.value(), " got ", frameFormat));
        }
    } else if (frameFormat != modelConfig.default_depth_format) {
        throw std::runtime_error(formatError("depth format mismatch, expected ", modelConfig.default_depth_format, " got ", frameFormat));
    }
}

// Encode color frame to raw_image
vsdk::Camera::raw_image encodeColorFrame(std::shared_ptr<ob::Frame> color) {
    vsdk::Camera::raw_image image;
    image.source_name = kColorSourceName;

    std::uint8_t* colorData = (std::uint8_t*)color->getData();
    if (colorData == nullptr) {
        throw std::runtime_error("color data is null");
    }
    uint32_t colorDataSize = color->dataSize();

    if (color->getFormat() == OB_FORMAT_MJPG) {
        image.mime_type = kColorMimeTypeJPEG;
        image.bytes.assign(colorData, colorData + colorDataSize);
    } else if (color->getFormat() == OB_FORMAT_RGB) {
        image.mime_type = kColorMimeTypePNG;
        auto width = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getWidth();
        auto height = color->getStreamProfile()->as<ob::VideoStreamProfile>()->getHeight();
        image.bytes = encoding::encode_to_png(colorData, width, height);
    } else {
        throw std::runtime_error(
            formatError("unsupported color format: ", ob::TypeHelper::convertOBFormatTypeToString(color->getFormat())));
    }
    return image;
}

}  // anonymous namespace

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

// Helper function to check if a video stream profile matches the given resolution and format
bool profileMatchesSpec(std::shared_ptr<ob::VideoStreamProfile> vsp,
                        std::optional<DeviceResolution> deviceRes,
                        std::optional<DeviceFormat> deviceFormat,
                        const OrbbecModelConfig& modelConfig,
                        bool isColor) {
    if (deviceFormat.has_value()) {
        std::string requestedFormat = isColor ? (deviceFormat->color_format.has_value() ? deviceFormat->color_format.value() : "")
                                              : (deviceFormat->depth_format.has_value() ? deviceFormat->depth_format.value() : "");
        if (!requestedFormat.empty() && ob::TypeHelper::convertOBFormatTypeToString(vsp->getFormat()) != requestedFormat) {
            return false;
        }
    } else {
        std::string defaultFormat = isColor ? modelConfig.default_color_format : modelConfig.default_depth_format;
        if (ob::TypeHelper::convertOBFormatTypeToString(vsp->getFormat()) != defaultFormat) {
            return false;
        }
    }

    if (deviceRes.has_value()) {
        auto requestedRes = isColor ? deviceRes->color_resolution : deviceRes->depth_resolution;
        if (requestedRes.has_value() && (vsp->getWidth() != requestedRes->width || vsp->getHeight() != requestedRes->height)) {
            return false;
        }
    } else {
        auto defaultRes = isColor ? modelConfig.default_color_resolution : modelConfig.default_depth_resolution;
        if (vsp->getWidth() != defaultRes.width || vsp->getHeight() != defaultRes.height) {
            return false;
        }
    }

    return true;
}

// Helper function to find matching stream profiles
std::pair<std::shared_ptr<ob::StreamProfile>, std::shared_ptr<ob::StreamProfile>> findMatchingProfiles(
    std::shared_ptr<ob::Pipeline> pipe,
    std::optional<DeviceResolution> deviceRes,
    std::optional<DeviceFormat> deviceFormat,
    const OrbbecModelConfig& modelConfig) {
    auto colorStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_COLOR);
    auto depthStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_DEPTH);

    auto colorSpCount = colorStreamProfiles->getCount();
    auto depthSpCount = depthStreamProfiles->getCount();

    for (uint32_t i = 0; i < colorSpCount; i++) {
        auto colorProfile = colorStreamProfiles->getProfile(i);
        auto colorVsp = colorProfile->as<ob::VideoStreamProfile>();

        if (!profileMatchesSpec(colorVsp, deviceRes, deviceFormat, modelConfig, true)) {
            continue;
        }

        for (uint32_t j = 0; j < depthSpCount; j++) {
            auto depthProfile = depthStreamProfiles->getProfile(j);
            auto depthVsp = depthProfile->as<ob::VideoStreamProfile>();

            // make sure the color and depth stream have the same fps
            if (colorVsp->getFps() != depthVsp->getFps()) {
                continue;
            }

            if (!profileMatchesSpec(depthVsp, deviceRes, deviceFormat, modelConfig, false)) {
                continue;
            }

            return std::make_pair(colorProfile, depthProfile);
        }
    }

    return std::make_pair(nullptr, nullptr);
}

// Helper function to build error message for missing profiles
std::string buildProfileErrorMsg(bool isColor, std::optional<DeviceResolution> deviceRes, std::optional<DeviceFormat> deviceFormat) {
    std::ostringstream buffer;
    buffer << service_name << " does not support the requested " << (isColor ? "color" : "depth") << " resolution/format: ";
    auto res = isColor ? (deviceRes.has_value() ? deviceRes->color_resolution : std::nullopt)
                       : (deviceRes.has_value() ? deviceRes->depth_resolution : std::nullopt);
    if (res.has_value()) {
        buffer << "resolution " << res->width << "x" << res->height;
    }
    auto fmt = isColor ? (deviceFormat.has_value() ? deviceFormat->color_format : std::nullopt)
                       : (deviceFormat.has_value() ? deviceFormat->depth_format : std::nullopt);
    if (fmt.has_value()) {
        buffer << ", format " << fmt.value();
    }
    return buffer.str();
}

// create a config for software depth-to-color alignment with specified resolution/format
std::shared_ptr<ob::Config> createSwD2CAlignConfig(std::shared_ptr<ob::Pipeline> pipe,
                                                   std::optional<DeviceResolution> deviceRes,
                                                   std::optional<DeviceFormat> deviceFormat,
                                                   const OrbbecModelConfig& modelConfig) {
    if (deviceRes.has_value()) {
        VIAM_SDK_LOG(info) << "[createSwD2CAlignConfig] resolution specified: " << deviceRes->to_string();
    }
    if (deviceFormat.has_value()) {
        VIAM_SDK_LOG(info) << "[createSwD2CAlignConfig] format specified: " << deviceFormat->to_string();
    }

    // Find matching color and depth profiles
    auto [colorProfile, depthProfile] = findMatchingProfiles(pipe, deviceRes, deviceFormat, modelConfig);

    if (!colorProfile || !depthProfile) {
        VIAM_SDK_LOG(warn) << "[createSwD2CAlignConfig] Could not find matching stream profiles for software depth-to-color alignment that "
                           << "also match the given resolution and format specification ("
                           << (deviceRes.has_value() ? deviceRes->to_string() : "none") << ", "
                           << (deviceFormat.has_value() ? deviceFormat->to_string() : "none") << ")\n";
        return nullptr;
    }

    auto config = std::make_shared<ob::Config>();
    config->enableStream(colorProfile);
    config->enableStream(depthProfile);
    config->setAlignMode(ALIGN_D2C_SW_MODE);  // Use software alignment
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    auto colorVsp = colorProfile->as<ob::VideoStreamProfile>();
    auto depthVsp = depthProfile->as<ob::VideoStreamProfile>();
    VIAM_SDK_LOG(info) << "Using software depth-to-color alignment with color " << colorVsp->getWidth() << "x" << colorVsp->getHeight()
                       << " and depth " << depthVsp->getWidth() << "x" << depthVsp->getHeight();

    return config;
}

// create a config for hardware depth-to-color alignment
std::shared_ptr<ob::Config> createHwD2CAlignConfig(std::shared_ptr<ob::Pipeline> pipe,
                                                   std::optional<DeviceResolution> deviceRes,
                                                   std::optional<DeviceFormat> deviceFormat,
                                                   const OrbbecModelConfig& modelConfig) {
    if (deviceRes.has_value()) {
        VIAM_SDK_LOG(info) << "[createHwD2CAlignConfig] resolution specified: " << deviceRes->to_string();
    }
    if (deviceFormat.has_value()) {
        VIAM_SDK_LOG(info) << "[createHwD2CAlignConfig] format specified: " << deviceFormat->to_string();
    }

    // Find matching color and depth profiles that support hardware D2C alignment
    auto colorStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_COLOR);
    auto depthStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_DEPTH);

    auto colorSpCount = colorStreamProfiles->getCount();
    auto depthSpCount = depthStreamProfiles->getCount();

    for (uint32_t i = 0; i < colorSpCount; i++) {
        auto colorProfile = colorStreamProfiles->getProfile(i);
        auto colorVsp = colorProfile->as<ob::VideoStreamProfile>();

        if (!profileMatchesSpec(colorVsp, deviceRes, deviceFormat, modelConfig, true)) {
            continue;
        }

        for (uint32_t j = 0; j < depthSpCount; j++) {
            auto depthProfile = depthStreamProfiles->getProfile(j);
            auto depthVsp = depthProfile->as<ob::VideoStreamProfile>();

            // make sure the color and depth stream have the same fps
            if (colorVsp->getFps() != depthVsp->getFps()) {
                continue;
            }

            if (!profileMatchesSpec(depthVsp, deviceRes, deviceFormat, modelConfig, false)) {
                continue;
            }

            // Check if the given stream profiles support hardware depth-to-color alignment
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
                hwD2CAlignConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
                return hwD2CAlignConfig;
            } else {
                VIAM_SDK_LOG(warn) << "[createHwD2CAlignConfig] color stream " << colorVsp->getWidth() << "x" << colorVsp->getHeight()
                                   << "@" << colorVsp->getFps() << " and depth stream " << depthVsp->getWidth() << "x"
                                   << depthVsp->getHeight() << "@" << depthVsp->getFps()
                                   << " do NOT support hardware depth-to-color alignment\n";
            }
        }
    }

    VIAM_SDK_LOG(warn) << "[createHwD2CAlignConfig] Could not find matching stream profiles for hardware depth-to-color alignment that "
                          "also match the given resolution and format specification ("
                       << (deviceRes.has_value() ? deviceRes->to_string() : "none") << ", "
                       << (deviceFormat.has_value() ? deviceFormat->to_string() : "none") << ")\n";
    return nullptr;
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
        uint64_t diff = timeSinceFrameUs(nowUs, getBestTimestampUs(color));
        if (diff > maxFrameAgeUs) {
            std::cerr << "color frame is " << diff << "us older than now, nowUs: " << nowUs << " frameTimeUs " << getBestTimestampUs(color)
                      << "\n";
        }
        diff = timeSinceFrameUs(nowUs, getBestTimestampUs(depth));
        if (diff > maxFrameAgeUs) {
            std::cerr << "depth frame is " << diff << "us older than now, nowUs: " << nowUs << " frameTimeUs " << getBestTimestampUs(depth)
                      << "\n";
        }

        auto it = frame_set_by_serial().find(serialNumber);
        if (it != frame_set_by_serial().end()) {
            std::shared_ptr<ob::Frame> prevColor = it->second->getFrame(OB_FRAME_COLOR);
            std::shared_ptr<ob::Frame> prevDepth = it->second->getFrame(OB_FRAME_DEPTH);
            if (prevColor != nullptr && prevDepth != nullptr) {
                diff = timeSinceFrameUs(getBestTimestampUs(color), getBestTimestampUs(prevColor));
                if (diff > maxFrameAgeUs) {
                    std::cerr << "previous color frame is " << diff
                              << "us older than current color frame. previousUs: " << getBestTimestampUs(prevColor)
                              << " currentUs: " << getBestTimestampUs(color) << "\n";
                }
                diff = timeSinceFrameUs(getBestTimestampUs(depth), getBestTimestampUs(prevDepth));
                if (diff > maxFrameAgeUs) {
                    std::cerr << "previous depth frame is " << diff
                              << "us older than current depth frame. previousUs: " << getBestTimestampUs(prevDepth)
                              << " currentUs: " << getBestTimestampUs(depth) << "\n";
                }
            }
        }
        frame_set_by_serial()[serialNumber] = frameSet;
    };
}

void configureDevice(std::string serialNumber, OrbbecModelConfig const& modelConfig) {
    VIAM_SDK_LOG(info) << "[configureDevice] Configuring device " << serialNumber;
    std::lock_guard<std::mutex> lock(devices_by_serial_mu());
    auto search = devices_by_serial().find(serialNumber);
    if (search == devices_by_serial().end()) {
        std::ostringstream buffer;
        buffer << service_name << ": unable to configure undetected device " << serialNumber;
        throw std::invalid_argument(buffer.str());
    }

    std::unique_ptr<ViamOBDevice>& my_dev = search->second;

    // Enable global timestamp if supported
    if (my_dev->device->isGlobalTimestampSupported()) {
        my_dev->device->enableGlobalTimestamp(true);
        VIAM_SDK_LOG(info) << "[configureDevice] Global timestamp enabled for device " << serialNumber;
    } else {
        VIAM_SDK_LOG(info) << "[configureDevice] Global timestamp not supported for device " << serialNumber;
    }

    // Initialize fields if not already set
    if (!my_dev->pipe) {
        my_dev->pipe = std::make_shared<ob::Pipeline>(my_dev->device);
        my_dev->pipe->enableFrameSync();
    }
    if (!my_dev->pointCloudFilter) {
        my_dev->pointCloudFilter = std::make_shared<ob::PointCloudFilter>();
        my_dev->pointCloudFilter->setCreatePointFormat(OB_FORMAT_RGB_POINT);
    }
    if (!my_dev->align) {
        my_dev->align = std::make_shared<ob::Align>(OB_STREAM_COLOR);
    }
    if (my_dev->postProcessDepthFilters.empty()) {
        auto depthSensor = my_dev->device->getSensor(OB_SENSOR_DEPTH);
        if (depthSensor == nullptr) {
            throw std::runtime_error("Current device does not have a depth sensor.");
        }
        my_dev->postProcessDepthFilters = depthSensor->createRecommendedFilters();
        my_dev->applyEnabledPostProcessDepthFilters = false;
    }

    // Get user-specified resolution/format if any
    std::optional<DeviceResolution> resolution_opt;
    std::optional<DeviceFormat> format_opt;
    {
        std::lock_guard<std::mutex> configLock(config_by_serial_mu());
        if (config_by_serial().count(serialNumber) > 0) {
            resolution_opt = config_by_serial().at(serialNumber).device_resolution;
            format_opt = config_by_serial().at(serialNumber).device_format;
            VIAM_SDK_LOG(info) << "[configureDevice] Resolution from config: "
                               << (resolution_opt.has_value() ? resolution_opt->to_string() : "not specified, Format from config: ")
                               << (format_opt.has_value() ? format_opt->to_string() : "not specified");
        } else {
            VIAM_SDK_LOG(info) << "[configureDevice] No custom config for " << serialNumber << ", using defaults";
        }
    }

    // Create the pipeline config (with user settings if provided, otherwise defaults)
    // Lets try hardware depth-to-color alignment first, if it fails, try software alignment
    auto config = createHwD2CAlignConfig(my_dev->pipe, resolution_opt, format_opt, modelConfig);
    if (config == nullptr) {
        VIAM_SDK_LOG(warn) << "Device " << serialNumber << " does not support hardware depth-to-color alignment, trying software alignment";
        // Use software alignment
        if (resolution_opt.has_value() || format_opt.has_value()) {
            config = createSwD2CAlignConfig(my_dev->pipe, resolution_opt, format_opt, modelConfig);
            if (config == nullptr) {
                std::ostringstream buffer;
                buffer << service_name << ": unable to configure device " << serialNumber
                       << " with software D2C alignment for the requested resolution/format";
                throw std::runtime_error(buffer.str());
            }
        } else {
            // If the user didn't specify a resolution or format, use the first available profiles
            auto colorStreamProfiles = my_dev->pipe->getStreamProfileList(OB_SENSOR_COLOR);
            auto depthStreamProfiles = my_dev->pipe->getStreamProfileList(OB_SENSOR_DEPTH);
            if (colorStreamProfiles->getCount() > 0 && depthStreamProfiles->getCount() > 0) {
                config = std::make_shared<ob::Config>();
                config->enableStream(colorStreamProfiles->getProfile(0));
                config->enableStream(depthStreamProfiles->getProfile(0));
                config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
                VIAM_SDK_LOG(info) << "Created basic config for device " << serialNumber;
            } else {
                throw std::runtime_error("No stream profiles available");
            }
        }
        if (config != nullptr) {
            VIAM_SDK_LOG(info) << "Device " << serialNumber << " supports software depth-to-color alignment, using it";
        }
    } else {
        VIAM_SDK_LOG(info) << "Device " << serialNumber << " supports hardware depth-to-color alignment, using it";
    }

    if (config == nullptr) {
        std::ostringstream buffer;
        buffer << service_name << ": unable to configure device " << serialNumber << " - no valid stream configuration found";
        throw std::runtime_error(buffer.str());
    }
    my_dev->config = config;
}

void startDevice(std::string serialNumber, OrbbecModelConfig const& modelConfig) {
    VIAM_SDK_LOG(info) << service_name << ": starting device " << serialNumber;
    std::lock_guard<std::mutex> lock(devices_by_serial_mu());
    auto search = devices_by_serial().find(serialNumber);
    if (search == devices_by_serial().end()) {
        std::ostringstream buffer;
        buffer << service_name << ": unable to start undetected device" << serialNumber;
        throw std::invalid_argument(buffer.str());
    }

    std::shared_ptr<ob::DeviceInfo> deviceInfo = search->second->device->getDeviceInfo();
    if (deviceInfo == nullptr) {
        std::ostringstream buffer;
        buffer << service_name << ": unable to get device info for device " << serialNumber;
        throw std::runtime_error(buffer.str());
    }
    std::string deviceName = deviceInfo->name();

    VIAM_SDK_LOG(info) << "Device " << serialNumber << " is supported: " << deviceName;

    std::unique_ptr<ViamOBDevice>& my_dev = search->second;
    if (my_dev->started) {
        std::ostringstream buffer;
        buffer << service_name << ": device " << serialNumber << " is already started";
        throw std::runtime_error(buffer.str());
    }

    // Start the pipeline with the configuration
    my_dev->pipe->start(my_dev->config, frameCallback(serialNumber));
    my_dev->started = true;
    VIAM_SDK_LOG(info) << "Started pipeline for device " << serialNumber;
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
void Orbbec::validate_sensor(std::pair<std::string, viam::sdk::ProtoValue> const& sensor_pair, const OrbbecModelConfig& modelConfig) {
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
        if (!modelConfig.supported_color_formats.count(*format)) {
            std::ostringstream buffer;
            buffer << "color sensor format must be one of: ";
            for (const auto& type : modelConfig.supported_color_formats) {
                buffer << type << " ";
            }
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::invalid_argument(buffer.str());
        }

    } else if (sensor_type == "depth") {
        if (!modelConfig.supported_depth_formats.count(*format)) {
            std::ostringstream buffer;
            buffer << "depth sensor format must be one of: ";
            for (const auto& type : modelConfig.supported_depth_formats) {
                buffer << type << " ";
            }
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::invalid_argument(buffer.str());
        }
    } else {
        throw std::invalid_argument("sensor type must be color or depth");
    }
}

std::vector<std::string> Orbbec::validateOrbbecModel(vsdk::ResourceConfig cfg, OrbbecModelConfig const& modelConfig) {
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
                validate_sensor(sensor_pair, modelConfig);
            }
        }
        auto color_width_uint32 =
            static_cast<std::uint32_t>(*sensors->at("color").get<viam::sdk::ProtoStruct>()->at("width").get<double>());
        auto color_height_uint32 =
            static_cast<std::uint32_t>(*sensors->at("color").get<viam::sdk::ProtoStruct>()->at("height").get<double>());
        if (modelConfig.color_to_depth_supported_resolutions.count({color_width_uint32, color_height_uint32}) == 0) {
            std::ostringstream buffer;
            buffer << "color resolution must be one of: ";
            for (const auto& res : modelConfig.color_to_depth_supported_resolutions) {
                buffer << "{" << res.first.to_string() << "} ";
            }
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::invalid_argument(buffer.str());
        }
        auto depth_width_uint32 =
            static_cast<std::uint32_t>(*sensors->at("depth").get<viam::sdk::ProtoStruct>()->at("width").get<double>());
        auto depth_height_uint32 =
            static_cast<std::uint32_t>(*sensors->at("depth").get<viam::sdk::ProtoStruct>()->at("height").get<double>());
        if (modelConfig.color_to_depth_supported_resolutions.at({color_width_uint32, color_height_uint32})
                .count({depth_width_uint32, depth_height_uint32}) == 0) {
            std::ostringstream buffer;
            buffer << "color/depth resolution combination not supported, for color resolution " << "{" << color_width_uint32 << ", "
                   << color_height_uint32 << "}, depth resolution must be one of: ";
            for (const auto& res : modelConfig.color_to_depth_supported_resolutions.at({color_width_uint32, color_height_uint32})) {
                buffer << "{" << res.to_string() << "} ";
            }
            VIAM_SDK_LOG(error) << buffer.str();
            throw std::invalid_argument(buffer.str());
        }
    }

    return {};
}

std::vector<std::string> Orbbec::validateAstra2(vsdk::ResourceConfig cfg) {
    return validateOrbbecModel(cfg, ASTRA2_CONFIG);
}

std::vector<std::string> Orbbec::validateGemini335Le(vsdk::ResourceConfig cfg) {
#if !defined(__linux__)
    return {"viam:orbbec:gemini_335le is only supported on Linux hosts"};
#else
    return validateOrbbecModel(cfg, GEMINI_335LE_CONFIG);
#endif
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

void applyExperimentalConfig(std::unique_ptr<ViamOBDevice>& my_dev, vsdk::ProtoStruct const& config) {
    if (config.count("device_properties") > 0) {
        auto device_properties = config.at("device_properties").get<vsdk::ProtoStruct>();
        if (device_properties) {
            device_control::setDeviceProperties(my_dev->device, *device_properties, "Orbbec Constructor");
            VIAM_SDK_LOG(info) << "[applyExperimentalConfig] device_properties applied";
        }
    }
    if (config.count("post_process_depth_filters") > 0) {
        auto filters = config.at("post_process_depth_filters").get<vsdk::ProtoStruct>();
        if (filters) {
            device_control::setPostProcessDepthFilters(my_dev->postProcessDepthFilters, *filters, "Orbbec Constructor");
            VIAM_SDK_LOG(info) << "[applyExperimentalConfig] postProcessDepthFilters:  "
                               << device_control::postProcessDepthFiltersToString(my_dev->postProcessDepthFilters);
        }
    }
    if (config.count("apply_post_process_depth_filters") > 0) {
        auto apply_filters = config.at("apply_post_process_depth_filters").get<bool>();
        if (apply_filters) {
            device_control::applyPostProcessDepthFilters(my_dev, *apply_filters, "Orbbec Constructor");
            VIAM_SDK_LOG(info) << "[applyExperimentalConfig] apply_post_process_depth_filters: "
                               << my_dev->applyEnabledPostProcessDepthFilters;
        }
    }
}

Orbbec::Orbbec(vsdk::Dependencies deps, vsdk::ResourceConfig cfg, std::shared_ptr<ob::Context> ctx)
    : Camera(cfg.name()), serial_number_(getSerialNumber(cfg)), ob_ctx_(std::move(ctx)) {
    VIAM_RESOURCE_LOG(info) << "Orbbec constructor start " << serial_number_;
    auto config = configure(deps, cfg);
    {
        std::lock_guard<std::mutex> lock(config_by_serial_mu());
        config_by_serial().insert_or_assign(serial_number_, *config);
        VIAM_RESOURCE_LOG(info) << "initial config_by_serial_: " << config_by_serial().at(serial_number_).to_string();
    }

    {
        std::lock_guard<std::mutex> lock(devices_by_serial_mu());
        auto search = devices_by_serial().find(serial_number_);
        if (search == devices_by_serial().end()) {
            std::ostringstream buffer;
            buffer << service_name << ": unable to start undetected device" << serial_number_;
            throw std::invalid_argument(buffer.str());
        }
        std::unique_ptr<ViamOBDevice>& my_dev = search->second;
        applyExperimentalConfig(my_dev, cfg.attributes());
    }

    {
        std::lock_guard<std::mutex> lock(devices_by_serial_mu());
        auto search = devices_by_serial().find(serial_number_);
        if (search != devices_by_serial().end()) {
            firmware_version_ = search->second->device->getDeviceInfo()->firmwareVersion();

            // Detect model and set model_config_
            std::shared_ptr<ob::DeviceInfo> deviceInfo = search->second->device->getDeviceInfo();
            model_config_ = OrbbecModelConfig::forDevice(deviceInfo->name());
            VIAM_SDK_LOG(info) << "Detected model: " << model_config_->viam_model_suffix;
        }
    }

    if (!model_config_.has_value()) {
        throw std::runtime_error("Failed to detect Orbbec model configuration");
    }

    configureDevice(serial_number_, model_config_.value());
    startDevice(serial_number_, model_config_.value());
    {
        std::lock_guard<std::mutex> lock(serial_by_resource_mu());
        serial_by_resource()[config->resource_name] = serial_number_;
    }

    VIAM_RESOURCE_LOG(info) << "Orbbec constructor end " << serial_number_;
}

Orbbec::~Orbbec() {
    VIAM_RESOURCE_LOG(info) << "Orbbec destructor start " << serial_number_;
    std::string prev_serial_number;
    std::string prev_resource_name;
    {
        const std::lock_guard<std::mutex> lock(config_by_serial_mu());
        if (config_by_serial().count(serial_number_) == 0) {
            VIAM_RESOURCE_LOG(error) << "Orbbec destructor: device with serial number " << serial_number_
                                     << " is not in config_by_serial, skipping erase";
        } else {
            prev_serial_number = config_by_serial().at(serial_number_).serial_number;
            prev_resource_name = config_by_serial().at(serial_number_).resource_name;
        }
    }
    stopDevice(prev_serial_number, prev_resource_name);
    VIAM_RESOURCE_LOG(info) << "Orbbec destructor end " << serial_number_;
}

void Orbbec::reconfigure(const vsdk::Dependencies& deps, const vsdk::ResourceConfig& cfg) {
    VIAM_RESOURCE_LOG(info) << "[reconfigure] Orbbec reconfigure start";
    std::string prev_serial_number;
    std::string prev_resource_name;
    {
        const std::lock_guard<std::mutex> lock_serial(serial_number_mu_);
        const std::lock_guard<std::mutex> lock(config_by_serial_mu());
        if (config_by_serial().count(serial_number_) == 0) {
            std::ostringstream buffer;
            buffer << "[reconfigure] device with serial number " << serial_number_ << " is not in config_by_serial, skipping reconfigure";
            VIAM_RESOURCE_LOG(error) << buffer.str();
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
        VIAM_RESOURCE_LOG(info) << "[reconfigure] updated config_by_serial_: " << config_by_serial().at(new_serial_number).to_string();
    }

    {
        std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
        frame_set_by_serial().erase(prev_serial_number);
    }

    // set firmware version member variable and apply experimental config
    {
        std::lock_guard<std::mutex> lock_serial(serial_number_mu_);
        std::lock_guard<std::mutex> lock(devices_by_serial_mu());
        auto search = devices_by_serial().find(serial_number_);
        if (search != devices_by_serial().end()) {
            firmware_version_ = search->second->device->getDeviceInfo()->firmwareVersion();

            // Detect model and set model_config_
            std::shared_ptr<ob::DeviceInfo> deviceInfo = search->second->device->getDeviceInfo();
            model_config_ = OrbbecModelConfig::forDevice(deviceInfo->name());

            std::unique_ptr<ViamOBDevice>& my_dev = search->second;
            applyExperimentalConfig(my_dev, cfg.attributes());
        }
    }

    if (!model_config_.has_value()) {
        throw std::runtime_error("Failed to detect Orbbec model configuration during reconfigure");
    }

    configureDevice(new_serial_number, model_config_.value());
    startDevice(new_serial_number, model_config_.value());
    {
        std::lock_guard<std::mutex> lock(serial_by_resource_mu());
        serial_by_resource()[new_resource_name] = new_serial_number;
    }
    VIAM_RESOURCE_LOG(info) << "[reconfigure] Orbbec reconfigure end";
}

vsdk::Camera::raw_image Orbbec::get_image(std::string mime_type, const vsdk::ProtoStruct& extra) {
    try {
        VIAM_RESOURCE_LOG(debug) << "[get_image] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(serial_number_mu_);
            serial_number = serial_number_;
        }

        if (model_config_.has_value()) {
            checkFirmwareVersion(firmware_version_, model_config_->min_firmware_version, model_config_->viam_model_suffix);
        }

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

        std::optional<DeviceFormat> res_format_opt;
        {
            std::lock_guard<std::mutex> lock(config_by_serial_mu());
            if (config_by_serial().count(serial_number) == 0) {
                throw std::invalid_argument("device with serial number " + serial_number + " is not in config_by_serial");
            }
            res_format_opt = config_by_serial().at(serial_number).device_format;
        }

        validateColorFrame(color, res_format_opt, *model_config_);
        vsdk::Camera::raw_image response = encodeColorFrame(color);
        VIAM_RESOURCE_LOG(debug) << "[get_image] end";
        return response;
    } catch (const std::exception& e) {
        VIAM_RESOURCE_LOG(error) << "[get_image] error: " << e.what();
        throw std::runtime_error("failed to create image: " + std::string(e.what()));
    }
}

vsdk::Camera::properties Orbbec::get_properties() {
    try {
        VIAM_RESOURCE_LOG(debug) << "[get_properties] start";

        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(serial_number_mu_);
            if (serial_number_.empty()) {
                throw std::runtime_error("serial number is null");
            }
            serial_number = serial_number_;
        }

        OBCameraIntrinsic intrinsic_props;
        OBCameraDistortion distortion_props;
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
            auto const camera_params = my_dev->pipe->getCameraParam();
            intrinsic_props = camera_params.rgbIntrinsic;
            distortion_props = camera_params.rgbDistortion;
        }

        vsdk::Camera::properties p{};
        p.supports_pcd = true;
        p.intrinsic_parameters.width_px = intrinsic_props.width;
        p.intrinsic_parameters.height_px = intrinsic_props.height;
        p.intrinsic_parameters.focal_x_px = intrinsic_props.fx;
        p.intrinsic_parameters.focal_y_px = intrinsic_props.fy;
        p.intrinsic_parameters.center_x_px = intrinsic_props.cx;
        p.intrinsic_parameters.center_y_px = intrinsic_props.cy;
        p.distortion_parameters.model = device_control::distortionTypeToString(distortion_props.model);

        // TODO: These should be named parameters in the struct, not relying on order
        // If this is ever changed, make sure to update the README
        p.distortion_parameters.parameters = std::vector<double>{distortion_props.p1,
                                                                 distortion_props.p2,
                                                                 distortion_props.k1,
                                                                 distortion_props.k2,
                                                                 distortion_props.k3,
                                                                 distortion_props.k4,
                                                                 distortion_props.k5,
                                                                 distortion_props.k6};

        VIAM_RESOURCE_LOG(debug) << "[get_properties] end";
        return p;
    } catch (const std::exception& e) {
        VIAM_RESOURCE_LOG(error) << "[get_properties] error: " << e.what();
        throw std::runtime_error("failed to create properties: " + std::string(e.what()));
    }
}

vsdk::Camera::image_collection Orbbec::get_images(std::vector<std::string> filter_source_names, const vsdk::ProtoStruct& extra) {
    try {
        VIAM_RESOURCE_LOG(debug) << "[get_images] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(serial_number_mu_);
            serial_number = serial_number_;
        }

        if (model_config_.has_value()) {
            checkFirmwareVersion(firmware_version_, model_config_->min_firmware_version, model_config_->viam_model_suffix);
        }

        std::shared_ptr<ob::FrameSet> fs = nullptr;
        {
            std::lock_guard<std::mutex> lock(frame_set_by_serial_mu());
            auto search = frame_set_by_serial().find(serial_number);
            if (search == frame_set_by_serial().end()) {
                throw std::runtime_error("no frame yet");
            }
            fs = search->second;
        }
        if (fs == nullptr) {
            throw std::runtime_error("no frameset");
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
        std::optional<DeviceFormat> device_format_opt;
        {
            std::lock_guard<std::mutex> lock(config_by_serial_mu());
            if (config_by_serial().count(serial_number) == 0) {
                throw std::runtime_error("device with serial number " + serial_number + " is not in config_by_serial");
            }
            device_format_opt = config_by_serial().at(serial_number).device_format;
        }

        if (should_process_color) {
            color = fs->getFrame(OB_FRAME_COLOR);
            validateColorFrame(color, device_format_opt, *model_config_);

            vsdk::Camera::raw_image color_image = encodeColorFrame(color);
            color_image.source_name = kColorSourceName;
            response.images.emplace_back(std::move(color_image));
        }

        if (should_process_depth) {
            depth = fs->getFrame(OB_FRAME_DEPTH);
            validateDepthFrame(depth, device_format_opt, *model_config_);

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
            VIAM_RESOURCE_LOG(error) << "[get_images] error: no camera sources matched the filter";
            return response;
        }

        uint64_t colorTS = color ? getBestTimestampUs(color) : 0;
        uint64_t depthTS = depth ? getBestTimestampUs(depth) : 0;
        uint64_t timestamp = 0;

        if (colorTS > 0 && depthTS > 0) {
            if (colorTS != depthTS) {
                VIAM_RESOURCE_LOG(info) << "color and depth timestamps differ, defaulting to "
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
        VIAM_RESOURCE_LOG(debug) << "[get_images] end";
        return response;
    } catch (const std::exception& e) {
        VIAM_RESOURCE_LOG(error) << "[get_images] error: " << e.what();
        throw std::runtime_error("failed to create images: " + std::string(e.what()));
    }
}

vsdk::ProtoStruct Orbbec::do_command(const vsdk::ProtoStruct& command) {
    bool call_get_properties = false;
    try {
        constexpr char firmware_key[] = "update_firmware";
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
            for (auto const& [key, value] : command) {
                if (key == firmware_key) {
                    VIAM_RESOURCE_LOG(info) << "Received firmware update command";
                    vsdk::ProtoStruct resp = viam::sdk::ProtoStruct{};
                    if (!model_config_.has_value()) {
                        throw std::runtime_error("Model configuration not available for firmware update");
                    }
                    if (firmware_version_.find(model_config_->min_firmware_version) != std::string::npos) {
                        std::ostringstream buffer;
                        buffer << "Device firmware already on version " << model_config_->min_firmware_version;
                        resp.emplace(firmware_key, buffer.str());
                        VIAM_RESOURCE_LOG(info) << buffer.str();
                        return resp;
                    }
                    VIAM_RESOURCE_LOG(info) << "Updating device firmware...";
                    if (dev->started) {
                        dev->pipe->stop();
                        dev->started = false;
                    }

                    try {
                        if (model_config_->firmware_url.has_value()) {
                            VIAM_RESOURCE_LOG(info) << "Updating device firmware from URL: " << model_config_->firmware_url.value();
                            firmware::updateFirmware(dev, ob_ctx_, model_config_->firmware_url.value(), logger_);
                        } else {
                            throw std::runtime_error("Firmware update not supported for this model");
                        }
                        firmware_version_ = model_config_->min_firmware_version;
                    } catch (const std::exception& e) {
                        std::ostringstream buffer;
                        buffer << "firmware update failed: " << e.what();
                        VIAM_RESOURCE_LOG(error) << buffer.str();
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
                    return resp;
                } else if (key == "dump_pcl_files") {
                    if (!value.is_a<bool>()) {
                        VIAM_RESOURCE_LOG(error) << "[do_command] dump_pcl_files: expected bool, got " << value.kind();
                        return vsdk::ProtoStruct{{"error", "expected bool"}};
                    }
                    dev->dumpPCLFiles = value.get_unchecked<bool>();
                    return {{"dump_pcl_files", dev->dumpPCLFiles}};
                } else if (key == "apply_post_process_depth_filters") {
                    return device_control::applyPostProcessDepthFilters(dev, value, key);
                } else if (key == "get_recommended_post_process_depth_filters") {
                    return device_control::getRecommendedPostProcessDepthFilters(dev->device);
                } else if (key == "set_recommended_post_process_depth_filters") {
                    return device_control::setRecommendedPostProcessDepthFilters(dev);
                } else if (key == "get_post_process_depth_filters") {
                    return device_control::getPostProcessDepthFilters(dev->postProcessDepthFilters, key);
                } else if (key == "set_post_process_depth_filters") {
                    return device_control::setPostProcessDepthFilters(dev->postProcessDepthFilters, value, key);
                } else if (key == "get_depth_unit") {
                    return device_control::getDepthUnit(dev->device, key);
                } else if (key == "set_depth_unit") {
                    return device_control::setDepthUnit(dev->device, value, key);
                } else if (key == "get_device_properties") {
                    return device_control::getDeviceProperties(dev->device, key);
                } else if (key == "set_device_properties") {
                    return device_control::setDeviceProperties(dev->device, value, key);
                } else if (key == "get_device_property") {
                    return device_control::getDeviceProperty(dev->device, value, key);
                } else if (key == "set_device_property") {
                    return device_control::setDeviceProperty(dev->device, value, key);
                } else if (key == "get_device_info") {
                    return device_control::getDeviceInfo(dev->device, key);
                } else if (key == "get_orbbec_sdk_version") {
                    return device_control::getOrbbecSDKVersion(key);
                } else if (key == "get_camera_params") {
                    return device_control::getCameraParams<ob::Pipeline, ob::VideoStreamProfile>(dev->pipe);
                } else if (key == "get_camera_temperature") {
                    return device_control::getCameraTemperature(dev->device, key);
                } else if (key == "create_module_config") {
                    return device_control::createModuleConfig<ViamOBDevice, ob::VideoStreamProfile>(dev);
                } else if (key == "call_get_properties") {
                    call_get_properties = true;
                }
            }
        }  // unlock devices_by_serial_mu_
        if (call_get_properties) {
            VIAM_RESOURCE_LOG(info) << "[do_command] calling get_properties";
            auto const props = get_properties();
            VIAM_RESOURCE_LOG(info) << "[do_command] get_properties called";
            vsdk::ProtoStruct resp;
            resp["supports_pcd"] = props.supports_pcd;
            resp["intrinsic_parameters"] = vsdk::ProtoStruct{{"width_px", props.intrinsic_parameters.width_px},
                                                             {"height_px", props.intrinsic_parameters.height_px},
                                                             {"focal_x_px", props.intrinsic_parameters.focal_x_px},
                                                             {"focal_y_px", props.intrinsic_parameters.focal_y_px},
                                                             {"center_x_px", props.intrinsic_parameters.center_x_px},
                                                             {"center_y_px", props.intrinsic_parameters.center_y_px}};
            vsdk::ProtoStruct distortion_params;
            distortion_params["model"] = props.distortion_parameters.model;
            std::vector<viam::sdk::ProtoValue> proto_params;
            for (const auto& val : props.distortion_parameters.parameters) {
                proto_params.emplace_back(val);  // Each double becomes a ProtoValue
            }
            distortion_params["parameters"] = viam::sdk::ProtoValue(proto_params);
            resp["distortion_parameters"] = distortion_params;
            VIAM_RESOURCE_LOG(info) << "[do_command] get_properties returning";
            return resp;
        }
    } catch (const std::exception& e) {
        VIAM_RESOURCE_LOG(error) << service_name << ": exception caught: " << e.what();
    }
    return vsdk::ProtoStruct{};
}

vsdk::Camera::point_cloud Orbbec::get_point_cloud(std::string mime_type, const vsdk::ProtoStruct& extra) {
    try {
        VIAM_RESOURCE_LOG(debug) << "[get_point_cloud] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(serial_number_mu_);
            serial_number = serial_number_;
        }

        if (model_config_.has_value()) {
            checkFirmwareVersion(firmware_version_, model_config_->min_firmware_version, model_config_->viam_model_suffix);
        }

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
        std::shared_ptr<ob::Frame> depth = fs->getFrame(OB_FRAME_DEPTH);

        validateColorFrame(color, std::nullopt, *model_config_);
        validateDepthFrame(depth, std::nullopt, *model_config_);

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

        // If we have any post process depth filters configured, apply them now
        if (my_dev->applyEnabledPostProcessDepthFilters && my_dev->postProcessDepthFilters.size() > 0) {
            for (auto& filter : my_dev->postProcessDepthFilters) {
                depth = filter->process(depth);
            }
            fs->pushFrame(depth);
        }

        std::vector<unsigned char> data;
        data = RGBPointsToPCD(my_dev->pointCloudFilter->process(my_dev->align->process(fs)), scale * mmToMeterMultiple);

        // Write PCD to file if dumpPCLFiles is set
        if (my_dev->dumpPCLFiles) {
            auto timestamp = getNowUs();
            // Get the path stored in the environment variable VIAM_MODULE_DATA
            // If the environment variable is not set, use the current working directory
            // to store the PCD file
            std::stringstream outfile_name;
            std::string viam_module_data = std::getenv("VIAM_MODULE_DATA");
            if (!viam_module_data.empty()) {
                std::filesystem::path dir_path(viam_module_data);
                if (std::filesystem::exists(dir_path) && std::filesystem::is_directory(dir_path)) {
                    outfile_name << viam_module_data << "/pointcloud_" << timestamp << ".pcd";
                } else {
                    VIAM_RESOURCE_LOG(warn) << "VIAM_MODULE_DATA is set to " << viam_module_data
                                            << " but is not a valid directory, using current working directory to store PCD file";
                    outfile_name << "pointcloud_" << timestamp << ".pcd";
                }
            } else {
                VIAM_RESOURCE_LOG(warn) << "VIAM_MODULE_DATA is not set, using current working directory to store PCD file";
                outfile_name << "pointcloud_" << timestamp << ".pcd";
            }
            std::ofstream outfile(outfile_name.str(), std::ios::out | std::ios::binary);
            outfile.write((const char*)&data[0], data.size());

            std::filesystem::path file_path(outfile_name.str());
            std::filesystem::path absolute_path = std::filesystem::absolute(file_path);
            std::string absolute_path_str = absolute_path.string();
            outfile.close();
            VIAM_RESOURCE_LOG(info) << "[get_point_cloud] wrote PCD to location: " << absolute_path_str;
        }

        VIAM_RESOURCE_LOG(debug) << "[get_point_cloud] end";
        return vsdk::Camera::point_cloud{kPcdMimeType, data};
    } catch (const std::exception& e) {
        VIAM_RESOURCE_LOG(error) << "[get_point_cloud] error: " << e.what();
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
    VIAM_SDK_LOG(info) << "[registerDevice] Registering device " << serialNumber;

#ifdef _WIN32
    // Setup Windows device registry for Orbbec cameras
    try {
        windows_registry::setupWindowsDeviceRegistry(dev);
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to setup windows device registry: " + std::string(e.what()));
    }
#endif

    {
        std::lock_guard<std::mutex> lock(devices_by_serial_mu());
        std::unique_ptr<ViamOBDevice> my_dev = std::make_unique<ViamOBDevice>();
        my_dev->device = dev;
        my_dev->serialNumber = serialNumber;
        devices_by_serial()[serialNumber] = std::move(my_dev);
        VIAM_SDK_LOG(info) << "[registerDevice] Successfully registered device " << serialNumber;
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
                        // Determine model configuration
                        std::optional<OrbbecModelConfig> modelConfig = OrbbecModelConfig::forDevice(info->name());
                        if (!modelConfig.has_value()) {
                            VIAM_SDK_LOG(error) << "Failed to determine model configuration for device " << serial_number;
                            continue;
                        }
                        configureDevice(serial_number, modelConfig.value());
                        startDevice(serial_number, modelConfig.value());
                        serial_by_resource()[resource_name] = serial_number;
                    }
                }
            }
        }
    } catch (ob::Error& e) {
        VIAM_SDK_LOG(error) << "setDeviceChangedCallback\n"
                            << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nname:" << e.getName()
                            << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    }
}

void startOrbbecSDK(ob::Context& ctx) {
    ctx.setDeviceChangedCallback(deviceChangedCallback);

    // Enable network device enumeration for Ethernet cameras (like Gemini 335Le)
    try {
        ctx.enableNetDeviceEnumeration(true);
        VIAM_SDK_LOG(info) << "Enabled network device enumeration for Ethernet cameras";
    } catch (ob::Error& e) {
        VIAM_SDK_LOG(warn) << "Failed to enable network device enumeration: " << e.what();
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(warn) << "Failed to enable network device enumeration: " << e.what();
    }

    try {
        // Use SDK's native device discovery (works for both USB and network devices)
        std::shared_ptr<ob::DeviceList> devList = ctx.queryDeviceList();
        int devCount = devList->getCount();

        if (devCount == 0) {
            VIAM_SDK_LOG(warn) << "No Orbbec devices found";
            return;
        }

        VIAM_SDK_LOG(info) << "Found " << devCount << " Orbbec devices";

        for (size_t i = 0; i < devCount; i++) {
            try {
                // Get device information from DeviceList without triggering USB control transfers
                std::string deviceName = devList->name(i);
                std::string serialNumber = devList->serialNumber(i);
                std::string connectionType = devList->connectionType(i);
                std::string ipAddress = devList->ipAddress(i);

                std::stringstream deviceInfoString;
                deviceInfoString << "Device " << (i + 1) << " - Name: " << deviceName << ", Serial: " << serialNumber
                                 << ", Connection: " << connectionType;
                if (!ipAddress.empty()) {
                    deviceInfoString << ", IP: " << ipAddress;
                }
                VIAM_SDK_LOG(info) << deviceInfoString.str();

                // Only create device if we can get the serial number
                if (!serialNumber.empty()) {
                    try {
                        std::shared_ptr<ob::Device> dev = devList->getDevice(i);
                        registerDevice(serialNumber, dev);
                        VIAM_SDK_LOG(info) << "Successfully registered device " << serialNumber;
                    } catch (ob::Error& devError) {
                        VIAM_SDK_LOG(warn) << "Failed to create device for " << serialNumber << ": " << devError.what();
                    }
                } else {
                    VIAM_SDK_LOG(warn) << "Skipping device " << (i + 1) << " - no serial number available";
                }
            } catch (ob::Error& deviceError) {
                VIAM_SDK_LOG(warn) << "Failed to process device " << (i + 1) << ": " << deviceError.what();
                // Continue with other devices even if one fails
            }
        }
    } catch (ob::Error& e) {
        VIAM_SDK_LOG(error) << "Failed to query Orbbec devices: " << e.what() << " (function: " << e.getFunction()
                            << ", args: " << e.getArgs() << ", name: " << e.getName() << ", type: " << e.getExceptionType() << ")";
        VIAM_SDK_LOG(warn)
            << "Continuing without Orbbec devices - check network connectivity for Ethernet cameras or USB connection for USB cameras";
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "Failed to query Orbbec devices: " << e.what();
        VIAM_SDK_LOG(warn)
            << "Continuing without Orbbec devices - check network connectivity for Ethernet cameras or USB connection for USB cameras";
    } catch (...) {
        VIAM_SDK_LOG(error) << "Failed to query Orbbec devices: unknown error";
        VIAM_SDK_LOG(warn)
            << "Continuing without Orbbec devices - check network connectivity for Ethernet cameras or USB connection for USB cameras";
    }
}
// ORBBEC SDK DEVICE REGISTRY END

}  // namespace orbbec
