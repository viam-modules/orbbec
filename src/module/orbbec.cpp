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

#include <math.h>
#include <chrono>
#include <cstdint>
#include <cstdio>

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

#include <libobsensor/ObSensor.hpp>

namespace orbbec {

namespace vsdk = ::viam::sdk;

vsdk::Model Orbbec::model("viam", "orbbec", "astra2");

const std::string kColorSourceName = "color";
const std::string kColorMimeTypeJPEG = "image/jpeg";
const std::string kDepthSourceName = "depth";
const std::string kDepthMimeTypeViamDep = "image/vnd.viam.dep";
const std::string kPcdMimeType = "pointcloud/pcd";

// CONSTANTS BEGIN
constexpr char service_name[] = "viam_orbbec";
const float mmToMeterMultiple = 0.001;
const uint64_t maxFrameAgeUs = 1e6;  // time until a frame is considered stale, in microseconds (equal to 1 sec)

// CONSTANTS END

// STRUCTS BEGIN
struct PointXYZRGB {
    float x, y, z;
    unsigned int rgb;
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

struct raw_camera_image {
    using deleter_type = void (*)(unsigned char*);
    using uniq = std::unique_ptr<unsigned char[], deleter_type>;

    static constexpr deleter_type free_deleter = [](unsigned char* ptr) { free(ptr); };

    static constexpr deleter_type array_delete_deleter = [](unsigned char* ptr) { delete[] ptr; };

    uniq bytes;
    size_t size;
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
// GLOBALS END

// HELPERS BEGIN
//
uint64_t getNowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

uint64_t timeSinceFrameUs(uint64_t nowUs, uint64_t imageTimeUs) {
    if (nowUs > imageTimeUs) {
        return nowUs - imageTimeUs;
    }
    return 0;
}

std::vector<unsigned char> RGBPointsToPCD(std::shared_ptr<ob::Frame> frame, float scale) {
    int numPoints = frame->dataSize() / sizeof(OBColorPoint);
    VIAM_SDK_LOG(info) << "[RGBPointsToPCD] input numPoints: " << numPoints;

    OBColorPoint* points = (OBColorPoint*)frame->data();
    std::vector<PointXYZRGB> pcdPoints;

    for (int i = 0; i < numPoints; i++) {
        OBColorPoint& p = points[i];
        unsigned int r = (unsigned int)p.r;
        unsigned int g = (unsigned int)p.g;
        unsigned int b = (unsigned int)p.b;
        unsigned int rgb = (r << 16) | (g << 8) | b;
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
    std::vector<unsigned char> pcdBytes;
    pcdBytes.insert(pcdBytes.end(), headerStr.begin(), headerStr.end());
    for (auto& p : pcdPoints) {
        unsigned char* x = (unsigned char*)&p.x;
        unsigned char* y = (unsigned char*)&p.y;
        unsigned char* z = (unsigned char*)&p.z;
        unsigned char* rgb = (unsigned char*)&p.rgb;

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
            VIAM_SDK_LOG(info) << "using width: " << vsp->getWidth() << " height: " << vsp->getHeight()
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
std::shared_ptr<ob::Config> createHwD2CAlignConfig(std::shared_ptr<ob::Pipeline> pipe) {
    auto colorStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_COLOR);
    auto depthStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_DEPTH);

    // Iterate through all color and depth stream profiles to find a match for
    // hardware depth-to-color alignment
    auto colorSpCount = colorStreamProfiles->getCount();
    auto depthSpCount = depthStreamProfiles->getCount();
    for (uint32_t i = 0; i < colorSpCount; i++) {
        auto colorProfile = colorStreamProfiles->getProfile(i);
        auto colorVsp = colorProfile->as<ob::VideoStreamProfile>();
        for (uint32_t j = 0; j < depthSpCount; j++) {
            auto depthProfile = depthStreamProfiles->getProfile(j);
            auto depthVsp = depthProfile->as<ob::VideoStreamProfile>();

            // make sure the color and depth stream have the same fps, due to some
            // models may not support different fps
            if (colorVsp->getFps() != depthVsp->getFps()) {
                continue;
            }

            // Check if the given stream profiles support hardware depth-to-color
            // alignment
            if (checkIfSupportHWD2CAlign(pipe, colorProfile, depthProfile)) {
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
            }
        }
    }
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

void startDevice(std::string serialNumber, std::string resourceName) {
    VIAM_SDK_LOG(info) << service_name << ": starting device " << serialNumber;
    std::lock_guard<std::mutex> lock(devices_by_serial_mu());

    auto search = devices_by_serial().find(serialNumber);
    if (search == devices_by_serial().end()) {
        std::ostringstream buffer;
        buffer << service_name << ": unable to start undetected device" << serialNumber;
        throw std::invalid_argument(buffer.str());
    }

    std::unique_ptr<ViamOBDevice>& my_dev = search->second;
    if (my_dev->started) {
        std::ostringstream buffer;
        buffer << service_name << ": unable to start already started device" << serialNumber;
        throw std::invalid_argument(buffer.str());
    }

    auto frameCallback = [serialNumber](std::shared_ptr<ob::FrameSet> frameSet) {
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

    my_dev->pipe->start(my_dev->config, std::move(frameCallback));
    my_dev->started = true;

    displayPresets(my_dev->device);

    VIAM_SDK_LOG(info) << "[startDevice] starting device with streams: ";
    auto streamProfiles = my_dev->pipe->getConfig()->getEnabledStreamProfileList();
    for (uint32_t i = 0; i < streamProfiles->getCount(); i++) {
        auto sp = streamProfiles->getProfile(i);
        auto vsp = sp->as<ob::VideoStreamProfile>();
        VIAM_SDK_LOG(info) << "[startDevice]   stream " << i << " w:" << vsp->getWidth() << ", h:" << vsp->getHeight()
                           << ", format:" << ob::TypeHelper::convertOBFormatTypeToString(vsp->getFormat()) << ", fps:" << vsp->getFps();
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
        VIAM_SDK_LOG(error) << service_name << ": unable to stop device that is not currently running " << serialNumber;
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

raw_camera_image encodeDepthRAW(const unsigned char* data, const uint64_t width, const uint64_t height, const bool littleEndian) {
    viam::sdk::Camera::depth_map m = xt::xarray<uint16_t>::from_shape({height, width});
    std::copy(reinterpret_cast<const uint16_t*>(data), reinterpret_cast<const uint16_t*>(data) + height * width, m.begin());

    for (size_t i = 0; i < m.size(); i++) {
        m[i] = m[i] * mmToMeterMultiple;
    }

    std::vector<unsigned char> encodedData = viam::sdk::Camera::encode_depth_map(m);

    unsigned char* rawBuf = new unsigned char[encodedData.size()];
    std::memcpy(rawBuf, encodedData.data(), encodedData.size());

    return raw_camera_image{raw_camera_image::uniq(rawBuf, raw_camera_image::array_delete_deleter), encodedData.size()};
}

// HELPERS END

// RESOURCE BEGIN
std::vector<std::string> validate(vsdk::ResourceConfig cfg) {
    auto attrs = cfg.attributes();

    if (attrs.count("serial_number")) {
        if (!attrs["serial_number"].get<std::string>()) {
            throw std::invalid_argument("serial_number must be a string");
        }
    }
    return {};
}

Orbbec::Orbbec(vsdk::Dependencies deps, vsdk::ResourceConfig cfg)
    : Camera(cfg.name()), config_(configure_(std::move(deps), std::move(cfg))) {
    VIAM_SDK_LOG(info) << "Orbbec constructor start " << config_->serial_number;
    startDevice(config_->serial_number, config_->resource_name);
    {
        std::lock_guard<std::mutex> lock(serial_by_resource_mu());
        serial_by_resource()[config_->resource_name] = config_->serial_number;
    }
    VIAM_SDK_LOG(info) << "Orbbec constructor end " << config_->serial_number;
}

Orbbec::~Orbbec() {
    if (config_ == nullptr) {
        VIAM_SDK_LOG(error) << "Orbbec destructor start: config_ is null, no available serial number";
    } else {
        VIAM_SDK_LOG(info) << "Orbbec destructor start " << config_->serial_number;
    }
    std::string prev_serial_number;
    std::string prev_resource_name;
    {
        const std::lock_guard<std::mutex> lock(config_mu_);
        prev_serial_number = config_->serial_number;
        prev_resource_name = config_->resource_name;
    }
    stopDevice(prev_serial_number, prev_resource_name);
    if (config_ == nullptr) {
        VIAM_SDK_LOG(error) << "Orbbec destructor end: config_ is null, no available serial number";
    } else {
        VIAM_SDK_LOG(info) << "Orbbec destructor end " << config_->serial_number;
    }
}

void Orbbec::reconfigure(const vsdk::Dependencies& deps, const vsdk::ResourceConfig& cfg) {
    VIAM_SDK_LOG(info) << "Orbbec reconfigure start";
    std::string prev_serial_number;
    std::string prev_resource_name;
    {
        const std::lock_guard<std::mutex> lock(config_mu_);
        prev_serial_number = config_->serial_number;
        prev_resource_name = config_->resource_name;
    }
    stopDevice(prev_serial_number, prev_resource_name);
    std::string new_serial_number;
    std::string new_resource_name;
    {
        const std::lock_guard<std::mutex> lock(config_mu_);
        config_.reset();
        config_ = configure_(deps, cfg);
        new_serial_number = config_->serial_number;
        new_resource_name = config_->resource_name;
    }
    startDevice(new_serial_number, new_resource_name);
    {
        std::lock_guard<std::mutex> lock(serial_by_resource_mu());
        serial_by_resource()[config_->resource_name] = new_serial_number;
    }
    VIAM_SDK_LOG(info) << "Orbbec reconfigure end";
}

vsdk::Camera::raw_image Orbbec::get_image(std::string mime_type, const vsdk::ProtoStruct& extra) {
    try {
        VIAM_SDK_LOG(debug) << "[get_image] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(config_mu_);
            serial_number = config_->serial_number;
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
        if (color == nullptr) {
            throw std::invalid_argument("no color frame");
        }

        if (color->getFormat() != OB_FORMAT_MJPG) {
            throw std::invalid_argument("color frame was not in jpeg format");
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

        unsigned char* colorData = (unsigned char*)color->getData();
        if (colorData == nullptr) {
            throw std::runtime_error("[get_image] color data is null");
        }
        uint32_t colorDataSize = color->dataSize();

        vsdk::Camera::raw_image response;
        response.source_name = kColorSourceName;
        response.mime_type = kColorMimeTypeJPEG;
        response.bytes.assign(colorData, colorData + colorDataSize);
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
            const std::lock_guard<std::mutex> lock(config_mu_);
            if (config_ == nullptr) {
                throw std::runtime_error("native config is null");
            }
            if (config_->serial_number.empty()) {
                throw std::runtime_error("native config serial number is empty");
            }
            serial_number = config_->serial_number;
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
                buffer << service_name << ": device with serial number " << serial_number << " is not longer started";
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

vsdk::Camera::image_collection Orbbec::get_images() {
    try {
        VIAM_SDK_LOG(debug) << "[get_images] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(config_mu_);
            serial_number = config_->serial_number;
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

        if (color->getFormat() != OB_FORMAT_MJPG) {
            throw std::invalid_argument("color frame was not in jpeg format");
        }

        std::shared_ptr<ob::Frame> depth = fs->getFrame(OB_FRAME_DEPTH);
        if (depth == nullptr) {
            throw std::invalid_argument("no depth frame");
        }

        diff = timeSinceFrameUs(nowUs, depth->getSystemTimeStampUs());
        if (diff > maxFrameAgeUs) {
            std::ostringstream buffer;
            buffer << "no recent depth frame: check USB connection, diff: " << diff << "us";
        }

        unsigned char* colorData = (unsigned char*)color->getData();
        if (colorData == nullptr) {
            throw std::runtime_error("[get_image] color data is null");
        }
        uint32_t colorDataSize = color->dataSize();

        vsdk::Camera::raw_image color_image;
        color_image.source_name = kColorSourceName;
        color_image.mime_type = kColorMimeTypeJPEG;
        color_image.bytes.assign(colorData, colorData + colorDataSize);

        unsigned char* depthData = (unsigned char*)depth->getData();
        if (depthData == nullptr) {
            throw std::runtime_error("[get_images] depth data is null");
        }
        auto depthVid = depth->as<ob::VideoFrame>();
        raw_camera_image rci = encodeDepthRAW(depthData, depthVid->getWidth(), depthVid->getHeight(), false);

        vsdk::Camera::raw_image depth_image;
        depth_image.source_name = kDepthSourceName;
        depth_image.mime_type = kDepthMimeTypeViamDep;
        depth_image.bytes.assign(rci.bytes.get(), rci.bytes.get() + rci.size);

        vsdk::Camera::image_collection response;
        response.images.emplace_back(std::move(color_image));
        response.images.emplace_back(std::move(depth_image));

        uint64_t colorTS = color->getSystemTimeStampUs();
        uint64_t depthTS = depth->getSystemTimeStampUs();
        if (colorTS != depthTS) {
            VIAM_SDK_LOG(info) << "color and depth timestamps differ, defaulting to "
                                  "older of the two"
                               << "color timestamp was " << colorTS << " depth timestamp was " << depthTS;
        }
        // use the older of the two timestamps
        uint64_t timestamp = (colorTS > depthTS) ? depthTS : colorTS;

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
    try {
        for (auto const& [key, value] : command) {
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

            auto const serialNumber = config_->serial_number;
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

                if (key == "set_depth_soft_filter") {
                    if (not value.is_a<bool>()) {
                        VIAM_SDK_LOG(error) << "[do_command] set_depth_soft_filter: expected bool, got " << value.kind();
                        return vsdk::ProtoStruct{{"error", "expected bool"}};
                    }
                    setDepthSoftFilter(my_dev->device, value.get_unchecked<bool>());
                    return {{"depth_soft_filter", my_dev->device->getBoolProperty(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL)}};
                }

                if (key == "get_depth_gain") {
                    if (my_dev->device->isPropertySupported(OB_PROP_DEPTH_GAIN_INT, OB_PERMISSION_READ)) {
                        OBIntPropertyRange valueRange = my_dev->device->getIntPropertyRange(OB_PROP_DEPTH_GAIN_INT);
                        VIAM_SDK_LOG(info) << "Depth current gain max:" << valueRange.max << ", min:" << valueRange.min << std::endl;
                        int value = my_dev->device->getIntProperty(OB_PROP_DEPTH_GAIN_INT);
                        std::cout << "Depth current gain:" << value << std::endl;
                        return {{"current_depth_gain", value}, {"min_depth_gain", valueRange.min}, {"max_depth_gain", valueRange.max}};
                    }
                    return vsdk::ProtoStruct{{"error", "Depth gain property not supported"}};
                }

                if (key == "set_depth_gain") {
                    try {
                        if (my_dev->device->isPropertySupported(OB_PROP_DEPTH_GAIN_INT, OB_PERMISSION_WRITE)) {
                            if (!value.is_a<double>()) {
                                VIAM_SDK_LOG(error) << "Depth gain value is not a double";
                                return vsdk::ProtoStruct{{"error", "Depth gain value is not a double"}};
                            }
                            double new_value = value.get_unchecked<double>();
                            OBIntPropertyRange valueRange = my_dev->device->getIntPropertyRange(OB_PROP_DEPTH_GAIN_INT);
                            VIAM_SDK_LOG(info) << "Depth gain range: min=" << valueRange.min << ", max=" << valueRange.max;
                            if (new_value < valueRange.min || new_value > valueRange.max) {
                                VIAM_SDK_LOG(error) << "Depth gain value out of range: " << new_value;
                                return vsdk::ProtoStruct{{"error", "Depth gain value out of range"}};
                            }
                            VIAM_SDK_LOG(info) << "Setting depth gain to: " << new_value;
                            my_dev->device->setIntProperty(OB_PROP_DEPTH_GAIN_INT, (int)new_value);
                            auto updated_value = my_dev->device->getIntProperty(OB_PROP_DEPTH_GAIN_INT);
                            VIAM_SDK_LOG(info) << "Depth set gain: " << updated_value;
                            return {{"current_depth_gain", updated_value},
                                    {"min_depth_gain", valueRange.min},
                                    {"max_depth_gain", valueRange.max}};
                        }
                        return vsdk::ProtoStruct{{"error", "Depth gain property not supported"}};
                    } catch (const std::exception& e) {
                        VIAM_SDK_LOG(error) << "Exception in set_depth_gain: " << e.what();
                        return vsdk::ProtoStruct{{"error", std::string("Exception: ") + e.what()}};
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
            const std::lock_guard<std::mutex> lock(config_mu_);
            serial_number = config_->serial_number;
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

        unsigned char* colorData = (unsigned char*)color->getData();
        if (colorData == nullptr) {
            throw std::runtime_error("[get_image] color data is null");
        }
        uint32_t colorDataSize = color->dataSize();

        std::shared_ptr<ob::DepthFrame> depthFrame = depth->as<ob::DepthFrame>();
        float scale = depthFrame->getValueScale();

        unsigned char* depthData = (unsigned char*)depth->getData();
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

        std::vector<unsigned char> data =
            RGBPointsToPCD(my_dev->pointCloudFilter->process(my_dev->align->process(fs)), scale * mmToMeterMultiple);
        // std::vector<unsigned char> data = RGBPointsToPCD(my_dev->pointCloudFilter->process(fs), scale * mmToMeterMultiple);

        // auto timestamp = getNowUs();
        // std::stringstream outfile_name;
        // outfile_name << "pointcloud_" << timestamp << ".pcd";
        // std::ofstream outfile(outfile_name.str(), std::ios::out | std::ios::binary);
        // outfile.write((const char*)&data[0], data.size());
        // std::filesystem::path file_path(outfile_name.str());
        // std::filesystem::path absolute_path = std::filesystem::absolute(file_path);
        // std::string absolute_path_str = absolute_path.string();
        // outfile.close();
        // VIAM_SDK_LOG(info) << "[get_point_cloud] wrote PCD to location: " << absolute_path_str;

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

std::unique_ptr<orbbec::ObResourceConfig> Orbbec::configure_(vsdk::Dependencies dependencies, vsdk::ResourceConfig configuration) {
    auto attrs = configuration.attributes();

    std::string serial_number_from_config;
    if (!attrs.count("serial_number")) {
        throw std::invalid_argument("serial_number is a required argument");
    }

    const std::string* serial_val = attrs["serial_number"].get<std::string>();
    if (serial_val == nullptr) {
        throw std::invalid_argument("serial_number must be a string");
    }

    serial_number_from_config = *serial_val;

    auto native_config = std::make_unique<orbbec::ObResourceConfig>(serial_number_from_config, configuration.name());

    return native_config;
}
// RESOURCE END

// ORBBEC SDK DEVICE REGISTRY START
void registerDevice(std::string serialNumber, std::shared_ptr<ob::Device> dev) {
    VIAM_SDK_LOG(info) << "starting " << serialNumber;
    std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>(dev);
    pipe->enableFrameSync();
    std::shared_ptr<ob::Config> config = createHwD2CAlignConfig(pipe);
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
                        startDevice(serial_number, resource_name);
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
