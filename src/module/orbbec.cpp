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

#include <libobsensor/ObSensor.hpp>

namespace orbbec {

namespace vsdk = ::viam::sdk;

vsdk::Model Orbbec::model("viam", "orbbec", "astra2");

const std::string kColorSourceName = "color";
const std::string kColorMimeTypeJPEG = "image/jpeg";
const std::string kDepthSourceName = "depth";
const std::string kDepthMimeTypeViamDep = "image/vnd.viam.dep";
const std::string kPcdMimeType = "pointcloud/pcd";
const std::string firmwareUrl = "https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Astra2_Release_2.8.20.zip";

// CONSTANTS BEGIN
constexpr char service_name[] = "viam_orbbec";
const float mmToMeterMultiple = 0.001;
const uint64_t maxFrameAgeUs = 1e6;  // time until a frame is considered stale, in microseconds (equal to 1 sec)
std::string requiredFirmwareVer = "2.8.20";

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

class rebootNeededError : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

int count = 0;
// GLOBALS END

// HELPERS BEGIN

void checkFirmwareVersion(const std::string& serial_number) {
    const std::lock_guard<std::mutex> lock(devices_by_serial_mu());
    auto search = devices_by_serial().find(serial_number);
    if (search == devices_by_serial().end()) {
        std::ostringstream buffer;
        buffer << "device with serial number " << serial_number << " not found";
        throw std::invalid_argument(buffer.str());
    }

    auto& device = search->second;
    auto info = device->device->getDeviceInfo();
    std::string version = info->firmwareVersion();

    if (version.find(requiredFirmwareVer) == std::string::npos) {
        std::ostringstream buffer;
        buffer << "Unsupported firmware version. Required: " << requiredFirmwareVer << ", Current: " << version
               << ". Call update_firmware do command to upgrade.";
        throw std::runtime_error(buffer.str());
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

std::vector<unsigned char> RGBPointsToPCD(std::shared_ptr<ob::Frame> frame, float scale) {
    int numPoints = frame->dataSize() / sizeof(OBColorPoint);

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
            VIAM_SDK_LOG(info) << "using width: " << vsp->getWidth() << " height: " << vsp->getHeight() << " format: " << vsp->getFormat()
                               << " fps: " << vsp->getFps() << "\n";
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
                hwD2CAlignConfig->enableStream(colorProfile);       // enable color stream
                hwD2CAlignConfig->enableStream(depthProfile);       // enable depth stream
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

size_t writeCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    auto* buffer = static_cast<std::vector<char>*>(userp);
    size_t totalSize = size * nmemb;
    buffer->insert(buffer->end(), static_cast<char*>(contents), static_cast<char*>(contents) + totalSize);
    return totalSize;
}

void updateFirmware(std::unique_ptr<ViamOBDevice>& my_dev) {
// On linux, orbbec reccomendsto set libuvc backend for firmware update
#if defined(__linux__)
    ob_ctx_->setUvcBackendType(OB_UVC_BACKEND_TYPE_LIBUVC);
#endif

    CURL* curl = curl_easy_init();
    if (!curl) {
        throw std::invalid_argument("curl easy init failed");
    }

    // Download the firmware and write it to a buffer
    std::vector<char> zipBuffer;
    curl_easy_setopt(curl, CURLOPT_URL, firmwareUrl.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &zipBuffer);
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::ostringstream buffer;
        buffer << "curl early perform failed: " << curl_easy_strerror(res);
        throw std::invalid_argument(buffer.str());
    }
    curl_easy_cleanup(curl);

    // create zip file
    zip_error_t ziperror;
    zip_error_init(&ziperror);
    zip_source_t* src = zip_source_buffer_create(zipBuffer.data(), zipBuffer.size(), 0, &ziperror);
    if (!src) {
        std::ostringstream buffer;
        buffer << "failed to create zip buffer: " << zip_error_strerror(&ziperror);
        throw std::runtime_error(buffer.str());
    }

    zip_t* zip = zip_open_from_source(src, 0, &ziperror);
    if (!zip) {
        zip_source_free(src);
        std::ostringstream buffer;
        buffer << "failed to open zip from source: " << zip_error_strerror(&ziperror);
        throw std::runtime_error(buffer.str());
    }

    // Confirm there is only 1 file inside the zip
    if (zip_get_num_entries(zip, 0) != 1) {
        zip_close(zip);
        throw std::runtime_error("unexpected number of files in firmware zip");
    }

    const char* fileName = zip_get_name(zip, 0, 0);
    if (!fileName) {
        zip_close(zip);
        throw std::runtime_error("couldn't get bin file name");
    }

    // open the .bin file inside the zip
    zip_file_t* binFile = zip_fopen(zip, fileName, 0);
    if (!binFile) {
        zip_close(zip);
        throw std::runtime_error("failed to open the firmware bin file");
    }

    // get stats of the file for size info
    zip_stat_t stats;
    zip_stat_init(&stats);
    if (zip_stat(zip, fileName, 0, &stats) != 0) {
        zip_close(zip);
        throw std::invalid_argument("failed to stat file");
    }

    std::vector<uint8_t> binData(stats.size);
    zip_int64_t bytesRead = zip_fread(binFile, binData.data(), stats.size);
    if (bytesRead == -1) {
        zip_close(zip);
        zip_fclose(binFile);
        zip_error_t* err = zip_file_get_error(binFile);
        std::ostringstream buffer;
        buffer << "failed to read bin: " << zip_error_strerror(err);
        throw std::runtime_error(buffer.str());
    }

    if (bytesRead != stats.size) {
        std::ostringstream buffer;
        buffer << "failed to fully read binary file, file size: " << stats.size << "bytes read: " << bytesRead;
        throw std::runtime_error(buffer.str());
    }

    zip_close(zip);
    zip_fclose(binFile);

    auto firmwareUpdateCallback = [](OBFwUpdateState state, const char* message, uint8_t percent) {
        switch (state) {
            case STAT_VERIFY_SUCCESS:
                VIAM_SDK_LOG(debug) << "Image file verification success";
                break;
            case STAT_FILE_TRANSFER:
                VIAM_SDK_LOG(debug) << "File transfer in progress";
                break;
            case STAT_DONE:
                VIAM_SDK_LOG(debug) << "Update completed";
                break;
            case STAT_IN_PROGRESS:
                VIAM_SDK_LOG(debug) << "Upgrade in progress";
                break;
            case STAT_START:
                VIAM_SDK_LOG(debug) << "Starting the upgrade";
                break;
            case STAT_VERIFY_IMAGE:
                VIAM_SDK_LOG(debug) << "Verifying image file";
                break;
            default:
                VIAM_SDK_LOG(error) << "Unknown status or error";
                break;
        }
        VIAM_SDK_LOG(info) << "Firmware update in progress: " << message;
    };
    my_dev->device->updateFirmwareFromData(binData.data(), binData.size(), std::move(firmwareUpdateCallback), false);
#if defined(__linux__)
    ob_ctx_->setUvcBackendType(OB_UVC_BACKEND_TYPE_AUTO);
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
    my_dev->pipe->start(my_dev->config, frameCallback(serialNumber));
    my_dev->started = true;
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
    return {};
}

Orbbec::Orbbec(vsdk::Dependencies deps, vsdk::ResourceConfig cfg, std::shared_ptr<ob::Context> ctx)
    : Camera(cfg.name()), config_(configure_(std::move(deps), std::move(cfg))), ob_ctx_(std::move(ctx)) {
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
    VIAM_SDK_LOG(info) << "Orbbec destructor end " << config_->serial_number;
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
        config_ = configure(deps, cfg);
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

        checkFirmwareVersion(serial_number);

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

vsdk::Camera::image_collection Orbbec::get_images() {
    try {
        VIAM_SDK_LOG(debug) << "[get_images] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(config_mu_);
            serial_number = config_->serial_number;
        }

        checkFirmwareVersion(serial_number);

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
    viam::sdk::ProtoStruct resp = viam::sdk::ProtoStruct{};
    constexpr char firmware_key[] = "update_firmware";
    for (const auto& kv : command) {
        if (kv.first == firmware_key) {
            std::string serial_number;
            {
                const std::lock_guard<std::mutex> lock(config_mu_);
                serial_number = config_->serial_number;
            }
            {
                // note: under lock for the entire firmware update
                const std::lock_guard<std::mutex> lock(devices_by_serial_mu());
                auto search = devices_by_serial().find(serial_number);
                if (search == devices_by_serial().end()) {
                    throw std::invalid_argument("device is not connected");
                }

                std::unique_ptr<ViamOBDevice>& dev = search->second;
                std::shared_ptr<ob::DeviceInfo> info = dev->device->getDeviceInfo();
                std::string version = info->firmwareVersion();
                if (version.find(requiredFirmwareVer) != std::string::npos) {
                    std::ostringstream buffer;
                    buffer << "device firmware already on required version " << requiredFirmwareVer;
                    resp.emplace(firmware_key, buffer.str());
                    break;
                }
                if (dev->started) {
                    dev->pipe->stop();
                    dev->started = false;
                }

                VIAM_SDK_LOG(info) << "Updating device firmware...";
                try {
                    updateFirmware(dev);
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
            const std::lock_guard<std::mutex> lock(config_mu_);
            serial_number = config_->serial_number;
        }

        checkFirmwareVersion(serial_number);

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

        std::vector<unsigned char> data =
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

void printDeviceList(const std::string& prompt, std::shared_ptr<ob::DeviceList> deviceList) {
    auto count = deviceList->getCount();
    if (count == 0) {
        return;
    }
    std::cout << count << " device(s) " << prompt << ": " << std::endl;
    for (uint32_t i = 0; i < count; i++) {
        auto uid = deviceList->getUid(i);
        auto vid = deviceList->getVid(i);
        auto pid = deviceList->getPid(i);
        auto serialNumber = deviceList->getSerialNumber(i);
        auto connection = deviceList->getConnectionType(i);
        std::cout << " serial number: " << serialNumber << ", connection: " << connection << std::endl;
    }
    std::cout << std::endl;
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
            VIAM_SDK_LOG(info) << "deleting the device from devices_by_serial";
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
};

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
