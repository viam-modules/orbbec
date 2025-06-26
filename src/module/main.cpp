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

#include <math.h>
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

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/rpc/server.hpp>

#include <libobsensor/ObSensor.hpp>

namespace {

namespace vsdk = ::viam::sdk;

// CONSTANTS BEGIN
constexpr char service_name[] = "viam_orbbec";
const float mmToMeterMultiple = 0.001;
// CONSTANTS END

// STRUCTS BEGIN
struct PointXYZRGB {
    float x, y, z;
    unsigned int rgb;
};

struct ViamOBDevice {
    ~ViamOBDevice() {
        VIAM_SDK_LOG(info) << "deleting ViamOBDevice " << serial_number << "\n";
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
std::mutex devices_by_serial_mu;
std::unordered_map<std::string, std::unique_ptr<ViamOBDevice>> devices_by_serial;

std::mutex frame_set_by_serial_mu;
std::unordered_map<std::string, std::shared_ptr<ob::FrameSet>> frame_set_by_serial;
// GLOBALS END

// HELPERS BEGIN
std::vector<unsigned char> RGBPointsToPCD(std::shared_ptr<ob::Frame> frame, float scale) {
    int numPoints = frame->dataSize() / sizeof(OBColorPoint);

    OBColorPoint* points = (OBColorPoint*)frame->data();
    std::vector<PointXYZRGB> pcdPoints;

    for (int i = 0; i < numPoints; i++) {
        OBColorPoint& p = points[i];
        unsigned int r = (unsigned int)(p.r);
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
            // Found a matching depth stream profile, it is means the given stream
            // profiles support hardware depth-to-color alignment
            return true;
        }
    }
    return false;
}

// create a config for hardware depth-to-color alignment
std::shared_ptr<ob::Config> createHwD2CAlignConfig(std::shared_ptr<ob::Pipeline> pipe) {
    auto coloStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_COLOR);
    auto depthStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_DEPTH);

    // Iterate through all color and depth stream profiles to find a match for
    // hardware depth-to-color alignment
    auto colorSpCount = coloStreamProfiles->getCount();
    auto depthSpCount = depthStreamProfiles->getCount();
    for (uint32_t i = 0; i < colorSpCount; i++) {
        auto colorProfile = coloStreamProfiles->getProfile(i);
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

void startDevice(std::string serialNumber) {
    VIAM_SDK_LOG(info) << service_name << ": starting device " << serialNumber;
    std::lock_guard<std::mutex> lock(devices_by_serial_mu);

    if (auto search = devices_by_serial.find(serialNumber); search == devices_by_serial.end()) {
        std::ostringstream buffer;
        buffer << service_name << ": unable to start undetected device" << serialNumber;
        throw std::invalid_argument(buffer.str());
    }

    std::unique_ptr<ViamOBDevice>& my_dev = devices_by_serial[serialNumber];
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

        std::lock_guard<std::mutex> lock(frame_set_by_serial_mu);
        frame_set_by_serial[serialNumber] = frameSet;
    };

    my_dev->pipe->start(my_dev->config, std::move(frameCallback));
    my_dev->started = true;
    VIAM_SDK_LOG(info) << service_name << ": device started " << serialNumber;
}

void stopDevice(std::string serialNumber) {
    std::lock_guard<std::mutex> lock(devices_by_serial_mu);

    if (auto search = devices_by_serial.find(serialNumber); search == devices_by_serial.end()) {
        VIAM_SDK_LOG(error) << service_name << ": unable to stop undetected device " << serialNumber;
        return;
    }

    std::unique_ptr<ViamOBDevice>& my_dev = devices_by_serial[serialNumber];
    if (!my_dev->started) {
        VIAM_SDK_LOG(error) << service_name << ": unable to stop device that is not currently running " << serialNumber;
        return;
    }

    my_dev->pipe->stop();
    my_dev->started = false;
}

void stopStreams() {
    std::vector<std::shared_ptr<ob::Pipeline>> pipes;
    std::lock_guard<std::mutex> lock(devices_by_serial_mu);
    for (auto& [key, ViamOBDevice] : devices_by_serial) {
        VIAM_SDK_LOG(info) << "stop stream " << key << "\n";
        ViamOBDevice->pipe->stop();
    }
    devices_by_serial.clear();
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
std::vector<std::string> validate(vsdk::ResourceConfig cfg) {
    auto attrs = cfg.attributes();

    if (attrs.count("serial_number")) {
        if (!attrs["serial_number"].get<std::string>()) {
            throw std::invalid_argument("serial_number must be a string");
        }
    }
    return {};
}

class Orbbec : public vsdk::Camera, public vsdk::Reconfigurable {
   public:
    Orbbec(vsdk::Dependencies deps, vsdk::ResourceConfig cfg) : Camera(cfg.name()), state_(configure_(std::move(deps), std::move(cfg))) {
        VIAM_SDK_LOG(info) << "Orbbec constructor start " << state_->serial_number;
        startDevice(state_->serial_number);
        VIAM_SDK_LOG(info) << "Orbbec constructor end " << state_->serial_number;
    }

    ~Orbbec() {
        VIAM_SDK_LOG(info) << "Orbbec destructor start " << state_->serial_number;
        std::string prev_serial_number;
        {
            const std::lock_guard<std::mutex> lock(state_mu_);
            prev_serial_number = state_->serial_number;
        }
        stopDevice(prev_serial_number);
        VIAM_SDK_LOG(info) << "Orbbec destructor end " << state_->serial_number;
    }

    void reconfigure(const vsdk::Dependencies& deps, const vsdk::ResourceConfig& cfg) {
        VIAM_SDK_LOG(info) << "Orbbec reconfigure start";
        std::string prev_serial_number;
        {
            const std::lock_guard<std::mutex> lock(state_mu_);
            prev_serial_number = state_->serial_number;
        }
        stopDevice(prev_serial_number);
        std::string new_serial_number;
        {
            const std::lock_guard<std::mutex> lock(state_mu_);
            state_.reset();
            state_ = configure_(deps, cfg);
            new_serial_number = state_->serial_number;
        }
        startDevice(new_serial_number);
        VIAM_SDK_LOG(info) << "Orbbec reconfigure end";
    }

    vsdk::Camera::raw_image get_image(std::string mime_type, const vsdk::ProtoStruct& extra) {
        VIAM_SDK_LOG(info) << "[get_image] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(state_mu_);
            serial_number = state_->serial_number;
        }
        std::shared_ptr<ob::FrameSet> fs = nullptr;
        {
            std::lock_guard<std::mutex> lock(frame_set_by_serial_mu);
            if (auto search = frame_set_by_serial.find(serial_number); search == frame_set_by_serial.end()) {
                throw std::invalid_argument("no frame yet");
            }
            fs = frame_set_by_serial[serial_number];
        }
        std::shared_ptr<ob::Frame> color = fs->getFrame(OB_FRAME_COLOR);
        if (color == nullptr) {
            throw std::invalid_argument("no color frame");
        }

        if (color->getFormat() != OB_FORMAT_MJPG) {
            throw std::invalid_argument("color frame was not in jpeg format");
        }

        // VIAM_SDK_LOG(info) << "[get_image] color frame system timestamp: "
        //                    << color->getSystemTimeStampUs();
        // TODO: Returen error if frame timestamp is older than 5 seconds as that
        // indicates we no longer have a working camera

        unsigned char* colorData = (unsigned char*)color->getData();
        uint32_t colorDataSize = color->dataSize();

        vsdk::Camera::raw_image response;
        response.source_name = "color";
        response.mime_type = "image/jpeg";
        response.bytes.assign(colorData, colorData + colorDataSize);
        return response;
    }

    vsdk::Camera::properties get_properties() {
        VIAM_SDK_LOG(info) << "[get_properties] start";

        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(state_mu_);
            serial_number = state_->serial_number;
        }

        OBCameraIntrinsic props;
        {
            const std::lock_guard<std::mutex> lock(devices_by_serial_mu);
            if (auto search = devices_by_serial.find(serial_number); search == devices_by_serial.end()) {
                std::ostringstream buffer;
                buffer << service_name << ": device with serial number " << serial_number << " is no longer connected";
                throw std::invalid_argument(buffer.str());
            }

            std::unique_ptr<ViamOBDevice>& my_dev = devices_by_serial[serial_number];
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

        return p;
    }
    struct raw_camera_image {
        using deleter_type = void (*)(unsigned char*);
        using uniq = std::unique_ptr<unsigned char[], deleter_type>;

        static constexpr deleter_type free_deleter = [](unsigned char* ptr) { free(ptr); };

        static constexpr deleter_type array_delete_deleter = [](unsigned char* ptr) { delete[] ptr; };

        uniq bytes;
        size_t size;
    };

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

    vsdk::Camera::image_collection get_images() {
        VIAM_SDK_LOG(info) << "[get_images] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(state_mu_);
            serial_number = state_->serial_number;
        }
        std::shared_ptr<ob::FrameSet> fs = nullptr;
        {
            std::lock_guard<std::mutex> lock(frame_set_by_serial_mu);
            if (auto search = frame_set_by_serial.find(serial_number); search == frame_set_by_serial.end()) {
                throw std::invalid_argument("no frame yet");
            }
            fs = frame_set_by_serial[serial_number];
        }

        std::shared_ptr<ob::Frame> color = fs->getFrame(OB_FRAME_COLOR);
        if (color == nullptr) {
            throw std::invalid_argument("no color frame");
        }

        if (color->getFormat() != OB_FORMAT_MJPG) {
            throw std::invalid_argument("color frame was not in jpeg format");
        }

        unsigned char* colorData = (unsigned char*)color->getData();
        uint32_t colorDataSize = color->dataSize();

        vsdk::Camera::raw_image color_image;
        color_image.source_name = "color";
        color_image.mime_type = "image/jpeg";
        color_image.bytes.assign(colorData, colorData + colorDataSize);

        std::shared_ptr<ob::Frame> depth = fs->getFrame(OB_FRAME_DEPTH);
        if (depth == nullptr) {
            throw std::invalid_argument("no depth frame");
        }

        unsigned char* depthData = (unsigned char*)depth->getData();
        auto depthVid = depth->as<ob::VideoFrame>();
        raw_camera_image rci = encodeDepthRAW(depthData, depthVid->getWidth(), depthVid->getHeight(), false);

        vsdk::Camera::raw_image depth_image;
        depth_image.source_name = "depth";
        depth_image.mime_type = "image/vnd.viam.dep";
        depth_image.bytes.assign(rci.bytes.get(), rci.bytes.get() + rci.size);

        vsdk::Camera::image_collection response;
        response.images.emplace_back(std::move(color_image));
        response.images.emplace_back(std::move(depth_image));

        uint64_t colorTS = color->getSystemTimeStampUs();
        uint64_t depthTS = depth->getSystemTimeStampUs();
        if (colorTS != depthTS) {
            VIAM_SDK_LOG(info) << "color and depth timestamps differ, defaulting to "
                                  "color";
            VIAM_SDK_LOG(info) << "color timestamp was " << colorTS << "depth timestamp was " << depthTS;
        }
        uint64_t timestamp = colorTS == 0 ? depthTS : colorTS;
        std::chrono::microseconds latestTimestamp(timestamp);
        response.metadata.captured_at = vsdk::time_pt{std::chrono::duration_cast<std::chrono::nanoseconds>(latestTimestamp)};
        VIAM_SDK_LOG(info) << "[get_images] stop";
        return response;
    }

    vsdk::ProtoStruct do_command(const vsdk::ProtoStruct& command) {
        VIAM_SDK_LOG(error) << "do_command not implemented";
        return vsdk::ProtoStruct{};
    }

    std::string pointcloudMime = "pointcloud/pcd";
    vsdk::Camera::point_cloud get_point_cloud(std::string mime_type, const vsdk::ProtoStruct& extra) {
        VIAM_SDK_LOG(info) << "[get_point_cloud] start";
        std::string serial_number;
        {
            const std::lock_guard<std::mutex> lock(state_mu_);
            serial_number = state_->serial_number;
        }
        std::shared_ptr<ob::FrameSet> fs = nullptr;
        {
            std::lock_guard<std::mutex> lock(frame_set_by_serial_mu);
            if (auto search = frame_set_by_serial.find(serial_number); search == frame_set_by_serial.end()) {
                throw std::invalid_argument("no frame yet");
            }
            fs = frame_set_by_serial[serial_number];
        }

        std::shared_ptr<ob::Frame> color = fs->getFrame(OB_FRAME_COLOR);
        if (color == nullptr) {
            throw std::invalid_argument("no color frame");
        }

        unsigned char* colorData = (unsigned char*)color->getData();
        uint32_t colorDataSize = color->dataSize();

        std::shared_ptr<ob::Frame> depth = fs->getFrame(OB_FRAME_DEPTH);
        if (depth == nullptr) {
            throw std::invalid_argument("no depth frame");
        }
        std::shared_ptr<ob::DepthFrame> depthFrame = depth->as<ob::DepthFrame>();
        float scale = depthFrame->getValueScale();

        unsigned char* depthData = (unsigned char*)depth->getData();

        // NOTE: UNDER LOCK
        std::lock_guard<std::mutex> lock(devices_by_serial_mu);
        if (auto search = devices_by_serial.find(serial_number); search == devices_by_serial.end()) {
            throw std::invalid_argument("device is not connected");
        }

        std::unique_ptr<ViamOBDevice>& my_dev = devices_by_serial[serial_number];
        if (!my_dev->started) {
            throw std::invalid_argument("device is not started");
        }

        std::vector<unsigned char> data =
            RGBPointsToPCD(my_dev->pointCloudFilter->process(my_dev->align->process(fs)), scale * mmToMeterMultiple);

        VIAM_SDK_LOG(info) << "[get_point_cloud] stop";
        return vsdk::Camera::point_cloud{pointcloudMime, data};
    }

    std::vector<vsdk::GeometryConfig> get_geometries(const vsdk::ProtoStruct& extra) {
        VIAM_SDK_LOG(error) << "get_geometries not implemented";
        return std::vector<vsdk::GeometryConfig>{};
    }

   private:
    struct state_ {
        std::string serial_number;

        explicit state_(std::string serial_number) : serial_number(serial_number) {}
    };
    std::mutex state_mu_;
    std::unique_ptr<struct state_> state_;

    static std::unique_ptr<struct state_> configure_(vsdk::Dependencies dependencies, vsdk::ResourceConfig configuration) {
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

        auto state = std::make_unique<struct state_>(serial_number_from_config);

        return state;
    }
};
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
        std::lock_guard<std::mutex> lock(devices_by_serial_mu);
        std::unique_ptr<ViamOBDevice> my_dev = std::make_unique<ViamOBDevice>();

        my_dev->pipe = pipe;
        my_dev->device = dev;
        my_dev->serial_number = serialNumber;
        my_dev->pointCloudFilter = pointCloudFilter;
        my_dev->align = align;
        my_dev->config = config;

        devices_by_serial[serialNumber] = std::move(my_dev);
    }
}

void deviceChangedCallback(const std::shared_ptr<ob::DeviceList> removedList, const std::shared_ptr<ob::DeviceList> deviceList) {
    try {
        int devCount = removedList->getCount();
        printDeviceList(removedList);
        for (size_t i = 0; i < devCount; i++) {
            if (i == 0) {
                VIAM_SDK_LOG(info) << " Devices Removed:\n";
            }
            std::lock_guard<std::mutex> lock(devices_by_serial_mu);
            std::string serial_number = removedList->serialNumber(i);
            if (auto search = devices_by_serial.find(serial_number); search == devices_by_serial.end()) {
                std::cerr << serial_number
                          << "was in removedList of device change callback but not "
                             "in devices_by_serial\n";
                continue;
            }
            devices_by_serial.erase(serial_number);
        }

        devCount = deviceList->getCount();
        for (size_t i = 0; i < devCount; i++) {
            if (i == 0) {
                VIAM_SDK_LOG(info) << " Devices added:\n";
            }
            std::shared_ptr<ob::Device> dev = deviceList->getDevice(i);
            std::shared_ptr<ob::DeviceInfo> info = dev->getDeviceInfo();
            printDeviceInfo(info);
            registerDevice(info->getSerialNumber(), dev);
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

int serve(int argc, char** argv) try {
    // Every Viam C++ SDK program must have one and only one Instance object
    // which is created before any other C++ SDK objects and stays alive until
    // all Viam C++ SDK objects are destroyed.
    vsdk::Instance inst;

    ob::Context ctx;
    // TODO: Make this enabled when user boots at debug level
    // ctx.setLoggerSeverity(OB_LOG_SEVERITY_DEBUG);
    startOrbbecSDK(ctx);

    // Create a new model registration for the service.
    std::shared_ptr<vsdk::ModelRegistration> mr = std::make_shared<vsdk::ModelRegistration>(
        // Identify that this resource offers the Camera API
        vsdk::API::get<vsdk::Camera>(),

        // Declare a model triple for this service.
        vsdk::Model{"viam", "orbbec", "astra2"},

        // Define the factory for instances of the resource.
        [](vsdk::Dependencies deps, vsdk::ResourceConfig cfg) { return std::make_unique<Orbbec>(deps, cfg); },
        validate);

    std::vector<std::shared_ptr<vsdk::ModelRegistration>> mrs = {mr};
    auto module_service = std::make_shared<vsdk::ModuleService>(argc, argv, mrs);

    // Start the module service.
    module_service->serve();

    return EXIT_SUCCESS;
} catch (const std::exception& ex) {
    std::cerr << "ERROR: A std::exception was thrown from `serve`: " << ex.what() << std::endl;
    return EXIT_FAILURE;
} catch (...) {
    std::cerr << "ERROR: An unknown exception was thrown from `serve`" << std::endl;
    return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string usage = "usage: orbbec /path/to/unix/socket";

    if (argc < 2) {
        std::cout << "ERROR: insufficient arguments\n";
        std::cout << usage << "\n";
        return EXIT_FAILURE;
    }

    return serve(argc, argv);
}
