// This is a command line utility to interact with the
// orbbec SDK.
// It is only intended as a viam developer debugging tool

#include <math.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <libobsensor/ObSensor.hpp>
#include <memory>
#include <mutex>
#include <ostream>
#include <sstream>
#include <vector>

static const double min_distance = 1e-6;
struct PointXYZRGB {
    float x, y, z;
    unsigned int rgb;
};

bool validPoint(OBColorPoint p) {
    return fabs(p.x) >= min_distance || fabs(p.y) >= min_distance || fabs(p.z) >= min_distance;
}

std::vector<unsigned char> RGBPointsToPCD(std::shared_ptr<ob::Frame> frame) {
    int numPoints = frame->dataSize() / sizeof(OBColorPoint);

    OBColorPoint* points = (OBColorPoint*)frame->data();
    std::vector<PointXYZRGB> pcdPoints;

    for (int i = 0; i < numPoints; i++) {
        OBColorPoint& p = points[i];
        if (validPoint(p)) {
            unsigned int r = (unsigned int)p.r;
            unsigned int g = (unsigned int)p.g;
            unsigned int b = (unsigned int)p.b;
            unsigned int rgb = (r << 16) | (g << 8) | b;
            PointXYZRGB pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = p.z;
            pt.rgb = rgb;
            pcdPoints.push_back(pt);
        }
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

    std::cout << "pointcloud size: " << pcdPoints.size() << "\n";
    return pcdBytes;
}

struct my_device {
    ~my_device() {
        std::cout << "deleting device\n";
    }
    std::string serial_number;
    std::shared_ptr<ob::Device> device;
    std::shared_ptr<ob::Pipeline> pipe;
    std::shared_ptr<ob::PointCloudFilter> pointCloudFilter;
    std::shared_ptr<ob::Align> align;
    OBCameraParam param;
};

std::mutex devices_by_serial_mu;
std::unordered_map<std::string, std::unique_ptr<my_device>> devices_by_serial;

std::mutex frame_set_by_serial_mu;
std::unordered_map<std::string, std::shared_ptr<ob::FrameSet>> frame_set_by_serial;

void printDeviceList(const std::shared_ptr<ob::DeviceList> devList) {
    int devCount = devList->getCount();
    for (size_t i = 0; i < devCount; i++) {
        std::cout << "DeviceListElement:" << i << std::endl;
        std::cout << "  Name:              " << devList->name(i) << std::endl;
        std::cout << "  Serial Number:     " << devList->serialNumber(i) << std::endl;
        std::cout << "  UID:               " << devList->uid(i) << std::endl;

        std::cout << "  VID:               " << devList->vid(i) << std::endl;

        std::cout << "  PID:               " << devList->pid(i) << std::endl;

        std::cout << "  Connection Type:   " << devList->connectionType(i) << std::endl;
    }
}
void printDeviceInfo(const std::shared_ptr<ob::DeviceInfo> info) {
    std::cout << "DeviceInfo:\n"
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
            std::cout << "using width: " << vsp->getWidth() << " height: " << vsp->getHeight() << " format: " << vsp->getFormat()
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
                hwD2CAlignConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
                return hwD2CAlignConfig;
            }
        }
    }
    return nullptr;
}

void startStream(std::string serialNumber, std::shared_ptr<ob::Device> dev) {
    std::cout << "starting " << serialNumber << std::endl;
    std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>(dev);
    pipe->enableFrameSync();
    auto config = createHwD2CAlignConfig(pipe);
    if (config == nullptr) {
        std::cerr << "Current device does not support hardware depth-to-color "
                     "alignment."
                  << std::endl;
        return;
    }

    // TODO: sort to find best stream profiles with same resolution and fps
    std::shared_ptr<ob::PointCloudFilter> pointCloudFilter = std::make_shared<ob::PointCloudFilter>();
    // NOTE: Swap this to depth if you want to align to depth
    std::shared_ptr<ob::Align> align = std::make_shared<ob::Align>(OB_STREAM_COLOR);

    pointCloudFilter->setCreatePointFormat(OB_FORMAT_RGB_POINT);

    {
        std::lock_guard<std::mutex> lock(devices_by_serial_mu);
        std::unique_ptr<my_device> my_dev = std::make_unique<my_device>();

        my_dev->pipe = pipe;
        my_dev->device = dev;
        my_dev->serial_number = serialNumber;
        my_dev->pointCloudFilter = pointCloudFilter;
        my_dev->align = align;

        // start pipeline and pass the callback function to receive the frames
        pipe->start(config, [serialNumber, align, pointCloudFilter](std::shared_ptr<ob::FrameSet> frameSet) {
            std::cout << "callback called\n";
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

            std::vector<unsigned char> data = RGBPointsToPCD(pointCloudFilter->process(align->process(frameSet)));
            std::ofstream outfile("my.pcd", std::ios::out | std::ios::binary);
            outfile.write((const char*)&data[0], data.size());
            outfile.close();

            std::lock_guard<std::mutex> lock(frame_set_by_serial_mu);
            frame_set_by_serial[serialNumber] = frameSet;
        });
        devices_by_serial[serialNumber] = std::move(my_dev);
    }
}

void stopStreams() {
    std::vector<std::shared_ptr<ob::Pipeline>> pipes;
    std::lock_guard<std::mutex> lock(devices_by_serial_mu);
    for (auto& [key, my_device] : devices_by_serial) {
        std::cout << "stop stream " << key << "\n";
        my_device->pipe->stop();
    }
    devices_by_serial.clear();
}

void listDevices(const ob::Context& ctx) {
    try {
        auto devList = ctx.queryDeviceList();
        int devCount = devList->getCount();
        std::cout << "devCount: " << devCount << "\n";

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

int main() {
    std::cout << "starting orbbec program" << std::endl;

    ob::Context ctx;
    ctx.setLoggerSeverity(OB_LOG_SEVERITY_DEBUG);

    ctx.setDeviceChangedCallback([](const std::shared_ptr<ob::DeviceList> removedList, const std::shared_ptr<ob::DeviceList> deviceList) {
        try {
            int devCount = removedList->getCount();
            printDeviceList(removedList);
            for (size_t i = 0; i < devCount; i++) {
                if (i == 0) {
                    std::cout << " Devices Removed:\n";
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
                    std::cout << " Devices added:\n";
                }
                std::shared_ptr<ob::Device> dev = deviceList->getDevice(i);
                std::shared_ptr<ob::DeviceInfo> info = dev->getDeviceInfo();
                printDeviceInfo(info);
                startStream(info->getSerialNumber(), dev);
            }
        } catch (ob::Error& e) {
            std::cerr << "setDeviceChangedCallback\n"
                      << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nname:" << e.getName() << "\nmessage:" << e.what()
                      << "\ntype:" << e.getExceptionType() << std::endl;
        }
    });

    std::shared_ptr<ob::DeviceList> devList = ctx.queryDeviceList();
    int devCount = devList->getCount();
    for (size_t i = 0; i < devCount; i++) {
        if (i == 0) {
            std::cout << "devCount: " << devCount << "\n";
        }
        std::shared_ptr<ob::Device> dev = devList->getDevice(i);
        std::shared_ptr<ob::DeviceInfo> info = dev->getDeviceInfo();
        printDeviceInfo(info);
        startStream(info->getSerialNumber(), dev);
    }

    std::cout << "waiting for key press\n";
    std::cin.get();
    std::cout << "stopping orbbec program" << std::endl;

    stopStreams();
    return 0;
}
