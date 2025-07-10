#include "discovery.hpp"

#include <iostream>
#include <string>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/rpc/server.hpp>
#include <viam/sdk/services/discovery.hpp>

#include <libobsensor/ObSensor.hpp>

namespace discovery {
namespace vsdk = ::viam::sdk;
constexpr char service_name[] = "orbbec_discovery";

std::vector<std::string> validate(vsdk::ResourceConfig cfg) {
    return {};
}

OrbbecDiscovery::OrbbecDiscovery(vsdk::Dependencies dependencies, vsdk::ResourceConfig configuration) : Discovery(configuration.name()) {
    VIAM_SDK_LOG(info) << "[constructor] start";
}

void OrbbecDiscovery::reconfigure(const vsdk::Dependencies& despendencies, const vsdk::ResourceConfig& configuration) {}

std::vector<vsdk::ResourceConfig> OrbbecDiscovery::discover_resources(const vsdk::ProtoStruct& extra) {
    std::vector<vsdk::ResourceConfig> configs;

    ob::Context ctx;
    std::shared_ptr<ob::DeviceList> devList = ctx.queryDeviceList();
    int devCount = devList->getCount();
    for (size_t i = 0; i < devCount; i++) {
        if (i == 0) {
            VIAM_SDK_LOG(info) << "devCount: " << devCount << "\n";
        }
        std::shared_ptr<ob::Device> dev = devList->getDevice(i);
        std::shared_ptr<ob::DeviceInfo> info = dev->getDeviceInfo();
        // print device info
        vsdk::ProtoStruct attributes;
        attributes.emplace("serial_number", info->serialNumber());
        auto model = vsdk::Model{"viam", "orbbec", "astra2"};

        vsdk::orientation_vector o{0, 0, 0, 1};
        vsdk::translation t{0.0, 0.0, 0.0};
        vsdk::GeometryConfig g;

        vsdk::LinkConfig link(t, o, g, "hi");

        vsdk::ResourceConfig config("camera", "1", "viam", attributes, "rdk:component:camera", model, link);
        configs.push_back(config);
    }
    return configs;
}

vsdk::ProtoStruct OrbbecDiscovery::do_command(const vsdk::ProtoStruct& command) {
    VIAM_SDK_LOG(error) << "do_command not implemented";
    return vsdk::ProtoStruct{};
}

}  // namespace discovery
