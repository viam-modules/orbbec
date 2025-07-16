#include "discovery.hpp"
#include "orbbec.hpp"

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
vsdk::Model OrbbecDiscovery::model = vsdk::Model("viam", "orbbec", "discovery");

OrbbecDiscovery::OrbbecDiscovery(vsdk::Dependencies dependencies, vsdk::ResourceConfig configuration) : Discovery(configuration.name()) {}

void OrbbecDiscovery::reconfigure(const vsdk::Dependencies& despendencies, const vsdk::ResourceConfig& configuration) {}

std::vector<vsdk::ResourceConfig> OrbbecDiscovery::discover_resources(const vsdk::ProtoStruct& extra) {
    std::vector<vsdk::ResourceConfig> configs;

    ob::Context ctx;
    std::shared_ptr<ob::DeviceList> devList = ctx.queryDeviceList();
    int devCount = devList->getCount();

    if (devCount == 0) {
        VIAM_SDK_LOG(warn) << "No Orbbec devices found during discovery";
        return {};
    }

    VIAM_SDK_LOG(info) << "Discovered " << devCount << " devices";

    for (size_t i = 0; i < devCount; i++) {
        std::shared_ptr<ob::Device> dev = devList->getDevice(i);
        std::shared_ptr<ob::DeviceInfo> info = dev->getDeviceInfo();
        orbbec::printDeviceInfo(info);

        char name[10];
        sprintf(name, "orbbec-%d", i + 1);

        vsdk::ProtoStruct attributes;
        attributes.emplace("serial_number", info->serialNumber());

        vsdk::ResourceConfig config(
            "camera", name, "viam", attributes, "rdk:component:camera", orbbec::Orbbec::model, vsdk::LinkConfig{}, vsdk::log_level::info);
        configs.push_back(config);
    }
    return configs;
}

vsdk::ProtoStruct OrbbecDiscovery::do_command(const vsdk::ProtoStruct& command) {
    VIAM_SDK_LOG(error) << "do_command not implemented";
    return vsdk::ProtoStruct{};
}

}  // namespace discovery
