#include "discovery.hpp"
#include "orbbec.hpp"

namespace discovery {

namespace vsdk = ::viam::sdk;
vsdk::Model OrbbecDiscovery::model = vsdk::Model("viam", "orbbec", "discovery");

OrbbecDiscovery::OrbbecDiscovery(vsdk::Dependencies dependencies, vsdk::ResourceConfig configuration, std::shared_ptr<ob::Context> ctx)
    : Discovery(configuration.name()), ob_ctx_(std::move(ctx)) {}

std::vector<vsdk::ResourceConfig> OrbbecDiscovery::discover_resources(const vsdk::ProtoStruct& extra) {
    std::vector<vsdk::ResourceConfig> configs;

    std::shared_ptr<ob::DeviceList> devList = ob_ctx_->queryDeviceList();
    int devCount = devList->getCount();

    if (devCount == 0) {
        VIAM_SDK_LOG(warn) << "No Orbbec devices found during discovery";
        return {};
    }

    VIAM_SDK_LOG(info) << "Discovered " << devCount << " devices";

    for (size_t i = 0; i < devCount; i++) {
        std::shared_ptr<ob::Device> dev = devList->getDevice(i);
        std::shared_ptr<ob::DeviceInfo> info = dev->getDeviceInfo();

        std::ostringstream name;
        name << "orbbec-" << i + 1;

        vsdk::ProtoStruct attributes;
        attributes.emplace("serial_number", info->serialNumber());

        vsdk::ResourceConfig config(
            "camera", std::move(name.str()), "viam", attributes, "rdk:component:camera", orbbec::Orbbec::model, vsdk::log_level::info);
        configs.push_back(config);
    }
    return configs;
}

vsdk::ProtoStruct OrbbecDiscovery::do_command(const vsdk::ProtoStruct& command) {
    VIAM_SDK_LOG(error) << "do_command not implemented";
    return vsdk::ProtoStruct{};
}

}  // namespace discovery
