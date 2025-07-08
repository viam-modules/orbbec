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
        VIAM_SDK_LOG(info) << "here ";
        vsdk::ProtoStruct attributes;
        attributes.emplace("serial_number", info->serialNumber());
        auto model = vsdk::Model{"viam", "orbbec", "astra2"};
        vsdk::LinkConfig link;

        VIAM_SDK_LOG(info) << "here before config";
        vsdk::ResourceConfig config("component", "1", "rdk", attributes, "viam:orbbec:astra2", model, link);
        configs.push_back(config);
    }
    VIAM_SDK_LOG(info) << "here returning config";
    return configs;
}

vsdk::ProtoStruct OrbbecDiscovery::do_command(const vsdk::ProtoStruct& command) {
    VIAM_SDK_LOG(error) << "do_command not implemented";
    return vsdk::ProtoStruct{};
}

// int serve(int argc, char** argv) try {
//     // Every Viam C++ SDK program must have one and only one Instance object
//     // which is created before any other C++ SDK objects and stays alive until
//     // all Viam C++ SDK objects are destroyed.
//     vsdk::Instance inst;

//     // Create a new model registration for the service.
//     std::shared_ptr<vsdk::ModelRegistration> mr = std::make_shared<vsdk::ModelRegistration>(
//         vsdk::API::get<vsdk::Discovery>(),

//         // Declare a model triple for this service.
//         vsdk::Model{"viam", "orbbec", "discovery"},

//         // Define the factory for instances of the resource.
//         [](vsdk::Dependencies deps, vsdk::ResourceConfig cfg) { return std::make_unique<OrbbecDiscovery>(deps, cfg); },
//         validate);

//     std::vector<std::shared_ptr<vsdk::ModelRegistration>> mrs = {mr};
//     auto module_service = std::make_shared<vsdk::ModuleService>(argc, argv, mrs);

//     // Start the module service.
//     module_service->serve();
// } catch (...) {
//     std::cerr << "ERROR: An unknown exception was thrown from `serve`" << std::endl;
//     return EXIT_FAILURE;
// }

// }  // namespace

// int main(int argc, char* argv[]) {
//     const std::string usage = "usage: orbbec /path/to/unix/socket";

//     if (argc < 2) {
//         std::cout << service_name << "ERROR: insufficient arguments\n";
//         std::cout << usage << std::endl;
//         return EXIT_FAILURE;
//     }

//     return serve(argc, argv);
// }

}  // namespace discovery
