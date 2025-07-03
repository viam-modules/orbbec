#include <iostream>
#include <string>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/services/discovery.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/rpc/server.hpp>


namespace {
namespace vsdk = ::viam::sdk;
constexpr char service_name[] = "orbbec_discovery";

class Discovery : public vsdk::DiscoveryService, public vsdk::Stoppable, public vsdk::Reconfigurable {
   public:
    explicit Discovery(vsdk::Dependencies dependencies, vsdk::ResourceConfig configuration)
        : DiscoveryService(configuration.name()), state_(configure_(std::move(dependencies), std::move(configuration))) {}

    ~Discovery() final {}

    void reconfigure(const vsdk::Dependencies& despendencies, const vsdk::ResourceConfig& configuration) final try {
        const std::unique_lock<std::shared_mutex> state_wlock(state_lock_);
        stop_locked_(state_wlock);
        state_ = configure_(std::move(dependencies), std::move(configuration));
    } catch (...) {
        throw;
    }
}

std::vector<ResourceConfig>
discover_resources(const ProtoStruct& extra) {
}

int serve(int argc, char** argv) try {
    // Every Viam C++ SDK program must have one and only one Instance object
    // which is created before any other C++ SDK objects and stays alive until
    // all Viam C++ SDK objects are destroyed.
    vsdk::Instance inst;

    // Create a new model registration for the service.
    std::shared_ptr<vsdk::ModelRegistration> mr = std::make_shared<vsdk::ModelRegistration>(
        vsdk::API::get<vsdk::DiscoveryService>(),

        // Declare a model triple for this service.
        vsdk::Model{"viam", "orbbec", "discovery"},

        // Define the factory for instances of the resource.
        [](vsdk::Dependencies deps, vsdk::ResourceConfig cfg) { return std::make_unique<Orbbec>(deps, cfg); },
        validate);

    std::vector<std::shared_ptr<vsdk::ModelRegistration>> mrs = {mr};
    auto module_service = std::make_shared<vsdk::ModuleService>(argc, argv, mrs);

    // Start the module service.
    module_service->serve();
} catch (...) {
    std::cerr << "ERROR: An unknown exception was thrown from `serve`" << std::endl;
    return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string usage = "usage: orbbec /path/to/unix/socket";

    if (argc < 2) {
        std::cout << service_name << "ERROR: insufficient arguments\n";
        std::cout << usage << std::endl;
        return EXIT_FAILURE;
    }

    return serve(argc, argv);
}
