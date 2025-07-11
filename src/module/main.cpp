#include "discovery.hpp"
#include "orbbec.hpp"

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/services/discovery.hpp>

#include <iostream>

#include <libobsensor/ObSensor.hpp>

namespace vsdk = ::viam::sdk;

template <typename ResourceType, typename ResourceImpl>
std::vector<std::shared_ptr<vsdk::ModelRegistration>> create_model_registrations(const std::vector<vsdk::Model>& models) {
    using namespace std::placeholders;

    constexpr auto factory = [](const vsdk::Dependencies& deps, const vsdk::ResourceConfig& config) {
        return std::make_unique<ResourceImpl>(deps, config);
    };

    const auto api = vsdk::API::get<ResourceType>();
    std::vector<std::shared_ptr<vsdk::ModelRegistration>> registrations;

    for (const auto& model : models) {
        registrations.push_back(std::make_shared<vsdk::ModelRegistration>(api, model, std::bind(factory, _1, _2)));
    }

    return registrations;
}

std::vector<std::shared_ptr<vsdk::ModelRegistration>> create_all_model_registrations() {
    std::vector<std::shared_ptr<vsdk::ModelRegistration>> registrations;

    std::vector<std::shared_ptr<vsdk::ModelRegistration>> camera_regs = create_model_registrations<vsdk::Camera, orbbec::Orbbec>({
        orbbec::Orbbec::model,
    });

    std::vector<std::shared_ptr<vsdk::ModelRegistration>> discovery_reg =
        create_model_registrations<vsdk::Discovery, discovery::OrbbecDiscovery>({
            discovery::OrbbecDiscovery::model,
        });

    registrations.insert(registrations.end(), camera_regs.begin(), camera_regs.end());
    registrations.insert(registrations.end(), discovery_reg.begin(), discovery_reg.end());

    return registrations;
}

int serve(int argc, char** argv) try {
    // Every Viam C++ SDK program must have one and only one Instance object
    // which is created before any other C++ SDK objects and stays alive until
    // all Viam C++ SDK objects are destroyed.
    vsdk::Instance inst;

    ob::Context ctx;
    for (size_t i = 0; i < argc; i++) {
        if (std::string(argv[i]) == "--log-level=debug") {
            ctx.setLoggerSeverity(OB_LOG_SEVERITY_DEBUG);
        }
    }
    orbbec::startOrbbecSDK(ctx);
    std::make_shared<vsdk::ModuleService>(argc, argv, create_all_model_registrations())->serve();

    return EXIT_SUCCESS;
} catch (const std::exception& ex) {
    std::cerr << "ERROR: A std::exception was thrown from `serve`: " << ex.what() << std::endl;
    return EXIT_FAILURE;
} catch (...) {
    std::cerr << "ERROR: An unknown exception was thrown from `serve`" << std::endl;
    return EXIT_FAILURE;
}

int main(int argc, char* argv[]) {
    std::cout << "Orbbec C++ SDK version: " << ob::Version::getMajor() << "." << ob::Version::getMinor() << "." << ob::Version::getPatch()
              << "\n";

    const std::string usage = "usage: orbbec /path/to/unix/socket";

    if (argc < 2) {
        std::cout << "ERROR: insufficient arguments\n";
        std::cout << usage << "\n";
        return EXIT_FAILURE;
    }

    return serve(argc, argv);
};
