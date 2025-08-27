#include "discovery.hpp"
#include "orbbec.hpp"

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/module/service.hpp>

#include <iostream>
#include <memory>

#include <libobsensor/ObSensor.hpp>

namespace vsdk = ::viam::sdk;

std::vector<std::shared_ptr<vsdk::ModelRegistration>> create_all_model_registrations(std::shared_ptr<ob::Context> ctx) {
    std::vector<std::shared_ptr<vsdk::ModelRegistration>> registrations;

    registrations.push_back(std::make_shared<vsdk::ModelRegistration>(
        vsdk::API::get<vsdk::Camera>(), orbbec::Orbbec::model, [ctx](vsdk::Dependencies deps, vsdk::ResourceConfig config) {
            return std::make_unique<orbbec::Orbbec>(std::move(deps), std::move(config), ctx);
        }));

    registrations.push_back(std::make_shared<vsdk::ModelRegistration>(
        vsdk::API::get<vsdk::Discovery>(),
        discovery::OrbbecDiscovery::model,
        [ctx](vsdk::Dependencies deps, vsdk::ResourceConfig config) {
            return std::make_unique<discovery::OrbbecDiscovery>(std::move(deps), std::move(config), ctx);
        },
        orbbec::Orbbec::validate));

    return registrations;
}

int serve(int argc, char** argv) try {
    // Every Viam C++ SDK program must have one and only one Instance object
    // which is created before any other C++ SDK objects and stays alive until
    // all Viam C++ SDK objects are destroyed.
    vsdk::Instance inst;

    auto ctx = std::make_shared<ob::Context>();
    for (size_t i = 0; i < argc; i++) {
        if (std::string(argv[i]) == "--log-level=debug") {
            ctx->setLoggerSeverity(OB_LOG_SEVERITY_DEBUG);
        }
    }

    orbbec::startOrbbecSDK(*ctx);
    auto module_service = std::make_shared<vsdk::ModuleService>(argc, argv, create_all_model_registrations(ctx));
    module_service->serve();

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
