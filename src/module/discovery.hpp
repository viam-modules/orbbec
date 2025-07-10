#pragma once
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/services/discovery.hpp>

namespace vsdk = ::viam::sdk;

namespace discovery {

class OrbbecDiscovery : public vsdk::Discovery, public vsdk::Reconfigurable {
   public:
    explicit OrbbecDiscovery(vsdk::Dependencies dependencies, vsdk::ResourceConfig configuration);
    void reconfigure(const vsdk::Dependencies& despendencies, const vsdk::ResourceConfig& configuration) override;
    std::vector<vsdk::ResourceConfig> discover_resources(const vsdk::ProtoStruct& extra) override;
    vsdk::ProtoStruct do_command(const vsdk::ProtoStruct& command) override;
    static vsdk::Model model;
};
}  // namespace discovery