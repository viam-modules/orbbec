#pragma once
#include <memory>

#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/services/discovery.hpp>

#include <libobsensor/ObSensor.hpp>

namespace discovery {

class OrbbecDiscovery : public viam::sdk::Discovery {
   public:
    explicit OrbbecDiscovery(viam::sdk::Dependencies dependencies, viam::sdk::ResourceConfig configuration);
    std::vector<viam::sdk::ResourceConfig> discover_resources(const viam::sdk::ProtoStruct& extra) override;
    viam::sdk::ProtoStruct do_command(const viam::sdk::ProtoStruct& command) override;
    static viam::sdk::Model model;

   private:
    std::shared_ptr<ob::Context> ob_ctx_;
};
}  // namespace discovery
