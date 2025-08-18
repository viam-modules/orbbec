#pragma once
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include <libobsensor/ObSensor.hpp>

namespace orbbec {

// The native config struct for orbbec resources.
struct ObResourceConfig {
    std::string resource_name;
    std::string serial_number;

    explicit ObResourceConfig(std::string const& serial_number, std::string const& resource_name)
        : serial_number(serial_number), resource_name(resource_name) {}
};

void startOrbbecSDK(ob::Context& ctx);
void printDeviceInfo(const std::shared_ptr<ob::DeviceInfo> info);

class Orbbec final : public viam::sdk::Camera, public viam::sdk::Reconfigurable {
   public:
    Orbbec(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg, ob::Context& ctx);
    ~Orbbec();
    void reconfigure(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg) override;
    viam::sdk::ProtoStruct do_command(const viam::sdk::ProtoStruct& command) override;
    raw_image get_image(std::string mime_type, const viam::sdk::ProtoStruct& extra) override;
    image_collection get_images() override;
    point_cloud get_point_cloud(std::string mime_type, const viam::sdk::ProtoStruct& extra) override;
    properties get_properties() override;
    std::vector<viam::sdk::GeometryConfig> get_geometries(const viam::sdk::ProtoStruct& extra) override;

    static viam::sdk::GeometryConfig geometry;
    static viam::sdk::Model model;

   private:
    std::unique_ptr<ObResourceConfig> config_;
    std::mutex config_mu_;
    ob::Context& ctx_;
    static std::unique_ptr<ObResourceConfig> configure_(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg);
};

}  // namespace orbbec
