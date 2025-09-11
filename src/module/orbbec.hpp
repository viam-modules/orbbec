#pragma once
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include <libobsensor/ObSensor.hpp>

#include <optional>

namespace orbbec {

struct Resolution {
    int width{};
    int height{};
};

struct DeviceResolution {
    Resolution color_resolution{};
    Resolution depth_resolution{};
};
// The native config struct for orbbec resources.
struct ObResourceConfig {
    std::string resource_name;
    std::string serial_number;
    std::optional<DeviceResolution> device_resolution;

    explicit ObResourceConfig(std::string const& serial_number,
                              std::string const& resource_name,
                              std::optional<DeviceResolution> device_resolution = std::nullopt)
        : serial_number(serial_number), resource_name(resource_name), device_resolution(device_resolution) {}
    std::string to_string() const {
        std::ostringstream os;
        os << "ObResourceConfig(resource_name=" << resource_name << ", serial_number=" << serial_number;
        if (device_resolution.has_value()) {
            os << ", color_resolution=" << device_resolution->color_resolution.width << "x" << device_resolution->color_resolution.height;
            os << ", depth_resolution=" << device_resolution->depth_resolution.width << "x" << device_resolution->depth_resolution.height;
        } else {
            os << ", device_resolution=nullopt";
        }
        os << ")";
        return os.str();
    }
};

void startOrbbecSDK(ob::Context& ctx);
void printDeviceInfo(const std::shared_ptr<ob::DeviceInfo> info);

class Orbbec final : public viam::sdk::Camera, public viam::sdk::Reconfigurable {
   public:
    Orbbec(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg, std::shared_ptr<ob::Context> ctx);
    ~Orbbec();
    void reconfigure(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg) override;
    viam::sdk::ProtoStruct do_command(const viam::sdk::ProtoStruct& command) override;
    raw_image get_image(std::string mime_type, const viam::sdk::ProtoStruct& extra) override;
    image_collection get_images() override;
    point_cloud get_point_cloud(std::string mime_type, const viam::sdk::ProtoStruct& extra) override;
    properties get_properties() override;
    std::vector<viam::sdk::GeometryConfig> get_geometries(const viam::sdk::ProtoStruct& extra) override;
    static std::vector<std::string> validate(viam::sdk::ResourceConfig cfg);
    static viam::sdk::GeometryConfig geometry;
    static viam::sdk::Model model;

   private:
    std::shared_ptr<ob::Context> ob_ctx_;
    std::unique_ptr<ObResourceConfig> config_;
    std::mutex config_mu_;
    std::string firmware_version_;
    static std::unique_ptr<ObResourceConfig> configure(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg);
};

}  // namespace orbbec
