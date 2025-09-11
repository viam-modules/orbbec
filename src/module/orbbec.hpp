#pragma once
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include <libobsensor/ObSensor.hpp>

#include <optional>
#include <unordered_set>

namespace orbbec {

struct Resolution {
    int width{};
    int height{};

    std::string to_string() const {
        std::ostringstream os;
        os << "Resolution(" << width << "x" << height << ")";
        return os.str();
    }
};

struct DeviceResolution {
    std::optional<Resolution> color_resolution{};
    std::optional<Resolution> depth_resolution{};

    std::string to_string() const {
        std::ostringstream os;
        os << "DeviceResolution(";
        if (color_resolution.has_value())
            os << "color_resolution=" << color_resolution->width << "x" << color_resolution->height;
        else
            os << "color_resolution=nullopt";
        if (depth_resolution.has_value())
            os << ", depth_resolution=" << depth_resolution->width << "x" << depth_resolution->height;
        else
            os << ", depth_resolution=nullopt";
        os << ")";
        return os.str();
    }
};

struct DeviceFormat {
    std::optional<std::string> color_format{};
    std::optional<std::string> depth_format{};

    std::string to_string() const {
        std::ostringstream os;
        os << "DeviceFormat(";
        if (color_format.has_value())
            os << "color_format=" << *color_format;
        else
            os << "color_format=nullopt";
        if (depth_format.has_value())
            os << ", depth_format=" << *depth_format;
        else
            os << ", depth_format=nullopt";
        os << ")";
        return os.str();
    }
};

// The native config struct for orbbec resources.
struct ObResourceConfig {
    std::string resource_name;
    std::string serial_number;
    std::optional<DeviceResolution> device_resolution;
    std::optional<DeviceFormat> device_format;

    explicit ObResourceConfig(std::string const& serial_number,
                              std::string const& resource_name,
                              std::optional<DeviceResolution> device_resolution,
                              std::optional<DeviceFormat> device_format)
        : serial_number(serial_number), resource_name(resource_name), device_resolution(device_resolution), device_format(device_format) {}
    std::string to_string() const {
        std::ostringstream os;
        os << "ObResourceConfig(resource_name=" << resource_name << ", serial_number=" << serial_number;
        if (device_resolution.has_value()) {
            os << ", device_resolution=" << device_resolution->to_string();
        } else {
            os << ", device_resolution=nullopt";
        }
        if (device_format.has_value()) {
            os << ", device_format=" << device_format->to_string();
        } else {
            os << ", device_format=nullopt";
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
    static const std::unordered_set<std::string> supported_color_formats;
    static const std::unordered_set<std::string> supported_depth_formats;
    static const std::string default_color_format;
    static const std::string default_depth_format;
    static const Resolution default_color_resolution;
    static const Resolution default_depth_resolution;

   private:
    std::shared_ptr<ob::Context> ob_ctx_;
    std::unique_ptr<ObResourceConfig> config_;
    std::mutex config_mu_;
    std::string firmware_version_;
    static std::unique_ptr<ObResourceConfig> configure(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg);
};

}  // namespace orbbec
