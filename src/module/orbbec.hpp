#pragma once
#include <viam/sdk/common/pose.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/spatialmath/geometry.hpp>

#include <libobsensor/ObSensor.hpp>

#include <optional>
#include <set>
#include <unordered_set>

namespace orbbec {

// Attribute key constants shared across translation units
inline const std::string kAttrSerialNumber = "serial_number";
inline const std::string kAttrSensors = "sensors";
inline const std::string kAttrWidth = "width";
inline const std::string kAttrHeight = "height";
inline const std::string kAttrFormat = "format";

// Source name constants
inline const std::string kColorSourceName = "color";
inline const std::string kDepthSourceName = "depth";

struct Resolution {
    uint32_t width{};
    uint32_t height{};

    bool operator>(const Resolution& other) const {
        return (width > other.width) || (width == other.width && height > other.height);
    }

    std::string to_string() const {
        std::ostringstream os;
        os << "(" << width << "x" << height << ")";
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
        os << "(";
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
        os << "(resource_name=" << resource_name << ", serial_number=" << serial_number;
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

struct OrbbecModelConfig {
    std::unordered_set<std::string> model_names;  // "Astra 2" or "Gemini 335Le"
    std::string viam_model_suffix;                // "astra2" or "gemini_335le"
    Resolution default_color_resolution;
    Resolution default_depth_resolution;
    std::optional<std::string> firmware_url;
    std::string min_firmware_version;
    std::map<Resolution, std::set<Resolution, std::greater<Resolution>>, std::greater<Resolution>> color_to_depth_supported_resolutions;
    std::unordered_set<std::string> supported_color_formats;
    std::unordered_set<std::string> supported_depth_formats;
    std::string default_color_format;
    std::string default_depth_format;

    // Bounding box of the camera body, and the pose of that box's center. The pose is expressed
    // in the color sensor's optical frame, since that is the frame this module reports all of its
    // data in: get_properties returns the RGB intrinsics and the point cloud is depth-to-color
    // aligned. The color sensor is not centered on the body, so the box center is offset from it.
    viam::sdk::box body_box;
    viam::sdk::pose body_box_pose;

    // Get config for a device name
    static std::optional<OrbbecModelConfig> forDevice(const std::string& device_name);

    // Check if a device name is supported
    static bool isSupported(const std::string& device_name);
};

struct ViamOBDevice {
    ~ViamOBDevice() {
        std::cout << "deleting ViamOBDevice " << serialNumber << "\n";
    }
    std::string serialNumber{};
    std::shared_ptr<ob::Device> device{};
    bool started{};
    std::shared_ptr<ob::Pipeline> pipe{};
    std::shared_ptr<ob::PointCloudFilter> pointCloudFilter{};
    std::shared_ptr<ob::Align> align{};
    std::shared_ptr<ob::Config> config{};
    std::vector<std::shared_ptr<ob::Filter>> postProcessDepthFilters{};
    bool applyEnabledPostProcessDepthFilters{};
    bool dumpPCLFiles{};
    uint64_t lastTimestampLogTime{};
};

void startOrbbecSDK(ob::Context& ctx);
void printDeviceInfo(const std::shared_ptr<ob::DeviceInfo> info);

// viam-server rebuilds the resource on a config change rather than calling a reconfigure method,
// so everything a reconfigure would have done lives in the constructor and destructor. The SDK
// destroys the old instance before constructing the replacement, so the destructor must leave the
// device in a state the constructor can start from.
class Orbbec final : public viam::sdk::Camera {
   public:
    Orbbec(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg, std::shared_ptr<ob::Context> ctx);
    ~Orbbec();
    viam::sdk::ProtoStruct do_command(const viam::sdk::ProtoStruct& command) override;
    viam::sdk::ProtoStruct get_status() override;
    image_collection get_images(std::vector<std::string> filter_source_names, const viam::sdk::ProtoStruct& extra) override;
    point_cloud get_point_cloud(std::string mime_type, const viam::sdk::ProtoStruct& extra) override;
    properties get_properties() override;
    std::vector<viam::sdk::GeometryConfig> get_geometries(const viam::sdk::ProtoStruct& extra) override;
    static std::vector<std::string> validateAstra2(viam::sdk::ResourceConfig cfg);
    static std::vector<std::string> validateGemini335Le(viam::sdk::ResourceConfig cfg);
    static std::vector<std::string> validateOrbbecModel(viam::sdk::ResourceConfig cfg, OrbbecModelConfig const& modelConfig);
    static viam::sdk::Model model_astra2;
    static viam::sdk::Model model_gemini_335le;

   private:
    std::shared_ptr<ob::Context> ob_ctx_;
    std::string firmware_version_;
    std::mutex serial_number_mu_;
    std::string serial_number_;
    std::optional<OrbbecModelConfig> model_config_;
    static std::unique_ptr<ObResourceConfig> configure(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg);
    static void validate_sensor(std::pair<std::string, viam::sdk::ProtoValue> const& sensor_pair, const OrbbecModelConfig& modelConfig);

    // Create a config for hardware depth-to-color alignment
    static std::shared_ptr<ob::Config> createHwD2CAlignConfig(std::shared_ptr<ob::Pipeline> pipe,
                                                              std::optional<DeviceResolution> deviceRes,
                                                              std::optional<DeviceFormat> deviceFormat,
                                                              const OrbbecModelConfig& modelConfig);
};

}  // namespace orbbec
