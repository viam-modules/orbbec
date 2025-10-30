#include "discovery.hpp"
#include "orbbec.hpp"

namespace discovery {

namespace vsdk = ::viam::sdk;
vsdk::Model OrbbecDiscovery::model = vsdk::Model("viam", "orbbec", "discovery");

OrbbecDiscovery::OrbbecDiscovery(vsdk::Dependencies dependencies, vsdk::ResourceConfig configuration, std::shared_ptr<ob::Context> ctx)
    : Discovery(configuration.name()), ob_ctx_(std::move(ctx)) {}

std::vector<vsdk::ResourceConfig> OrbbecDiscovery::discover_resources(const vsdk::ProtoStruct& extra) {
    std::vector<vsdk::ResourceConfig> configs;

    // Enable network device enumeration for Ethernet cameras (like Gemini 335Le)
    try {
        ob_ctx_->enableNetDeviceEnumeration(true);
        VIAM_RESOURCE_LOG(info) << "Enabled network device enumeration for Ethernet cameras";
    } catch (ob::Error& e) {
        VIAM_RESOURCE_LOG(error) << "Failed to enable network device enumeration: " << e.what();
    } catch (const std::exception& e) {
        VIAM_RESOURCE_LOG(error) << "Failed to enable network device enumeration: " << e.what();
    }

    try {
        // Use SDK's native device discovery (works for both USB and network devices)
        std::shared_ptr<ob::DeviceList> devList = ob_ctx_->queryDeviceList();
        int devCount = devList->getCount();

        if (devCount == 0) {
            VIAM_RESOURCE_LOG(warn) << "No Orbbec devices found during discovery";
            return {};
        }

        VIAM_RESOURCE_LOG(info) << "Discovered " << devCount << " Orbbec devices";

        for (size_t i = 0; i < devCount; i++) {
            try {
                std::string deviceName = devList->name(i);
                std::string serialNumber = devList->serialNumber(i);
                std::string connectionType = devList->connectionType(i);
                std::string ipAddress = devList->ipAddress(i);

                std::stringstream deviceInfoString;
                deviceInfoString << "Device " << (i + 1) << " - Name: " << deviceName << ", Serial: " << serialNumber
                                 << ", Connection: " << connectionType;
                if (!ipAddress.empty()) {
                    deviceInfoString << ", IP: " << ipAddress;
                }

                VIAM_RESOURCE_LOG(info) << deviceInfoString.str();

                std::ostringstream name;
                name << "orbbec-" << i + 1;

                vsdk::ProtoStruct attributes;
                attributes.emplace("serial_number", serialNumber);

                // Detect model and create appropriate resource
                std::string viamModelSuffix;
                std::optional<orbbec::OrbbecModelConfig> modelConfig = orbbec::OrbbecModelConfig::forDevice(deviceName);
                if (!modelConfig.has_value()) {
                    VIAM_RESOURCE_LOG(error) << "Failed to determine model configuration for device " << deviceName;
                    continue;
                }
                viamModelSuffix = modelConfig->viam_model_suffix;

                vsdk::ResourceConfig config("camera",
                                            std::move(name.str()),
                                            "viam",
                                            attributes,
                                            "rdk:component:camera",
                                            vsdk::Model("viam", "orbbec", viamModelSuffix),
                                            vsdk::log_level::info);
                configs.push_back(config);

                VIAM_RESOURCE_LOG(info) << "Successfully configured device " << (i + 1) << " with serial: " << serialNumber;
            } catch (ob::Error& deviceError) {
                VIAM_RESOURCE_LOG(error) << "Failed to get device info for device " << (i + 1) << ": " << deviceError.what();
                // Continue with other devices even if one fails
            }
        }
    } catch (ob::Error& e) {
        VIAM_RESOURCE_LOG(error) << "Failed to discover Orbbec devices: " << e.what() << " (function: " << e.getFunction()
                                 << ", args: " << e.getArgs() << ", name: " << e.getName() << ", type: " << e.getExceptionType() << ")";
        VIAM_RESOURCE_LOG(error) << "Discovery failed - check network connectivity for Ethernet cameras or USB connection for USB cameras";
    } catch (const std::exception& e) {
        VIAM_RESOURCE_LOG(error) << "Failed to discover Orbbec devices: " << e.what();
        VIAM_RESOURCE_LOG(error) << "Discovery failed - check network connectivity for Ethernet cameras or USB connection for USB cameras";
    } catch (...) {
        VIAM_RESOURCE_LOG(error) << "Failed to discover Orbbec devices: unknown error";
        VIAM_RESOURCE_LOG(error) << "Discovery failed - check network connectivity for Ethernet cameras or USB connection for USB cameras";
    }

    return configs;
}

vsdk::ProtoStruct OrbbecDiscovery::do_command(const vsdk::ProtoStruct& command) {
    VIAM_RESOURCE_LOG(error) << "do_command not implemented";
    return vsdk::ProtoStruct{};
}

}  // namespace discovery
