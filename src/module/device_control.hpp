#pragma once
#include <viam/sdk/common/proto_value.hpp>

namespace device_control {

inline double depthPrecisionLevelToUnit(OBDepthPrecisionLevel precision) {
    switch (precision) {
        case OB_PRECISION_1MM:
            return 1.0;
        case OB_PRECISION_0MM8:
            return 0.8;
        case OB_PRECISION_0MM4:
            return 0.4;
        case OB_PRECISION_0MM1:
            return 0.1;
        case OB_PRECISION_0MM2:
            return 0.2;
        case OB_PRECISION_0MM5:
            return 0.5;
        case OB_PRECISION_0MM05:
            return 0.05;
        default:
            break;
    }
    throw std::invalid_argument("Unregistered depth precision level");
}

inline bool areAlmostEqual(double a, double b, double epsilon = 1e-5) {
    return std::abs(a - b) < epsilon;
}

inline OBDepthPrecisionLevel depthUnitToPrecisionLevel(double unit) {
    if (areAlmostEqual(1.0, unit)) {
        return OB_PRECISION_1MM;
    } else if (areAlmostEqual(0.8, unit)) {
        return OB_PRECISION_0MM8;
    } else if (areAlmostEqual(0.4, unit)) {
        return OB_PRECISION_0MM4;
    } else if (areAlmostEqual(0.1, unit)) {
        return OB_PRECISION_0MM1;
    } else if (areAlmostEqual(0.2, unit)) {
        return OB_PRECISION_0MM2;
    } else if (areAlmostEqual(0.5, unit)) {
        return OB_PRECISION_0MM5;
    } else if (areAlmostEqual(0.05, unit)) {
        return OB_PRECISION_0MM05;
    }
    throw std::invalid_argument("Unsupported unit to depth precision level");
}

inline bool isPrimaryTypeProperty(OBPropertyItem propertyItem) {
    return propertyItem.type == OB_INT_PROPERTY || propertyItem.type == OB_FLOAT_PROPERTY || propertyItem.type == OB_BOOL_PROPERTY;
}

inline std::string permissionToString(OBPermissionType permission) {
    switch (permission) {
        case OB_PERMISSION_DENY:
            return "deny";
        case OB_PERMISSION_READ:
            return "read";
        case OB_PERMISSION_WRITE:
            return "write";
        case OB_PERMISSION_READ_WRITE:
            return "read_write";
        case OB_PERMISSION_ANY:
            return "any";
        default:
            return "unknown";
    }
}

inline std::string propertyTypeToString(OBPropertyType type) {
    switch (type) {
        case OB_BOOL_PROPERTY:
            return "bool";
        case OB_INT_PROPERTY:
            return "int";
        case OB_FLOAT_PROPERTY:
            return "float";
        case OB_STRUCT_PROPERTY:
            return "struct";
        default:
            return "unknown";
    }
}

inline std::string distortionTypeToString(OBCameraDistortionModel model) {
    switch (model) {
        case OB_DISTORTION_NONE:
            return "NONE";
            break;
        case OB_DISTORTION_MODIFIED_BROWN_CONRADY:
            return "MODIFIED_BROWN_CONRADY";
            break;
        case OB_DISTORTION_INVERSE_BROWN_CONRADY:
            return "INVERSE_BROWN_CONRADY";
            break;
        case OB_DISTORTION_BROWN_CONRADY:
            return "BROWN_CONRADY";
            break;
        case OB_DISTORTION_BROWN_CONRADY_K6:
            return "BROWN_CONRADY_K6";
            break;
        case OB_DISTORTION_KANNALA_BRANDT4:
            return "KANNALA_BRANDT4";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}

viam::sdk::ProtoStruct getOrbbecSDKVersion(std::string const& command) {
    viam::sdk::ProtoStruct resp;
    std::stringstream ss;
    ss << ob::Version::getMajor() << "." << ob::Version::getMinor() << "." << ob::Version::getPatch();
    resp["version"] = ss.str();
    resp["stage_version"] = ob::Version::getStageVersion();
    return resp;
}

template <typename DeviceT>
viam::sdk::ProtoStruct getDeviceInfo(std::shared_ptr<DeviceT>& device, std::string const& command) {
    std::shared_ptr<ob::DeviceInfo> info = device->getDeviceInfo();
    viam::sdk::ProtoStruct info_struct;
    info_struct["pid"] = info->getPid();
    info_struct["vid"] = info->getVid();
    info_struct["name"] = info->getName();
    info_struct["uid"] = info->getUid();
    info_struct["serial_number"] = info->getSerialNumber();
    info_struct["firmware_version"] = info->getFirmwareVersion();
    info_struct["connection_type"] = info->getConnectionType();
    info_struct["hardware_version"] = info->getHardwareVersion();
    info_struct["supported_min_sdk_version"] = info->getSupportedMinSdkVersion();
    info_struct["asic_name"] = info->getAsicName();
    return info_struct;
}

template <typename DeviceT>
viam::sdk::ProtoStruct getDeviceProperty(std::shared_ptr<DeviceT> device, viam::sdk::ProtoValue const& value, std::string const& command) {
    if (!value.template is_a<std::string>()) {
        return {{"error", "value must be a struct"}};
    }
    std::string property = value.template get_unchecked<std::string>();
    uint32_t size = device->getSupportedPropertyCount();
    for (uint32_t i = 0; i < size; i++) {
        OBPropertyItem property_item = device->getSupportedProperty(i);
        if (property_item.name == property) {
            viam::sdk::ProtoStruct property_struct;
            property_struct["name"] = property_item.name;
            property_struct["id"] = property_item.id;
            property_struct["type"] = propertyTypeToString(property_item.type);
            property_struct["permission"] = permissionToString(property_item.permission);
            if (property_item.type == OB_INT_PROPERTY) {
                OBIntPropertyRange valueRange = device->getIntPropertyRange(property_item.id);
                property_struct["current"] = valueRange.cur;
                property_struct["min"] = valueRange.min;
                property_struct["max"] = valueRange.max;
                property_struct["default"] = valueRange.def;
            } else if (property_item.type == OB_FLOAT_PROPERTY) {
                OBFloatPropertyRange valueRange = device->getFloatPropertyRange(property_item.id);
                property_struct["current"] = valueRange.cur;
                property_struct["min"] = valueRange.min;
                property_struct["max"] = valueRange.max;
                property_struct["default"] = valueRange.def;
            } else if (property_item.type == OB_BOOL_PROPERTY) {
                OBBoolPropertyRange valueRange = device->getBoolPropertyRange(property_item.id);
                property_struct["current"] = valueRange.cur;
                property_struct["default"] = valueRange.def;
            } else if (property_item.type == OB_STRUCT_PROPERTY) {
                property_struct["current"] = "struct_property";
            } else {
                property_struct["current"] = "unknown_type";
            }
            return property_struct;
        }
    }
    return {};
}

template <typename DeviceT>
viam::sdk::ProtoStruct setDeviceProperty(std::shared_ptr<DeviceT> device,
                                         const viam::sdk::ProtoValue& property,
                                         std::string const& command) {
    if (!property.template is_a<viam::sdk::ProtoStruct>()) {
        return {{"error", "property must be a struct"}};
    }
    auto const& property_map = property.template get_unchecked<viam::sdk::ProtoStruct>();
    if (property_map.size() == 0) {
        return {{"error", "property map is empty"}};
    }
    if (property_map.size() > 1) {
        return {{"error", "property map must contain exactly one entry"}};
    }

    uint32_t size = device->getSupportedPropertyCount();
    for (uint32_t i = 0; i < size; i++) {
        OBPropertyItem property_item = device->getSupportedProperty(i);
        if (property_item.name == property_map.begin()->first) {
            // Set the property value
            if (property_item.type == OB_INT_PROPERTY) {
                if (!property_map.begin()->second.is_a<double>()) {
                    return {{"error", "Invalid type for int property"}};
                }
                int int_value = property_map.begin()->second.get_unchecked<double>();
                device->setIntProperty(property_item.id, int_value);
            } else if (property_item.type == OB_FLOAT_PROPERTY) {
                if (!property_map.begin()->second.is_a<double>()) {
                    return {{"error", "Invalid type for float property"}};
                }
                float float_value = property_map.begin()->second.get_unchecked<double>();
                device->setFloatProperty(property_item.id, float_value);
            } else if (property_item.type == OB_BOOL_PROPERTY) {
                if (!property_map.begin()->second.is_a<bool>()) {
                    return {{"error", "Invalid type for bool property"}};
                }
                bool bool_value = property_map.begin()->second.get_unchecked<bool>();
                device->setBoolProperty(property_item.id, bool_value);
            } else {
                return {{"error", "Unsupported property type"}};
            }
            return getDeviceProperty(device, property_item.name, command);
        }
    }

    return {};
}

// Get property list
template <typename DeviceT>
viam::sdk::ProtoStruct getDeviceProperties(std::shared_ptr<DeviceT> device,
                                           std::string const& command,
                                           std::unordered_set<std::string> const& filter = {}) {
    viam::sdk::ProtoStruct properties;
    uint32_t size = device->getSupportedPropertyCount();
    for (uint32_t i = 0; i < size; i++) {
        OBPropertyItem property_item = device->getSupportedProperty(i);
        if (isPrimaryTypeProperty(property_item) && property_item.permission != OB_PERMISSION_DENY &&
            (filter.empty() || filter.count(property_item.name) > 0)) {
            properties[property_item.name] = getDeviceProperty(device, property_item.name, command);
        }
    }
    return properties;
}

template <typename DeviceT>
viam::sdk::ProtoStruct setDeviceProperties(std::shared_ptr<DeviceT> device,
                                           const viam::sdk::ProtoValue& properties,
                                           std::string const& command) {
    if (device == nullptr) {
        return {{"error", "device is null"}};
    }
    if (!properties.template is_a<viam::sdk::ProtoStruct>()) {
        return {{"error", "properties must be a struct"}};
    }
    std::unordered_set<std::string> writable_properties;
    auto const& properties_map = properties.template get_unchecked<viam::sdk::ProtoStruct>();
    int const supportedPropertyCount = device->getSupportedPropertyCount();
    for (int i = 0; i < supportedPropertyCount; i++) {
        OBPropertyItem property_item = device->getSupportedProperty(i);
        if (properties_map.count(property_item.name) > 0) {
            if (property_item.permission == OB_PERMISSION_DENY || property_item.permission == OB_PERMISSION_READ) {
                std::stringstream error_ss;
                error_ss << "Property " << property_item.name << " is not writable, skipping.";
                VIAM_SDK_LOG(warn) << error_ss.str();
                return {{"error", error_ss.str()}};
                continue;
            }
            try {
                auto const& value_struct = properties_map.at(property_item.name).get<viam::sdk::ProtoStruct>();
                if (!value_struct) {
                    std::stringstream error_ss;
                    error_ss << "Value for property " << property_item.name << " is not a struct, skipping.";
                    VIAM_SDK_LOG(warn) << error_ss.str();
                    continue;
                }
                auto const& value = value_struct->at("current");
                writable_properties.insert(property_item.name);
                if (property_item.type == OB_INT_PROPERTY && value.is_a<double>()) {
                    int int_value = static_cast<int>(value.get_unchecked<double>());
                    device->setIntProperty(property_item.id, int_value);
                    VIAM_SDK_LOG(info) << "[setDeviceProperties] Set int property " << property_item.name << " to " << int_value;
                } else if (property_item.type == OB_FLOAT_PROPERTY && value.is_a<double>()) {
                    double float_value = value.get_unchecked<double>();
                    device->setFloatProperty(property_item.id, float_value);
                    VIAM_SDK_LOG(info) << "[setDeviceProperties] Set float property " << property_item.name << " to " << float_value;
                } else if (property_item.type == OB_BOOL_PROPERTY && value.is_a<bool>()) {
                    bool bool_value = value.get_unchecked<bool>();
                    device->setBoolProperty(property_item.id, bool_value);
                    VIAM_SDK_LOG(info) << "[setDeviceProperties] Set bool property " << property_item.name << " to "
                                       << (bool_value ? "true" : "false");
                } else {
                    VIAM_SDK_LOG(warn) << "[setDeviceProperties] Type mismatch or unsupported type for property " << property_item.name
                                       << ", skipping.";
                }
            } catch (ob::Error& e) {
                std::stringstream error_ss;
                error_ss << "Failed to set property " << property_item.name << ": " << e.what();
                VIAM_SDK_LOG(error) << error_ss.str();
                return {{"error", error_ss.str()}};
            }
        }
    }
    return getDeviceProperties<DeviceT>(device, command, {});
}

template <typename DeviceT>
std::string getCurrentDepthWorkMode(std::shared_ptr<DeviceT> device) {
    // Query the current camera depth mode
    auto curDepthMode = device->getCurrentDepthWorkMode();
    std::cout << "current depth work mode: " << curDepthMode.name << std::endl;

    return curDepthMode.name;
}

template <typename FilterT>
viam::sdk::ProtoStruct getFilterInfo(std::shared_ptr<FilterT> filter) {
    viam::sdk::ProtoStruct filterInfo;
    filterInfo["enabled"] = filter->isEnabled();
    auto configSchemaVec = filter->getConfigSchemaVec();
    for (auto& configSchema : configSchemaVec) {
        VIAM_SDK_LOG(info) << "    - {" << configSchema.name << ", " << configSchema.type << ", " << configSchema.min << ", "
                           << configSchema.max << ", " << configSchema.step << ", " << configSchema.def << ", " << configSchema.desc << "}";
        viam::sdk::ProtoStruct schemaStruct;
        schemaStruct["type"] = configSchema.type;
        schemaStruct["min"] = configSchema.min;
        schemaStruct["max"] = configSchema.max;
        schemaStruct["step"] = configSchema.step;
        schemaStruct["def"] = configSchema.def;
        schemaStruct["desc"] = configSchema.desc;
        // Add the current value
        double currentValue = filter->getConfigValue(configSchema.name);
        schemaStruct["curr_value"] = currentValue;
        filterInfo[configSchema.name] = schemaStruct;
    }
    return filterInfo;
}

template <typename FilterT>
viam::sdk::ProtoStruct filterListToProtoStruct(std::vector<std::shared_ptr<FilterT>> const& recommendedDepthFilters,
                                               std::unordered_set<std::string> const& writable_properties = {},
                                               bool const only_enabled_filters = false) {
    viam::sdk::ProtoStruct recommendedFilters;
    for (const auto& filter : recommendedDepthFilters) {
        if ((only_enabled_filters && !filter->isEnabled()) ||
            (!writable_properties.empty() && writable_properties.count(filter->getName()) == 0)) {
            continue;
        }
        VIAM_SDK_LOG(info) << "[filterListToProtoStruct] " << filter->getName() << ": " << (filter->isEnabled() ? "enabled" : "disabled");

        viam::sdk::ProtoStruct filterInfo = getFilterInfo(filter);
        recommendedFilters[filter->getName()] = filterInfo;
    }
    return recommendedFilters;
}

template <typename FilterT>
std::string postProcessDepthFiltersToString(std::vector<std::shared_ptr<FilterT>> const& filters) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < filters.size(); i++) {
        oss << filters[i]->getName() << "(" << (filters[i]->isEnabled() ? "enabled" : "disabled") << ")";
        if (i != filters.size() - 1) {
            oss << ", ";
        }
    }
    oss << "]";
    return oss.str();
}

template <typename DeviceT>
viam::sdk::ProtoStruct getRecommendedPostProcessDepthFilters(std::shared_ptr<DeviceT>& device, bool const only_enabled_filters = false) {
    auto depthSensor = device->getSensor(OB_SENSOR_DEPTH);
    auto depthFilterList = depthSensor->createRecommendedFilters();

    return filterListToProtoStruct(depthFilterList, {}, only_enabled_filters);
}

template <typename ViamDeviceT>
viam::sdk::ProtoStruct setRecommendedPostProcessDepthFilters(std::unique_ptr<ViamDeviceT>& dev) {
    auto device = dev->device;
    auto depthSensor = device->getSensor(OB_SENSOR_DEPTH);
    auto depthFilterList = depthSensor->createRecommendedFilters();
    dev->postProcessDepthFilters = depthFilterList;

    return filterListToProtoStruct(dev->postProcessDepthFilters);
}

template <typename FilterT>
viam::sdk::ProtoStruct setPostProcessDepthFilters(std::vector<std::shared_ptr<FilterT>>& currentDepthFilters,
                                                  viam::sdk::ProtoStruct const& newFiltersConfig) {
    VIAM_SDK_LOG(error) << "[setPostProcessDepthFilters] begin";
    std::unordered_set<std::string> writable_properties;
    for (auto& filter : currentDepthFilters) {
        if (newFiltersConfig.count(filter->getName())) {
            auto const& filterConfigValue = newFiltersConfig.at(filter->getName());
            if (!filterConfigValue.template is_a<viam::sdk::ProtoStruct>()) {
                VIAM_SDK_LOG(error) << "[setPostProcessDepthFilters] Filter config for " << filter->getName() << " is not a struct";
                continue;
            }
            auto& filterConfig = filterConfigValue.template get_unchecked<viam::sdk::ProtoStruct>();
            for (const auto& [key, value] : filterConfig) {
                if (key == "enabled") {
                    filter->enable(value.template get_unchecked<bool>());
                }
            }
            // Set only valid config names
            for (auto& configSchema : filter->getConfigSchemaVec()) {
                std::string paramName = configSchema.name;
                if (filterConfig.count(paramName)) {
                    auto& paramValue = filterConfig.at(paramName);
                    if (paramValue.template is_a<double>()) {
                        filter->setConfigValue(paramName, paramValue.template get_unchecked<double>());
                    }
                }
            }
            writable_properties.insert(filter->getName());
        }
    }
    VIAM_SDK_LOG(error) << "[setPostProcessDepthFilters] writable properties size: " << writable_properties.size();
    return filterListToProtoStruct(currentDepthFilters, writable_properties);
}

template <typename FilterT>
viam::sdk::ProtoStruct getPostProcessDepthFilters(std::vector<std::shared_ptr<FilterT>> const& depthFilters,
                                                  std::string const& command,
                                                  std::unordered_set<std::string> const& writable_properties = {}) {
    VIAM_SDK_LOG(info) << "[getPostProcessDepthFilters]" << command
                       << ": getting current post process depth filters, writable properties size: " << writable_properties.size();
    try {
        bool const only_enabled_filters = true;
        auto const current_post_process_depth_filters = filterListToProtoStruct(depthFilters, writable_properties, only_enabled_filters);
        return {{command, current_post_process_depth_filters}};
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "[do_command]" << command << ": " << e.what();
        return viam::sdk::ProtoStruct{{"error", e.what()}};
    }
}

template <typename FilterT>
viam::sdk::ProtoStruct setPostProcessDepthFilters(std::vector<std::shared_ptr<FilterT>>& currentDepthFilters,
                                                  viam::sdk::ProtoValue const& newDepthFilters,
                                                  std::string const& command) {
    try {
        if (!newDepthFilters.is_a<viam::sdk::ProtoStruct>()) {
            std::stringstream error_ss;
            error_ss << "[setPostProcessDepthFilters] set_depth_filters: expected struct, got " << newDepthFilters.kind();
            VIAM_SDK_LOG(error) << error_ss.str();
            return viam::sdk::ProtoStruct{{"error", error_ss.str()}};
        }
        auto const updatedFilters =
            setPostProcessDepthFilters(currentDepthFilters, newDepthFilters.get_unchecked<viam::sdk::ProtoStruct>());

        return {{command, updatedFilters}};
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "[setPostProcessDepthFilters] set_post_process_depth_filters: " << e.what();
        return viam::sdk::ProtoStruct{{"error", e.what()}};
    }
}

template <typename ViamDeviceT>
viam::sdk::ProtoStruct applyPostProcessDepthFilters(std::unique_ptr<ViamDeviceT>& my_dev,
                                                    viam::sdk::ProtoValue const& value,
                                                    std::string const& command) {
    if (!value.is_a<bool>()) {
        VIAM_SDK_LOG(error) << "[do_command] apply_recommended_depth_filters: expected bool, got " << value.kind();
        return viam::sdk::ProtoStruct{{"error", "expected bool"}};
    }
    my_dev->applyEnabledPostProcessDepthFilters = value.get_unchecked<bool>();
    return viam::sdk::ProtoStruct{{command, my_dev->applyEnabledPostProcessDepthFilters}};
}

template <typename DeviceT>
viam::sdk::ProtoStruct getDepthUnit(std::shared_ptr<DeviceT>& device, std::string const& command) {
    if (!device) {
        return {{"error", "Device not found."}};
    }
    try {
        if (device->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_READ)) {
            double value = depthPrecisionLevelToUnit((OBDepthPrecisionLevel)device->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT));
            // value is aproximated to the nearest 0.01
            // value = std::round(value * 100.0) / 100.0;
            viam::sdk::ProtoList availableDepthPrecisionLevelsProto;
            availableDepthPrecisionLevelsProto.push_back("1.0");
            availableDepthPrecisionLevelsProto.push_back("0.8");
            availableDepthPrecisionLevelsProto.push_back("0.5");
            availableDepthPrecisionLevelsProto.push_back("0.4");
            availableDepthPrecisionLevelsProto.push_back("0.2");
            availableDepthPrecisionLevelsProto.push_back("0.1");
            availableDepthPrecisionLevelsProto.push_back("0.05");

            return {{"depth_precision_level_mm", value}, {"available_depth_precision_levels_mm", availableDepthPrecisionLevelsProto}};
        } else {
            return {{"error", "Depth precision level property is not supported."}};
        }
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "[getDepthUnit] " << e.what();
        return viam::sdk::ProtoStruct{{"error", e.what()}};
    }
}

template <typename DeviceT>
viam::sdk::ProtoStruct setDepthUnit(std::shared_ptr<DeviceT>& device, viam::sdk::ProtoValue const& value, std::string const& command) {
    if (!device) {
        return {{"error", "Device not found."}};
    }
    if (!value.template is_a<double>()) {
        return {{"error", "Invalid value type for Depth Unit. Expected double."}};
    }
    double const depthUnit = value.get_unchecked<double>();
    try {
        if (device->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_WRITE)) {
            device->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, depthUnitToPrecisionLevel(depthUnit));
            return getDepthUnit(device, command);
        } else {
            return {{"error", "Depth unit property is not supported."}};
        }
    } catch (ob::Error& e) {
        std::stringstream error_ss;
        error_ss << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what()
                 << "\ntype:" << e.getExceptionType() << std::endl;
        return {{"error", error_ss.str()}};
    }
}

template <typename PipelineT, typename VideoStreamProfileT>
viam::sdk::ProtoStruct getCameraParams(std::shared_ptr<PipelineT> pipe) {
    if (pipe == nullptr) {
        return {{"error", "pipe is null"}};
    }
    auto const config = pipe->getConfig();
    if (!config) {
        return {{"error", "failed to get pipeline config"}};
    }

    auto enabledStreamProfileListPtr = config->getEnabledStreamProfileList();

    viam::sdk::ProtoStruct result;
    if (enabledStreamProfileListPtr) {
        auto const& enabledStreamProfileList = *enabledStreamProfileListPtr;
        auto const count = enabledStreamProfileList.getCount();
        for (int i = 0; i < count; i++) {
            auto sp = enabledStreamProfileList.getProfile(i);
            auto vsp = sp->template as<VideoStreamProfileT>();
            std::string sensorName = ob::TypeHelper::convertOBStreamTypeToString(sp->getType());
            viam::sdk::ProtoStruct profile;
            profile["width"] = static_cast<int>(vsp->getWidth());
            profile["height"] = static_cast<int>(vsp->getHeight());
            profile["format"] = ob::TypeHelper::convertOBFormatTypeToString(vsp->getFormat());
            profile["fps"] = static_cast<int>(vsp->getFps());

            viam::sdk::ProtoStruct intrinsics_struct;
            auto intrinsics = vsp->getIntrinsic();
            intrinsics_struct["fx"] = static_cast<double>(intrinsics.fx);
            intrinsics_struct["fy"] = static_cast<double>(intrinsics.fy);
            intrinsics_struct["cx"] = static_cast<double>(intrinsics.cx);
            intrinsics_struct["cy"] = static_cast<double>(intrinsics.cy);
            intrinsics_struct["width"] = static_cast<double>(intrinsics.width);
            intrinsics_struct["height"] = static_cast<double>(intrinsics.height);
            profile["intrinsics"] = intrinsics_struct;

            viam::sdk::ProtoStruct distortion_struct;
            auto distortion = vsp->getDistortion();
            distortion_struct["k1"] = static_cast<double>(distortion.k1);
            distortion_struct["k2"] = static_cast<double>(distortion.k2);
            distortion_struct["k3"] = static_cast<double>(distortion.k3);
            distortion_struct["k4"] = static_cast<double>(distortion.k4);
            distortion_struct["k5"] = static_cast<double>(distortion.k5);
            distortion_struct["k6"] = static_cast<double>(distortion.k6);
            distortion_struct["p1"] = static_cast<double>(distortion.p1);
            distortion_struct["p2"] = static_cast<double>(distortion.p2);
            distortion_struct["model"] = distortionTypeToString(distortion.model);
            profile["distortion"] = distortion_struct;

            result[sensorName] = profile;
        }
    }

    return result;
}

template <typename DeviceT>
viam::sdk::ProtoStruct getCameraTemperature(std::shared_ptr<DeviceT> device, std::string const& command) {
    if (!device) {
        return {{"error", "Device not found."}};
    }
    try {
        if (device->isPropertySupported(OB_STRUCT_DEVICE_TEMPERATURE, OB_PERMISSION_READ)) {
            OBDeviceTemperature obTemp;
            uint32_t dataSize = sizeof(OBDeviceTemperature);
            device->getStructuredData(OB_STRUCT_DEVICE_TEMPERATURE, reinterpret_cast<uint8_t*>(&obTemp), &dataSize);

            viam::sdk::ProtoStruct temperatureStruct;
            temperatureStruct["main_board_temp_c"] = obTemp.mainBoardTemp;
            temperatureStruct["cpu_temp_c"] = obTemp.cpuTemp;
            temperatureStruct["rgb_temp_c"] = obTemp.rgbTemp;
            temperatureStruct["ir_left_temp_c"] = obTemp.irLeftTemp;
            temperatureStruct["ir_right_temp_c"] = obTemp.irRightTemp;
            temperatureStruct["chip_top_temp_c"] = obTemp.chipTopTemp;
            temperatureStruct["chip_bottom_temp_c"] = obTemp.chipBottomTemp;

            return {{command, temperatureStruct}};
        } else {
            return {{"error", "Device temperature property is not supported."}};
        }
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "[getCameraTemperature] " << e.what();
        return viam::sdk::ProtoStruct{{"error", e.what()}};
    }
}

template <typename ViamDeviceT, typename VideoStreamProfileT>
viam::sdk::ProtoStruct createModuleConfig(std::unique_ptr<ViamDeviceT>& dev) {
    if (dev == nullptr) {
        return {{"error", "device is null"}};
    }
    if (dev->pipe == nullptr) {
        return {{"error", "pipe is null"}};
    }
    auto const config = dev->pipe->getConfig();
    if (!config) {
        return {{"error", "failed to get pipeline config"}};
    }

    auto const device = dev->device;
    if (device == nullptr) {
        return {{"error", "device is null"}};
    }

    auto enabledStreamProfileListPtr = config->getEnabledStreamProfileList();
    if (!enabledStreamProfileListPtr) {
        return {{"error", "failed to get enabled stream profile list"}};
    }

    auto const& enabledStreamProfileList = *enabledStreamProfileListPtr;
    auto const count = enabledStreamProfileList.getCount();

    viam::sdk::ProtoStruct sensors;
    viam::sdk::ProtoStruct depth_sensor;
    viam::sdk::ProtoStruct color_sensor;
    for (int i = 0; i < count; i++) {
        auto sp = enabledStreamProfileList.getProfile(i);
        if (sp == nullptr) {
            return {{"error", "failed to get stream profile"}};
        }
        if (sp->getType() != OB_STREAM_COLOR && sp->getType() != OB_STREAM_DEPTH) {
            continue;
        }
        if (sp->getType() == OB_STREAM_DEPTH) {
            depth_sensor["width"] = static_cast<int>(sp->template as<VideoStreamProfileT>()->getWidth());
            depth_sensor["height"] = static_cast<int>(sp->template as<VideoStreamProfileT>()->getHeight());
            depth_sensor["format"] = ob::TypeHelper::convertOBFormatTypeToString(sp->template as<VideoStreamProfileT>()->getFormat());
            sensors["depth"] = depth_sensor;

        } else if (sp->getType() == OB_STREAM_COLOR) {
            color_sensor["width"] = static_cast<int>(sp->template as<VideoStreamProfileT>()->getWidth());
            color_sensor["height"] = static_cast<int>(sp->template as<VideoStreamProfileT>()->getHeight());
            color_sensor["format"] = ob::TypeHelper::convertOBFormatTypeToString(sp->template as<VideoStreamProfileT>()->getFormat());
            sensors["color"] = color_sensor;
        }
    }

    viam::sdk::ProtoStruct result;
    result["serial_number"] = dev->serialNumber;
    result["sensors"] = sensors;
    result["post_process_depth_filters"] =
        getPostProcessDepthFilters(dev->postProcessDepthFilters, "create_module_config")["create_module_config"];
    result["apply_post_process_depth_filters"] = dev->applyEnabledPostProcessDepthFilters;
    result["device_properties"] = getDeviceProperties(device, "create_module_config");

    return result;
}
}  // namespace device_control
