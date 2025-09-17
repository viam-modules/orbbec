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
                // For struct properties, we can add more detailed handling if needed
                // uint32_t bufferSize = 65536;  // Choose a size you expect to be sufficient
                // std::vector<uint8_t> buffer(bufferSize);
                // device->getStructuredData(property_item.id, buffer.data(), &bufferSize);
                // VIAM_SDK_LOG(info) << "Retrieved structured data for property " << property_item.id << ", size: " << bufferSize;
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
                if (not property_map.begin()->second.is_a<double>()) {
                    return {{"error", "Invalid type for int property"}};
                }
                int int_value = property_map.begin()->second.get_unchecked<double>();
                device->setIntProperty(property_item.id, int_value);
            } else if (property_item.type == OB_FLOAT_PROPERTY) {
                if (not property_map.begin()->second.is_a<double>()) {
                    return {{"error", "Invalid type for float property"}};
                }
                float float_value = property_map.begin()->second.get_unchecked<double>();
                device->setFloatProperty(property_item.id, float_value);
            } else if (property_item.type == OB_BOOL_PROPERTY) {
                if (not property_map.begin()->second.is_a<bool>()) {
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
    viam::sdk::ProtoList properties_list;
    uint32_t size = device->getSupportedPropertyCount();
    for (uint32_t i = 0; i < size; i++) {
        OBPropertyItem property_item = device->getSupportedProperty(i);
        if (isPrimaryTypeProperty(property_item) && property_item.permission != OB_PERMISSION_DENY &&
            (filter.empty() || filter.count(property_item.name) > 0)) {
            properties_list.push_back(getDeviceProperty(device, property_item.name, command));
        }
    }
    std::sort(properties_list.begin(), properties_list.end(), [](const viam::sdk::ProtoValue& a, const viam::sdk::ProtoValue& b) {
        auto const& a_map = a.get_unchecked<viam::sdk::ProtoStruct>();
        auto const& b_map = b.get_unchecked<viam::sdk::ProtoStruct>();
        return a_map.at("name").get_unchecked<std::string>() < b_map.at("name").get_unchecked<std::string>();
    });
    properties["properties"] = properties_list;
    return properties;
}

template <typename DeviceT>
viam::sdk::ProtoStruct setDeviceProperties(std::shared_ptr<DeviceT> device,
                                           const viam::sdk::ProtoValue& properties,
                                           std::string const& command) {
    if (!properties.template is_a<viam::sdk::ProtoStruct>()) {
        return {{"error", "properties must be a struct"}};
    }
    std::unordered_set<std::string> writable_properties;
    auto const& properties_map = properties.template get_unchecked<viam::sdk::ProtoStruct>();
    int const supportedPropertyCount = device->getSupportedPropertyCount();
    for (int i = 0; i < supportedPropertyCount; i++) {
        OBPropertyItem property_item = device->getSupportedProperty(i);
        if (properties_map.count(property_item.name) > 0) {
            auto const& value = properties_map.at(property_item.name);
            if (property_item.permission == OB_PERMISSION_DENY || property_item.permission == OB_PERMISSION_READ) {
                std::stringstream error_ss;
                error_ss << "Property " << property_item.name << " is not writable, skipping.";
                VIAM_SDK_LOG(warn) << error_ss.str();
                return {{"error", error_ss.str()}};
                continue;
            }
            try {
                writable_properties.insert(property_item.name);
                if (property_item.type == OB_INT_PROPERTY && value.is_a<double>()) {
                    int int_value = static_cast<int>(value.get_unchecked<double>());
                    device->setIntProperty(property_item.id, int_value);
                    VIAM_SDK_LOG(info) << "Set int property " << property_item.name << " to " << int_value;
                } else if (property_item.type == OB_FLOAT_PROPERTY && value.is_a<double>()) {
                    double float_value = value.get_unchecked<double>();
                    device->setFloatProperty(property_item.id, float_value);
                    VIAM_SDK_LOG(info) << "Set float property " << property_item.name << " to " << float_value;
                } else if (property_item.type == OB_BOOL_PROPERTY && value.is_a<bool>()) {
                    bool bool_value = value.get_unchecked<bool>();
                    device->setBoolProperty(property_item.id, bool_value);
                    VIAM_SDK_LOG(info) << "Set bool property " << property_item.name << " to " << (bool_value ? "true" : "false");
                } else {
                    VIAM_SDK_LOG(warn) << "Type mismatch or unsupported type for property " << property_item.name << ", skipping.";
                    return {{"error", "Type mismatch or unsupported type"}};
                }
            } catch (ob::Error& e) {
                std::stringstream error_ss;
                error_ss << "Failed to set property " << property_item.name << ": " << e.what();
                VIAM_SDK_LOG(error) << error_ss.str();
                return {{"error", error_ss.str()}};
            }
        }
    }
    return getDeviceProperties(device, command, writable_properties);
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
                                               std::unordered_set<std::string> const& writable_properties = {}) {
    viam::sdk::ProtoStruct recommendedFilters;
    for (const auto& filter : recommendedDepthFilters) {
        if (!writable_properties.empty() && writable_properties.count(filter->getName()) == 0) {
            continue;
        }
        VIAM_SDK_LOG(info) << "[filterListToProtoStruct] " << filter->getName() << ": " << (filter->isEnabled() ? "enabled" : "disabled");

        viam::sdk::ProtoStruct filterInfo = getFilterInfo(filter);
        recommendedFilters[filter->getName()] = filterInfo;
    }
    return recommendedFilters;
}

template <typename DeviceT>
viam::sdk::ProtoStruct getRecommendedPostProcessDepthFilters(std::shared_ptr<DeviceT>& device) {
    auto depthSensor = device->getSensor(OB_SENSOR_DEPTH);
    auto depthFilterList = depthSensor->createRecommendedFilters();

    return filterListToProtoStruct(depthFilterList);
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
        auto const current_post_process_depth_filters = filterListToProtoStruct(depthFilters, writable_properties);
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
}  // namespace device_control
