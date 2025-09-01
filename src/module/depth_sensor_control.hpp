#pragma once
#include <viam/sdk/common/proto_value.hpp>

namespace depth_sensor_control {
template <typename DeviceT>
void setDepthSoftFilter(std::shared_ptr<DeviceT> device, bool enable) {
    try {
        if (device->isPropertySupported(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL, OB_PERMISSION_WRITE)) {
            device->setBoolProperty(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL, enable);
            VIAM_SDK_LOG(info) << "[setDepthSoftFilter] " << (enable ? "enabled" : "disabled") << " depth soft filter" << std::endl;
        } else {
            VIAM_SDK_LOG(error) << "[setDepthSoftFilter] OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL is not supported";
        }
    } catch (ob::Error& e) {
        std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what()
                  << "\ntype:" << e.getExceptionType() << std::endl;
        exit(EXIT_FAILURE);
    }
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
viam::sdk::ProtoStruct filterListToProtoStruct(std::vector<std::shared_ptr<FilterT>> const& recommendedDepthFilters) {
    viam::sdk::ProtoStruct recommendedFilters;
    for (const auto& filter : recommendedDepthFilters) {
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
        }
    }
    return filterListToProtoStruct(currentDepthFilters);
}

template <typename FilterT>
viam::sdk::ProtoStruct getPostProcessDepthFilters(std::vector<std::shared_ptr<FilterT>> const& depthFilters, std::string const& command) {
    try {
        auto const current_post_process_depth_filters = filterListToProtoStruct(depthFilters);
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
        if (not newDepthFilters.is_a<viam::sdk::ProtoStruct>()) {
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
    if (not value.is_a<bool>()) {
        VIAM_SDK_LOG(error) << "[do_command] apply_recommended_depth_filters: expected bool, got " << value.kind();
        return viam::sdk::ProtoStruct{{"error", "expected bool"}};
    }
    my_dev->applyEnabledPostProcessDepthFilters = value.get_unchecked<bool>();
    return viam::sdk::ProtoStruct{{command, my_dev->applyEnabledPostProcessDepthFilters}};
}

template <typename DeviceT>
viam::sdk::ProtoStruct getDepthSoftFilter(std::shared_ptr<DeviceT>& device, std::string const& command) {
    return {{command, device->getBoolProperty(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL)}};
}

template <typename DeviceT>
viam::sdk::ProtoStruct setDepthSoftFilter(std::shared_ptr<DeviceT>& device,
                                          viam::sdk::ProtoValue const& value,
                                          std::string const& command) {
    if (not value.template is_a<bool>()) {
        VIAM_SDK_LOG(error) << "[do_command] set_depth_soft_filter: expected bool, got " << value.kind();
        return viam::sdk::ProtoStruct{{"error", "expected bool"}};
    }
    bool const enable = value.template get_unchecked<bool>();
    try {
        if (device->isPropertySupported(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL, OB_PERMISSION_WRITE)) {
            device->setBoolProperty(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL, enable);
            VIAM_SDK_LOG(info) << "[setDepthSoftFilter] " << (enable ? "enabled" : "disabled") << " depth soft filter" << std::endl;
        } else {
            VIAM_SDK_LOG(error) << "[setDepthSoftFilter] OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL is not supported";
        }
    } catch (ob::Error& e) {
        VIAM_SDK_LOG(error) << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what()
                            << "\ntype:" << e.getExceptionType() << std::endl;
        return viam::sdk::ProtoStruct{{"error", e.what()}};
    }

    depth_sensor_control::setDepthSoftFilter(device, value.template get_unchecked<bool>(), command);
    return getDepthSoftFilter(device, command);
}

template <typename DeviceT>
viam::sdk::ProtoStruct getDepthGain(std::shared_ptr<DeviceT>& device) {
    try {
        if (device->isPropertySupported(OB_PROP_DEPTH_GAIN_INT, OB_PERMISSION_READ)) {
            OBIntPropertyRange valueRange = device->getIntPropertyRange(OB_PROP_DEPTH_GAIN_INT);
            VIAM_SDK_LOG(info) << "Depth current gain max:" << valueRange.max << ", min:" << valueRange.min << std::endl;
            int value = device->getIntProperty(OB_PROP_DEPTH_GAIN_INT);
            std::cout << "Depth current gain:" << value << std::endl;
            return {{"current_depth_gain", value}, {"min_depth_gain", valueRange.min}, {"max_depth_gain", valueRange.max}};
        }
        return viam::sdk::ProtoStruct{{"error", "Depth gain property not supported"}};
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "[getDepthGain] " << e.what();
        return viam::sdk::ProtoStruct{{"error", e.what()}};
    }
}

template <typename DeviceT>
viam::sdk::ProtoStruct setDepthGain(std::shared_ptr<DeviceT>& device, viam::sdk::ProtoValue const& value) {
    try {
        if (device->isPropertySupported(OB_PROP_DEPTH_GAIN_INT, OB_PERMISSION_WRITE)) {
            if (!value.is_a<double>()) {
                VIAM_SDK_LOG(error) << "Depth gain value is not a double";
                return viam::sdk::ProtoStruct{{"error", "Depth gain value is not a double"}};
            }
            double new_value = value.get_unchecked<double>();
            OBIntPropertyRange valueRange = device->getIntPropertyRange(OB_PROP_DEPTH_GAIN_INT);
            VIAM_SDK_LOG(info) << "Depth gain range: min=" << valueRange.min << ", max=" << valueRange.max;
            if (new_value < valueRange.min || new_value > valueRange.max) {
                VIAM_SDK_LOG(error) << "Depth gain value out of range: " << new_value;
                return viam::sdk::ProtoStruct{{"error", "Depth gain value out of range"}};
            }
            VIAM_SDK_LOG(info) << "Setting depth gain to: " << new_value;
            device->setIntProperty(OB_PROP_DEPTH_GAIN_INT, (int)new_value);
            return getDepthGain(device);
        }
        return viam::sdk::ProtoStruct{{"error", "Depth gain property not supported"}};
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "Exception in set_depth_gain: " << e.what();
        return viam::sdk::ProtoStruct{{"error", std::string("Exception: ") + e.what()}};
    }
}

template <typename DeviceT>
viam::sdk::ProtoStruct getDepthAutoExposure(std::shared_ptr<DeviceT>& device, std::string const& command) {
    if (device) {
        try {
            if (device->isPropertySupported(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, OB_PERMISSION_READ)) {
                bool value = device->getBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
                return {{command, value}};
            } else {
                return {{"error", "Depth Auto-Exposure switch property is not supported."}};
            }
        } catch (ob::Error& e) {
            std::stringstream error_ss;
            error_ss << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what()
                     << "\ntype:" << e.getExceptionType() << std::endl;
            return {{"error", error_ss.str()}};
        }
    }
}

template <typename DeviceT>
viam::sdk::ProtoStruct setDepthAutoExposure(std::shared_ptr<DeviceT>& device,
                                            viam::sdk::ProtoValue const& value,
                                            std::string const& command) {
    if (not value.template is_a<bool>()) {
        return {{"error", "Invalid value type for Depth Auto-Exposure. Expected boolean."}};
    }
    bool const enable = value.get_unchecked<bool>();
    if (device) {
        try {
            if (device->isPropertySupported(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
                device->setBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, enable);
                return getDepthAutoExposure(device, command);
            } else {
                return {{"error", "Depth Auto-Exposure switch property is not supported."}};
            }
        } catch (ob::Error& e) {
            std::stringstream error_ss;
            error_ss << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what()
                     << "\ntype:" << e.getExceptionType() << std::endl;
            return {{"error", error_ss.str()}};
        }
    }
}

}  // namespace depth_sensor_control
