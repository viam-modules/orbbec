// Copyright 2025 Viam Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "orbbec_windows_registry.hpp"

#ifdef _WIN32
#include <windows.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <libobsensor/ObSensor.hpp>

namespace orbbec {
namespace windows_registry {

std::string uint16ToHex(uint16_t value) {
    std::stringstream ss;
    ss << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << value;
    return ss.str();
}

void setupWindowsDeviceRegistry(std::shared_ptr<ob::Device> device) {
    // On windows, we must add a metadata value to the windows device registry for the device to work correctly.
    // Adapted from the orbbec SDK setup script:
    // https://github.com/orbbec/OrbbecSDK_v2/blob/main/scripts/env_setup/obsensor_metadata_win10.ps1
    try {
        const char* command = "powershell -Command \"Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser -Force\"";
        int result = std::system(command);
        if (result != 0) {
            // command failed, try the backup
            command = "powershell -Command \"Set-ExecutionPolicy -ExecutionPolicy Unrestricted -Scope CurrentUser -Force\"";
            int result = std::system(command);
            if (result != 0) {
                VIAM_SDK_LOG(error) << "Could not set execution policy";
            }
        }

        // Base registry paths
        std::vector<std::string> searchTrees = {
            // KSCATEGORY_CAPTURE class,used for video capture devices
            "SYSTEM\\CurrentControlSet\\Control\\DeviceClasses\\{e5323777-f976-4f5b-9b55-b94699c46e44}",
            // KSCATEGORY_RENDER class, used for rendering devices
            "SYSTEM\\CurrentControlSet\\Control\\DeviceClasses\\{65E8773D-8F56-11D0-A3B9-00A0C9223196}"};

        uint16_t vid = device->getDeviceInfo()->vid();
        uint16_t pid = device->getDeviceInfo()->pid();
        std::string baseDeviceId = "##?#USB#VID_" + uint16ToHex(vid) + "&PID_" + uint16ToHex(pid);

        for (const auto& subtree : searchTrees) {
            // Open the device registry key
            HKEY hkey;
            if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, subtree.c_str(), 0, KEY_ENUMERATE_SUB_KEYS, &hkey) != ERROR_SUCCESS) {
                VIAM_SDK_LOG(error) << "Could not open device registry key: " << subtree;
                continue;
            }

            char name[512];
            DWORD nameSize;
            DWORD index = 0;
            while (true) {
                // reset before each call
                nameSize = sizeof(name);

                // enumerate all of the keys in the devices folder we previously opened
                LONG result = RegEnumKeyExA(hkey, index, name, &nameSize, NULL, NULL, NULL, NULL);
                if (result == ERROR_NO_MORE_ITEMS) {
                    break;  // normal end of enumeration
                } else if (result != ERROR_SUCCESS) {
                    VIAM_SDK_LOG(error) << "device registry key enumeration failed: " << result;
                    break;
                }
                std::string subKeyName(name);
                // find the enteries for our orbbec device.
                if (subKeyName.find("USB#VID_" + uint16ToHex(vid) + "&PID_" + uint16ToHex(pid)) == std::string::npos) {
                    // not a match for an orbbec device, go to next key
                    ++index;
                    continue;
                }

                std::string deviceParamsKey = subtree + "\\" + subKeyName + "\\#GLOBAL\\Device Parameters";
                std::string valueName = "MetadataBufferSizeInKB0";
                HKEY hDeviceKey;
                // open the orbbec device parameters key
                result = RegOpenKeyExA(HKEY_LOCAL_MACHINE, deviceParamsKey.c_str(), 0, KEY_READ | KEY_SET_VALUE, &hDeviceKey);
                if (result != ERROR_SUCCESS) {
                    VIAM_SDK_LOG(error) << "Couldn't open windows registry device parameters key: " << deviceParamsKey;
                    ++index;
                    continue;
                }
                // check if the metadata value already exists
                DWORD data;
                DWORD dataSize = sizeof(data);
                result = RegQueryValueExA(hDeviceKey, valueName.c_str(), nullptr, nullptr, reinterpret_cast<BYTE*>(&data), &dataSize);
                if (result == ERROR_FILE_NOT_FOUND) {
                    // value does not exist yet, create it.
                    DWORD value = 5;
                    result =
                        RegSetValueExA(hDeviceKey, valueName.c_str(), 0, REG_DWORD, reinterpret_cast<const BYTE*>(&value), sizeof(value));
                    if (result != ERROR_SUCCESS) {
                        VIAM_SDK_LOG(error) << "Couldn't set metadata registry value for key " << deviceParamsKey;
                    } else {
                        VIAM_SDK_LOG(info) << "Created value " << valueName << " = " << value << " for key " << deviceParamsKey;
                    }
                } else if (result == ERROR_SUCCESS) {
                    VIAM_SDK_LOG(info) << "Value already exists on key " << deviceParamsKey << ", skipping.";
                } else {
                    VIAM_SDK_LOG(error) << "Error reading metadata value for key " << deviceParamsKey << ": " << result;
                }
                RegCloseKey(hDeviceKey);
                ++index;
            }
            RegCloseKey(hkey);
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to update windows device registry keys: " + std::string(e.what()));
    }
}

} // namespace windows_registry
} // namespace orbbec

#endif // _WIN32
