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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <libobsensor/ObSensor.hpp>

#include <viam/sdk/log/logging.hpp>

namespace orbbec {

struct ViamOBDevice;

namespace firmware {

// Callback for writing downloaded data to buffer
size_t writeFileCallback(void* contents, size_t size, size_t nmemb, void* userp);

// Main firmware update function
void updateFirmware(std::unique_ptr<ViamOBDevice>& my_dev,
                    std::shared_ptr<ob::Context> ctx,
                    const std::string& firmware_url,
                    viam::sdk::LogSource& logger);

}  // namespace firmware
}  // namespace orbbec
