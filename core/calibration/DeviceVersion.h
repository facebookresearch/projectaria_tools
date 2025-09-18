/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <string>

namespace projectaria::tools::calibration {

/**
 * @brief Enum class for different versions of Aria glasses
 */
enum class DeviceVersion {
  NotValid,
  Gen1,
  Gen2,
};

/** @brief converts the enum to readable std::string */
std::string getName(DeviceVersion deviceVersion);

/** @brief get enum from string stored in calibration */
DeviceVersion fromDeviceClassName(const std::string& name);

} // namespace projectaria::tools::calibration
