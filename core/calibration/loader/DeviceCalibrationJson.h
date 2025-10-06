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

#include <optional>
#include <string>

#include <calibration/DeviceCalibration.h>

namespace projectaria::tools::calibration {

std::optional<DeviceCalibration> deviceCalibrationFromJson(const std::string& calibJsonStr);

std::string deviceCalibrationToJson(const DeviceCalibration& deviceCalibration);

} // namespace projectaria::tools::calibration
