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

#include <calibration/CameraConfigBuilder.h>
#include <calibration/SensorCalibration.h>
#include "nlohmann/json.hpp"

namespace projectaria::tools::calibration {

// Default to Gen1 builder to be backward compatible
CameraCalibration parseCameraCalibrationFromJson(
    const nlohmann::json& json,
    const CameraConfigBuilder& configBuilder);
ImuCalibration parseImuCalibrationFromJson(const nlohmann::json& json);
MagnetometerCalibration parseMagnetometerCalibrationFromJson(
    const nlohmann::json& json,
    const DeviceVersion& deviceVersion);
BarometerCalibration parseBarometerCalibrationFromJson(const nlohmann::json& json);
MicrophoneCalibration parseMicrophoneCalibrationFromJson(const nlohmann::json& json);

} // namespace projectaria::tools::calibration
