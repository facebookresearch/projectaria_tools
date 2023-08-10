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

#include <calibration/SensorCalibration.h>
#define RAPIDJSON_NAMESPACE rapidjson
#define RAPIDJSON_HAS_STDSTRING 1
#include <rapidjson/document.h>

namespace projectaria::tools::calibration {

CameraCalibration parseCameraCalibrationFromJson(const rapidjson::Value& json);
ImuCalibration parseImuCalibrationFromJson(const rapidjson::Value& json);
MagnetometerCalibration parseMagnetometerCalibrationFromJson(const rapidjson::Value& json);
BarometerCalibration parseBarometerCalibrationFromJson(const rapidjson::Value& json);
MicrophoneCalibration parseMicrophoneCalibrationFromJson(const rapidjson::Value& json);

} // namespace projectaria::tools::calibration
