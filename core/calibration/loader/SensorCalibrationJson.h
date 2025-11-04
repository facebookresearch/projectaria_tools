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

// For Gen1 device, in factory calibration json:
//    `rectified_in_T = magMatFromJson * (raw_in_uT - bias_in_uT). `.
// We want to align to the following to match IMU convention:
//    `rectified_in_T = magMat.inv() * (raw_in_T - bias_in_T)`.
// Therefore we need to do some patches as follows:
// 1. Note that `raw_in_uT` has been transformed to `raw_in_T` in
// ${PROJECT}/core/data_provider/RecordReaderInterface.cpp
//
// `magMatFromJson`: Gen1: muT -> T, Gen2: T -> T.
// NOTE: the scale factor read from Json and saved to json are the same. Here is the math
// magMatForJson = -magMat.inverse() * scaleMatToJson (taking inverse on each side and multiply by
// -1)
// (-magMatForJson.inverse()) = matMat * (1.0 / scaleMatToJson)
// -magMatForJson.inverse() * scaleMatToJson = matMat
double getMagRectificationMatrixScaleJsonToUnity(const DeviceVersion& deviceVersion);
// `biasFromJson`: Gen1: muT -> T, Gen2: Gauss -> T.
double getMagBiasScaleJsonToTesla(const DeviceVersion& deviceVersion);

// Helper function to convert DeviceVersion enum to string
std::string deviceVersionToString(const DeviceVersion& version);

// Helper function to convert CameraProjection::ModelType to string
std::string projectionModelToString(const CameraProjection::ModelType& modelType);

std::string projectionModuleToDescriptionString(const CameraProjection::ModelType& modelType);

// Helper function to serialize camera calibration to JSON
nlohmann::json cameraCalibrationToJson(const CameraCalibration& camCalib);
// Helper function to serialize IMU calibration to JSON
nlohmann::json imuCalibrationToJson(const ImuCalibration& imuCalib);

// Helper function to serialize magnetometer calibration to JSON
nlohmann::json magnetometerCalibrationToJson(
    const MagnetometerCalibration& magCalib,
    const DeviceVersion& deviceVersion);

// Helper function to serialize barometer calibration to JSON
nlohmann::json barometerCalibrationToJson(const BarometerCalibration& baroCalib);
// Helper function to serialize microphone calibration to JSON
nlohmann::json microphoneCalibrationToJson(const MicrophoneCalibration& micCalib);
} // namespace projectaria::tools::calibration
