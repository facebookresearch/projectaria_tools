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

#include <calibration/loader/SensorCalibrationJson.h>

#include <calibration/SensorCalibration.h>
#include <data_provider/json_io/JsonHelpers.h>

#include <logging/Checks.h>
#define DEFAULT_LOG_CHANNEL "SensorCalibrationJson"
#include <logging/Log.h>

#include <stdexcept>

using namespace projectaria::tools::json;

namespace projectaria::tools::calibration {

CameraCalibration parseCameraCalibrationFromJson(
    const nlohmann::json& json,
    const CameraConfigBuilder& configBuilder) {
  // Parse projection params
  const std::string projectionModelName = json["Projection"]["Name"];
  CameraProjection::ModelType modelName;
  if (projectionModelName == "FisheyeRadTanThinPrism") {
    modelName = CameraProjection::ModelType::Fisheye624;
  } else if (projectionModelName == "KannalaBrandtK3") {
    modelName = CameraProjection::ModelType::KannalaBrandtK3;
  } else if (projectionModelName == "Fisheye62") {
    modelName = CameraProjection::ModelType::Fisheye62;
  }

  Eigen::VectorXd projectionParams =
      json::eigenVectorFromJson<double>(json["Projection"]["Params"]);

  const std::string label = json["Label"];
  const std::string serialNumber = json["SerialNumber"];
  const auto T_Device_Camera = se3FromJson<double>(json["T_Device_Camera"]);
  const double timeOffsetSecDeviceCamera = json.contains("TimeOffsetSec_Device_Camera")
      ? static_cast<double>(json["TimeOffsetSec_Device_Camera"])
      : 0.0;

  // Obtain camera config from builder
  const auto maybeConfig = configBuilder.getCameraConfigData(label);
  if (!maybeConfig.has_value()) {
    throw std::runtime_error(
        fmt::format("No config found for camera {} from Config builder", label));
  }

  CameraCalibration camCalib(
      label,
      modelName,
      projectionParams,
      T_Device_Camera,
      maybeConfig->imageWidth,
      maybeConfig->imageHeight,
      maybeConfig->maybeValidRadius,
      maybeConfig->maxSolidAngle,
      serialNumber,
      timeOffsetSecDeviceCamera);
  return camCalib;
}

namespace {
std::pair<Eigen::Matrix3d, Eigen::Vector3d> parseRectModelFromJson(const nlohmann::json& json) {
  return {
      eigenMatrixFromJson<double, 3, 3>(json["Model"]["RectificationMatrix"]),
      eigenVectorFromJson<double, 3>(json["Bias"]["Offset"])};
}
} // namespace

ImuCalibration parseImuCalibrationFromJson(const nlohmann::json& json) {
  const auto& label = json["Label"];
  const auto [accelMat, accelBias] = parseRectModelFromJson(json["Accelerometer"]);
  const auto [gyroMat, gyroBias] = parseRectModelFromJson(json["Gyroscope"]);
  const double timeOffsetSecDeviceAccel = static_cast<double>(json["Accelerometer"]["TimeOffsetSec_Device_Accel"]);
  const double timeOffsetSecDeviceGyro = static_cast<double>(json["Gyroscope"]["TimeOffsetSec_Device_Gyro"]);
  const auto T_Device_Imu = se3FromJson<double>(json["T_Device_Imu"]);

  return ImuCalibration(label, accelMat, accelBias, gyroMat, gyroBias, T_Device_Imu,
      timeOffsetSecDeviceAccel, timeOffsetSecDeviceGyro);
}

MagnetometerCalibration parseMagnetometerCalibrationFromJson(
    const nlohmann::json& json,
    const DeviceVersion& deviceVersion) {
  const auto& label = json["Label"];
  const auto [magMatFromJson, biasFromJson] = parseRectModelFromJson(json);

  // For Gen1 device, in factory calibration json:
  //    `rectified_in_T = magMatFromJson * (raw_in_uT - bias_in_uT). `.
  // We want to align to the following to match IMU convention:
  //    `rectified_in_T = magMat.inv() * (raw_in_T - bias_in_T)`.
  // Therefore we need to do some patches as follows:
  // 1. Note that `raw_in_uT` has been transformed to `raw_in_T` in
  // ${PROJECT}/core/data_provider/RecordReaderInterface.cpp
  //
  // 2. `magMatFromJson`: Gen1: muT -> T, Gen2: T -> T.
  double rectificationMatrixScale = deviceVersion == DeviceVersion::Gen1 ? 1e-6 : 1.0;
  // 3. `biasFromJson`: Gen1: muT -> T, Gen2: Gauss -> T.
  double biasScale = deviceVersion == DeviceVersion::Gen1 ? 1e-6 : 1e-4;

  // We also intentionally flip the sign to compensate for a factory convention difference.
  auto magMat = -magMatFromJson.inverse() * rectificationMatrixScale;

  auto biasInTesla = biasFromJson * biasScale;

  XR_CHECK(magMat.determinant() > 0, "Magnetometer calibration matrix should have >0 determinant.");

  return {label, magMat, biasInTesla};
}

BarometerCalibration parseBarometerCalibrationFromJson(const nlohmann::json& json) {
  const auto& label = json["Label"];
  double slope = json["PressureModel"]["Slope"];
  double offsetPa = json["PressureModel"]["OffsetPa"];

  return BarometerCalibration(label, slope, offsetPa);
}

MicrophoneCalibration parseMicrophoneCalibrationFromJson(const nlohmann::json& json) {
  const auto& label = json["Label"];
  const double dSensitivity1KDbv = json["DSensitivity1KDbv"];
  return MicrophoneCalibration(label, dSensitivity1KDbv);
}
} // namespace projectaria::tools::calibration
