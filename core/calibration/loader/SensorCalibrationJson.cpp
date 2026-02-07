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

#include <logging/Checks.h>
#define DEFAULT_LOG_CHANNEL "SensorCalibrationJson"
#include <logging/Log.h>

#include <stdexcept>

namespace projectaria::tools::calibration {
namespace {

// Circular mask radius value for full resolution RGB and SLAM cameras
const double kSlamValidRadius = 330;
const double kRgbValidRadius = 1415;

// Relaxed thresholds for max solid angle within which hopefully the camera model remains monotonic
const double kSlamMaxSolidAngleRad = 1.4;
const double kRgbMaxSolidAngleRad = 1;
const double kEtMaxSolidAngleRad = 0.75;

Eigen::VectorXd parseVectorXdFromJson(const nlohmann::json& json) {
  Eigen::VectorXd vec(json.size());
  for (size_t i = 0; i < json.size(); ++i) {
    vec(i) = json[i];
  }
  return vec;
}

Eigen::Vector3d parseVector3dFromJson(const nlohmann::json& json) {
  XR_CHECK(json.size() == 3, "Expects a 3d vector from json, actual size: {}", json.size());

  return Eigen::Vector3d(json[0], json[1], json[2]);
}

Eigen::Matrix3d parseMatrix3dFromJson(const nlohmann::json& json) {
  XR_CHECK(json.size() == 3, "Expects 3 rows from matrix, actual number of rows: {}", json.size());

  Eigen::Matrix3d mat;
  for (size_t i = 0; i < 3; ++i) {
    mat.row(i) = parseVector3dFromJson(json[i]).transpose().eval();
  }
  return mat;
}

Sophus::SE3d parseSe3dFromJson(const nlohmann::json& json) {
  Eigen::Vector3d translation = parseVector3dFromJson(json["Translation"]);

  XR_CHECK(
      json["UnitQuaternion"].size() == 2,
      "Expects UnitQuaternion to have two components, actual size: {}",
      json.size());
  double qReal = json["UnitQuaternion"][0];
  Eigen::Vector3d qImag = parseVector3dFromJson(json["UnitQuaternion"][1]);

  Eigen::Quaterniond rotation(qReal, qImag.x(), qImag.y(), qImag.z());
  return {rotation, translation};
}
} // namespace

CameraCalibration parseCameraCalibrationFromJson(const nlohmann::json& json) {
  // Parse projection params
  const std::string projectionModelName = json["Projection"]["Name"];
  CameraProjection::ModelType modelName{};
  if (projectionModelName == "FisheyeRadTanThinPrism") {
    modelName = CameraProjection::ModelType::Fisheye624;
  } else if (projectionModelName == "KannalaBrandtK3") {
    modelName = CameraProjection::ModelType::KannalaBrandtK3;
  } else if (projectionModelName == "Fisheye62") {
    modelName = CameraProjection::ModelType::Fisheye62;
  }
  Eigen::VectorXd projectionParams = parseVectorXdFromJson(json["Projection"]["Params"]);

  const std::string label = json["Label"];
  const std::string serialNumber = json["SerialNumber"];
  const auto T_Device_Camera = parseSe3dFromJson(json["T_Device_Camera"]);
  const double timeOffsetSecDeviceCamera = json.contains("TimeOffsetSec_Device_Camera")
      ? static_cast<double>(json["TimeOffsetSec_Device_Camera"])
      : 0.0;

  std::optional<double> validRadius;
  int width = 0;
  int height = 0;
  double maxSolidAngle = 0.0;
  // Handle sensor valid radius and camera resolution (full res during calibration)
  if (label == "camera-rgb") {
    validRadius = kRgbValidRadius;
    width = 2880;
    height = 2880;
    maxSolidAngle = kRgbMaxSolidAngleRad;
  } else if (label == "camera-slam-left" || label == "camera-slam-right") {
    validRadius = kSlamValidRadius;
    width = 640;
    height = 480;
    maxSolidAngle = kSlamMaxSolidAngleRad;
  } else if (label == "camera-et-left" || label == "camera-et-right") {
    width = 640;
    height = 480;
    maxSolidAngle = kEtMaxSolidAngleRad;
  } else {
    const std::string error = fmt::format("Unrecognized camera label for Aria: {}", label);
    XR_LOGE("{}", error);
    throw std::runtime_error{error};
  }

  CameraCalibration camCalib(
      label,
      modelName,
      projectionParams,
      T_Device_Camera,
      width,
      height,
      validRadius,
      maxSolidAngle,
      serialNumber,
      timeOffsetSecDeviceCamera);
  return camCalib;
}

namespace {
std::pair<Eigen::Matrix3d, Eigen::Vector3d> parseRectModelFromJson(const nlohmann::json& json) {
  return {
      parseMatrix3dFromJson(json["Model"]["RectificationMatrix"]),
      parseVector3dFromJson(json["Bias"]["Offset"])};
}
} // namespace

ImuCalibration parseImuCalibrationFromJson(const nlohmann::json& json) {
  const auto& label = json["Label"];
  const auto [accelMat, accelBias] = parseRectModelFromJson(json["Accelerometer"]);
  const auto [gyroMat, gyroBias] = parseRectModelFromJson(json["Gyroscope"]);
  const auto T_Device_Imu = parseSe3dFromJson(json["T_Device_Imu"]);

  return ImuCalibration(label, accelMat, accelBias, gyroMat, gyroBias, T_Device_Imu);
}

MagnetometerCalibration parseMagnetometerCalibrationFromJson(const nlohmann::json& json) {
  const auto& label = json["Label"];
  const auto [magMatFromJson, biasInMicroTesla] = parseRectModelFromJson(json);

  // In factory calibration json:
  //    `rectified_in_T = magMatFromJson * (raw_in_uT - bias_in_uT). `.
  // We want to align to the following to match IMU convention:
  //    `rectified_in_T = magMat.inv() * (raw_in_T - bias_in_T)`.
  // Therefore we need to do some patches as follows:

  // 1. Note that `raw_in_uT` has been transformed to `raw_in_T` in
  // ${PROJECT}/core/data_provider/RecordReaderInterface.cpp

  // 2. `magMat` is transformed as:
  auto magMat = (-magMatFromJson.inverse() * 1e-6); /* where the extra `-` sign is an intentional
  patch to correct a sign error in factory calibration process. */

  // 3. `bias` is transformed as:
  auto biasInTesla = biasInMicroTesla * 1e-6;

  return MagnetometerCalibration(label, magMat, biasInTesla);
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
