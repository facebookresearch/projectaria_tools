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
  CameraProjection::ModelType modelName{};
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

  // load saved config, if available - if not, then defaults will be loaded
  std::optional<CameraConfigData> maybeConfig;
  if (json.contains("ConfigData")) {
    auto jsonConfig = json["ConfigData"];
    maybeConfig = CameraConfigData{
        .imageWidth = jsonConfig["ImageWidth"],
        .imageHeight = jsonConfig["ImageHeight"],
        .maxSolidAngle = jsonConfig["MaxSolidAngle"],
    };
    if (jsonConfig.contains("ValidRadius")) {
      maybeConfig->maybeValidRadius = static_cast<double>(jsonConfig["ValidRadius"]);
    }
  } else { // Obtain default camera config from builder
    maybeConfig = configBuilder.getCameraConfigData(label);
    if (!maybeConfig.has_value()) {
      throw std::runtime_error(
          fmt::format("No config found for camera {} from Config builder", label));
    }
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
  const double timeOffsetSecDeviceAccel =
      static_cast<double>(json["Accelerometer"]["TimeOffsetSec_Device_Accel"]);
  const double timeOffsetSecDeviceGyro =
      static_cast<double>(json["Gyroscope"]["TimeOffsetSec_Device_Gyro"]);
  const auto T_Device_Imu = se3FromJson<double>(json["T_Device_Imu"]);

  return ImuCalibration(
      label,
      accelMat,
      accelBias,
      gyroMat,
      gyroBias,
      T_Device_Imu,
      timeOffsetSecDeviceAccel,
      timeOffsetSecDeviceGyro);
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
  double rectificationMatrixScale = getMagRectificationMatrixScaleJsonToUnity(deviceVersion);
  // 3. `biasFromJson`: Gen1: muT -> T, Gen2: Gauss -> T.
  double biasScale = getMagBiasScaleJsonToTesla(deviceVersion);

  // We also intentionally flip the sign to compensate for a factory convention difference.
  auto magMat = -magMatFromJson.inverse() * rectificationMatrixScale;

  auto biasInTesla = biasFromJson * biasScale;

  if (!(magMat.determinant() > 0)) {
    throw std::runtime_error("Magnetometer calibration matrix should have >0 determinant.");
  }

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

// Helper function to convert DeviceVersion enum to string
std::string deviceVersionToString(const DeviceVersion& version) {
  switch (version) {
    case DeviceVersion::Gen1:
      return "Aria";
    case DeviceVersion::Gen2:
      return "Oatmeal";
    case DeviceVersion::NotValid:
    default:
      return "NotValid";
  }
}

// Helper function to convert CameraProjection::ModelType to string
std::string projectionModelToString(const CameraProjection::ModelType& modelType) {
  switch (modelType) {
    case CameraProjection::ModelType::Fisheye624:
      return "FisheyeRadTanThinPrism";
    case CameraProjection::ModelType::KannalaBrandtK3:
      return "KannalaBrandtK3";
    case CameraProjection::ModelType::Fisheye62:
      return "Fisheye62";
    default:
      return "Unknown";
  }
}

std::string projectionModuleToDescriptionString(const CameraProjection::ModelType& modelType) {
  switch (modelType) {
    case CameraProjection::ModelType::Fisheye624:
      return "fx, cx, cy, k_0, k_1, k_2, k_3, k_4, k_5, p_0, p_1, s_0, s_1, s_2, s_3";
    case CameraProjection::ModelType::KannalaBrandtK3:
      return "fx, fy, cx, cy, kb0, kb1, kb2, kb3";
    case CameraProjection::ModelType::Fisheye62:
      return "see Fisheye62.h";
    default:
      return "Unknown";
  }
}

// Helper function to serialize camera calibration to JSON
nlohmann::json cameraCalibrationToJson(const CameraCalibration& camCalib) {
  nlohmann::json camJson;

  camJson["Calibrated"] = true;
  camJson["Label"] = camCalib.getLabel();
  camJson["SerialNumber"] = camCalib.getSerialNumber();
  camJson["T_Device_Camera"] = json::se3ToJson(camCalib.getT_Device_Camera());

  // Projection parameters
  camJson["Projection"]["Description"] = projectionModuleToDescriptionString(camCalib.modelName());
  camJson["Projection"]["Name"] = projectionModelToString(camCalib.modelName());
  camJson["Projection"]["Params"] = json::eigenVectorToJson(camCalib.projectionParams());

  // Time offset if non-zero
  if (camCalib.getTimeOffsetSecDeviceCamera() != 0.0) {
    camJson["TimeOffsetSec_Device_Camera"] = camCalib.getTimeOffsetSecDeviceCamera();
  }

  // Store config, so we can correctly load an already configured camera without re-applying
  auto& jsonConfig = camJson["ConfigData"];
  jsonConfig["ImageWidth"] = camCalib.getImageSize().x();
  jsonConfig["ImageHeight"] = camCalib.getImageSize().y();
  jsonConfig["MaxSolidAngle"] = camCalib.getMaxSolidAngle();
  if (camCalib.getValidRadius().has_value()) {
    jsonConfig["ValidRadius"] = *camCalib.getValidRadius();
  }

  return camJson;
}

// Helper function to serialize IMU calibration to JSON
nlohmann::json imuCalibrationToJson(const ImuCalibration& imuCalib) {
  nlohmann::json imuJson;

  imuJson["Label"] = imuCalib.getLabel();
  imuJson["SerialNumber"] = "";
  imuJson["Calibrated"] = true;
  imuJson["T_Device_Imu"] = json::se3ToJson(imuCalib.getT_Device_Imu());

  // Accelerometer calibration
  imuJson["Accelerometer"]["Model"]["Name"] = "UpperTriagonalLinear";
  imuJson["Accelerometer"]["Model"]["RectificationMatrix"] =
      json::eigenMatrixToJson(imuCalib.getAccelModel().getRectification());
  imuJson["Accelerometer"]["Bias"]["Offset"] =
      json::eigenVectorToJson(imuCalib.getAccelModel().getBias());
  imuJson["Accelerometer"]["Bias"]["Name"] = "Constant";
  imuJson["Accelerometer"]["TimeOffsetSec_Device_Accel"] = imuCalib.getTimeOffsetSecDeviceAccel();

  // Gyroscope calibration
  imuJson["Gyroscope"]["Bias"]["Name"] = "Constant";

  imuJson["Gyroscope"]["Bias"]["Offset"] =
      json::eigenVectorToJson(imuCalib.getGyroModel().getBias());
  imuJson["Gyroscope"]["Model"]["Name"] = "Linear";
  imuJson["Gyroscope"]["Model"]["RectificationMatrix"] =
      json::eigenMatrixToJson(imuCalib.getGyroModel().getRectification());
  imuJson["Gyroscope"]["TimeOffsetSec_Device_Gyro"] = imuCalib.getTimeOffsetSecDeviceGyro();

  return imuJson;
}

double getMagRectificationMatrixScaleJsonToUnity(const DeviceVersion& deviceVersion) {
  return deviceVersion == DeviceVersion::Gen1 ? 1e-6 : 1.0;
}

double getMagBiasScaleJsonToTesla(const DeviceVersion& deviceVersion) {
  return deviceVersion == DeviceVersion::Gen1 ? 1e-6 : 1e-4;
}

// Helper function to serialize magnetometer calibration to JSON
nlohmann::json magnetometerCalibrationToJson(
    const MagnetometerCalibration& magCalib,
    const DeviceVersion& deviceVersion) {
  nlohmann::json magJson;

  magJson["Label"] = magCalib.getLabel();

  const auto& magMat = magCalib.getModel().getRectification();
  const auto& biasInTesla = magCalib.getModel().getBias();

  Eigen::Matrix3d magMatForJson =
      -magMat.inverse() * getMagRectificationMatrixScaleJsonToUnity(deviceVersion);
  Eigen::Vector3d biasForJson = biasInTesla / getMagBiasScaleJsonToTesla(deviceVersion);

  magJson["SerialNumber"] = "";
  magJson["Model"]["Name"] = "Linear";
  magJson["Model"]["RectificationMatrix"] = json::eigenMatrixToJson(magMatForJson);
  magJson["Bias"]["Offset"] = json::eigenVectorToJson(biasForJson);
  magJson["Bias"]["Name"] = "Constant";

  return magJson;
}

// Helper function to serialize barometer calibration to JSON
nlohmann::json barometerCalibrationToJson(const BarometerCalibration& baroCalib) {
  nlohmann::json baroJson;

  baroJson["Label"] = baroCalib.getLabel();
  baroJson["PressureModel"]["Name"] = "Linear";
  baroJson["PressureModel"]["Slope"] = baroCalib.getSlope();
  baroJson["PressureModel"]["OffsetPa"] = baroCalib.getOffsetPa();
  baroJson["SerialNumber"] = "";

  return baroJson;
}

// Helper function to serialize microphone calibration to JSON
nlohmann::json microphoneCalibrationToJson(const MicrophoneCalibration& micCalib) {
  nlohmann::json micJson;

  micJson["Label"] = micCalib.getLabel();
  micJson["DSensitivity1KDbv"] = micCalib.getDSensitivity1KDbv();
  micJson["SerialNumber"] = "";

  return micJson;
}
} // namespace projectaria::tools::calibration
