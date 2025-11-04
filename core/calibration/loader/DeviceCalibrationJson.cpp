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

#include <calibration/loader/DeviceCalibrationJson.h>
#include <calibration/loader/SensorCalibrationJson.h>
#include <data_provider/json_io/JsonHelpers.h>

#include <logging/Checks.h>
#define DEFAULT_LOG_CHANNEL "DeviceCalibrationFactory"
#include <logging/Log.h>

#include <string>

namespace projectaria::tools::calibration {

namespace {
// A helper function to obtain the origin sensor label
// label. If the origin label is not valid, return an empty string.
std::string getOriginSensorLabel(const nlohmann::json& originSpecJson) {
  // Parse in Origin Sensor Label
  const std::string originType = originSpecJson["Type"];

  // First check that origin type is valid
  if (originType != "Custom") {
    XR_LOGW(
        "Origin Specification's Type in calibration.json should be 'Custom' instead of {}",
        originType);
    return "";
  } else {
    return originSpecJson["ChildLabel"];
  }
}

} // namespace

void patchSyntheticHomeCalib(nlohmann::json& json) {
  XR_CHECK(json.contains("DeviceClassInfo"));
  // fix device class info
  if (json["DeviceClassInfo"]["BuildVersion"] == std::string("SimulatedDevice")) {
    json["DeviceClassInfo"]["DeviceClass"] = "Aria";
  } else {
    return; // recording data, no need to patch
  }
  // fix origin specification
  if (json.contains("OriginSpecification")) {
    json["OriginSpecification"]["Type"] = "Custom";
    json["OriginSpecification"]["ChildLabel"] = "imu-left";
  }

  // fix imu label
  if (json.contains("ImuCalibrations")) {
    auto jsonArray = json["ImuCalibrations"];
    XR_CHECK(jsonArray.size() == 1);
    for (auto& imuJson : jsonArray) {
      imuJson["Label"] = "imu-left";
    }
  }
}
// calibration accessor for Aria device
std::optional<DeviceCalibration> deviceCalibrationFromJson(const std::string& calibJsonStr) {
  auto json = nlohmann::json::parse(calibJsonStr.c_str());
  patchSyntheticHomeCalib(json);

  // Parse in Device version and subtype
  std::string deviceSubtype;
  DeviceVersion deviceVersion = DeviceVersion::NotValid;
  if (json.contains("DeviceClassInfo")) {
    deviceVersion = fromDeviceClassName(json["DeviceClassInfo"]["DeviceClass"]);
    deviceSubtype = json["DeviceClassInfo"]["BuildVersion"];
  }

  // Create a Camera Config builder to parse in camera default config information
  CameraConfigBuilder cameraConfigBuilder(deviceVersion);

  std::map<std::string, CameraCalibration> cameraCalibs;
  std::map<std::string, ImuCalibration> imuCalibs;
  {
    if (json.contains("CameraCalibrations")) {
      for (const auto& camJson : json["CameraCalibrations"]) {
        CameraCalibration camCalib = parseCameraCalibrationFromJson(camJson, cameraConfigBuilder);
        auto label = camCalib.getLabel();
        cameraCalibs.emplace(label, std::move(camCalib));
      }
    }
    if (json.contains("ImuCalibrations")) {
      for (const auto& imuJson : json["ImuCalibrations"]) {
        ImuCalibration imuCalib = parseImuCalibrationFromJson(imuJson);
        auto label = imuCalib.getLabel();
        imuCalibs.emplace(label, std::move(imuCalib));
      }
    }
  }

  std::map<std::string, MagnetometerCalibration> magnetometerCalibs;
  std::map<std::string, BarometerCalibration> barometerCalibs;
  std::map<std::string, MicrophoneCalibration> microphoneCalibs;

  if (json.contains("MagCalibrations")) {
    for (const auto& magnetometerJson : json["MagCalibrations"]) {
      MagnetometerCalibration magnetometerCalib =
          parseMagnetometerCalibrationFromJson(magnetometerJson, deviceVersion);
      auto label = magnetometerCalib.getLabel();
      magnetometerCalibs.emplace(label, std::move(magnetometerCalib));
    }
  }
  if (json.contains("BaroCalibrations")) {
    for (const auto& barometerJson : json["BaroCalibrations"]) {
      BarometerCalibration barometerCalib = parseBarometerCalibrationFromJson(barometerJson);
      auto label = barometerCalib.getLabel();
      barometerCalibs.emplace(label, std::move(barometerCalib));
    }
  }
  if (json.contains("MicCalibrations")) {
    for (const auto& microphoneJson : json["MicCalibrations"]) {
      MicrophoneCalibration microphoneCalib = parseMicrophoneCalibrationFromJson(microphoneJson);
      auto label = microphoneCalib.getLabel();
      microphoneCalibs.emplace(label, std::move(microphoneCalib));
    }
  }

  // Check validity of origin label, and only create DeviceCadExtrinsics if it is valid
  const auto originSensorLabel = getOriginSensorLabel(json["OriginSpecification"]);
  DeviceCadExtrinsics deviceCadExtrinsics(deviceVersion, deviceSubtype, originSensorLabel);

  return DeviceCalibration{
      cameraCalibs,
      imuCalibs,
      magnetometerCalibs,
      barometerCalibs,
      microphoneCalibs,
      deviceCadExtrinsics,
      deviceSubtype,
      originSensorLabel,
      deviceVersion};
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
  imuJson["Accelerometer"]["TimeOffsetSec_Device_Accel"] = 0;
  imuJson["Accelerometer"]["Model"]["Name"] = "UpperTriagonalLinear";
  imuJson["Accelerometer"]["Model"]["RectificationMatrix"] =
      json::eigenMatrixToJson(imuCalib.getAccelModel().getRectification());
  imuJson["Accelerometer"]["Bias"]["Offset"] =
      json::eigenVectorToJson(imuCalib.getAccelModel().getBias());
  imuJson["Accelerometer"]["Bias"]["Name"] = "Constant";

  // Gyroscope calibration
  imuJson["Gyroscope"]["Bias"]["Name"] = "Constant";

  imuJson["Gyroscope"]["Bias"]["Offset"] =
      json::eigenVectorToJson(imuCalib.getGyroModel().getBias());
  imuJson["Gyroscope"]["Model"]["RectificationMatrix"] =
      json::eigenMatrixToJson(imuCalib.getGyroModel().getRectification());
  imuJson["Gyroscope"]["Model"]["Name"] = "LinearGSensitivity";
  auto zeroMat = Eigen::Matrix<double, 3, 3>();
  zeroMat.setZero();
  imuJson["Gyroscope"]["Model"]["GSensitivityMatrix"] = json::eigenMatrixToJson(zeroMat);
  imuJson["Gyroscope"]["TimeOffsetSec_Device_Gyro"] = 0;

  return imuJson;
}

// Helper function to serialize magnetometer calibration to JSON
nlohmann::json magnetometerCalibrationToJson(
    const MagnetometerCalibration& magCalib,
    const DeviceVersion& deviceVersion) {
  nlohmann::json magJson;

  magJson["Label"] = magCalib.getLabel();

  const auto& magMat = magCalib.getModel().getRectification();
  const auto& biasInTesla = magCalib.getModel().getBias();

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

  Eigen::Matrix3d magMatForJson = -magMat.inverse() * rectificationMatrixScale;
  Eigen::Vector3d biasForJson = biasInTesla / biasScale;

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

// Main function to serialize DeviceCalibration to JSON string
std::string deviceCalibrationToJson(const DeviceCalibration& deviceCalib) {
  nlohmann::json json;

  json["AlgorithmName"] = "RealCalib";
  json["AlgorithmVersion"] = "1.0 Beta";
  json["CalibrationSource"] = "Unknown";
  json["Serial"] = ""; // TODO add serial number in DeviceCalibration

  // Device class information
  json["DeviceClassInfo"]["BuildVersion"] = deviceCalib.getDeviceSubtype();
  json["DeviceClassInfo"]["DeviceClass"] = deviceVersionToString(deviceCalib.getDeviceVersion());

  // Origin specification
  json["OriginSpecification"]["Type"] = "Custom";
  json["OriginSpecification"]["ChildLabel"] = deviceCalib.getOriginLabel();

  // Camera calibrations
  if (!deviceCalib.getCameraCalibs().empty()) {
    json["CameraCalibrations"] = nlohmann::json::array();
    for (const auto& [label, camCalib] : deviceCalib.getCameraCalibs()) {
      json["CameraCalibrations"].push_back(cameraCalibrationToJson(camCalib));
    }
  }

  // IMU calibrations
  if (!deviceCalib.getImuCalibs().empty()) {
    json["ImuCalibrations"] = nlohmann::json::array();
    for (const auto& [label, imuCalib] : deviceCalib.getImuCalibs()) {
      json["ImuCalibrations"].push_back(imuCalibrationToJson(imuCalib));
    }
  }

  // Magnetometer calibrations
  if (!deviceCalib.getMagnetometerCalibs().empty()) {
    json["MagCalibrations"] = nlohmann::json::array();
    for (const auto& [label, magCalib] : deviceCalib.getMagnetometerCalibs()) {
      json["MagCalibrations"].push_back(
          magnetometerCalibrationToJson(magCalib, deviceCalib.getDeviceVersion()));
    }
  }

  // Barometer calibrations
  if (!deviceCalib.getBarometerCalibs().empty()) {
    json["BaroCalibrations"] = nlohmann::json::array();
    for (const auto& [label, baroCalib] : deviceCalib.getBarometerCalibs()) {
      json["BaroCalibrations"].push_back(barometerCalibrationToJson(baroCalib));
    }
  }

  // Microphone calibrations
  if (!deviceCalib.getMicrophoneCalibs().empty()) {
    json["MicCalibrations"] = nlohmann::json::array();
    for (const auto& [label, micCalib] : deviceCalib.getMicrophoneCalibs()) {
      json["MicCalibrations"].push_back(microphoneCalibrationToJson(micCalib));
    }
  }

  // Return formatted JSON string
  return json.dump(2); // Pretty print with 2 spaces indentation
}

} // namespace projectaria::tools::calibration
