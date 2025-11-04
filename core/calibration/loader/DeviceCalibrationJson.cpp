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
