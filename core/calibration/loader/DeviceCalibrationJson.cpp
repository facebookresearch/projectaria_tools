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

#include <logging/Checks.h>
#define DEFAULT_LOG_CHANNEL "DeviceCalibrationFactory"
#include <logging/Log.h>

#include <string>

namespace projectaria::tools::calibration {

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

  std::string deviceSubtype;
  if (json.contains("DeviceClassInfo")) {
    deviceSubtype = json["DeviceClassInfo"]["BuildVersion"];
  }

  // Check if dataset is ASE simulated
  bool aseSimulated = false;
  if (json.contains("AlgorithmName") && json["AlgorithmName"] == "UnrealSim") {
    aseSimulated = true;
  }

  std::map<std::string, CameraCalibration> cameraCalibs;
  std::map<std::string, ImuCalibration> imuCalibs;
  {
    if (json.contains("CameraCalibrations")) {
      for (const auto& camJson : json["CameraCalibrations"]) {
        CameraCalibration camCalib = parseCameraCalibrationFromJson(camJson, aseSimulated);
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
          parseMagnetometerCalibrationFromJson(magnetometerJson);
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

  std::string originLabel;
  // Parse in Origin Sensor Label
  if (json.contains("OriginSpecification")) {
    const std::string originType = json["OriginSpecification"]["Type"];
    if (originType == "Custom") {
      originLabel = json["OriginSpecification"]["ChildLabel"];
    } else {
      XR_LOGE(
          "Origin Specification's Type in calibration.json should be 'Custom' instead of {}",
          originType);
    }
  }

  DeviceCadExtrinsics deviceCadExtrinsics(deviceSubtype, originLabel);

  return DeviceCalibration{
      cameraCalibs,
      imuCalibs,
      magnetometerCalibs,
      barometerCalibs,
      microphoneCalibs,
      deviceCadExtrinsics,
      deviceSubtype,
      originLabel};
}
} // namespace projectaria::tools::calibration
