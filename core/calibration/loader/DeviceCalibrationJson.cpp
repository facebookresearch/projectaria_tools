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

#include <string>

#include <cereal/external/rapidjson/document.h>

#include <calibration/loader/SensorCalibrationJson.h>

#include <logging/Checks.h>
#define DEFAULT_LOG_CHANNEL "DeviceCalibrationFactory"
#include <logging/Log.h>

namespace projectaria::tools::calibration {

// calibration accessor for Aria device
std::optional<DeviceCalibration> deviceCalibrationFromJson(const std::string& calibJsonStr) {
  fb_rapidjson::Document json;
  json.Parse(calibJsonStr.c_str());

  std::map<std::string, CameraCalibration> cameraCalibs;
  std::map<std::string, ImuCalibration> imuCalibs;
  {
    if (json.FindMember("CameraCalibrations") != json.MemberEnd()) {
      for (const auto& camJson : json["CameraCalibrations"].GetArray()) {
        CameraCalibration camCalib = parseCameraCalibrationFromJson(camJson);
        auto label = camCalib.getLabel();
        cameraCalibs.emplace(label, std::move(camCalib));
      }
    }
    if (json.FindMember("ImuCalibrations") != json.MemberEnd()) {
      for (const auto& imuJson : json["ImuCalibrations"].GetArray()) {
        ImuCalibration imuCalib = parseImuCalibrationFromJson(imuJson);
        auto label = imuCalib.getLabel();
        imuCalibs.emplace(label, std::move(imuCalib));
      }
    }
  }

  std::string deviceSubtype;
  std::map<std::string, MagnetometerCalibration> magnetometerCalibs;
  std::map<std::string, BarometerCalibration> barometerCalibs;
  std::map<std::string, MicrophoneCalibration> microphoneCalibs;
  if (json.FindMember("DeviceClassInfo") != json.MemberEnd()) {
    deviceSubtype = json["DeviceClassInfo"]["BuildVersion"].GetString();
  }
  if (json.FindMember("MagCalibrations") != json.MemberEnd()) {
    for (const auto& magnetometerJson : json["MagCalibrations"].GetArray()) {
      MagnetometerCalibration magnetometerCalib =
          parseMagnetometerCalibrationFromJson(magnetometerJson);
      auto label = magnetometerCalib.getLabel();
      magnetometerCalibs.emplace(label, std::move(magnetometerCalib));
    }
  }
  if (json.FindMember("BaroCalibrations") != json.MemberEnd()) {
    for (const auto& barometerJson : json["BaroCalibrations"].GetArray()) {
      BarometerCalibration barometerCalib = parseBarometerCalibrationFromJson(barometerJson);
      auto label = barometerCalib.getLabel();
      barometerCalibs.emplace(label, std::move(barometerCalib));
    }
  }
  if (json.FindMember("MicCalibrations") != json.MemberEnd()) {
    for (const auto& microphoneJson : json["MicCalibrations"].GetArray()) {
      MicrophoneCalibration microphoneCalib = parseMicrophoneCalibrationFromJson(microphoneJson);
      auto label = microphoneCalib.getLabel();
      microphoneCalibs.emplace(label, std::move(microphoneCalib));
    }
  }

  std::string originLabel;
  // Parse in Origin Sensor Label
  if (json.FindMember("OriginSpecification") != json.MemberEnd()) {
    const std::string originType = json["OriginSpecification"]["Type"].GetString();
    if (originType == "Custom") {
      originLabel = json["OriginSpecification"]["ChildLabel"].GetString();
    } else {
      XR_LOGE(
          "Origin Specification's Type in calibration.json should be 'Custom' instead of {}",
          originType);
    }
  }

  DeviceCadExtrinsics deviceCadExtrinsics = DeviceCadExtrinsics(deviceSubtype, originLabel);

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
