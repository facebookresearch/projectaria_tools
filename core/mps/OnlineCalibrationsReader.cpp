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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

#include <calibration/loader/SensorCalibrationJson.h>
#include "OnlineCalibration.h"

namespace projectaria::tools::mps {

namespace {
// loading new online calibration format after Jun 2023
OnlineCalibration readSingleOnlineCalibFromJson(const nlohmann::json& doc) {
  OnlineCalibration calib;
  calib.trackingTimestamp = std::chrono::microseconds(doc["tracking_timestamp_us"]);
  calib.utcTimestamp = std::chrono::nanoseconds(doc["utc_timestamp_ns"]);

  for (int cameraIdx = 0; cameraIdx < doc["CameraCalibrations"].size(); ++cameraIdx) {
    const auto& camJson = doc["CameraCalibrations"][cameraIdx];

    calibration::CameraCalibration parsedCameraCalib =
        calibration::parseCameraCalibrationFromJson(camJson);

    if (doc.count("ImageSizes")) {
      // Note that in the online calibration format, the image size is potentially stored in the
      // json file and we need to set them properly because the parsing function will always set the
      // image size to the device sensor spec. The image size field is ready after the MPS
      // trajectory version 1.1.0.
      const auto& imageSize = doc["ImageSizes"][cameraIdx];
      std::optional<double> maybeReadOutTimeSec = {};
      if (doc.count("ReadoutTimesSec")) {
        for (const auto& readOutTimeInfo : doc["ReadoutTimesSec"]) {
          // First field is the camera idx, and second field is the readout time in seconds
          if (readOutTimeInfo[0].get<int>() == cameraIdx) {
            maybeReadOutTimeSec = readOutTimeInfo[1].get<double>();
            break;
          }
        }
      }

      calibration::CameraCalibration onlineCameraCalib(
          parsedCameraCalib.getLabel(),
          parsedCameraCalib.modelName(),
          parsedCameraCalib.projectionParams(),
          parsedCameraCalib.getT_Device_Camera(),
          imageSize[0].get<int>(),
          imageSize[1].get<int>(),
          parsedCameraCalib.getValidRadius(),
          parsedCameraCalib.getMaxSolidAngle(),
          parsedCameraCalib.getSerialNumber(),
          parsedCameraCalib.getTimeOffsetSecDeviceCamera(),
          maybeReadOutTimeSec);
      calib.cameraCalibs.push_back(std::move(onlineCameraCalib));
    } else {
      calib.cameraCalibs.push_back(std::move(parsedCameraCalib));
    }
  }
  for (const auto& imuJson : doc["ImuCalibrations"]) {
    calib.imuCalibs.push_back(calibration::parseImuCalibrationFromJson(imuJson));
  }
  return calib;
}

void replaceSubStr(std::string& str, const std::string& from, const std::string& to) {
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}

std::string fixJsonString(const std::string& inputJsonStr) {
  std::string fixedJsonStr = inputJsonStr;
  replaceSubStr(fixedJsonStr, "True", "true");
  replaceSubStr(fixedJsonStr, "False", "false");
  replaceSubStr(fixedJsonStr, "\'", "\"");
  return fixedJsonStr;
}

// loading old online calib format before Jun 2023
OnlineCalibration readSingleOnlineCalibFromJsonString(const nlohmann::json& doc) {
  OnlineCalibration calib;
  calib.trackingTimestamp = std::chrono::microseconds(doc["tracking_timestamp_us"]);
  calib.utcTimestamp = std::chrono::nanoseconds(doc["utc_timestamp_ns"]);
  // cam
  nlohmann::json camDoc = nlohmann::json::parse(fixJsonString(doc["CameraCalibrations"]));
  for (const auto& camJson : camDoc) {
    calib.cameraCalibs.push_back(calibration::parseCameraCalibrationFromJson(camJson));
  }
  // imu
  nlohmann::json imuDoc = nlohmann::json::parse(fixJsonString(doc["ImuCalibrations"]));
  for (const auto& imuJson : imuDoc) {
    calib.imuCalibs.push_back(calibration::parseImuCalibrationFromJson(imuJson));
  }
  return calib;
}
} // namespace

OnlineCalibrations readOnlineCalibration(const std::string& filepath) {
  std::ifstream infile(filepath);
  if (infile) {
    std::string jsonCalibrationString;
    OnlineCalibrations onlineCalibs;

    while (std::getline(infile, jsonCalibrationString)) {
      nlohmann::json doc = nlohmann::json::parse(jsonCalibrationString.c_str());

      // whether the calibration content is saved in string
      // this is for backward compatibility of MPS online calib results release before Jun 2023.
      //
      // Before June 2023: calibration content is in quote
      // After June 2023:  we update the JSON format for simplicity, calibration content is no
      // longer in quote
      const bool contentInStr = doc["tracking_timestamp_us"].is_string();

      if (contentInStr) {
        onlineCalibs.push_back(readSingleOnlineCalibFromJsonString(doc));
      } else {
        onlineCalibs.push_back(readSingleOnlineCalibFromJson(doc));
      }
    }

    std::cout << "Loaded #onlineCalib records: " << onlineCalibs.size() << std::endl;
    return onlineCalibs;

  } else {
    std::cerr << "[readOnlineCalibration] Can't open the provided file path." << std::endl;
  }
  return {};
}

} // namespace projectaria::tools::mps
