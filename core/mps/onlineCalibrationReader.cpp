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

#define RAPIDJSON_HAS_STDSTRING 1
#include <calibration/loader/SensorCalibrationJson.h>
#include "OnlineCalibration.h"

namespace projectaria::tools::mps {

namespace {
// loading new online calib format after Jun 2023
OnlineCalibration readSingleOnlineCalibFromJson(const rapidjson::Document& doc) {
  OnlineCalibration calib;
  calib.trackingTimestamp = std::chrono::microseconds(doc["tracking_timestamp_us"].GetInt64());
  calib.utcTimestamp = std::chrono::nanoseconds(doc["utc_timestamp_ns"].GetInt64());
  for (const auto& camJson : doc["CameraCalibrations"].GetArray()) {
    calib.cameraCalibs.push_back(calibration::parseCameraCalibrationFromJson(camJson));
  }
  for (const auto& imuJson : doc["ImuCalibrations"].GetArray()) {
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
OnlineCalibration readSingleOnlineCalibFromJsonString(const rapidjson::Document& doc) {
  OnlineCalibration calib;
  calib.trackingTimestamp =
      std::chrono::microseconds(std::stoul(doc["tracking_timestamp_us"].GetString()));
  calib.utcTimestamp = std::chrono::nanoseconds(std::stoul(doc["utc_timestamp_ns"].GetString()));
  // cam
  rapidjson::Document camDoc;
  camDoc.Parse(fixJsonString(doc["CameraCalibrations"].GetString()));
  for (int i = 0; i < camDoc.GetArray().Size(); i++) {
    const auto& camJson = camDoc.GetArray()[i];
    calib.cameraCalibs.push_back(calibration::parseCameraCalibrationFromJson(camJson));
  }
  // imu
  rapidjson::Document imuDoc;
  imuDoc.Parse(fixJsonString(doc["ImuCalibrations"].GetString()));
  for (int i = 0; i < imuDoc.GetArray().Size(); i++) {
    const auto& imuJson = imuDoc.GetArray()[i];
    calib.imuCalibs.push_back(calibration::parseImuCalibrationFromJson(imuJson));
  }
  return calib;
}
} // namespace

OnlineCalibrations readOnlineCalibration(const std::string& filepath) {
  std::ifstream infile(filepath);
  if (infile) {
    std::string jsonCalibrationString = "";
    OnlineCalibrations onlineCalibs;

    while (std::getline(infile, jsonCalibrationString)) {
      rapidjson::Document doc;
      doc.Parse(jsonCalibrationString.c_str());

      // whether the calibration content is saved in string
      // this is for backward compatibility of MPS online calib results release before Jun 2023.
      //
      // Before June 2023: calibration content is in quote
      // After June 2023:  we update the JSON format for simplicity, calibration content is no
      // longer in quote
      const bool contentInStr = doc["tracking_timestamp_us"].IsString();

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
