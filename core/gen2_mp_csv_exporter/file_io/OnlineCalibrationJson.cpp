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

#include <gen2_mp_csv_exporter/file_io/OnlineCalibrationJson.h>

namespace projectaria::tools::mps {

using namespace projectaria::tools::calibration;

namespace {
// Helper function to convert Eigen::VectorXd to JSON
nlohmann::json vectorToJson(const Eigen::VectorXd& vec) {
  nlohmann::json j;
  for (int i = 0; i < vec.size(); ++i) {
    j.push_back(vec(i));
  }
  return j;
}

// Helper function to convert Eigen::Matrix to JSON
nlohmann::json matrixToJson(const Eigen::Matrix3d& matrix) {
  nlohmann::json j;
  for (int i = 0; i < matrix.rows(); ++i) {
    j.push_back(vectorToJson(matrix.row(i)));
  }
  return j;
}

// Helper function to convert Sophus::SE3d to JSON
nlohmann::json se3ToJson(const Sophus::SE3d& se3) {
  return {
      {"Translation", std::vector<double>(se3.translation().data(), se3.translation().data() + 3)},
      {"UnitQuaternion",
       {se3.unit_quaternion().w(),
        std::vector<double>(
            se3.unit_quaternion().vec().data(), se3.unit_quaternion().vec().data() + 3)}}};
}

nlohmann::json to_json(const ImuCalibration& imuCalib) {
  return nlohmann::json{
      {"Calibrated", true}, // Assuming calibration status is true
      {"Gyroscope",
       {{"TimeOffsetSec_Device_Gyro", 0.0}, // Placeholder value
        {"Bias",
         {{"Name", "Constant"}, {"Offset", vectorToJson(imuCalib.getGyroModel().getBias())}}},
        {"Model",
         {{"Name", "LinearGSensitivity"},
          {"RectificationMatrix", matrixToJson(imuCalib.getGyroModel().getRectification())},
          {"GSensitivityMatrix", {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}}}}},
      {"Accelerometer",
       {{"TimeOffsetSec_Device_Accel", 0.0}, // Placeholder value
        {"Bias",
         {{"Name", "Constant"}, {"Offset", vectorToJson(imuCalib.getAccelModel().getBias())}}},
        {"Model",
         {{"Name", "UpperTriagonalLinear"},
          {"RectificationMatrix", matrixToJson(imuCalib.getAccelModel().getRectification())}}}}},
      {"T_Device_Imu", se3ToJson(imuCalib.getT_Device_Imu())},
      {"SerialNumber", ""}, // Placeholder value
      {"Label", imuCalib.getLabel()}};
}

nlohmann::json to_json(const CameraCalibration& camCalib) {
  return nlohmann::json{
      {"Calibrated", true}, // Assuming calibration status is true
      {"Projection",
       {
           {"Params", vectorToJson(camCalib.projectionParams())},
           {"Description", "see FisheyeRadTanThinPrism.h"}, // Placeholder value
           {"Name", "FisheyeRadTanThinPrism"} // Placeholder value
       }},
      {"T_Device_Camera", se3ToJson(camCalib.getT_Device_Camera())},
      {"SerialNumber", camCalib.getSerialNumber()},
      {"Label", camCalib.getLabel()}};
}

nlohmann::json to_json(const OnlineCalibration& onlineCalib) {
  nlohmann::json j;
  j = nlohmann::json{
      {"utc_timestamp_ns", onlineCalib.utcTimestamp.count()},
      {"tracking_timestamp_us", onlineCalib.trackingTimestamp.count()},
      {"ReadoutTimesSec", nlohmann::json::array()},
      {"ImuCalibrations", nlohmann::json::array()},
      {"ImageSizes", nlohmann::json::array()},
      {"CameraCalibrations", nlohmann::json::array()}};
  // populate camera calibrations
  for (const auto& camCalib : onlineCalib.cameraCalibs) {
    j["CameraCalibrations"].push_back(to_json(camCalib));
    j["ReadoutTimesSec"].push_back({2, camCalib.getReadOutTimeSec().value_or(0.0)});
    j["ImageSizes"].push_back({camCalib.getImageSize().x(), camCalib.getImageSize().y()});
  }

  // populate imu calibrations
  for (const auto& imuCalib : onlineCalib.imuCalibs) {
    j["ImuCalibrations"].push_back(to_json(imuCalib));
  }

  return j;
}
} // namespace

bool MpvOnlineCalibJsonWriter::openFile(const std::string& jsonFilename) {
  jsonFile_.open(jsonFilename, std::ios::out);
  return jsonFile_.is_open();
}

void MpvOnlineCalibJsonWriter::insertOnlineCalibData(const OnlineCalibration& calib) {
  if (jsonFile_.is_open()) {
    nlohmann::json calibJson = to_json(calib);
    jsonFile_ << calibJson.dump() << std::endl; // Write each entry as a single line
  }
}

void MpvOnlineCalibJsonWriter::closeFile() {
  if (jsonFile_.is_open()) {
    jsonFile_.close();
  }
}

} // namespace projectaria::tools::mps
