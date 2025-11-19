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

#include <calibration/DeviceCadExtrinsics.h>

#include <filesystem>
#include <fstream>
#include <istream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include <calibration/ConfigFileLoader.h>

#include <logging/Checks.h>
#define DEFAULT_LOG_CHANNEL "DeviceCadExtrinsics"
#include <logging/Log.h>

namespace projectaria::tools::calibration {

namespace {
// helper function for separating a csv line into tokens
std::vector<std::string> tokenize(const std::string& input, char delimiter) {
  std::vector<std::string> tokens;
  std::stringstream stream;
  stream << input;
  std::string token;

  while (std::getline(stream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

// Helper function for parsing a csv line as a <label,SE3d pose>,
// the csv line should have the format of:
//      label, trans_x, trans_y, trans_z, YAxis_x , YAxis_y, YAxis_z, ZAxis_x, ZAxis_y, ZAxis_z
std::optional<std::pair<std::string, Sophus::SE3d>> readSingleCsvLine(const std::string& line) {
  // break csv line into tokens
  std::vector<std::string> tokens = tokenize(line, ',');
  if (tokens.empty()) {
    return {};
  }

  if (tokens.size() != 10) {
    XR_LOGE(
        "Parsing single CSV line failed, expected 10 tokens in line, got {} \n {}",
        tokens.size(),
        line);
    return {};
  }
  std::string label = tokens[0];

  // Parse translation
  const Eigen::Vector3d translation = {
      std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3])};

  // Parse unit vector Y and Z
  Eigen::Vector3d unitY = {std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6])};
  Eigen::Vector3d unitZ = {std::stod(tokens[7]), std::stod(tokens[8]), std::stod(tokens[9])};

  // Normalize unitY and unitZ
  unitY = unitY.normalized();
  unitZ = unitZ.normalized();

  // Construct a SO3 from unitY and unit Z
  static const double eps = 1e-3;
  if (std::abs(unitY.dot(unitZ)) > eps) {
    XR_LOGE("unitY and unitZ vectors are not orthonormal.");
    return {};
  }
  Eigen::Matrix3d rot;
  rot.col(1) = unitY;
  rot.col(2) = unitZ;
  rot.col(0) = rot.col(1).cross(rot.col(2));
  if (rot.determinant() < 0.99) {
    throw std::runtime_error("Rotation matrix determinant must be >= 0.99");
  }

  return std::make_pair(label, Sophus::SE3d(Sophus::makeRotationMatrix(rot), translation));
}

// Helper function to load DeviceCadExtrinsics as a map<label, SE3> from a csv file
std::optional<std::unordered_map<std::string, Sophus::SE3d>> loadCadExtrinsicsFromCsvFile(
    const DeviceVersion& deviceVersion,
    const std::string& deviceSubType) {
  // Load CAD extrinsics csv file
  const std::string csvStr = loadDeviceCadExtrinsicsCsvAsString(deviceVersion, deviceSubType);
  std::istringstream csvStream(csvStr);

  // Read csv lines into label->pose mapping
  std::unordered_map<std::string, Sophus::SE3d> labelToT_Cpf_Sensor;
  std::string csvLine;
  // Skip first line
  std::getline(csvStream, csvLine);
  while (std::getline(csvStream, csvLine)) {
    const auto maybeLabelAndPose = readSingleCsvLine(csvLine);
    if (!maybeLabelAndPose.has_value()) {
      fmt::print(
          "WARNING: parsing csv line to label and pose has failed, this line will be skipped: \n {}\n",
          csvLine);
      continue;
    }
    labelToT_Cpf_Sensor.emplace(maybeLabelAndPose.value().first, maybeLabelAndPose.value().second);
  }

  return labelToT_Cpf_Sensor;
}

} // namespace

DeviceCadExtrinsics::DeviceCadExtrinsics(
    const DeviceVersion& deviceVersion,
    const std::string& deviceSubType,
    const std::string& originSensorLabel)
    : deviceVersion_(deviceVersion),
      deviceSubType_(deviceSubType),
      originSensorLabel_(originSensorLabel) {
  // First, skip simulated devices on purpose
  const std::unordered_set<std::string> simulatedDeviceSubtypes = {
      "SimulatedDevice", "DtcSimulatedDevice_DVT-S"};
  if (simulatedDeviceSubtypes.find(deviceSubType) != simulatedDeviceSubtypes.end()) {
    fmt::print("WARNING: No CAD available for simulated device\n");
    return;
  }

  // Check that origin sensor label is valid, throw on failure.
  // This is for catching legacy calibration.json files
  // that have the wrong origin label as "ME-defined-CPF".
  if (originSensorLabel != "slam-front-left" && originSensorLabel != "camera-slam-left") {
    const std::string errorMsg = fmt::format(
        "Error: Origin Specification's child label in calibration.json is wrong. It should be the first slam camera label instead of {}",
        originSensorLabel);
    throw std::runtime_error{errorMsg};
  }

  // Load CAD extrinsics from config files
  const auto maybeLabelToT_Cpf_Sensor = loadCadExtrinsicsFromCsvFile(deviceVersion, deviceSubType);
  if (!maybeLabelToT_Cpf_Sensor.has_value()) {
    const std::string error = fmt::format(
        "ERROR: Cannot load CAD extrinsics for device {}: {}. ",
        getName(deviceVersion),
        deviceSubType);
    fmt::print("{}\n", error);
    throw std::runtime_error{error};
  }
  labelToT_Cpf_Sensor_ = maybeLabelToT_Cpf_Sensor.value();

  // Ensure that origin sensor label is valid
  if (labelToT_Cpf_Sensor_.find(originSensorLabel) == labelToT_Cpf_Sensor_.end()) {
    const std::string error = fmt::format(
        "ERROR: The origin sensor label in DeviceCalibration is wrong. Currently it is {}, however we expect it to be one of the sensors (Specifically, the first slam camera) ",
        originSensorLabel);
    fmt::print("{}\n", error);
    throw std::runtime_error{error};
  }
  // Set T_Device_CPF according to origin sensor label
  T_Device_Cpf_ = labelToT_Cpf_Sensor_.at(originSensorLabel).inverse();
}

std::optional<Sophus::SE3d> DeviceCadExtrinsics::getT_Device_Sensor(
    const std::string& label) const {
  const auto maybePose = labelToT_Cpf_Sensor_.find(label);
  if (maybePose != labelToT_Cpf_Sensor_.end()) {
    return T_Device_Cpf_ * maybePose->second;
  } else {
    return {};
  }
}

Sophus::SE3d DeviceCadExtrinsics::getT_Device_Cpf() const {
  return T_Device_Cpf_;
}

} // namespace projectaria::tools::calibration
