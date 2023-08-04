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
  XR_CHECK_GE(rot.determinant(), 0.99);

  return std::make_pair(label, Sophus::SE3d(Sophus::makeRotationMatrix(rot), translation));
}

// A helper function for constructing CAD extrinsics for DVT-S
std::unordered_map<std::string, Sophus::SE3d> constructCadForDvtSmall() {
  std::unordered_map<std::string, Sophus::SE3d> cadMapT_Device_Sensor;

  const std::vector<std::string> cadCsvLines = {
      "camera-slam-left, 0.069051, 0.002372, 0.009254, 0.793353, 0.000000, -0.608761, 0.607927, -0.052336, 0.792266",
      "camera-slam-right, -0.069051, 0.002372, 0.009254, 0.793353,0.000000, 0.608761, -0.607927, -0.052336, 0.792266",
      "camera-rgb, 0.056000,0.007121,0.012883,1.000000,0.000000,0.000000,0.000000,-0.130526,0.991445",
      "camera-et-left, 0.054298, -0.018500, 0.006210, 0.034478, -0.508118, -0.860597, -0.674245, 0.623794, -0.395316",
      "camera-et-right, -0.054298, -0.018500, 0.006210, -0.034478, -0.508118, -0.860597, 0.674245, 0.623794, -0.395316",
      "imu-left, 0.064079,0.001432,0.003612,-0.083456,-0.990691,-0.107544,-0.607365,0.136127,-0.782673",
      "imu-right, -0.065182, 0.001893, 0.002645, -0.793353,0.000000, -0.608761, 0.595593, 0.206874, -0.776191",
      "mag0, 0.064372, -0.005868, 0.000699, 0.587481, -0.015929, -0.809081, -0.808020, 0.043280, -0.587563",
      "baro0, -0.009258, 0.010842, 0.017200, 0.000000, 0.000000, -1.000000, 0.173648, 0.984808, 0.000000",
      "mic0, -0.045906 ,-0.027938 ,0.006667 ,0.97508224 ,-0.160939007 ,0.152686805 ,-0.14019156 ,-0.980440 ,-0.138144",
      "mic1, 0.009161 ,0.010231 ,0.017250 ,-0.984808 ,-0.173648 ,0.000000 ,-0.173648 ,0.984808 ,0.000000",
      "mic2, 0.045905 ,-0.027931 ,0.006668 ,-0.975082 ,-0.160938 ,0.152687 ,0.140190 ,-0.980440 ,-0.138146",
      "mic3, 0.063471 ,0.012034 ,0.005566 ,-0.017386 ,0.001521 ,0.999848 ,0.087156 ,0.996195 ,0.000000",
      "mic4, -0.052398 ,0.013200 ,0.012160 ,0.965337 ,0.033710 ,0.258819 ,-0.034899 ,0.999391 ,0.000000",
      "mic5, 0.069856 ,0.008270 ,-0.093105 ,0.002466 ,-0.996176 ,0.087334 ,0.990147 ,-0.009795 ,-0.139689",
      "mic6, -0.069822 ,0.008268 ,-0.093138 ,-0.002487 ,-0.996176 ,0.087333 ,-0.990081 ,-0.009815 ,-0.140151"};

  for (const auto& line : cadCsvLines) {
    const auto maybeLabelAndPose = readSingleCsvLine(line);
    XR_CHECK(maybeLabelAndPose.has_value(), "Reading csv line has failed: {}", line);
    cadMapT_Device_Sensor.emplace(
        maybeLabelAndPose.value().first, maybeLabelAndPose.value().second);
  }

  return cadMapT_Device_Sensor;
}

// A helper function for constructing CAD extrinsics for DVT-L
std::unordered_map<std::string, Sophus::SE3d> constructCadForDvtLarge() {
  std::unordered_map<std::string, Sophus::SE3d> cadMapT_Device_Sensor;

  const std::vector<std::string> cadCsvLines = {
      "camera-slam-left,0.071351,0.002372,0.008454,0.793353,0.000000,-0.608761,0.607927,-0.052336,0.792266",
      "camera-slam-right,-0.071351,0.002372,0.008454,0.793353,0.000000,0.608761,-0.607927,-0.052336,0.792266",
      "camera-rgb,0.058250,0.007186,0.012096,1.000000,0.000000,0.000000,0.000000,-0.130526,0.991445",
      "camera-et-left,0.055753,-0.019589,0.004786,0.034096,-0.508436,-0.860424,-0.674299,0.623745,-0.395300",
      "camera-et-right,-0.055753,-0.019589,0.004786,-0.034096,-0.508436,-0.860424,0.674299,0.623745,-0.395300",
      "imu-left, 0.063225, 0.001595, 0.002873, -0.083456, -0.990691, -0.107544, -0.607365, 0.136127, -0.782673",
      "imu-right,-0.067482,0.001893,0.001845,-0.793353,0.000000,-0.608761,0.595593,0.206874,-0.776191",
      "mag0, 0.066201, -0.005760, -0.001777, 0.588330, 0.006520, -0.808595, -0.808020, 0.043280, -0.587563",
      "baro0, -0.009258, 0.010842, 0.017200, 0.000000, 0.000000, -1.000000, 0.173648, 0.984808, 0.000000",
      "mic0, -0.04613750996 ,-0.02929690419 ,0.006233332458 ,0.981006 ,-0.109790 ,0.159915 ,-0.087780 ,-0.986430 ,-0.138744",
      "mic1, 0.009200472201 ,0.01030418902 ,0.01725 ,-0.984807753 ,-0.173648178 ,0 ,-0.173648178 ,0.984807753 ,0",
      "mic2, 0.046138 ,-0.029297 ,0.006233 ,-0.981006 ,-0.109790 ,0.159915 ,0.087780 ,-0.986430 ,-0.138744",
      "mic3, 0.065925 ,0.011961 ,0.004305 ,-0.017386 ,0.001521 ,0.999848 ,0.087156 ,0.996195 ,0.000000",
      "mic4, -0.054800 ,0.013142 ,0.010960 ,0.965337 ,0.033710 ,0.258819 ,-0.034899 ,0.999391 ,0.000000",
      "mic5, 0.072318 ,0.008272 ,-0.094955 ,0.002477 ,-0.996176 ,0.087334 ,0.990114 ,-0.009805 ,-0.139920",
      "mic6, -0.072319 ,0.008271 ,-0.094955 ,-0.002477 ,-0.996176 ,0.087334 ,-0.990114 ,-0.009805 ,-0.139920"};

  for (const auto& line : cadCsvLines) {
    const auto maybeLabelAndPose = readSingleCsvLine(line);
    XR_CHECK(maybeLabelAndPose.has_value(), "Reading csv line has failed: {}", line);
    cadMapT_Device_Sensor.emplace(
        maybeLabelAndPose.value().first, maybeLabelAndPose.value().second);
  }

  return cadMapT_Device_Sensor;
}
} // namespace

DeviceCadExtrinsics::DeviceCadExtrinsics(
    const std::string& deviceSubType,
    const std::string& originSensorLabel) {
  if (deviceSubType == "DVT-S") {
    labelToT_Cpf_Sensor_ = constructCadForDvtSmall();
  } else if (deviceSubType == "DVT-L") {
    labelToT_Cpf_Sensor_ = constructCadForDvtLarge();
  } else if (deviceSubType == "SimulatedDevice") {
    XR_LOGW("No CAD available for simulated device");
    return;
  } else {
    XR_FATAL_ERROR("Does not recognize device subtype: {}", deviceSubType);
  }

  T_Device_Cpf_ = labelToT_Cpf_Sensor_.at(originSensorLabel).inverse();
}

std::optional<Sophus::SE3d> DeviceCadExtrinsics::getT_Device_Sensor(
    const std::string& label) const {
  const auto maybePose = labelToT_Cpf_Sensor_.find(label);
  if (maybePose != labelToT_Cpf_Sensor_.end()) {
    return T_Device_Cpf_ * maybePose->second;
  } else {
    return std::nullopt;
  }
}

Sophus::SE3d DeviceCadExtrinsics::getT_Device_Cpf() const {
  return T_Device_Cpf_;
}

} // namespace projectaria::tools::calibration
