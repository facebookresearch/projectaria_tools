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

#include <calibration/NeuralBandImuCalibration.h>

#include <cmath>
#include <numbers>
#include <utility>

#include <nlohmann/json.hpp>

namespace projectaria::tools::calibration {

namespace {

constexpr double kGravityMSec2 = 9.80665;
constexpr double kDegToRad = std::numbers::pi / 180.0;

bool accelOnlineBiasAlreadyRemoved(NeuralBandImuCalibrationApplied applied) {
  return applied == NeuralBandImuCalibrationApplied::CalibratedAll ||
      applied == NeuralBandImuCalibrationApplied::OnlineBias;
}

bool gyroOnlineBiasAlreadyRemoved(NeuralBandImuCalibrationApplied applied) {
  return applied != NeuralBandImuCalibrationApplied::Uncalibrated;
}

Eigen::Vector3d readVec3OrZero(const nlohmann::json& obj, const char* key) {
  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  auto it = obj.find(key);
  if (it == obj.end() || !it->is_array() || it->size() != 3) {
    return out;
  }
  for (Eigen::Index i = 0; i < 3; ++i) {
    const auto& elem = (*it)[i];
    if (elem.is_number()) {
      out[i] = elem.get<double>();
    }
  }
  return out;
}

// Rectification matrices default to Identity so an untouched matrix is a
// safe passthrough (`Y = M * X` == X). Callers that need Zero as the sentinel
// value must pass it explicitly.
Eigen::Matrix3d
readMat3OrDefault(const nlohmann::json& obj, const char* key, const Eigen::Matrix3d& defaultVal) {
  auto it = obj.find(key);
  if (it == obj.end() || !it->is_array() || it->size() != 9) {
    return defaultVal;
  }
  Eigen::Matrix3d out = defaultVal;
  // Row-major flat array on the wire.
  for (Eigen::Index r = 0; r < 3; ++r) {
    for (Eigen::Index c = 0; c < 3; ++c) {
      const auto& elem = (*it)[r * 3 + c];
      if (elem.is_number()) {
        out(r, c) = elem.get<double>();
      }
    }
  }
  return out;
}

} // namespace

NeuralBandImuCalibration::NeuralBandImuCalibration(
    std::string label,
    float accelScalingFactor,
    float gyroScalingFactor,
    NeuralBandImuCalibrationApplied calibrationApplied,
    Eigen::Vector3d offlineAccelOffsetG,
    Eigen::Vector3d offlineGyroOffsetDps,
    Eigen::Vector3d onlineAccelOffsetG,
    Eigen::Vector3d onlineGyroOffsetDps,
    Eigen::Matrix3d accelCrossAxisRectMatrix,
    Eigen::Matrix3d gyroCrossAxisRectMatrix,
    Eigen::Matrix3d gyroLinearGRectMatrix,
    uint32_t sigfigs)
    : label_(std::move(label)),
      accelScalingFactor_(accelScalingFactor),
      gyroScalingFactor_(gyroScalingFactor),
      calibrationApplied_(calibrationApplied),
      offlineAccelOffsetG_(std::move(offlineAccelOffsetG)),
      offlineGyroOffsetDps_(std::move(offlineGyroOffsetDps)),
      onlineAccelOffsetG_(std::move(onlineAccelOffsetG)),
      onlineGyroOffsetDps_(std::move(onlineGyroOffsetDps)),
      accelCrossAxisRectMatrix_(std::move(accelCrossAxisRectMatrix)),
      gyroCrossAxisRectMatrix_(std::move(gyroCrossAxisRectMatrix)),
      gyroLinearGRectMatrix_(std::move(gyroLinearGRectMatrix)),
      sigfigs_(sigfigs) {}

const std::string& NeuralBandImuCalibration::getLabel() const {
  return label_;
}
float NeuralBandImuCalibration::getAccelScalingFactor() const {
  return accelScalingFactor_;
}
float NeuralBandImuCalibration::getGyroScalingFactor() const {
  return gyroScalingFactor_;
}
NeuralBandImuCalibrationApplied NeuralBandImuCalibration::getCalibrationApplied() const {
  return calibrationApplied_;
}
const Eigen::Vector3d& NeuralBandImuCalibration::getOfflineAccelOffsetG() const {
  return offlineAccelOffsetG_;
}
const Eigen::Vector3d& NeuralBandImuCalibration::getOfflineGyroOffsetDps() const {
  return offlineGyroOffsetDps_;
}
const Eigen::Vector3d& NeuralBandImuCalibration::getOnlineAccelOffsetG() const {
  return onlineAccelOffsetG_;
}
const Eigen::Vector3d& NeuralBandImuCalibration::getOnlineGyroOffsetDps() const {
  return onlineGyroOffsetDps_;
}
const Eigen::Matrix3d& NeuralBandImuCalibration::getAccelCrossAxisRectMatrix() const {
  return accelCrossAxisRectMatrix_;
}
const Eigen::Matrix3d& NeuralBandImuCalibration::getGyroCrossAxisRectMatrix() const {
  return gyroCrossAxisRectMatrix_;
}
const Eigen::Matrix3d& NeuralBandImuCalibration::getGyroLinearGRectMatrix() const {
  return gyroLinearGRectMatrix_;
}
uint32_t NeuralBandImuCalibration::getSigfigs() const {
  return sigfigs_;
}

Eigen::Vector3d NeuralBandImuCalibration::rawToRectifiedAccel(
    const Eigen::Vector3d& accelMSec2) const {
  if (accelOnlineBiasAlreadyRemoved(calibrationApplied_)) {
    return accelMSec2;
  }
  return accelMSec2 - onlineAccelOffsetG_ * kGravityMSec2;
}

Eigen::Vector3d NeuralBandImuCalibration::rawToRectifiedGyro(
    const Eigen::Vector3d& gyroRadSec) const {
  if (gyroOnlineBiasAlreadyRemoved(calibrationApplied_)) {
    return gyroRadSec;
  }
  return gyroRadSec - onlineGyroOffsetDps_ * kDegToRad;
}

Eigen::Vector3d NeuralBandImuCalibration::rectifiedToRawAccel(
    const Eigen::Vector3d& rectifiedMSec2) const {
  if (accelOnlineBiasAlreadyRemoved(calibrationApplied_)) {
    return rectifiedMSec2;
  }
  return rectifiedMSec2 + onlineAccelOffsetG_ * kGravityMSec2;
}

Eigen::Vector3d NeuralBandImuCalibration::rectifiedToRawGyro(
    const Eigen::Vector3d& rectifiedRadSec) const {
  if (gyroOnlineBiasAlreadyRemoved(calibrationApplied_)) {
    return rectifiedRadSec;
  }
  return rectifiedRadSec + onlineGyroOffsetDps_ * kDegToRad;
}

std::optional<NeuralBandImuCalibration> NeuralBandImuCalibration::fromParamsJson(
    const std::string& json,
    std::string label) {
  if (json.empty()) {
    return std::nullopt;
  }
  nlohmann::json parsed;
  try {
    parsed = nlohmann::json::parse(json);
  } catch (const nlohmann::json::exception&) {
    return std::nullopt;
  }
  if (!parsed.is_object()) {
    return std::nullopt;
  }
  auto accelScaleIt = parsed.find("imu_accel_scaling_factor");
  auto gyroScaleIt = parsed.find("imu_gyro_scaling_factor");
  auto appliedIt = parsed.find("imu_calibration_applied");
  auto imuObjIt = parsed.find("imu_calibration");
  if (accelScaleIt == parsed.end() || !accelScaleIt->is_number() || gyroScaleIt == parsed.end() ||
      !gyroScaleIt->is_number() || appliedIt == parsed.end() || !appliedIt->is_number_integer() ||
      imuObjIt == parsed.end() || !imuObjIt->is_object()) {
    return std::nullopt;
  }

  // Guard the enum-range check below: it assumes OnlineBias is the numeric max.
  static_assert(
      static_cast<int32_t>(NeuralBandImuCalibrationApplied::OnlineBias) == 3,
      "Update the fromParamsJson range check if a NeuralBandImuCalibrationApplied entry is added.");
  const int32_t appliedVal = appliedIt->get<int32_t>();
  if (appliedVal < static_cast<int32_t>(NeuralBandImuCalibrationApplied::Uncalibrated) ||
      appliedVal > static_cast<int32_t>(NeuralBandImuCalibrationApplied::OnlineBias)) {
    return std::nullopt;
  }

  const auto sigfigsIt = imuObjIt->find("sigfigs");
  const uint32_t sigfigs = (sigfigsIt != imuObjIt->end() && sigfigsIt->is_number_integer())
      ? sigfigsIt->get<uint32_t>()
      : 0;

  return NeuralBandImuCalibration{
      std::move(label),
      accelScaleIt->get<float>(),
      gyroScaleIt->get<float>(),
      static_cast<NeuralBandImuCalibrationApplied>(appliedVal),
      readVec3OrZero(*imuObjIt, "offline_accel_offset"),
      readVec3OrZero(*imuObjIt, "offline_gyro_offset"),
      readVec3OrZero(*imuObjIt, "online_accel_offset"),
      readVec3OrZero(*imuObjIt, "online_gyro_offset"),
      readMat3OrDefault(*imuObjIt, "accel_cross_axis_rect_matrix", Eigen::Matrix3d::Identity()),
      readMat3OrDefault(*imuObjIt, "gyro_cross_axis_rect_matrix", Eigen::Matrix3d::Identity()),
      readMat3OrDefault(*imuObjIt, "gyro_linear_g_rect_matrix", Eigen::Matrix3d::Zero()),
      sigfigs,
  };
}

} // namespace projectaria::tools::calibration
