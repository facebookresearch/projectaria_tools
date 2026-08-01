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

#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>

namespace projectaria::tools::calibration {

/**
 * @brief Indicates which bias-correction step the device firmware already
 * applied to the streamed IMU samples. Reader helpers gate on this to avoid
 * double-application.
 *
 * `CalibratedAll` (value 1) is a firmware-side misnomer — its behavior is
 * identical to `OnlineBias` (both remove accel + gyro online bias). It is
 * kept as a distinct enum value only because the wire proto emits it.
 */
enum class NeuralBandImuCalibrationApplied : int32_t {
  Uncalibrated = 0,
  CalibratedAll = 1,
  GyroOnlineBias = 2,
  OnlineBias = 3,
};

/**
 * @brief Wristband IMU calibration parsed from the `imu_*` sub-fields of the
 * `device.emg_calibration` JSON blob. Coexists with
 * `NeuralBandEmgCalibration`; the two classes parse the same JSON.
 *
 * `raw_to_rectified_accel/gyro()` subtract the remaining online bias per
 * `calibrationApplied`; cross-axis and g-sensitivity rectification matrices
 * are exposed but not applied (upstream semantics not yet finalized).
 * `offline_*` offsets are typically unpopulated on-wire today.
 * Wire float values arrive already divided by `10^sigfigs`; the field is
 * informational.
 */
class NeuralBandImuCalibration {
 public:
  NeuralBandImuCalibration() = default;
  explicit NeuralBandImuCalibration(
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
      uint32_t sigfigs);

  [[nodiscard]] const std::string& getLabel() const;
  /// Accel raw LSB → g conversion factor (g / LSB).
  [[nodiscard]] float getAccelScalingFactor() const;
  /// Gyro raw LSB → dps conversion factor (dps / LSB).
  [[nodiscard]] float getGyroScalingFactor() const;
  [[nodiscard]] NeuralBandImuCalibrationApplied getCalibrationApplied() const;
  [[nodiscard]] const Eigen::Vector3d& getOfflineAccelOffsetG() const;
  [[nodiscard]] const Eigen::Vector3d& getOfflineGyroOffsetDps() const;
  [[nodiscard]] const Eigen::Vector3d& getOnlineAccelOffsetG() const;
  [[nodiscard]] const Eigen::Vector3d& getOnlineGyroOffsetDps() const;
  [[nodiscard]] const Eigen::Matrix3d& getAccelCrossAxisRectMatrix() const;
  [[nodiscard]] const Eigen::Matrix3d& getGyroCrossAxisRectMatrix() const;
  [[nodiscard]] const Eigen::Matrix3d& getGyroLinearGRectMatrix() const;
  [[nodiscard]] uint32_t getSigfigs() const;

  /// Take m/s² accel from `NeuralBandAccelSample.accelMSec2` and subtract the
  /// remaining online bias (if firmware has not already done so per
  /// `calibrationApplied`). Cross-axis rectification is NOT applied.
  [[nodiscard]] Eigen::Vector3d rawToRectifiedAccel(const Eigen::Vector3d& accelMSec2) const;

  /// Take rad/s gyro from `NeuralBandGyroSample.gyroRadSec` and subtract the
  /// remaining online bias (if firmware has not already done so per
  /// `calibrationApplied`). Cross-axis and g-sensitivity rectification are
  /// NOT applied.
  [[nodiscard]] Eigen::Vector3d rawToRectifiedGyro(const Eigen::Vector3d& gyroRadSec) const;

  [[nodiscard]] Eigen::Vector3d rectifiedToRawAccel(const Eigen::Vector3d& rectifiedMSec2) const;
  [[nodiscard]] Eigen::Vector3d rectifiedToRawGyro(const Eigen::Vector3d& rectifiedRadSec) const;

  /// Parse the IMU sub-fields of the `device.emg_calibration` JSON blob.
  /// Returns nullopt on empty / malformed / non-object input or when any of
  /// `imu_accel_scaling_factor`, `imu_gyro_scaling_factor`,
  /// `imu_calibration_applied`, `imu_calibration` are missing. Missing
  /// sub-fields inside `imu_calibration` default to zero.
  static std::optional<NeuralBandImuCalibration> fromParamsJson(
      const std::string& json,
      std::string label = "imu");

 private:
  std::string label_;
  float accelScalingFactor_ = 0.0f;
  float gyroScalingFactor_ = 0.0f;
  NeuralBandImuCalibrationApplied calibrationApplied_ =
      NeuralBandImuCalibrationApplied::Uncalibrated;
  Eigen::Vector3d offlineAccelOffsetG_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d offlineGyroOffsetDps_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d onlineAccelOffsetG_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d onlineGyroOffsetDps_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d accelCrossAxisRectMatrix_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d gyroCrossAxisRectMatrix_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d gyroLinearGRectMatrix_ = Eigen::Matrix3d::Zero();
  uint32_t sigfigs_ = 0;
};

} // namespace projectaria::tools::calibration
