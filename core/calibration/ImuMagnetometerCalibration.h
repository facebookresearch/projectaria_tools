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

#include <string>

#include <sophus/se3.hpp>

namespace projectaria::tools::calibration {
/**
 * @brief A class representing a linear rectification model, where:
 * @code
 *    rectified = rectificationMatrix.inv() * (raw - bias)
 *    raw = rectificationMatrix * rectified + bias
 * @endcode
 * `raw` is sensor read-out value, and rectified is actual data in the physical space, e.g.
 * acceleration and angular velocity for IMU.
 *
 */
class LinearRectificationModel3d {
 public:
  /**
   * @brief Constructor that takes rectification matrix and bias.
   */
  LinearRectificationModel3d(
      const Eigen::Matrix3d& rectificationMatrix,
      const Eigen::Vector3d& bias);

  /**
   * @brief convert from raw to rectified data to compensate system error
   */
  Eigen::Vector3d rawToRectified(const Eigen::Vector3d& raw) const;
  /**
   * @brief inverse function of rawToRectified, for simulating raw sensor data from actual
   * (rectified) data
   */
  Eigen::Vector3d rectifiedToRaw(const Eigen::Vector3d& rectified) const;

  /**
   * @brief getter function for rectification matrix
   */
  Eigen::Matrix3d getRectification() const;

  /**
   * @brief getter function for bias vector
   */
  Eigen::Vector3d getBias() const;

 private:
  Eigen::Matrix3d rectificationMatrix_;
  Eigen::Vector3d bias_;
};

/**
 * @brief A class representing an IMU calibration model, including both accelerometer and gyroscope.
 * We assume the accelerometer and gyroscope for each IMU are co-located and thus they share the
 * same extrinsic.
 */
class ImuCalibration {
 public:
  /**
   * @brief Constructor with a list of parameters for ImuCalibration.
   * @param label The label of the imu, e.g. "imu-left".
   * @param rectificationMatrixAccel The rectification matrix for accelerometer intrinsics.
   * @param biasAccel The bias of accelerometer intrinsics.
   * @param rectificationMatrixGyro The rectification matrix for gyroscope intrinsics.
   * @param biasGyro The bias of gyroscope intrinsics.
   * @param T_Device_Imu The extrinsics of the IMU in Device frame.
   */
  ImuCalibration(
      const std::string& label,
      const Eigen::Matrix3d& rectificationMatrixAccel,
      const Eigen::Vector3d& biasAccel,
      const Eigen::Matrix3d& rectificationMatrixGyro,
      const Eigen::Vector3d& biasGyro,
      const Sophus::SE3d& T_Device_Imu);

  std::string getLabel() const;
  Sophus::SE3d getT_Device_Imu() const;

  /**
   * @brief convert from imu sensor readout to actual acceleration
   */
  Eigen::Vector3d rawToRectifiedAccel(const Eigen::Vector3d& raw) const;

  /**
   * @brief simulate imu accel sensor readout from actual acceleration
   */
  Eigen::Vector3d rectifiedToRawAccel(const Eigen::Vector3d& rectified) const;
  /**
   * @brief  convert from imu sensor readout to actual angular velocity
   */
  Eigen::Vector3d rawToRectifiedGyro(const Eigen::Vector3d& raw) const;
  /**
   * @brief simulate imu gyro sensor readout from actual angular velocity
   */
  Eigen::Vector3d rectifiedToRawGyro(const Eigen::Vector3d& rectified) const;

  /**
   * @brief get accelerometer linear rectification model that includes rectification matrix and bias
   */
  LinearRectificationModel3d getAccelModel() const;

  /**
   * @brief get gyroscope linear rectification model that includes rectification matrix and bias
   */
  LinearRectificationModel3d getGyroModel() const;

 private:
  std::string label_;
  LinearRectificationModel3d accel_;
  LinearRectificationModel3d gyro_;
  Sophus::SE3d T_Device_Imu_;
};

/**
 * @brief A class representing a magnetometer calibration model, including only the intrinsics of
 * the magnetometer. Its extrinsics can be obtained from `DeviceCadExtrinsics` class.
 */
class MagnetometerCalibration {
 public:
  /**
   * @brief Constructor with a list of parameters for MagnetometerCalibration.
   * @param label The label of the magnetometer, e.g. "mag0".
   * @param rectificationMatrix The rectification matrix for magnetometer intrinsics.
   * @param bias The bias of magnetometer intrinsics.
   */
  MagnetometerCalibration(
      const std::string& label,
      const Eigen::Matrix3d& rectificationMatrix,
      const Eigen::Vector3d& bias);

  std::string getLabel() const;
  /**
   * @brief convert from mag sensor readout to actual magnetic field
   */
  Eigen::Vector3d rawToRectified(const Eigen::Vector3d& raw) const;
  /**
   * @brief simulate mag sensor readout from actual magnetic field
   */
  Eigen::Vector3d rectifiedToRaw(const Eigen::Vector3d& rectified) const;
  /**
   * @brief get linear rectification model that includes rectification matrix and bias
   */
  LinearRectificationModel3d getModel() const;

 private:
  std::string label_;
  LinearRectificationModel3d model_;
};
} // namespace projectaria::tools::calibration
