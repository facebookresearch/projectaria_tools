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

#include <calibration/ImuMagnetometerCalibration.h>

namespace projectaria::tools::calibration {

/* LinearRectificationModel */
LinearRectificationModel3d::LinearRectificationModel3d(
    const Eigen::Matrix3d& rectificationMatrix,
    const Eigen::Vector3d& bias)
    : rectificationMatrix_(rectificationMatrix), bias_(bias) {}

Eigen::Vector3d LinearRectificationModel3d::rawToRectified(const Eigen::Vector3d& raw) const {
  return rectificationMatrix_.inverse() * (raw - bias_);
}

Eigen::Vector3d LinearRectificationModel3d::rectifiedToRaw(const Eigen::Vector3d& rectified) const {
  return rectificationMatrix_ * rectified + bias_;
}

/* ImuCalibration */
ImuCalibration::ImuCalibration(
    const std::string& label,
    const Eigen::Matrix3d& rectificationMatrixAccel,
    const Eigen::Vector3d& biasAccel,
    const Eigen::Matrix3d& rectificationMatrixGyro,
    const Eigen::Vector3d& biasGyro,
    const Sophus::SE3d& T_Device_Imu)
    : label_(label),
      accel_(rectificationMatrixAccel, biasAccel),
      gyro_(rectificationMatrixGyro, biasGyro),
      T_Device_Imu_(T_Device_Imu) {}

std::string ImuCalibration::getLabel() const {
  return label_;
}
Sophus::SE3d ImuCalibration::getT_Device_Imu() const {
  return T_Device_Imu_;
}

Eigen::Vector3d ImuCalibration::rawToRectifiedAccel(const Eigen::Vector3d& raw) const {
  return accel_.rawToRectified(raw);
}

Eigen::Vector3d ImuCalibration::rectifiedToRawAccel(const Eigen::Vector3d& rectified) const {
  return accel_.rectifiedToRaw(rectified);
}

Eigen::Vector3d ImuCalibration::rawToRectifiedGyro(const Eigen::Vector3d& raw) const {
  return gyro_.rawToRectified(raw);
}

Eigen::Vector3d ImuCalibration::rectifiedToRawGyro(const Eigen::Vector3d& rectified) const {
  return gyro_.rectifiedToRaw(rectified);
}

/* MagnetometerCalibration */
MagnetometerCalibration::MagnetometerCalibration(
    const std::string& label,
    const Eigen::Matrix3d& rectificationMatrix,
    const Eigen::Vector3d& bias)
    : label_(label), model_(rectificationMatrix, bias) {}

std::string MagnetometerCalibration::getLabel() const {
  return label_;
}

Eigen::Vector3d MagnetometerCalibration::rawToRectified(const Eigen::Vector3d& raw) const {
  return model_.rawToRectified(raw);
}

Eigen::Vector3d MagnetometerCalibration::rectifiedToRaw(const Eigen::Vector3d& rectified) const {
  return model_.rectifiedToRaw(rectified);
}
} // namespace projectaria::tools::calibration
