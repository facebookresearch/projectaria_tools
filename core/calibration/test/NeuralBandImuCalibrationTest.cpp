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

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace projectaria::tools::calibration {

namespace {

constexpr double kGravityMSec2 = 9.80665;
constexpr double kDegToRad = std::numbers::pi / 180.0;

constexpr const char* kFullJson = R"({
  "imu_accel_scaling_factor": 0.000244,
  "imu_gyro_scaling_factor": 0.07,
  "imu_calibration_applied": 2,
  "imu_calibration": {
    "sigfigs": 6,
    "offline_accel_offset": [0.01, -0.02, 0.03],
    "offline_gyro_offset": [1.0, -2.0, 3.0],
    "online_accel_offset": [0.001, 0.0, -0.001],
    "online_gyro_offset": [0.5, 0.0, -0.5],
    "accel_cross_axis_rect_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1],
    "gyro_cross_axis_rect_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1],
    "gyro_linear_g_rect_matrix": [0, 0, 0, 0, 0, 0, 0, 0, 0]
  }
})";

} // namespace

TEST(NeuralBandImuCalibrationTest, FromParamsJsonParsesAllFields) {
  const auto calib = NeuralBandImuCalibration::fromParamsJson(kFullJson);
  ASSERT_TRUE(calib.has_value());
  EXPECT_EQ(calib->getLabel(), "imu");
  EXPECT_FLOAT_EQ(calib->getAccelScalingFactor(), 0.000244f);
  EXPECT_FLOAT_EQ(calib->getGyroScalingFactor(), 0.07f);
  EXPECT_EQ(calib->getCalibrationApplied(), NeuralBandImuCalibrationApplied::GyroOnlineBias);
  EXPECT_EQ(calib->getSigfigs(), 6u);
  EXPECT_DOUBLE_EQ(calib->getOfflineAccelOffsetG().x(), 0.01);
  EXPECT_DOUBLE_EQ(calib->getOfflineGyroOffsetDps().z(), 3.0);
  EXPECT_DOUBLE_EQ(calib->getOnlineAccelOffsetG().x(), 0.001);
  EXPECT_DOUBLE_EQ(calib->getOnlineGyroOffsetDps().z(), -0.5);
  EXPECT_DOUBLE_EQ(calib->getAccelCrossAxisRectMatrix()(0, 0), 1.0);
}

TEST(NeuralBandImuCalibrationTest, FromParamsJsonHonorsLabelOverride) {
  const auto calib = NeuralBandImuCalibration::fromParamsJson(kFullJson, "wristband_imu");
  ASSERT_TRUE(calib.has_value());
  EXPECT_EQ(calib->getLabel(), "wristband_imu");
}

TEST(NeuralBandImuCalibrationTest, FromParamsJsonRejectsEmpty) {
  EXPECT_FALSE(NeuralBandImuCalibration::fromParamsJson("").has_value());
}

TEST(NeuralBandImuCalibrationTest, FromParamsJsonRejectsInvalidJson) {
  EXPECT_FALSE(NeuralBandImuCalibration::fromParamsJson("{not json").has_value());
}

TEST(NeuralBandImuCalibrationTest, FromParamsJsonRejectsWhenTopLevelIsNotObject) {
  EXPECT_FALSE(NeuralBandImuCalibration::fromParamsJson("[]").has_value());
}

TEST(NeuralBandImuCalibrationTest, FromParamsJsonRejectsWhenRequiredFieldMissing) {
  constexpr const char* kMissingAccelScale = R"({
    "imu_gyro_scaling_factor": 0.07,
    "imu_calibration_applied": 0,
    "imu_calibration": {}
  })";
  EXPECT_FALSE(NeuralBandImuCalibration::fromParamsJson(kMissingAccelScale).has_value());

  constexpr const char* kMissingImuObj = R"({
    "imu_accel_scaling_factor": 0.000244,
    "imu_gyro_scaling_factor": 0.07,
    "imu_calibration_applied": 0
  })";
  EXPECT_FALSE(NeuralBandImuCalibration::fromParamsJson(kMissingImuObj).has_value());
}

TEST(NeuralBandImuCalibrationTest, MissingSubFieldsDefaultToZero) {
  constexpr const char* kSparse = R"({
    "imu_accel_scaling_factor": 0.000244,
    "imu_gyro_scaling_factor": 0.07,
    "imu_calibration_applied": 0,
    "imu_calibration": {}
  })";
  const auto calib = NeuralBandImuCalibration::fromParamsJson(kSparse);
  ASSERT_TRUE(calib.has_value());
  EXPECT_EQ(calib->getSigfigs(), 0u);
  EXPECT_TRUE(calib->getOnlineAccelOffsetG().isZero());
  // Rectification matrices default to Identity so a missing field is a passthrough.
  EXPECT_TRUE(calib->getAccelCrossAxisRectMatrix().isIdentity());
  EXPECT_TRUE(calib->getGyroCrossAxisRectMatrix().isIdentity());
  // g-sensitivity coupling defaults to Zero (no additive coupling).
  EXPECT_TRUE(calib->getGyroLinearGRectMatrix().isZero());
}

TEST(NeuralBandImuCalibrationTest, RawToRectifiedAccelPassesThroughWhenBiasAlreadyRemoved) {
  const auto calib = NeuralBandImuCalibration::fromParamsJson(R"({
    "imu_accel_scaling_factor": 0.000244,
    "imu_gyro_scaling_factor": 0.07,
    "imu_calibration_applied": 3,
    "imu_calibration": {
      "online_accel_offset": [1.0, 2.0, 3.0]
    }
  })");
  ASSERT_TRUE(calib.has_value());
  const Eigen::Vector3d raw(0.5, -0.5, 0.25);
  EXPECT_TRUE(calib->rawToRectifiedAccel(raw).isApprox(raw));
}

TEST(NeuralBandImuCalibrationTest, RawToRectifiedAccelSubtractsBiasWhenUncalibrated) {
  const auto calib = NeuralBandImuCalibration::fromParamsJson(R"({
    "imu_accel_scaling_factor": 0.000244,
    "imu_gyro_scaling_factor": 0.07,
    "imu_calibration_applied": 0,
    "imu_calibration": {
      "online_accel_offset": [0.1, 0.0, 0.0]
    }
  })");
  ASSERT_TRUE(calib.has_value());
  const Eigen::Vector3d raw(1.0, 0.0, 0.0);
  const Eigen::Vector3d expected(1.0 - 0.1 * kGravityMSec2, 0.0, 0.0);
  EXPECT_TRUE(calib->rawToRectifiedAccel(raw).isApprox(expected));
}

TEST(NeuralBandImuCalibrationTest, RawToRectifiedGyroPassesThroughWhenBiasAlreadyRemoved) {
  const auto calib = NeuralBandImuCalibration::fromParamsJson(R"({
    "imu_accel_scaling_factor": 0.000244,
    "imu_gyro_scaling_factor": 0.07,
    "imu_calibration_applied": 2,
    "imu_calibration": {
      "online_gyro_offset": [10.0, 0.0, 0.0]
    }
  })");
  ASSERT_TRUE(calib.has_value());
  const Eigen::Vector3d raw(1.5, -0.75, 0.25);
  EXPECT_TRUE(calib->rawToRectifiedGyro(raw).isApprox(raw));
}

TEST(NeuralBandImuCalibrationTest, RawToRectifiedGyroSubtractsBiasWhenUncalibrated) {
  const auto calib = NeuralBandImuCalibration::fromParamsJson(R"({
    "imu_accel_scaling_factor": 0.000244,
    "imu_gyro_scaling_factor": 0.07,
    "imu_calibration_applied": 0,
    "imu_calibration": {
      "online_gyro_offset": [10.0, 0.0, 0.0]
    }
  })");
  ASSERT_TRUE(calib.has_value());
  const Eigen::Vector3d raw(1.0, 0.0, 0.0);
  const Eigen::Vector3d expected(1.0 - 10.0 * kDegToRad, 0.0, 0.0);
  EXPECT_TRUE(calib->rawToRectifiedGyro(raw).isApprox(expected));
}

TEST(NeuralBandImuCalibrationTest, RectifiedToRawRoundTrips) {
  const auto calib = NeuralBandImuCalibration::fromParamsJson(R"({
    "imu_accel_scaling_factor": 0.000244,
    "imu_gyro_scaling_factor": 0.07,
    "imu_calibration_applied": 0,
    "imu_calibration": {
      "online_accel_offset": [0.03, -0.02, 0.01],
      "online_gyro_offset": [5.0, -3.0, 1.0]
    }
  })");
  ASSERT_TRUE(calib.has_value());
  const Eigen::Vector3d accel(0.5, -0.5, 0.25);
  EXPECT_TRUE(calib->rectifiedToRawAccel(calib->rawToRectifiedAccel(accel)).isApprox(accel));
  const Eigen::Vector3d gyro(1.5, -0.75, 0.25);
  EXPECT_TRUE(calib->rectifiedToRawGyro(calib->rawToRectifiedGyro(gyro)).isApprox(gyro));
}

} // namespace projectaria::tools::calibration
