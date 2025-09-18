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
#include <data_provider/data_types/FrontendTypes.h>
#include <gtest/gtest.h>
#include <Eigen/Core>

namespace projectaria::tools::data_provider::test {

//  Compare two 3D arrays with a tolerance
template <typename T>
inline bool comparePtr3(const T* ptr1, const T* ptr2, T tolerance = static_cast<T>(1e-5)) {
  return Eigen::Matrix<T, 3, 1>(ptr1[0], ptr1[1], ptr1[2])
      .isApprox(Eigen::Matrix<T, 3, 1>(ptr2[0], ptr2[1], ptr2[2]), tolerance);
}

/**
 * Compare two quaternions represented as 4D arrays using angular distance
 * @tparam T Type of array elements (float or double)
 * @param a First Eigen::Quaternion
 * @param b Second Eigen::Quaternion
 * @param tolerance Angular distance tolerance in radians
 * @return True if quaternions are approximately equal within tolerance
 */
template <typename T>
inline bool compareQuaternion(
    const Eigen::Quaternion<T>& quaternion1,
    const Eigen::Quaternion<T>& quaternion2,
    T tolerance = static_cast<T>(1e-5)) {
  return std::abs(quaternion1.angularDistance(quaternion2)) < tolerance;
}

/**
 * Compare two quaternions represented as 4D arrays using angular distance
 * @tparam T Type of array elements (float or double)
 * @param a First quaternion array in xyzw format
 * @param b Second quaternion array in xyzw format
 * @param tolerance Angular distance tolerance in radians
 * @return True if quaternions are approximately equal within tolerance
 */
template <typename T>
inline bool
compareQuaternion(const T* quaternion1, const T* quaternion2, T tolerance = static_cast<T>(1e-5)) {
  return compareQuaternion(
      Eigen::Quaternion<T>(quaternion1[3], quaternion1[0], quaternion1[1], quaternion1[2]),
      Eigen::Quaternion<T>(quaternion2[3], quaternion2[0], quaternion2[1], quaternion2[2]),
      tolerance);
}

// Compare two OnlineCalibState objects
inline void compareOnlineCalibState(
    const projectaria::tools::data_provider::OnlineCalibState& state1,
    const projectaria::tools::data_provider::OnlineCalibState& state2) {
  // Compare camera parameters
  EXPECT_EQ(state1.camParameters.size(), state2.camParameters.size());
  for (size_t i = 0; i < state1.camParameters.size(); ++i) {
    EXPECT_EQ(state1.camParameters[i].type, state2.camParameters[i].type);
    EXPECT_EQ(state1.camParameters[i].intrinsics.size(), state2.camParameters[i].intrinsics.size());
    for (size_t j = 0; j < state1.camParameters[i].intrinsics.size(); ++j) {
      EXPECT_EQ(state1.camParameters[i].intrinsics[j], state2.camParameters[i].intrinsics[j]);
    }
    EXPECT_EQ(state1.camParameters[i].readoutTimeSec, state2.camParameters[i].readoutTimeSec);
    EXPECT_EQ(state1.camParameters[i].maxRadiusSquared, state2.camParameters[i].maxRadiusSquared);
    EXPECT_EQ(state1.camParameters[i].imageSize.width, state2.camParameters[i].imageSize.width);
    EXPECT_EQ(state1.camParameters[i].imageSize.height, state2.camParameters[i].imageSize.height);
  }

  // Compare camera extrinsics
  EXPECT_EQ(state1.T_Cam_BodyImu.size(), state2.T_Cam_BodyImu.size());
  for (size_t i = 0; i < state1.T_Cam_BodyImu.size(); ++i) {
    EXPECT_EQ(state1.T_Cam_BodyImu[i].translation(), state2.T_Cam_BodyImu[i].translation());
    EXPECT_EQ(
        state1.T_Cam_BodyImu[i].unit_quaternion().coeffs(),
        state2.T_Cam_BodyImu[i].unit_quaternion().coeffs());
  }

  // Compare time offsets
  EXPECT_EQ(state1.dt_Ref_Cam.size(), state2.dt_Ref_Cam.size());
  for (size_t i = 0; i < state1.dt_Ref_Cam.size(); ++i) {
    EXPECT_EQ(state1.dt_Ref_Cam[i], state2.dt_Ref_Cam[i]);
  }

  // Compare IMU parameters
  EXPECT_EQ(state1.imuModelParameters.size(), state2.imuModelParameters.size());
  for (size_t i = 0; i < state1.imuModelParameters.size(); ++i) {
    const auto& imu1 = state1.imuModelParameters[i];
    const auto& imu2 = state2.imuModelParameters[i];

    EXPECT_EQ(imu1.gyroScaleVec, imu2.gyroScaleVec);
    EXPECT_EQ(imu1.accelScaleVec, imu2.accelScaleVec);
    EXPECT_EQ(imu1.accelBiasMSec2, imu2.accelBiasMSec2);
    EXPECT_EQ(imu1.gyroBiasRadSec, imu2.gyroBiasRadSec);
    EXPECT_EQ(imu1.accelNonorth, imu2.accelNonorth);
    EXPECT_EQ(imu1.gyroNonorth, imu2.gyroNonorth);
    EXPECT_EQ(imu1.gyroGSensitivityRadSecPerMSec2, imu2.gyroGSensitivityRadSecPerMSec2);
    EXPECT_EQ(imu1.dtReferenceAccelSec, imu2.dtReferenceAccelSec);
    EXPECT_EQ(imu1.dtReferenceGyroSec, imu2.dtReferenceGyroSec);
    EXPECT_EQ(imu1.T_Imu_BodyImu.translation(), imu2.T_Imu_BodyImu.translation());
    EXPECT_EQ(
        imu1.T_Imu_BodyImu.unit_quaternion().coeffs(),
        imu2.T_Imu_BodyImu.unit_quaternion().coeffs());
    EXPECT_EQ(imu1.nominalSamplingPeriodSec, imu2.nominalSamplingPeriodSec);
    EXPECT_EQ(imu1.accelSaturationThresholdMSec2, imu2.accelSaturationThresholdMSec2);
    EXPECT_EQ(imu1.gyroSaturationThresholdRadSec, imu2.gyroSaturationThresholdRadSec);
  }
}

} // namespace projectaria::tools::data_provider::test
