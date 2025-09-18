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

#include <data_provider/json_io/FrontendTypesJson.h>
#include <data_provider/json_io/test/TestCompareUtils.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::json;

OnlineCalibState createExampleOnlineCalibState() {
  OnlineCalibState sampleState;
  // Initialize sampleState with test data
  sampleState.camParameters = {
      {"linear", {500.0f, 500.0f, 320.0f, 240.0f}, 0.0f, 1000000.0f, {640, 480}}};
  sampleState.T_Cam_BodyImu = {Sophus::SE3f(
      Eigen::Quaternionf(0.707f, 0.0f, 0.707f, 0.0f), Eigen::Vector3f(0.1f, 0.2f, 0.3f))};
  sampleState.dt_Ref_Cam = {1000000};
  ImuMeasurementModelParameters<float> imuParams;
  imuParams.gyroScaleVec = Eigen::Vector3f(0.95f, 1.05f, 1.00f);
  imuParams.accelScaleVec = Eigen::Vector3f(1.02f, 0.98f, 1.01f);
  imuParams.accelBiasMSec2 = Eigen::Vector3f(0.01f, -0.02f, 0.03f);
  imuParams.gyroBiasRadSec = Eigen::Vector3f(-0.01f, 0.02f, -0.03f);
  imuParams.accelNonorth = (Eigen::Matrix3f() << 0.999f,
                            0.001f,
                            0.002f,
                            -0.001f,
                            0.999f,
                            0.003f,
                            0.002f,
                            -0.003f,
                            0.999f)
                               .finished();
  imuParams.gyroNonorth = (Eigen::Matrix3f() << 0.998f,
                           0.002f,
                           -0.001f,
                           0.001f,
                           0.999f,
                           0.004f,
                           -0.002f,
                           -0.004f,
                           0.998f)
                              .finished();
  imuParams.gyroGSensitivityRadSecPerMSec2 =
      (Eigen::Matrix3f() << 0.0f, 0.001f, 0.002f, 0.001f, 0.0f, 0.003f, 0.002f, 0.003f, 0.0f)
          .finished();
  imuParams.dtReferenceAccelSec = 0.002f;
  imuParams.dtReferenceGyroSec = 0.003f;
  imuParams.T_Imu_BodyImu = Sophus::SE3f(
      Eigen::Quaternionf(0.707f, 0.0f, 0.707f, 0.0f), Eigen::Vector3f(0.05f, -0.05f, 0.1f));
  imuParams.nominalSamplingPeriodSec = 0.012f;
  imuParams.accelSaturationThresholdMSec2 = 120.0f;
  imuParams.gyroSaturationThresholdRadSec = 12.0f;

  sampleState.imuModelParameters = {imuParams};
  return sampleState;
}

void compare(
    const ProjectionModelParameters<float>& lhs,
    const ProjectionModelParameters<float>& rhs) {
  EXPECT_EQ(lhs.type, rhs.type);
  EXPECT_EQ(lhs.imageSize.height, rhs.imageSize.height);
  EXPECT_EQ(lhs.imageSize.width, rhs.imageSize.width);
  EXPECT_TRUE(isClose(lhs.intrinsics, rhs.intrinsics));
  EXPECT_EQ(lhs.readoutTimeSec, rhs.readoutTimeSec);
  EXPECT_EQ(lhs.maxRadiusSquared, rhs.maxRadiusSquared);
}

void compare(
    const ImuMeasurementModelParameters<float>& lhs,
    const ImuMeasurementModelParameters<float>& rhs) {
  EXPECT_TRUE(isClose(lhs.gyroScaleVec, rhs.gyroScaleVec));
  EXPECT_TRUE(isClose(lhs.accelScaleVec, rhs.accelScaleVec));
  EXPECT_TRUE(isClose(lhs.accelBiasMSec2, rhs.accelBiasMSec2));
  EXPECT_TRUE(isClose(lhs.gyroBiasRadSec, rhs.gyroBiasRadSec));
  EXPECT_TRUE(isClose(lhs.accelNonorth, rhs.accelNonorth));
  EXPECT_TRUE(isClose(lhs.gyroNonorth, rhs.gyroNonorth));
  EXPECT_TRUE(isClose(lhs.gyroGSensitivityRadSecPerMSec2, rhs.gyroGSensitivityRadSecPerMSec2));
  EXPECT_TRUE(isClose(lhs.dtReferenceAccelSec, rhs.dtReferenceAccelSec));
  EXPECT_TRUE(isClose(lhs.dtReferenceGyroSec, rhs.dtReferenceGyroSec));
  EXPECT_TRUE(isClose(lhs.T_Imu_BodyImu, rhs.T_Imu_BodyImu));
  EXPECT_TRUE(isClose(lhs.nominalSamplingPeriodSec, rhs.nominalSamplingPeriodSec));
  EXPECT_TRUE(isClose(lhs.accelSaturationThresholdMSec2, rhs.accelSaturationThresholdMSec2));
  EXPECT_TRUE(isClose(lhs.gyroSaturationThresholdRadSec, rhs.gyroSaturationThresholdRadSec));
}

void compare(const OnlineCalibState& lhs, const OnlineCalibState& rhs) {
  EXPECT_TRUE(isClose(lhs.dt_Ref_Cam, rhs.dt_Ref_Cam));
  EXPECT_TRUE(isClose(lhs.T_Cam_BodyImu, rhs.T_Cam_BodyImu));
  EXPECT_EQ(lhs.camParameters.size(), rhs.camParameters.size());

  for (int i = 0; i < lhs.camParameters.size(); ++i) {
    compare(lhs.camParameters[i], rhs.camParameters[i]);
  }
  EXPECT_EQ(lhs.imuModelParameters.size(), rhs.imuModelParameters.size());
  for (int i = 0; i < lhs.imuModelParameters.size(); ++i) {
    compare(lhs.imuModelParameters[i], rhs.imuModelParameters[i]);
  }
}

TEST(FrontendTypesJsonTest, roundtripTest) {
  // Round trip test OnlineCalibState -> Json -> OnlineCalibState
  OnlineCalibState sampleState = createExampleOnlineCalibState();

  std::string ocalString = onlineCalibStateToJsonStr(sampleState);

  const auto& convertedOnlineCalibState = onlineCalibStateFromJsonStr(ocalString);

  EXPECT_TRUE(convertedOnlineCalibState.has_value());

  compare(sampleState, convertedOnlineCalibState.value());
}
