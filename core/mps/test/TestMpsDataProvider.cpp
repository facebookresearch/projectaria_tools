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

#include <mps/MpsDataProvider.h>

#include <filesystem>

#include <gtest/gtest.h>

using namespace projectaria::tools::mps;
using namespace projectaria::tools::data_provider;

namespace fs = std::filesystem;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x))

static const std::string testDataFolder = XSTRING(TEST_FOLDER);
static const std::string mpsRootFolder = testDataFolder + "mps_sample";
static const std::string vrsPath = testDataFolder + "sample.vrs";

TEST(MpsDataPathsProvider, PathsAPI) {
  const auto dataPathsProvider = MpsDataPathsProvider(mpsRootFolder);
  const auto dataPaths = dataPathsProvider.getDataPaths();

  EXPECT_TRUE(fs::exists(dataPaths.eyegaze.generalEyegaze));
  EXPECT_TRUE(fs::exists(dataPaths.eyegaze.summary));
  EXPECT_TRUE(fs::exists(dataPaths.slam.closedLoopTrajectory));
  EXPECT_TRUE(fs::exists(dataPaths.slam.openLoopTrajectory));
  EXPECT_TRUE(fs::exists(dataPaths.slam.semidensePoints));
  EXPECT_TRUE(fs::exists(dataPaths.slam.semidenseObservations));
  EXPECT_TRUE(fs::exists(dataPaths.slam.onlineCalibration));
  EXPECT_TRUE(fs::exists(dataPaths.slam.summary));
  EXPECT_TRUE(fs::exists(dataPaths.handTracking.wristAndPalmPoses));
  EXPECT_TRUE(fs::exists(dataPaths.handTracking.summary));
}

TEST(MpsDataProvider, DataExists) {
  const auto dataPathsProvider = MpsDataPathsProvider(mpsRootFolder);
  const auto dataPaths = dataPathsProvider.getDataPaths();
  const auto dp = MpsDataProvider(dataPaths);

  EXPECT_TRUE(dp.hasGeneralEyeGaze());
  EXPECT_FALSE(dp.hasPersonalizedEyeGaze());
  EXPECT_TRUE(dp.hasOpenLoopPoses());
  EXPECT_TRUE(dp.hasClosedLoopPoses());
  EXPECT_TRUE(dp.hasOnlineCalibrations());
  EXPECT_TRUE(dp.hasSemidensePointCloud());
  EXPECT_TRUE(dp.hasSemidenseObservations());
  EXPECT_TRUE(dp.hasWristAndPalmPoses());
}

TEST(MpsDataProvider, DataQuerying) {
  const auto dataPathsProvider = MpsDataPathsProvider(mpsRootFolder);
  const auto dataPaths = dataPathsProvider.getDataPaths();
  auto dp = MpsDataProvider(dataPaths);

  // check semidense
  EXPECT_TRUE(dp.getSemidensePointCloud().size() > 0);
  EXPECT_TRUE(dp.getSemidenseObservations().size() > 0);

  // get the first set of data and check its validity
  auto maybeOpenLoopPose = dp.getOpenLoopPose(0);
  auto maybeClosedLoopPose = dp.getClosedLoopPose(0);
  auto maybeEyeGaze = dp.getGeneralEyeGaze(0);
  auto maybeOnlineCalib = dp.getOnlineCalibration(0);
  auto maybeWristAndPalmPose = dp.getWristAndPalmPose(0);
  auto maybeRgbPose = dp.getRgbCorrectedClosedLoopPose(0);
  auto maybeRgbTs = dp.getRgbCorrectedTimestampNs(0);
  EXPECT_TRUE(maybeOpenLoopPose.has_value());
  EXPECT_TRUE(maybeClosedLoopPose.has_value());
  EXPECT_TRUE(maybeEyeGaze.has_value());
  EXPECT_TRUE(maybeOnlineCalib.has_value());
  EXPECT_TRUE(maybeWristAndPalmPose.has_value());
  EXPECT_TRUE(maybeRgbPose.has_value());
  EXPECT_TRUE(maybeRgbTs.has_value());

  // check that invalid queries throws an exception
  EXPECT_THROW(dp.getPersonalizedEyeGaze(0), std::runtime_error);

  // test some false cases
  maybeOpenLoopPose = dp.getOpenLoopPose(0, TimeQueryOptions::Before);
  maybeClosedLoopPose = dp.getClosedLoopPose(0, TimeQueryOptions::Before);
  maybeEyeGaze = dp.getGeneralEyeGaze(0, TimeQueryOptions::Before);
  maybeOnlineCalib = dp.getOnlineCalibration(0, TimeQueryOptions::Before);
  maybeRgbPose = dp.getRgbCorrectedClosedLoopPose(0, TimeQueryOptions::Before);
  maybeRgbTs = dp.getRgbCorrectedTimestampNs(0, TimeQueryOptions::Before);
  EXPECT_FALSE(maybeOpenLoopPose.has_value());
  EXPECT_FALSE(maybeClosedLoopPose.has_value());
  EXPECT_FALSE(maybeEyeGaze.has_value());
  EXPECT_FALSE(maybeOnlineCalib.has_value());
  EXPECT_FALSE(maybeRgbPose.has_value());
  EXPECT_FALSE(maybeRgbTs.has_value());
}

TEST(MpsDataProvider, InterpolatedPoseQuerying) {
  const auto dataPathsProvider = MpsDataPathsProvider(mpsRootFolder);
  const auto dataPaths = dataPathsProvider.getDataPaths();
  auto dp = MpsDataProvider(dataPaths);
  EXPECT_TRUE(dp.hasClosedLoopPoses());

  // Test for case where query before the first timestamp
  auto maybeOpenLoopPose = dp.getInterpolatedClosedLoopPose(0);
  EXPECT_FALSE(maybeOpenLoopPose.has_value());

  // Test for case where query after the last timestamp
  maybeOpenLoopPose = dp.getInterpolatedClosedLoopPose(9999999999 * 1e3);
  EXPECT_FALSE(maybeOpenLoopPose.has_value());

  // Test for case where query is exactly at one timestamp
  maybeOpenLoopPose = dp.getInterpolatedClosedLoopPose(149202610 * 1e3);
  EXPECT_TRUE(maybeOpenLoopPose.has_value());
  EXPECT_EQ(maybeOpenLoopPose->trackingTimestamp.count(), 149202610);
  EXPECT_TRUE(maybeOpenLoopPose->deviceLinearVelocity_device.isApprox(
      Eigen::Vector3d(0.038913, -0.322935, 0.242351), 1e-4));

  // Test for cases of interpolation between 149203459 and 149204459
  const auto resultPose = dp.getInterpolatedClosedLoopPose(149203959000);
  EXPECT_TRUE(resultPose.has_value());
  EXPECT_EQ(resultPose->graphUid, "ea7288b8-b6a0-012d-8156-257aba6bd796");
  EXPECT_EQ(resultPose->qualityScore, 0.5);
  EXPECT_TRUE(resultPose->T_world_device.translation().isApprox(
      Sophus::Vector3d(-0.0001245, -0.006745, 0.0004035), 1e-4));

  int64_t result_utcTimestamp = resultPose->utcTimestamp.count();
  EXPECT_EQ(result_utcTimestamp, 1686767222002923694);

  EXPECT_TRUE(resultPose->gravity_world.isApprox(Eigen::Vector3d(0, 0, -9.81), 1e-4));
  EXPECT_TRUE(resultPose->angularVelocity_device.isApprox(
      Eigen::Vector3d(-0.051098, 0.056527, -0.0694205), 1e-4));
  EXPECT_TRUE(resultPose->deviceLinearVelocity_device.isApprox(
      Eigen::Vector3d(0.040185, -0.3245015, 0.2434535), 1e-4));
}
