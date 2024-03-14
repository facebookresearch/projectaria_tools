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

#include <gtest/gtest.h>
#include <mps/EyeGazeFormat.h>
#include <mps/EyeGazeReader.h>
#include "EyeGaze.h"

using namespace projectaria::tools::mps;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x))

static const std::string testDataFolder = XSTRING(TEST_FOLDER);
float EYEGAZE_ERROR_TOLERANCE = 0.001;

TEST(mps_eyegaze_valid_file, reader) {
  const auto eyegazeValues = readEyeGaze(testDataFolder + "mps_sample/eye_gaze/eyegaze.csv");
  EXPECT_TRUE(eyegazeValues.size() > 0);
  EXPECT_EQ("", eyegazeValues[0].session_uid); // They should be empty
  EXPECT_EQ(0, eyegazeValues[0].vergence.left_yaw);
}

TEST(mps_eyegaze_valid_file_with_session_uid, reader) {
  const auto eyegazeValues =
      readEyeGaze(testDataFolder + "mps_sample/eye_gaze/generalized_eye_gaze.csv");
  EXPECT_TRUE(eyegazeValues.size() > 0);
  EXPECT_NE("", eyegazeValues[0].session_uid); // They should be NON empty
  EXPECT_EQ(0, eyegazeValues[0].vergence.left_yaw);
}

TEST(mps_eyegaze_invalid_file, reader) {
  const auto eyegazeValues = readEyeGaze("");
  EXPECT_TRUE(eyegazeValues.empty());
}

TEST(mps_eyegaze_vergence_valid_file, reader) {
  const auto eyegazeVergenceValues =
      readEyeGaze(testDataFolder + "mps_sample/eye_gaze_vergence/generalized_gaze.csv");
  EXPECT_TRUE(eyegazeVergenceValues.size() > 0);
  EXPECT_NE("", eyegazeVergenceValues[0].session_uid); // They should be NON empty
  ASSERT_NEAR(0.0315, eyegazeVergenceValues[0].vergence.tx_left_eye, 0.0001);
  ASSERT_NEAR(-0.0315, eyegazeVergenceValues[0].vergence.tx_right_eye, 0.0001);
}

// Tests for helper functions

TEST(mps_eyegaze_get_gaze_point_at_depth, helper) {
  const auto gazePointAtDepth =
      getEyeGazePointAtDepth(-0.102910490016660, -0.288851886987686, 1.179526637404006);
  ASSERT_NEAR(-0.116201334110103, gazePointAtDepth.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.334355951558995, gazePointAtDepth.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(1.12516063, gazePointAtDepth.z(), EYEGAZE_ERROR_TOLERANCE);
}

TEST(mps_eyegaze_compute_depth_and_combined_gaze_direction, helper) {
  float depthM = NAN, combinedYawRads = NAN, combinedPitchRads = NAN;
  std::tie(depthM, combinedYawRads, combinedPitchRads) =
      computeDepthAndCombinedGazeDirection(-0.1264797, -0.07706982, -0.26359045);
  ASSERT_NEAR(1.31310134, depthM, EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.10183712, combinedYawRads, EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.26359045, combinedPitchRads, EYEGAZE_ERROR_TOLERANCE);
}

TEST(mps_eyegaze_get_gaze_intersection_point, helper) {
  const Eigen::Vector3d gazePoint = getGazeIntersectionPoint(-0.1264797, -0.07706982, -0.26359045);
  ASSERT_NEAR(-0.12892598, gazePoint.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.34047373, gazePoint.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(1.26162232, gazePoint.z(), EYEGAZE_ERROR_TOLERANCE);
}

TEST(mps_eyegaze_get_gaze_directions, helper) {
  Eigen::Vector3d leftDirection, rightDirection;
  std::tie(leftDirection, rightDirection) = getGazeVectors(-0.1264797, -0.07706982, -0.26359045);
  ASSERT_NEAR(-0.12185171, leftDirection.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.25860713, leftDirection.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.95826641, leftDirection.z(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.07434921, rightDirection.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.25982753, rightDirection.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.96278858, rightDirection.z(), EYEGAZE_ERROR_TOLERANCE);
}
