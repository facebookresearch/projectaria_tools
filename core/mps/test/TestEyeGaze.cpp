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
static const std::string testDataFolderGen2 = XSTRING(TEST_FOLDER_GEN2);
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
  // Regression: after adding the Gen2 union columns to `readEyeGazeVergence`, a Gen1
  // vergence file (no `yaw_rads_cpf`, no per-eye pitch) must NOT trigger the Gen2 backfill
  // path. Gen2-only fields stay at struct defaults.
  EXPECT_FALSE(eyegazeVergenceValues[0].spatial_gaze_point_valid);
  EXPECT_FALSE(eyegazeVergenceValues[0].combined_gaze_valid);
  EXPECT_TRUE(eyegazeVergenceValues[0].spatial_gaze_point_in_cpf.isZero());
  EXPECT_TRUE(eyegazeVergenceValues[0].combined_gaze_origin_in_cpf.isZero());
}

TEST(mps_eyegaze_gen2_valid_file, reader) {
  // Sample is the first 5 rows of a real Gen2 general_eye_gaze.csv recording.
  const auto eyegazeValues =
      readEyeGaze(testDataFolderGen2 + "mps_sample/eye_gaze/general_eye_gaze.csv");
  ASSERT_EQ(5u, eyegazeValues.size());

  const auto& row0 = eyegazeValues[0];
  EXPECT_EQ(1130889351, row0.trackingTimestamp.count());

  // Combined yaw/pitch are read directly from the file (Gen2 has them as columns).
  ASSERT_NEAR(0.07778381, row0.yaw, EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.36552525, row0.pitch, EYEGAZE_ERROR_TOLERANCE);

  // Per-eye yaws and the new Gen2 per-eye pitches.
  ASSERT_NEAR(0.01876480, row0.vergence.left_yaw, EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.13680240, row0.vergence.right_yaw, EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.36495323, row0.vergence.left_pitch, EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.36493456, row0.vergence.right_pitch, EYEGAZE_ERROR_TOLERANCE);

  // Eye origins (hardcoded ±0.0325 in the initial Gen2 release).
  ASSERT_NEAR(0.0325, row0.vergence.tx_left_eye, EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.0325, row0.vergence.tx_right_eye, EYEGAZE_ERROR_TOLERANCE);

  // Reader backfills 3D geometry. combined gaze origin is the midpoint of the two eye
  // origins (the CPF origin for these symmetric eyes). spatial gaze point comes from the ray
  // triangulation in `getGazeVergencePoint`; validity reflects whether the inputs converged
  // on a real point or fell through to the far-gaze fallback.
  EXPECT_TRUE(row0.combined_gaze_valid);
  ASSERT_NEAR(0.0, row0.combined_gaze_origin_in_cpf.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.0, row0.combined_gaze_origin_in_cpf.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.0, row0.combined_gaze_origin_in_cpf.z(), EYEGAZE_ERROR_TOLERANCE);
  // depth is the Euclidean distance from combined origin to spatial gaze point — must match
  // the norm of the backfilled spatial point (combined_origin is at zero here).
  EXPECT_NEAR(row0.spatial_gaze_point_in_cpf.norm(), row0.depth, EYEGAZE_ERROR_TOLERANCE);
  // Whichever branch ran, spatial gaze point must be non-zero (real or 10 m fallback).
  EXPECT_GT(row0.spatial_gaze_point_in_cpf.norm(), 0.f);

  // Gen2 schema has no confidence bounds / session_uid / depth column — those stay at
  // defaults (depth is *only* populated by the backfill, not read from the file).
  EXPECT_EQ(0.f, row0.yaw_low);
  EXPECT_EQ(0.f, row0.pitch_high);
  EXPECT_EQ("", row0.session_uid);
}

TEST(mps_eyegaze_get_gaze_vergence_point, helper) {
  // Two eyes converging on a known target. Constructs per-eye (yaw, pitch) using the same
  // sin/cos polar convention as the helper, so the rays exactly hit `target`.
  const Eigen::Vector3d leftOrigin{0.0325, 0.0, 0.0};
  const Eigen::Vector3d rightOrigin{-0.0325, 0.0, 0.0};
  const Eigen::Vector3d target{0.0, 0.0, 1.0};

  const auto yawPitchToTarget = [](const Eigen::Vector3d& origin, const Eigen::Vector3d& tgt) {
    const Eigen::Vector3d dir = (tgt - origin).normalized();
    return std::make_pair(
        static_cast<float>(std::atan2(dir.x(), dir.z())), static_cast<float>(std::asin(dir.y())));
  };
  const auto [leftYaw, leftPitch] = yawPitchToTarget(leftOrigin, target);
  const auto [rightYaw, rightPitch] = yawPitchToTarget(rightOrigin, target);

  const auto [vergence, isReal] =
      getGazeVergencePoint(leftOrigin, leftYaw, leftPitch, rightOrigin, rightYaw, rightPitch);
  EXPECT_TRUE(isReal);
  ASSERT_NEAR(target.x(), vergence.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(target.y(), vergence.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(target.z(), vergence.z(), EYEGAZE_ERROR_TOLERANCE);
}

TEST(mps_eyegaze_get_gaze_vergence_point_parallel, helper) {
  // Both eyes looking straight ahead — parallel rays trigger the weak-vergence guardrail.
  // The helper signals isReal=false and falls back to
  // cyclopean origin + fused direction * kVergenceFarFallbackDistanceM. For symmetric eyes
  // around the CPF origin, the fallback lands on the +Z axis.
  const Eigen::Vector3d leftOrigin{0.0325, 0.0, 0.0};
  const Eigen::Vector3d rightOrigin{-0.0325, 0.0, 0.0};
  const auto [vergence, isReal] = getGazeVergencePoint(leftOrigin, 0.f, 0.f, rightOrigin, 0.f, 0.f);
  EXPECT_FALSE(isReal);
  ASSERT_NEAR(0.0, vergence.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.0, vergence.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(kVergenceFarFallbackDistanceM, vergence.z(), EYEGAZE_ERROR_TOLERANCE);
}

TEST(mps_eyegaze_get_gaze_vergence_point_divergent, helper) {
  // Left eye at +X looks toward +X, right eye at -X looks toward -X — the rays diverge and
  // the closest-approach solution lies behind both eyes. The behind-eye guardrail must
  // trigger and produce the far-gaze fallback at +Z = kVergenceFarFallbackDistanceM.
  const Eigen::Vector3d leftOrigin{0.0325, 0.0, 0.0};
  const Eigen::Vector3d rightOrigin{-0.0325, 0.0, 0.0};
  const auto [vergence, isReal] =
      getGazeVergencePoint(leftOrigin, 0.1f, 0.0f, rightOrigin, -0.1f, 0.0f);
  EXPECT_FALSE(isReal);
  ASSERT_NEAR(0.0, vergence.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.0, vergence.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(kVergenceFarFallbackDistanceM, vergence.z(), EYEGAZE_ERROR_TOLERANCE);
}

TEST(mps_eyegaze_get_gaze_vergence_point_skew, helper) {
  // Yaws converge on (0, 0, 1) but pitches go opposite directions, so the per-eye rays are
  // highly skew and their closest-approach midpoint is uninformative. The miss-distance
  // guardrail must trigger and return the far-gaze fallback.
  const Eigen::Vector3d leftOrigin{0.0325, 0.0, 0.0};
  const Eigen::Vector3d rightOrigin{-0.0325, 0.0, 0.0};
  const auto [vergence, isReal] =
      getGazeVergencePoint(leftOrigin, -0.0325f, 0.5f, rightOrigin, 0.0325f, -0.5f);
  EXPECT_FALSE(isReal);
  ASSERT_NEAR(kVergenceFarFallbackDistanceM, vergence.norm(), EYEGAZE_ERROR_TOLERANCE);
}

TEST(mps_eyegaze_get_gaze_vergence_point_excessive_depth, helper) {
  // Shallow-angle convergent rays that intersect beyond kVergenceMaxReliableDepthM (6 m) but
  // still pass the vergence-angle, behind-eye, and miss-distance checks. Uses a wider IPD so
  // that the vergence angle exceeds the 0.75° threshold while the intersection depth exceeds
  // 6 m. The max-depth guardrail must trigger and return the far-gaze fallback.
  const Eigen::Vector3d leftOrigin{0.05, 0.0, 0.0};
  const Eigen::Vector3d rightOrigin{-0.05, 0.0, 0.0};
  // Per-eye yaws of ±0.008 rad converge at ≈ IPD/(2·tan(yaw)) ≈ 0.05/0.008 ≈ 6.25 m on +Z.
  const auto [vergence, isReal] =
      getGazeVergencePoint(leftOrigin, -0.008f, 0.f, rightOrigin, 0.008f, 0.f);
  EXPECT_FALSE(isReal);
  ASSERT_NEAR(kVergenceFarFallbackDistanceM, vergence.norm(), EYEGAZE_ERROR_TOLERANCE);
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
