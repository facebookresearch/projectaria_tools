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

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <mps/EyeGazeReader.h>
#include "EyeGaze.h"

using namespace projectaria::tools::mps;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x))

static const std::string testDataFolder = XSTRING(TEST_FOLDER);
static const std::string testDataFolderGen2 = XSTRING(TEST_FOLDER_GEN2);
float EYEGAZE_ERROR_TOLERANCE = 0.001;

namespace {

// One row of a Gen2 eye gaze CSV, from the bundled general_eye_gaze.csv sample.
const std::string kGen2Header =
    "tracking_timestamp_us,yaw_rads_cpf,pitch_rads_cpf,left_yaw_rads_cpf,right_yaw_rads_cpf,"
    "left_pitch_rads_cpf,right_pitch_rads_cpf,tx_left_eye_cpf,ty_left_eye_cpf,tz_left_eye_cpf,"
    "tx_right_eye_cpf,ty_right_eye_cpf,tz_right_eye_cpf";
const std::string kGen2Row =
    "1130889351,0.07778378,-0.36552523,0.01876476,0.13680238,-0.36495319,-0.36493456,"
    "0.0325,0.0,0.0,-0.0325,0.0,0.0";
// Both eyes looking straight ahead: the gaze rays never converge.
const std::string kStraightAheadGen2Row =
    "1,0.0,0.0,0.0,0.0,0.0,0.0,0.0325,0.0,0.0,-0.0325,0.0,0.0";

// Writes a one-row Gen2 CSV, `gazeCells` followed by `extraCells`, with `extraColumns` appended
// to the header, and removes it on destruction.
struct Gen2CsvFile {
  std::string path;
  Gen2CsvFile(
      const std::string& extraColumns,
      const std::string& extraCells,
      const std::string& gazeCells = kGen2Row) {
    path = (std::filesystem::temp_directory_path() /
            ("eye_gaze_test_" +
             std::to_string(
                 std::hash<std::string>{}(extraColumns + "|" + extraCells + "|" + gazeCells)) +
             ".csv"))
               .string();
    std::ofstream out(path);
    if (!out) {
      ADD_FAILURE() << "Cannot write " << path;
    }
    out << kGen2Header << extraColumns << "\n" << gazeCells << extraCells << "\n";
  }
  ~Gen2CsvFile() {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
  }
};

} // namespace

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
  // triangulation in `getGazeVergencePoint`, or is the far-gaze fallback when the rays do not
  // converge.
  EXPECT_TRUE(row0.combined_gaze_valid);
  ASSERT_NEAR(0.0, row0.combined_gaze_origin_in_cpf.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.0, row0.combined_gaze_origin_in_cpf.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.0, row0.combined_gaze_origin_in_cpf.z(), EYEGAZE_ERROR_TOLERANCE);
  // depth is the Euclidean distance from combined origin to spatial gaze point — must match
  // the norm of the backfilled spatial point (combined_origin is at zero here).
  EXPECT_NEAR(row0.spatial_gaze_point_in_cpf.norm(), row0.depth, EYEGAZE_ERROR_TOLERANCE);
  // Whichever branch ran, spatial gaze point must be non-zero (real or far-gaze fallback).
  EXPECT_GT(row0.spatial_gaze_point_in_cpf.norm(), 0.f);

  // Gen2 schema has no confidence bounds / session_uid / depth column — those stay at
  // defaults (depth is *only* populated by the backfill, not read from the file).
  EXPECT_EQ(0.f, row0.yaw_low);
  EXPECT_EQ(0.f, row0.pitch_high);
  EXPECT_EQ("", row0.session_uid);
}

TEST(mps_eyegaze_far_gaze_fallback_distance, helper) {
  // Same far-gaze distance as the on-device eye tracking and the MPS pipeline.
  EXPECT_EQ(100.0, kVergenceFarFallbackDistanceM);
}

TEST(mps_eyegaze_gen2_far_gaze, reader) {
  // The rays never converge, so the spatial point is the far-gaze fallback. It is still a real
  // gaze point, and valid.
  const Gen2CsvFile csv("", "", kStraightAheadGen2Row);
  const auto rows = readEyeGaze(csv.path);
  ASSERT_EQ(1u, rows.size());
  EXPECT_TRUE(rows[0].spatial_gaze_point_valid);
  ASSERT_NEAR(kVergenceFarFallbackDistanceM, rows[0].spatial_gaze_point_in_cpf.z(), 1e-3);
  ASSERT_NEAR(kVergenceFarFallbackDistanceM, rows[0].depth, 1e-3);
}

TEST(mps_eyegaze_gen2_far_gaze_off_axis, reader) {
  // Both eyes look 0.5 rad right and 0.5 rad up. The far-gaze point lies along the direction that
  // yaw and pitch describe in the projective convention, (tan(yaw), tan(pitch), 1).
  const Gen2CsvFile csv("", "", "1,0.5,0.5,0.5,0.5,0.5,0.5,0.0325,0.0,0.0,-0.0325,0.0,0.0");
  const auto rows = readEyeGaze(csv.path);
  ASSERT_EQ(1u, rows.size());
  const Eigen::Vector3d expected = kVergenceFarFallbackDistanceM *
      Eigen::Vector3d(std::tan(0.5), std::tan(0.5), 1.0).normalized();
  EXPECT_TRUE(rows[0].spatial_gaze_point_valid);
  ASSERT_NEAR(expected.x(), rows[0].spatial_gaze_point_in_cpf.x(), 1e-3);
  ASSERT_NEAR(expected.y(), rows[0].spatial_gaze_point_in_cpf.y(), 1e-3);
  ASSERT_NEAR(expected.z(), rows[0].spatial_gaze_point_in_cpf.z(), 1e-3);
}

TEST(mps_eyegaze_get_gaze_vergence_point, helper) {
  // Two eyes converging on an off-axis target. Per-eye (yaw, pitch) follow the projective
  // convention of the eye gaze outputs, yaw = atan2(x, z) and pitch = atan2(y, z), so the rays
  // exactly hit `target`.
  const Eigen::Vector3d leftOrigin{0.0325, 0.0, 0.0};
  const Eigen::Vector3d rightOrigin{-0.0325, 0.0, 0.0};
  const Eigen::Vector3d target{0.3, 0.2, 1.0};

  const auto yawPitchToTarget = [](const Eigen::Vector3d& origin, const Eigen::Vector3d& tgt) {
    const Eigen::Vector3d dir = tgt - origin;
    return std::make_pair(
        static_cast<float>(std::atan2(dir.x(), dir.z())),
        static_cast<float>(std::atan2(dir.y(), dir.z())));
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

TEST(mps_eyegaze_gen2_validity, reader) {
  const auto rows =
      readEyeGaze(testDataFolderGen2 + "mps_sample/eye_gaze_with_validity/general_eye_gaze.csv");
  ASSERT_EQ(5u, rows.size());

  // Both eyes valid.
  EXPECT_FLOAT_EQ(0.98431f, rows[0].vergence.left_valid_score);
  EXPECT_FLOAT_EQ(0.95612f, rows[0].vergence.right_valid_score);
  EXPECT_TRUE(rows[0].vergence.left_gaze_valid);
  EXPECT_TRUE(rows[0].vergence.right_gaze_valid);
  EXPECT_TRUE(rows[0].combined_gaze_valid);
  EXPECT_TRUE(rows[0].spatial_gaze_point_valid);

  // Left eye invalid: the combined and spatial flags need both eyes, the angles are still read.
  EXPECT_FALSE(rows[1].vergence.left_gaze_valid);
  EXPECT_TRUE(rows[1].vergence.right_gaze_valid);
  EXPECT_FALSE(rows[1].combined_gaze_valid);
  EXPECT_FALSE(rows[1].spatial_gaze_point_valid);
  ASSERT_NEAR(0.01539704, rows[1].vergence.left_yaw, EYEGAZE_ERROR_TOLERANCE);

  // The flags are read as written, not re-derived from the scores.
  EXPECT_TRUE(rows[2].vergence.left_gaze_valid);
  EXPECT_FALSE(rows[2].vergence.right_gaze_valid);

  EXPECT_FALSE(rows[3].vergence.left_gaze_valid);
  EXPECT_FALSE(rows[3].vergence.right_gaze_valid);
}

TEST(mps_eyegaze_gen2_without_validity, reader) {
  // Files written before the validity columns behave as before: every eye valid, no score.
  const auto rows = readEyeGaze(testDataFolderGen2 + "mps_sample/eye_gaze/general_eye_gaze.csv");
  ASSERT_EQ(5u, rows.size());
  for (const auto& row : rows) {
    EXPECT_TRUE(row.vergence.left_gaze_valid);
    EXPECT_TRUE(row.vergence.right_gaze_valid);
    EXPECT_TRUE(std::isnan(row.vergence.left_valid_score));
    EXPECT_TRUE(std::isnan(row.vergence.right_valid_score));
    EXPECT_TRUE(row.combined_gaze_valid);
    EXPECT_TRUE(row.spatial_gaze_point_valid);
  }
}

TEST(mps_eyegaze_gen2_scores_without_flags, reader) {
  const Gen2CsvFile csv(",left_eye_valid_score,right_eye_valid_score", ",0.2,0.9");
  const auto rows = readEyeGaze(csv.path);
  ASSERT_EQ(1u, rows.size());
  EXPECT_FLOAT_EQ(0.2f, rows[0].vergence.left_valid_score);
  EXPECT_FLOAT_EQ(0.9f, rows[0].vergence.right_valid_score);
  EXPECT_TRUE(rows[0].vergence.left_gaze_valid);
  EXPECT_TRUE(rows[0].vergence.right_gaze_valid);
}

TEST(mps_eyegaze_gen2_empty_validity_cells, reader) {
  // A model without a validity output leaves the cells empty: no score, flags as before.
  const Gen2CsvFile csv(
      ",left_eye_valid_score,right_eye_valid_score,left_eye_valid,right_eye_valid", ",,,,");
  const auto rows = readEyeGaze(csv.path);
  ASSERT_EQ(1u, rows.size());
  EXPECT_TRUE(std::isnan(rows[0].vergence.left_valid_score));
  EXPECT_TRUE(std::isnan(rows[0].vergence.right_valid_score));
  EXPECT_TRUE(rows[0].vergence.left_gaze_valid);
  EXPECT_TRUE(rows[0].vergence.right_gaze_valid);
  EXPECT_TRUE(rows[0].combined_gaze_valid);
}

TEST(mps_eyegaze_gen2_malformed_validity, reader) {
  EXPECT_TRUE(readEyeGaze(Gen2CsvFile(",left_eye_valid_score", ",0.5").path).empty());
  EXPECT_TRUE(readEyeGaze(Gen2CsvFile(",left_eye_valid,right_eye_valid", ",1,1").path).empty());
  EXPECT_TRUE(readEyeGaze(Gen2CsvFile(
                              ",left_eye_valid_score,right_eye_valid_score,left_eye_valid,"
                              "right_eye_valid",
                              ",0.5,0.5,2,1")
                              .path)
                  .empty());
  EXPECT_TRUE(
      readEyeGaze(Gen2CsvFile(",left_eye_valid_score,right_eye_valid_score", ",0.5,abc").path)
          .empty());
}

TEST(mps_eyegaze_gen2_malformed_validity_after_valid_rows, reader) {
  // A malformed validity cell rejects the whole file, not just the rows from it on.
  const std::string columns =
      ",left_eye_valid_score,right_eye_valid_score,left_eye_valid,right_eye_valid";
  for (const std::string badCells : {",0.9,abc,1,1", ",0.9,0.8,1,2"}) {
    std::string cells = ",0.9,0.8,1,1\n";
    cells += kGen2Row;
    cells += badCells;
    const Gen2CsvFile csv(columns, cells);
    EXPECT_TRUE(readEyeGaze(csv.path).empty()) << badCells;
  }
}

TEST(mps_eyegaze_gen2_valid_score_out_of_range, reader) {
  // Scores are in [0, 1]; a non-finite or clearly out-of-range score rejects the file.
  for (const std::string cells : {",0.5,inf", ",nan,0.5", ",0.5,2.5", ",-0.5,0.5"}) {
    EXPECT_TRUE(
        readEyeGaze(Gen2CsvFile(",left_eye_valid_score,right_eye_valid_score", cells).path).empty())
        << cells;
  }
}

TEST(mps_eyegaze_gen2_valid_score_rounding, reader) {
  // A score computed in float can land just outside [0, 1]; such a score still reads.
  const Gen2CsvFile csv(",left_eye_valid_score,right_eye_valid_score", ",-1e-7,1.0000001");
  const auto rows = readEyeGaze(csv.path);
  ASSERT_EQ(1u, rows.size());
  EXPECT_NEAR(0.f, rows[0].vergence.left_valid_score, 1e-6);
  EXPECT_NEAR(1.f, rows[0].vergence.right_valid_score, 1e-6);
}

TEST(mps_eyegaze_gen2_far_gaze_with_invalid_eye, reader) {
  // An invalid eye invalidates the spatial point; the point and depth are still filled in.
  const Gen2CsvFile csv(
      ",left_eye_valid_score,right_eye_valid_score,left_eye_valid,right_eye_valid",
      ",0.5,0.9,0,1",
      kStraightAheadGen2Row);
  const auto rows = readEyeGaze(csv.path);
  ASSERT_EQ(1u, rows.size());
  EXPECT_FALSE(rows[0].spatial_gaze_point_valid);
  ASSERT_NEAR(kVergenceFarFallbackDistanceM, rows[0].spatial_gaze_point_in_cpf.z(), 1e-3);
  ASSERT_NEAR(kVergenceFarFallbackDistanceM, rows[0].depth, 1e-3);
}

namespace {

// A frame as read from a file: flags at the default threshold 0.7, both eyes scored.
EyeGaze scoredFrame(float leftScore, float rightScore) {
  EyeGaze gaze;
  gaze.vergence.left_valid_score = leftScore;
  gaze.vergence.right_valid_score = rightScore;
  gaze.vergence.left_gaze_valid = leftScore >= 0.7f;
  gaze.vergence.right_gaze_valid = rightScore >= 0.7f;
  gaze.combined_gaze_valid = gaze.vergence.left_gaze_valid && gaze.vergence.right_gaze_valid;
  gaze.spatial_gaze_point_valid = gaze.combined_gaze_valid;
  return gaze;
}

} // namespace

TEST(mps_eyegaze_valid_score_threshold, lower_threshold_makes_an_eye_valid) {
  const EyeGaze gaze = applyValidScoreThreshold(scoredFrame(0.5f, 0.9f), 0.4f);
  EXPECT_TRUE(gaze.vergence.left_gaze_valid);
  EXPECT_TRUE(gaze.vergence.right_gaze_valid);
  EXPECT_TRUE(gaze.combined_gaze_valid);
  EXPECT_TRUE(gaze.spatial_gaze_point_valid);
}

TEST(mps_eyegaze_valid_score_threshold, higher_threshold_makes_an_eye_invalid) {
  const EyeGaze gaze = applyValidScoreThreshold(scoredFrame(0.75f, 0.9f), 0.8f);
  EXPECT_FALSE(gaze.vergence.left_gaze_valid);
  EXPECT_TRUE(gaze.vergence.right_gaze_valid);
  EXPECT_FALSE(gaze.combined_gaze_valid);
  EXPECT_FALSE(gaze.spatial_gaze_point_valid);
}

TEST(mps_eyegaze_valid_score_threshold, score_equal_to_threshold_is_valid) {
  const EyeGaze gaze = applyValidScoreThreshold(scoredFrame(0.6f, 0.6f), 0.6f);
  EXPECT_TRUE(gaze.vergence.left_gaze_valid);
  EXPECT_TRUE(gaze.vergence.right_gaze_valid);
}

TEST(mps_eyegaze_valid_score_threshold, frame_without_scores_is_unchanged) {
  EyeGaze frame;
  frame.vergence.left_gaze_valid = false;
  frame.combined_gaze_valid = false;
  frame.spatial_gaze_point_valid = false;
  const EyeGaze gaze = applyValidScoreThreshold(frame, 0.1f);
  EXPECT_FALSE(gaze.vergence.left_gaze_valid);
  EXPECT_TRUE(gaze.vergence.right_gaze_valid);
  EXPECT_FALSE(gaze.combined_gaze_valid);
  EXPECT_FALSE(gaze.spatial_gaze_point_valid);
}

TEST(mps_eyegaze_valid_score_threshold, depends_only_on_the_last_threshold) {
  const EyeGaze frame = scoredFrame(0.5f, 0.9f);
  const EyeGaze twice = applyValidScoreThreshold(applyValidScoreThreshold(frame, 0.4f), 0.7f);
  const EyeGaze once = applyValidScoreThreshold(frame, 0.7f);
  EXPECT_EQ(once.vergence.left_gaze_valid, twice.vergence.left_gaze_valid);
  EXPECT_EQ(once.combined_gaze_valid, twice.combined_gaze_valid);
  EXPECT_EQ(once.spatial_gaze_point_valid, twice.spatial_gaze_point_valid);
}

TEST(mps_eyegaze_valid_score_threshold, rejects_thresholds_outside_zero_to_one) {
  const EyeGaze frame = scoredFrame(0.5f, 0.9f);
  EXPECT_THROW(applyValidScoreThreshold(frame, -0.1f), std::invalid_argument);
  EXPECT_THROW(applyValidScoreThreshold(frame, 1.1f), std::invalid_argument);
  EXPECT_THROW(
      applyValidScoreThreshold(frame, std::numeric_limits<float>::quiet_NaN()),
      std::invalid_argument);
  EXPECT_THROW(applyValidScoreThreshold(EyeGazes{}, 2.f), std::invalid_argument);
}

TEST(mps_eyegaze_valid_score_threshold, applies_to_every_frame) {
  const EyeGazes gazes =
      applyValidScoreThreshold(EyeGazes{scoredFrame(0.5f, 0.9f), scoredFrame(0.3f, 0.9f)}, 0.4f);
  ASSERT_EQ(2u, gazes.size());
  EXPECT_TRUE(gazes[0].vergence.left_gaze_valid);
  EXPECT_FALSE(gazes[1].vergence.left_gaze_valid);
}
