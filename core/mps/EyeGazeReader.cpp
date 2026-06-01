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

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#ifndef CSV_IO_NO_THREAD
#define CSV_IO_NO_THREAD
#endif
#include "EyeGazeReader.h"
#include "fast-cpp-csv-parser/csv.h"

namespace projectaria::tools::mps {

namespace {
// Tunable guardrails for `getGazeVergencePoint`. The minimum vergence angle, miss-distance
// threshold, and maximum reliable depth match the constants used by the ray triangulation in
// the eye tracking pipeline that produces the CSV.
constexpr double kPi = 3.14159265358979323846; // M_PI is not standard C++.
constexpr double kVergenceMinAngleDeg = 0.75;
constexpr double kVergenceMaxMissDistanceM = 0.03; // 3 cm
constexpr double kVergenceMaxReliableDepthM = 6.0;
constexpr double kTinyNormSq = 1e-12; // Compared against `vec.dot(vec)` (a squared norm).

Eigen::Vector3d yawPitchToDirection(float yawRads, float pitchRads) {
  const double cp = std::cos(pitchRads);
  return {std::sin(yawRads) * cp, std::sin(pitchRads), std::cos(yawRads) * cp};
}

Eigen::Vector3d fuseDirections(const Eigen::Vector3d& d1, const Eigen::Vector3d& d2) {
  const Eigen::Vector3d sum = d1 + d2;
  if (sum.dot(sum) < kTinyNormSq) {
    return {0.0, 0.0, 1.0};
  }
  return sum.normalized();
}
} // namespace

std::pair<Eigen::Vector3d, bool> getGazeVergencePoint(
    const Eigen::Vector3d& leftEyeOrigin,
    float leftYawRads,
    float leftPitchRads,
    const Eigen::Vector3d& rightEyeOrigin,
    float rightYawRads,
    float rightPitchRads) {
  static const double kCosMinVergence = std::cos(kVergenceMinAngleDeg * kPi / 180.0);

  const Eigen::Vector3d d1 = yawPitchToDirection(leftYawRads, leftPitchRads);
  const Eigen::Vector3d d2 = yawPitchToDirection(rightYawRads, rightPitchRads);
  const Eigen::Vector3d cyclopeanOrigin = 0.5 * (leftEyeOrigin + rightEyeOrigin);

  auto farGazeFallback = [&]() -> Eigen::Vector3d {
    return cyclopeanOrigin + fuseDirections(d1, d2) * kVergenceFarFallbackDistanceM;
  };

  const double d1d2 = std::clamp(d1.dot(d2), -1.0, 1.0);
  const double denom = 1.0 - d1d2 * d1d2; // sin²(angle); valid because d1, d2 are unit vectors.

  // Weak vergence or numerically parallel rays.
  if (d1d2 > kCosMinVergence || std::abs(denom) < kTinyNormSq) {
    return {farGazeFallback(), false};
  }

  // Standard closest-points solve.
  const Eigen::Vector3d p12 = leftEyeOrigin - rightEyeOrigin;
  const double d1p12 = d1.dot(p12);
  const double d2p12 = d2.dot(p12);
  const double t = (d1d2 * d2p12 - d1p12) / denom;
  const double s = (d2p12 - d1d2 * d1p12) / denom;

  // Behind-eye / diverging.
  if (t < 0.0 || s < 0.0) {
    return {farGazeFallback(), false};
  }

  const Eigen::Vector3d pL = leftEyeOrigin + t * d1;
  const Eigen::Vector3d pR = rightEyeOrigin + s * d2;
  const Eigen::Vector3d vergencePoint = 0.5 * (pL + pR);

  // Skew rays whose closest points are too far apart.
  if ((pL - pR).norm() > kVergenceMaxMissDistanceM) {
    return {farGazeFallback(), false};
  }

  // Excessive forward depth.
  if (fuseDirections(d1, d2).dot(vergencePoint - cyclopeanOrigin) > kVergenceMaxReliableDepthM) {
    return {farGazeFallback(), false};
  }

  return {vergencePoint, true};
}

constexpr std::array<const char*, 9> kEyeGazeColumns = {
    "tracking_timestamp_us",
    "yaw_rads_cpf",
    "pitch_rads_cpf",
    "depth_m",
    "yaw_low_rads_cpf",
    "pitch_low_rads_cpf",
    "yaw_high_rads_cpf",
    "pitch_high_rads_cpf",
    "session_uid", // V2: Added for calibrated eye gaze sessions
};

// Columns understood by `readEyeGazeVergence`. The set is the union of the Gen1 vergence
// schema and the Gen2 ET schema (`yaw_rads_cpf`, `left_pitch_rads_cpf`,
// `right_pitch_rads_cpf`). Per-row column presence is detected via `has_column`, so Gen1
// files (no per-eye pitch) and Gen2 files (no depth / confidence bounds / session_uid) both
// parse correctly without separate readers.
constexpr std::array<const char*, 21> kEyeGazeVergenceColumns = {
    "tracking_timestamp_us",
    "left_yaw_rads_cpf",
    "right_yaw_rads_cpf",
    "pitch_rads_cpf",
    "depth_m",
    "left_yaw_low_rads_cpf",
    "right_yaw_low_rads_cpf",
    "pitch_low_rads_cpf",
    "left_yaw_high_rads_cpf",
    "right_yaw_high_rads_cpf",
    "pitch_high_rads_cpf",
    "tx_left_eye_cpf",
    "ty_left_eye_cpf",
    "tz_left_eye_cpf",
    "tx_right_eye_cpf",
    "ty_right_eye_cpf",
    "tz_right_eye_cpf",
    "session_uid",
    "yaw_rads_cpf",
    "left_pitch_rads_cpf",
    "right_pitch_rads_cpf",
};

EyeGazes readEyeGazeVergence(const std::string& path) {
  EyeGazes eyeGazeVergences;
  try {
    io::CSVReader<kEyeGazeVergenceColumns.size()> csv(path);

    // Read in the CSV header. `ignore_missing_column` makes columns absent from the file
    // (e.g. Gen1's `depth_m` in a Gen2 file, or Gen2's `left_pitch_rads_cpf` in a Gen1 file)
    // safe to bind: their corresponding `read_row` arguments retain whatever value they were
    // last set to, and the column scan does not mis-index the rest of the row.
    const auto readHeader = [&](auto&&... args) {
      csv.read_header(io::ignore_missing_column, args...);
    };
    std::apply(readHeader, kEyeGazeVergenceColumns);

    // Both schemas carry per-eye yaw + eye origins. Gen2 is identified by the combined
    // `yaw_rads_cpf` AND both per-eye pitch columns; Gen1 lacks those and instead carries
    // depth, low/high confidence bounds, and session_uid. Checking all three guards against
    // a future Gen1 variant that adds per-eye pitch without a combined-yaw column.
    const bool isGen2 = csv.has_column("yaw_rads_cpf") && csv.has_column("left_pitch_rads_cpf") &&
        csv.has_column("right_pitch_rads_cpf");

    EyeGaze eyeGazeVergence;
    std::int64_t tracking_timestamp_us = 0;
    float combinedYawFromFile = 0.f;

    while (csv.read_row(
        tracking_timestamp_us,
        eyeGazeVergence.vergence.left_yaw,
        eyeGazeVergence.vergence.right_yaw,
        eyeGazeVergence.pitch,
        eyeGazeVergence.depth,
        eyeGazeVergence.vergence.left_yaw_low,
        eyeGazeVergence.vergence.right_yaw_low,
        eyeGazeVergence.pitch_low,
        eyeGazeVergence.vergence.left_yaw_high,
        eyeGazeVergence.vergence.right_yaw_high,
        eyeGazeVergence.pitch_high,
        eyeGazeVergence.vergence.tx_left_eye,
        eyeGazeVergence.vergence.ty_left_eye,
        eyeGazeVergence.vergence.tz_left_eye,
        eyeGazeVergence.vergence.tx_right_eye,
        eyeGazeVergence.vergence.ty_right_eye,
        eyeGazeVergence.vergence.tz_right_eye,
        eyeGazeVergence.session_uid,
        combinedYawFromFile,
        eyeGazeVergence.vergence.left_pitch,
        eyeGazeVergence.vergence.right_pitch)) {
      eyeGazeVergence.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      if (isGen2) {
        // Gen2 MPS writes the combined yaw directly; use it as-is.
        eyeGazeVergence.yaw = combinedYawFromFile;

        // Backfill 3D geometry that the Gen2 CSV doesn't carry: combined gaze origin (midpoint
        // of the two eye origins), spatial gaze point (vergence of the per-eye rays), and
        // depth (Euclidean distance from combined origin to spatial gaze point). When the
        // helper signals that the result is a degenerate-geometry fallback rather than a real
        // vergence, mark spatial_gaze_point_valid = false so consumers can skip the synthetic
        // point.
        const Eigen::Vector3d leftOrigin{
            eyeGazeVergence.vergence.tx_left_eye,
            eyeGazeVergence.vergence.ty_left_eye,
            eyeGazeVergence.vergence.tz_left_eye};
        const Eigen::Vector3d rightOrigin{
            eyeGazeVergence.vergence.tx_right_eye,
            eyeGazeVergence.vergence.ty_right_eye,
            eyeGazeVergence.vergence.tz_right_eye};
        const Eigen::Vector3d combinedOrigin = 0.5 * (leftOrigin + rightOrigin);
        const auto [vergencePoint, isRealVergence] = getGazeVergencePoint(
            leftOrigin,
            eyeGazeVergence.vergence.left_yaw,
            eyeGazeVergence.vergence.left_pitch,
            rightOrigin,
            eyeGazeVergence.vergence.right_yaw,
            eyeGazeVergence.vergence.right_pitch);

        eyeGazeVergence.combined_gaze_origin_in_cpf = combinedOrigin.cast<float>();
        eyeGazeVergence.combined_gaze_valid = true;
        eyeGazeVergence.spatial_gaze_point_in_cpf = vergencePoint.cast<float>();
        eyeGazeVergence.spatial_gaze_point_valid = isRealVergence;
        // Don't expose the far-gaze fallback distance as a real depth. Consumers that ignore
        // `spatial_gaze_point_valid` would otherwise see 10 m and treat it as a measurement.
        eyeGazeVergence.depth =
            isRealVergence ? static_cast<float>((vergencePoint - combinedOrigin).norm()) : 0.f;
      } else {
        // Gen1 schema lacks combined yaw — derive it from the per-eye yaw + common pitch.
        float depthM = NAN, combinedYawRads = NAN, pitchRads = NAN;
        std::tie(depthM, combinedYawRads, pitchRads) = computeDepthAndCombinedGazeDirection(
            eyeGazeVergence.vergence.left_yaw,
            eyeGazeVergence.vergence.right_yaw,
            eyeGazeVergence.pitch);
        eyeGazeVergence.yaw = combinedYawRads;
      }
      eyeGazeVergences.push_back(eyeGazeVergence);
    }
    std::cout << "Loaded #EyeGazes: " << eyeGazeVergences.size() << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Failed to parse eye gaze vergence file: " << e.what() << std::endl;
  }
  return eyeGazeVergences; // Can be empty if input file was invalid
}

EyeGazes readEyeGaze(const std::string& path) {
  // First try to read eye gaze vergence file
  EyeGazes eyeGazes = readEyeGazeVergence(path);
  if (!eyeGazes.empty()) {
    return eyeGazes;
  }
  // try to load legacy file format for backward compatibility
  try {
    io::CSVReader<kEyeGazeColumns.size()> csv(path);

    // Read in the CSV header
    const auto readHeader = [&](auto&&... args) {
      csv.read_header(io::ignore_missing_column, args...);
    };
    std::apply(readHeader, kEyeGazeColumns);
    // check if the first 8 mandatory columns are present
    for (int i = 0; i < 8; i++) {
      if (!csv.has_column(kEyeGazeColumns[i])) {
        std::string columnName(kEyeGazeColumns[i]);
        throw std::runtime_error("Missing column: " + columnName);
      }
    }

    // Default: assume we have V1 file format
    int version = 1;
    if (csv.has_column(kEyeGazeColumns[8])) { // checking "session_uid"
      version = 2;
    }

    EyeGaze eyeGaze;
    std::int64_t tracking_timestamp_us = 0;

    while (csv.read_row(
        tracking_timestamp_us,
        eyeGaze.yaw,
        eyeGaze.pitch,
        eyeGaze.depth,
        eyeGaze.yaw_low,
        eyeGaze.pitch_low,
        eyeGaze.yaw_high,
        eyeGaze.pitch_high,
        eyeGaze.session_uid)) {
      if (version == 1) {
        eyeGaze.session_uid = ""; // V1 format does not have session UID
      }
      eyeGaze.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      eyeGazes.push_back(eyeGaze);
    }
    std::cout << "Loaded #EyeGazes: " << eyeGazes.size() << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Failed to parse eye gaze file: " << e.what() << std::endl;
  }
  return eyeGazes; // Can be empty if input file was invalid
}

} // namespace projectaria::tools::mps
