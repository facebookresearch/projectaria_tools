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

#include <array>
#include <iostream>

#ifndef CSV_IO_NO_THREAD
#define CSV_IO_NO_THREAD
#endif
#include "EyeGazeReader.h"
#include "fast-cpp-csv-parser/csv.h"

namespace projectaria::tools::mps {

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

constexpr std::array<const char*, 18> kEyeGazeVergenceColumns = {
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
};

EyeGazes readEyeGazeVergence(const std::string& path) {
  EyeGazes eyeGazeVergences;
  try {
    io::CSVReader<kEyeGazeVergenceColumns.size()> csv(path);

    // Read in the CSV header
    const auto readHeader = [&](auto&&... args) {
      csv.read_header(io::ignore_missing_column, args...);
    };
    std::apply(readHeader, kEyeGazeVergenceColumns);

    EyeGaze eyeGazeVergence;
    std::int64_t tracking_timestamp_us = 0;

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
        eyeGazeVergence.session_uid)) {
      eyeGazeVergence.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      // populate yaw
      float depthM = NAN, combinedYawRads = NAN, pitchRads = NAN;
      std::tie(depthM, combinedYawRads, pitchRads) = computeDepthAndCombinedGazeDirection(
          eyeGazeVergence.vergence.left_yaw,
          eyeGazeVergence.vergence.right_yaw,
          eyeGazeVergence.pitch);
      eyeGazeVergence.yaw = combinedYawRads;
      eyeGazeVergences.push_back(eyeGazeVergence);
    }

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

  } catch (std::exception& e) {
    std::cerr << "Failed to parse eye gaze file: " << e.what() << std::endl;
  }
  return eyeGazes; // Can be empty if input file was invalid
}

} // namespace projectaria::tools::mps
