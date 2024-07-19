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

#include <Eigen/Core>

#include <array>
#include <chrono>
#include <filesystem>
#include <iostream>

#include <exception>
#include <stdexcept>

#ifndef CSV_IO_NO_THREAD
#define CSV_IO_NO_THREAD
#endif
#include "fast-cpp-csv-parser/csv.h"

#include "HandTracking.h"
#include "HandTrackingReader.h"

namespace projectaria::tools::mps {

WristAndPalmPoses readWristAndPalmPosesV2(const std::string& filepath) {
  WristAndPalmPoses wristAndPalmPoses;
  try {
    io::CSVReader<27> csv(filepath);

    // Read in the CSV header
    csv.read_header(
        io::ignore_missing_column,
        "tracking_timestamp_us",
        "left_tracking_confidence",
        "tx_left_wrist_device",
        "ty_left_wrist_device",
        "tz_left_wrist_device",
        "tx_left_palm_device",
        "ty_left_palm_device",
        "tz_left_palm_device",
        "right_tracking_confidence",
        "tx_right_wrist_device",
        "ty_right_wrist_device",
        "tz_right_wrist_device",
        "tx_right_palm_device",
        "ty_right_palm_device",
        "tz_right_palm_device",
        // Normal vector fields start:
        "nx_left_palm_device",
        "ny_left_palm_device",
        "nz_left_palm_device",
        "nx_left_wrist_device",
        "ny_left_wrist_device",
        "nz_left_wrist_device",
        "nx_right_palm_device",
        "ny_right_palm_device",
        "nz_right_palm_device",
        "nx_right_wrist_device",
        "ny_right_wrist_device",
        "nz_right_wrist_device");

    // Must have these new columns to qualify as v2, i.e. nx/ny/nz_left/right_palm/wrist_device
    std::vector<std::string> normalFields = {
        "nx_left_palm_device",
        "ny_left_palm_device",
        "nz_left_palm_device",
        "nx_right_palm_device",
        "ny_right_palm_device",
        "nz_right_palm_device",
        "nx_left_wrist_device",
        "ny_left_wrist_device",
        "nz_left_wrist_device",
        "nx_right_wrist_device",
        "ny_right_wrist_device",
        "nz_right_wrist_device"};
    for (const auto& field : normalFields) {
      if (!csv.has_column(field)) {
        throw std::runtime_error("Missing column: " + field);
      }
    }

    int64_t tracking_timestamp_us = 0;
    WristAndPalmPose::OneSide leftWristAndPalmPose, rightWristAndPalmPose;

    while (true) {
      bool read_success = false;
      leftWristAndPalmPose.wristAndPalmNormal_device =
          rightWristAndPalmPose.wristAndPalmNormal_device = {
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
      read_success = csv.read_row(
          tracking_timestamp_us,
          leftWristAndPalmPose.confidence,
          leftWristAndPalmPose.wristPosition_device.x(),
          leftWristAndPalmPose.wristPosition_device.y(),
          leftWristAndPalmPose.wristPosition_device.z(),
          leftWristAndPalmPose.palmPosition_device.x(),
          leftWristAndPalmPose.palmPosition_device.y(),
          leftWristAndPalmPose.palmPosition_device.z(),
          rightWristAndPalmPose.confidence,
          rightWristAndPalmPose.wristPosition_device.x(),
          rightWristAndPalmPose.wristPosition_device.y(),
          rightWristAndPalmPose.wristPosition_device.z(),
          rightWristAndPalmPose.palmPosition_device.x(),
          rightWristAndPalmPose.palmPosition_device.y(),
          rightWristAndPalmPose.palmPosition_device.z(),
          // Normal vector fields start:
          leftWristAndPalmPose.wristAndPalmNormal_device->palmNormal_device.x(),
          leftWristAndPalmPose.wristAndPalmNormal_device->palmNormal_device.y(),
          leftWristAndPalmPose.wristAndPalmNormal_device->palmNormal_device.z(),
          leftWristAndPalmPose.wristAndPalmNormal_device->wristNormal_device.x(),
          leftWristAndPalmPose.wristAndPalmNormal_device->wristNormal_device.y(),
          leftWristAndPalmPose.wristAndPalmNormal_device->wristNormal_device.z(),
          rightWristAndPalmPose.wristAndPalmNormal_device->palmNormal_device.x(),
          rightWristAndPalmPose.wristAndPalmNormal_device->palmNormal_device.y(),
          rightWristAndPalmPose.wristAndPalmNormal_device->palmNormal_device.z(),
          rightWristAndPalmPose.wristAndPalmNormal_device->wristNormal_device.x(),
          rightWristAndPalmPose.wristAndPalmNormal_device->wristNormal_device.y(),
          rightWristAndPalmPose.wristAndPalmNormal_device->wristNormal_device.z());

      if (!read_success) {
        break;
      }
      WristAndPalmPose wristAndPalmPose;
      wristAndPalmPose.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      wristAndPalmPose.leftHand = leftWristAndPalmPose;
      wristAndPalmPose.rightHand = rightWristAndPalmPose;
      wristAndPalmPoses.push_back(wristAndPalmPose);
    }
    std::cout << "Loaded #WristAndPalmPose: " << wristAndPalmPoses.size() << std::endl;
  } catch (std::exception& e) {
    std::cout << "Warning: failed to parse wrist and palm file v2: " << filepath << e.what()
              << std::endl;
  }
  return wristAndPalmPoses;
}

WristAndPalmPoses readWristAndPalmPosesV1(const std::string& filepath) {
  WristAndPalmPoses wristAndPalmPoses;
  try {
    io::CSVReader<15> csv(filepath);

    // Read in the CSV header
    csv.read_header(
        io::ignore_missing_column,
        "tracking_timestamp_us",
        "left_tracking_confidence",
        "tx_left_wrist_device",
        "ty_left_wrist_device",
        "tz_left_wrist_device",
        "tx_left_palm_device",
        "ty_left_palm_device",
        "tz_left_palm_device",
        "right_tracking_confidence",
        "tx_right_wrist_device",
        "ty_right_wrist_device",
        "tz_right_wrist_device",
        "tx_right_palm_device",
        "ty_right_palm_device",
        "tz_right_palm_device");

    int64_t tracking_timestamp_us = 0;
    WristAndPalmPose::OneSide leftWristAndPalmPose, rightWristAndPalmPose;

    while (true) {
      bool read_success = false;
      read_success = csv.read_row(
          tracking_timestamp_us,
          leftWristAndPalmPose.confidence,
          leftWristAndPalmPose.wristPosition_device.x(),
          leftWristAndPalmPose.wristPosition_device.y(),
          leftWristAndPalmPose.wristPosition_device.z(),
          leftWristAndPalmPose.palmPosition_device.x(),
          leftWristAndPalmPose.palmPosition_device.y(),
          leftWristAndPalmPose.palmPosition_device.z(),
          rightWristAndPalmPose.confidence,
          rightWristAndPalmPose.wristPosition_device.x(),
          rightWristAndPalmPose.wristPosition_device.y(),
          rightWristAndPalmPose.wristPosition_device.z(),
          rightWristAndPalmPose.palmPosition_device.x(),
          rightWristAndPalmPose.palmPosition_device.y(),
          rightWristAndPalmPose.palmPosition_device.z());

      if (!read_success) {
        break;
      }
      WristAndPalmPose wristAndPalmPose;
      wristAndPalmPose.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      wristAndPalmPose.leftHand = leftWristAndPalmPose;
      wristAndPalmPose.rightHand = rightWristAndPalmPose;
      wristAndPalmPoses.push_back(wristAndPalmPose);
    }
    std::cout << "Loaded #WristAndPalmPose: " << wristAndPalmPoses.size() << std::endl;
  } catch (std::exception& e) {
    std::cout << "Warning: failed to parse wrist and palm file: " << filepath << e.what()
              << std::endl;
  }
  return wristAndPalmPoses;
}

// Read file using V2 reader first. If it returns empty list, then try V1 reader.
WristAndPalmPoses readWristAndPalmPoses(const std::string& filepath) {
  WristAndPalmPoses wristAndPalmPoses = readWristAndPalmPosesV2(filepath);
  if (wristAndPalmPoses.empty()) {
    wristAndPalmPoses = readWristAndPalmPosesV1(filepath);
  }
  if (wristAndPalmPoses.empty()) {
    std::cerr << "Failed to read any wrist and palm poses from file: " << filepath << std::endl;
  }
  return wristAndPalmPoses;
}
} // namespace projectaria::tools::mps
