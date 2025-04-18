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

enum class HandLandmarkIndex : uint8_t {
  THUMB_FINGERTIP = 0,
  INDEX_FINGERTIP = 1,
  MIDDLE_FINGERTIP = 2,
  RING_FINGERTIP = 3,
  PINKY_FINGERTIP = 4,

  WRIST = 5,

  THUMB_INTERMEDIATE = 6,
  THUMB_DISTAL = 7,

  INDEX_PROXIMAL = 8,
  INDEX_INTERMEDIATE = 9,
  INDEX_DISTAL = 10,

  MIDDLE_PROXIMAL = 11,
  MIDDLE_INTERMEDIATE = 12,
  MIDDLE_DISTAL = 13,

  RING_PROXIMAL = 14,
  RING_INTERMEDIATE = 15,
  RING_DISTAL = 16,

  PINKY_PROXIMAL = 17,
  PINKY_INTERMEDIATE = 18,
  PINKY_DISTAL = 19,

  PALM_CENTER = 20,
};

void assignWristAndPalm(WristAndPalmPose& landmarkPose) {
  if (landmarkPose.leftHand.has_value()) {
    landmarkPose.leftHand->wristPosition_device =
        landmarkPose.leftHand
            ->landmarkPositions_device[static_cast<uint8_t>(HandLandmarkIndex::WRIST)];
    landmarkPose.leftHand->palmPosition_device =
        landmarkPose.leftHand
            ->landmarkPositions_device[static_cast<uint8_t>(HandLandmarkIndex::PALM_CENTER)];
  }
  if (landmarkPose.rightHand.has_value()) {
    landmarkPose.rightHand->wristPosition_device =
        landmarkPose.rightHand
            ->landmarkPositions_device[static_cast<uint8_t>(HandLandmarkIndex::WRIST)];
    landmarkPose.rightHand->palmPosition_device =
        landmarkPose.rightHand
            ->landmarkPositions_device[static_cast<uint8_t>(HandLandmarkIndex::PALM_CENTER)];
  }
}

WristAndPalmPoses readWristAndPalmPosesV4(const std::string& filepath) {
  WristAndPalmPoses landmarkPoses;
  try {
    io::CSVReader<141> csv(filepath);

    // Read in the CSV header
    csv.read_header(
        io::ignore_missing_column,
        "tracking_timestamp_us",
        "left_tracking_confidence",
        "tx_left_landmark_0_device",
        "ty_left_landmark_0_device",
        "tz_left_landmark_0_device",
        "tx_left_landmark_1_device",
        "ty_left_landmark_1_device",
        "tz_left_landmark_1_device",
        "tx_left_landmark_2_device",
        "ty_left_landmark_2_device",
        "tz_left_landmark_2_device",
        "tx_left_landmark_3_device",
        "ty_left_landmark_3_device",
        "tz_left_landmark_3_device",
        "tx_left_landmark_4_device",
        "ty_left_landmark_4_device",
        "tz_left_landmark_4_device",
        "tx_left_landmark_5_device",
        "ty_left_landmark_5_device",
        "tz_left_landmark_5_device",
        "tx_left_landmark_6_device",
        "ty_left_landmark_6_device",
        "tz_left_landmark_6_device",
        "tx_left_landmark_7_device",
        "ty_left_landmark_7_device",
        "tz_left_landmark_7_device",
        "tx_left_landmark_8_device",
        "ty_left_landmark_8_device",
        "tz_left_landmark_8_device",
        "tx_left_landmark_9_device",
        "ty_left_landmark_9_device",
        "tz_left_landmark_9_device",
        "tx_left_landmark_10_device",
        "ty_left_landmark_10_device",
        "tz_left_landmark_10_device",
        "tx_left_landmark_11_device",
        "ty_left_landmark_11_device",
        "tz_left_landmark_11_device",
        "tx_left_landmark_12_device",
        "ty_left_landmark_12_device",
        "tz_left_landmark_12_device",
        "tx_left_landmark_13_device",
        "ty_left_landmark_13_device",
        "tz_left_landmark_13_device",
        "tx_left_landmark_14_device",
        "ty_left_landmark_14_device",
        "tz_left_landmark_14_device",
        "tx_left_landmark_15_device",
        "ty_left_landmark_15_device",
        "tz_left_landmark_15_device",
        "tx_left_landmark_16_device",
        "ty_left_landmark_16_device",
        "tz_left_landmark_16_device",
        "tx_left_landmark_17_device",
        "ty_left_landmark_17_device",
        "tz_left_landmark_17_device",
        "tx_left_landmark_18_device",
        "ty_left_landmark_18_device",
        "tz_left_landmark_18_device",
        "tx_left_landmark_19_device",
        "ty_left_landmark_19_device",
        "tz_left_landmark_19_device",
        "tx_left_landmark_20_device",
        "ty_left_landmark_20_device",
        "tz_left_landmark_20_device",
        "right_tracking_confidence",
        "tx_right_landmark_0_device",
        "ty_right_landmark_0_device",
        "tz_right_landmark_0_device",
        "tx_right_landmark_1_device",
        "ty_right_landmark_1_device",
        "tz_right_landmark_1_device",
        "tx_right_landmark_2_device",
        "ty_right_landmark_2_device",
        "tz_right_landmark_2_device",
        "tx_right_landmark_3_device",
        "ty_right_landmark_3_device",
        "tz_right_landmark_3_device",
        "tx_right_landmark_4_device",
        "ty_right_landmark_4_device",
        "tz_right_landmark_4_device",
        "tx_right_landmark_5_device",
        "ty_right_landmark_5_device",
        "tz_right_landmark_5_device",
        "tx_right_landmark_6_device",
        "ty_right_landmark_6_device",
        "tz_right_landmark_6_device",
        "tx_right_landmark_7_device",
        "ty_right_landmark_7_device",
        "tz_right_landmark_7_device",
        "tx_right_landmark_8_device",
        "ty_right_landmark_8_device",
        "tz_right_landmark_8_device",
        "tx_right_landmark_9_device",
        "ty_right_landmark_9_device",
        "tz_right_landmark_9_device",
        "tx_right_landmark_10_device",
        "ty_right_landmark_10_device",
        "tz_right_landmark_10_device",
        "tx_right_landmark_11_device",
        "ty_right_landmark_11_device",
        "tz_right_landmark_11_device",
        "tx_right_landmark_12_device",
        "ty_right_landmark_12_device",
        "tz_right_landmark_12_device",
        "tx_right_landmark_13_device",
        "ty_right_landmark_13_device",
        "tz_right_landmark_13_device",
        "tx_right_landmark_14_device",
        "ty_right_landmark_14_device",
        "tz_right_landmark_14_device",
        "tx_right_landmark_15_device",
        "ty_right_landmark_15_device",
        "tz_right_landmark_15_device",
        "tx_right_landmark_16_device",
        "ty_right_landmark_16_device",
        "tz_right_landmark_16_device",
        "tx_right_landmark_17_device",
        "ty_right_landmark_17_device",
        "tz_right_landmark_17_device",
        "tx_right_landmark_18_device",
        "ty_right_landmark_18_device",
        "tz_right_landmark_18_device",
        "tx_right_landmark_19_device",
        "ty_right_landmark_19_device",
        "tz_right_landmark_19_device",
        "tx_right_landmark_20_device",
        "ty_right_landmark_20_device",
        "tz_right_landmark_20_device",
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

    // Must have these new columns to qualify as v4, i.e. nx/ny/nz_left/right_palm/wrist_device
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
    WristAndPalmPose::OneSide leftLandmarkPose, rightLandmarkPose;

    while (true) {
      bool read_success = false;
      leftLandmarkPose.wristAndPalmNormal_device = rightLandmarkPose.wristAndPalmNormal_device = {
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
      read_success = csv.read_row(
          tracking_timestamp_us,
          leftLandmarkPose.confidence,
          leftLandmarkPose.landmarkPositions_device[0].x(),
          leftLandmarkPose.landmarkPositions_device[0].y(),
          leftLandmarkPose.landmarkPositions_device[0].z(),
          leftLandmarkPose.landmarkPositions_device[1].x(),
          leftLandmarkPose.landmarkPositions_device[1].y(),
          leftLandmarkPose.landmarkPositions_device[1].z(),
          leftLandmarkPose.landmarkPositions_device[2].x(),
          leftLandmarkPose.landmarkPositions_device[2].y(),
          leftLandmarkPose.landmarkPositions_device[2].z(),
          leftLandmarkPose.landmarkPositions_device[3].x(),
          leftLandmarkPose.landmarkPositions_device[3].y(),
          leftLandmarkPose.landmarkPositions_device[3].z(),
          leftLandmarkPose.landmarkPositions_device[4].x(),
          leftLandmarkPose.landmarkPositions_device[4].y(),
          leftLandmarkPose.landmarkPositions_device[4].z(),
          leftLandmarkPose.landmarkPositions_device[5].x(),
          leftLandmarkPose.landmarkPositions_device[5].y(),
          leftLandmarkPose.landmarkPositions_device[5].z(),
          leftLandmarkPose.landmarkPositions_device[6].x(),
          leftLandmarkPose.landmarkPositions_device[6].y(),
          leftLandmarkPose.landmarkPositions_device[6].z(),
          leftLandmarkPose.landmarkPositions_device[7].x(),
          leftLandmarkPose.landmarkPositions_device[7].y(),
          leftLandmarkPose.landmarkPositions_device[7].z(),
          leftLandmarkPose.landmarkPositions_device[8].x(),
          leftLandmarkPose.landmarkPositions_device[8].y(),
          leftLandmarkPose.landmarkPositions_device[8].z(),
          leftLandmarkPose.landmarkPositions_device[9].x(),
          leftLandmarkPose.landmarkPositions_device[9].y(),
          leftLandmarkPose.landmarkPositions_device[9].z(),
          leftLandmarkPose.landmarkPositions_device[10].x(),
          leftLandmarkPose.landmarkPositions_device[10].y(),
          leftLandmarkPose.landmarkPositions_device[10].z(),
          leftLandmarkPose.landmarkPositions_device[11].x(),
          leftLandmarkPose.landmarkPositions_device[11].y(),
          leftLandmarkPose.landmarkPositions_device[11].z(),
          leftLandmarkPose.landmarkPositions_device[12].x(),
          leftLandmarkPose.landmarkPositions_device[12].y(),
          leftLandmarkPose.landmarkPositions_device[12].z(),
          leftLandmarkPose.landmarkPositions_device[13].x(),
          leftLandmarkPose.landmarkPositions_device[13].y(),
          leftLandmarkPose.landmarkPositions_device[13].z(),
          leftLandmarkPose.landmarkPositions_device[14].x(),
          leftLandmarkPose.landmarkPositions_device[14].y(),
          leftLandmarkPose.landmarkPositions_device[14].z(),
          leftLandmarkPose.landmarkPositions_device[15].x(),
          leftLandmarkPose.landmarkPositions_device[15].y(),
          leftLandmarkPose.landmarkPositions_device[15].z(),
          leftLandmarkPose.landmarkPositions_device[16].x(),
          leftLandmarkPose.landmarkPositions_device[16].y(),
          leftLandmarkPose.landmarkPositions_device[16].z(),
          leftLandmarkPose.landmarkPositions_device[17].x(),
          leftLandmarkPose.landmarkPositions_device[17].y(),
          leftLandmarkPose.landmarkPositions_device[17].z(),
          leftLandmarkPose.landmarkPositions_device[18].x(),
          leftLandmarkPose.landmarkPositions_device[18].y(),
          leftLandmarkPose.landmarkPositions_device[18].z(),
          leftLandmarkPose.landmarkPositions_device[19].x(),
          leftLandmarkPose.landmarkPositions_device[19].y(),
          leftLandmarkPose.landmarkPositions_device[19].z(),
          leftLandmarkPose.landmarkPositions_device[20].x(),
          leftLandmarkPose.landmarkPositions_device[20].y(),
          leftLandmarkPose.landmarkPositions_device[20].z(),
          rightLandmarkPose.confidence,
          rightLandmarkPose.landmarkPositions_device[0].x(),
          rightLandmarkPose.landmarkPositions_device[0].y(),
          rightLandmarkPose.landmarkPositions_device[0].z(),
          rightLandmarkPose.landmarkPositions_device[1].x(),
          rightLandmarkPose.landmarkPositions_device[1].y(),
          rightLandmarkPose.landmarkPositions_device[1].z(),
          rightLandmarkPose.landmarkPositions_device[2].x(),
          rightLandmarkPose.landmarkPositions_device[2].y(),
          rightLandmarkPose.landmarkPositions_device[2].z(),
          rightLandmarkPose.landmarkPositions_device[3].x(),
          rightLandmarkPose.landmarkPositions_device[3].y(),
          rightLandmarkPose.landmarkPositions_device[3].z(),
          rightLandmarkPose.landmarkPositions_device[4].x(),
          rightLandmarkPose.landmarkPositions_device[4].y(),
          rightLandmarkPose.landmarkPositions_device[4].z(),
          rightLandmarkPose.landmarkPositions_device[5].x(),
          rightLandmarkPose.landmarkPositions_device[5].y(),
          rightLandmarkPose.landmarkPositions_device[5].z(),
          rightLandmarkPose.landmarkPositions_device[6].x(),
          rightLandmarkPose.landmarkPositions_device[6].y(),
          rightLandmarkPose.landmarkPositions_device[6].z(),
          rightLandmarkPose.landmarkPositions_device[7].x(),
          rightLandmarkPose.landmarkPositions_device[7].y(),
          rightLandmarkPose.landmarkPositions_device[7].z(),
          rightLandmarkPose.landmarkPositions_device[8].x(),
          rightLandmarkPose.landmarkPositions_device[8].y(),
          rightLandmarkPose.landmarkPositions_device[8].z(),
          rightLandmarkPose.landmarkPositions_device[9].x(),
          rightLandmarkPose.landmarkPositions_device[9].y(),
          rightLandmarkPose.landmarkPositions_device[9].z(),
          rightLandmarkPose.landmarkPositions_device[10].x(),
          rightLandmarkPose.landmarkPositions_device[10].y(),
          rightLandmarkPose.landmarkPositions_device[10].z(),
          rightLandmarkPose.landmarkPositions_device[11].x(),
          rightLandmarkPose.landmarkPositions_device[11].y(),
          rightLandmarkPose.landmarkPositions_device[11].z(),
          rightLandmarkPose.landmarkPositions_device[12].x(),
          rightLandmarkPose.landmarkPositions_device[12].y(),
          rightLandmarkPose.landmarkPositions_device[12].z(),
          rightLandmarkPose.landmarkPositions_device[13].x(),
          rightLandmarkPose.landmarkPositions_device[13].y(),
          rightLandmarkPose.landmarkPositions_device[13].z(),
          rightLandmarkPose.landmarkPositions_device[14].x(),
          rightLandmarkPose.landmarkPositions_device[14].y(),
          rightLandmarkPose.landmarkPositions_device[14].z(),
          rightLandmarkPose.landmarkPositions_device[15].x(),
          rightLandmarkPose.landmarkPositions_device[15].y(),
          rightLandmarkPose.landmarkPositions_device[15].z(),
          rightLandmarkPose.landmarkPositions_device[16].x(),
          rightLandmarkPose.landmarkPositions_device[16].y(),
          rightLandmarkPose.landmarkPositions_device[16].z(),
          rightLandmarkPose.landmarkPositions_device[17].x(),
          rightLandmarkPose.landmarkPositions_device[17].y(),
          rightLandmarkPose.landmarkPositions_device[17].z(),
          rightLandmarkPose.landmarkPositions_device[18].x(),
          rightLandmarkPose.landmarkPositions_device[18].y(),
          rightLandmarkPose.landmarkPositions_device[18].z(),
          rightLandmarkPose.landmarkPositions_device[19].x(),
          rightLandmarkPose.landmarkPositions_device[19].y(),
          rightLandmarkPose.landmarkPositions_device[19].z(),
          rightLandmarkPose.landmarkPositions_device[20].x(),
          rightLandmarkPose.landmarkPositions_device[20].y(),
          rightLandmarkPose.landmarkPositions_device[20].z(),
          // Normal vector fields start:
          leftLandmarkPose.wristAndPalmNormal_device->palmNormal_device.x(),
          leftLandmarkPose.wristAndPalmNormal_device->palmNormal_device.y(),
          leftLandmarkPose.wristAndPalmNormal_device->palmNormal_device.z(),
          leftLandmarkPose.wristAndPalmNormal_device->wristNormal_device.x(),
          leftLandmarkPose.wristAndPalmNormal_device->wristNormal_device.y(),
          leftLandmarkPose.wristAndPalmNormal_device->wristNormal_device.z(),
          rightLandmarkPose.wristAndPalmNormal_device->palmNormal_device.x(),
          rightLandmarkPose.wristAndPalmNormal_device->palmNormal_device.y(),
          rightLandmarkPose.wristAndPalmNormal_device->palmNormal_device.z(),
          rightLandmarkPose.wristAndPalmNormal_device->wristNormal_device.x(),
          rightLandmarkPose.wristAndPalmNormal_device->wristNormal_device.y(),
          rightLandmarkPose.wristAndPalmNormal_device->wristNormal_device.z());

      if (!read_success) {
        break;
      }
      WristAndPalmPose landmarkPose;
      landmarkPose.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      landmarkPose.leftHand = leftLandmarkPose;
      landmarkPose.rightHand = rightLandmarkPose;
      assignWristAndPalm(landmarkPose);
      landmarkPoses.push_back(landmarkPose);
    }
    std::cout << "Loaded #LandmarkPoses: " << landmarkPoses.size() << std::endl;
  } catch (std::exception& e) {
    std::cout << "Warning: failed to parse landmark file v4: " << filepath << e.what() << std::endl;
  }
  return landmarkPoses;
}

WristAndPalmPoses readWristAndPalmPosesV3(const std::string& filepath) {
  WristAndPalmPoses landmarkPoses;
  try {
    io::CSVReader<129> csv(filepath);

    // Read in the CSV header
    csv.read_header(
        io::ignore_missing_column,
        "tracking_timestamp_us",
        "left_tracking_confidence",
        "tx_left_landmark_0_device",
        "ty_left_landmark_0_device",
        "tz_left_landmark_0_device",
        "tx_left_landmark_1_device",
        "ty_left_landmark_1_device",
        "tz_left_landmark_1_device",
        "tx_left_landmark_2_device",
        "ty_left_landmark_2_device",
        "tz_left_landmark_2_device",
        "tx_left_landmark_3_device",
        "ty_left_landmark_3_device",
        "tz_left_landmark_3_device",
        "tx_left_landmark_4_device",
        "ty_left_landmark_4_device",
        "tz_left_landmark_4_device",
        "tx_left_landmark_5_device",
        "ty_left_landmark_5_device",
        "tz_left_landmark_5_device",
        "tx_left_landmark_6_device",
        "ty_left_landmark_6_device",
        "tz_left_landmark_6_device",
        "tx_left_landmark_7_device",
        "ty_left_landmark_7_device",
        "tz_left_landmark_7_device",
        "tx_left_landmark_8_device",
        "ty_left_landmark_8_device",
        "tz_left_landmark_8_device",
        "tx_left_landmark_9_device",
        "ty_left_landmark_9_device",
        "tz_left_landmark_9_device",
        "tx_left_landmark_10_device",
        "ty_left_landmark_10_device",
        "tz_left_landmark_10_device",
        "tx_left_landmark_11_device",
        "ty_left_landmark_11_device",
        "tz_left_landmark_11_device",
        "tx_left_landmark_12_device",
        "ty_left_landmark_12_device",
        "tz_left_landmark_12_device",
        "tx_left_landmark_13_device",
        "ty_left_landmark_13_device",
        "tz_left_landmark_13_device",
        "tx_left_landmark_14_device",
        "ty_left_landmark_14_device",
        "tz_left_landmark_14_device",
        "tx_left_landmark_15_device",
        "ty_left_landmark_15_device",
        "tz_left_landmark_15_device",
        "tx_left_landmark_16_device",
        "ty_left_landmark_16_device",
        "tz_left_landmark_16_device",
        "tx_left_landmark_17_device",
        "ty_left_landmark_17_device",
        "tz_left_landmark_17_device",
        "tx_left_landmark_18_device",
        "ty_left_landmark_18_device",
        "tz_left_landmark_18_device",
        "tx_left_landmark_19_device",
        "ty_left_landmark_19_device",
        "tz_left_landmark_19_device",
        "tx_left_landmark_20_device",
        "ty_left_landmark_20_device",
        "tz_left_landmark_20_device",
        "right_tracking_confidence",
        "tx_right_landmark_0_device",
        "ty_right_landmark_0_device",
        "tz_right_landmark_0_device",
        "tx_right_landmark_1_device",
        "ty_right_landmark_1_device",
        "tz_right_landmark_1_device",
        "tx_right_landmark_2_device",
        "ty_right_landmark_2_device",
        "tz_right_landmark_2_device",
        "tx_right_landmark_3_device",
        "ty_right_landmark_3_device",
        "tz_right_landmark_3_device",
        "tx_right_landmark_4_device",
        "ty_right_landmark_4_device",
        "tz_right_landmark_4_device",
        "tx_right_landmark_5_device",
        "ty_right_landmark_5_device",
        "tz_right_landmark_5_device",
        "tx_right_landmark_6_device",
        "ty_right_landmark_6_device",
        "tz_right_landmark_6_device",
        "tx_right_landmark_7_device",
        "ty_right_landmark_7_device",
        "tz_right_landmark_7_device",
        "tx_right_landmark_8_device",
        "ty_right_landmark_8_device",
        "tz_right_landmark_8_device",
        "tx_right_landmark_9_device",
        "ty_right_landmark_9_device",
        "tz_right_landmark_9_device",
        "tx_right_landmark_10_device",
        "ty_right_landmark_10_device",
        "tz_right_landmark_10_device",
        "tx_right_landmark_11_device",
        "ty_right_landmark_11_device",
        "tz_right_landmark_11_device",
        "tx_right_landmark_12_device",
        "ty_right_landmark_12_device",
        "tz_right_landmark_12_device",
        "tx_right_landmark_13_device",
        "ty_right_landmark_13_device",
        "tz_right_landmark_13_device",
        "tx_right_landmark_14_device",
        "ty_right_landmark_14_device",
        "tz_right_landmark_14_device",
        "tx_right_landmark_15_device",
        "ty_right_landmark_15_device",
        "tz_right_landmark_15_device",
        "tx_right_landmark_16_device",
        "ty_right_landmark_16_device",
        "tz_right_landmark_16_device",
        "tx_right_landmark_17_device",
        "ty_right_landmark_17_device",
        "tz_right_landmark_17_device",
        "tx_right_landmark_18_device",
        "ty_right_landmark_18_device",
        "tz_right_landmark_18_device",
        "tx_right_landmark_19_device",
        "ty_right_landmark_19_device",
        "tz_right_landmark_19_device",
        "tx_right_landmark_20_device",
        "ty_right_landmark_20_device",
        "tz_right_landmark_20_device");

    int64_t tracking_timestamp_us = 0;
    WristAndPalmPose::OneSide leftLandmarkPose, rightLandmarkPose;

    while (true) {
      bool read_success = false;
      read_success = csv.read_row(
          tracking_timestamp_us,
          leftLandmarkPose.confidence,
          leftLandmarkPose.landmarkPositions_device[0].x(),
          leftLandmarkPose.landmarkPositions_device[0].y(),
          leftLandmarkPose.landmarkPositions_device[0].z(),
          leftLandmarkPose.landmarkPositions_device[1].x(),
          leftLandmarkPose.landmarkPositions_device[1].y(),
          leftLandmarkPose.landmarkPositions_device[1].z(),
          leftLandmarkPose.landmarkPositions_device[2].x(),
          leftLandmarkPose.landmarkPositions_device[2].y(),
          leftLandmarkPose.landmarkPositions_device[2].z(),
          leftLandmarkPose.landmarkPositions_device[3].x(),
          leftLandmarkPose.landmarkPositions_device[3].y(),
          leftLandmarkPose.landmarkPositions_device[3].z(),
          leftLandmarkPose.landmarkPositions_device[4].x(),
          leftLandmarkPose.landmarkPositions_device[4].y(),
          leftLandmarkPose.landmarkPositions_device[4].z(),
          leftLandmarkPose.landmarkPositions_device[5].x(),
          leftLandmarkPose.landmarkPositions_device[5].y(),
          leftLandmarkPose.landmarkPositions_device[5].z(),
          leftLandmarkPose.landmarkPositions_device[6].x(),
          leftLandmarkPose.landmarkPositions_device[6].y(),
          leftLandmarkPose.landmarkPositions_device[6].z(),
          leftLandmarkPose.landmarkPositions_device[7].x(),
          leftLandmarkPose.landmarkPositions_device[7].y(),
          leftLandmarkPose.landmarkPositions_device[7].z(),
          leftLandmarkPose.landmarkPositions_device[8].x(),
          leftLandmarkPose.landmarkPositions_device[8].y(),
          leftLandmarkPose.landmarkPositions_device[8].z(),
          leftLandmarkPose.landmarkPositions_device[9].x(),
          leftLandmarkPose.landmarkPositions_device[9].y(),
          leftLandmarkPose.landmarkPositions_device[9].z(),
          leftLandmarkPose.landmarkPositions_device[10].x(),
          leftLandmarkPose.landmarkPositions_device[10].y(),
          leftLandmarkPose.landmarkPositions_device[10].z(),
          leftLandmarkPose.landmarkPositions_device[11].x(),
          leftLandmarkPose.landmarkPositions_device[11].y(),
          leftLandmarkPose.landmarkPositions_device[11].z(),
          leftLandmarkPose.landmarkPositions_device[12].x(),
          leftLandmarkPose.landmarkPositions_device[12].y(),
          leftLandmarkPose.landmarkPositions_device[12].z(),
          leftLandmarkPose.landmarkPositions_device[13].x(),
          leftLandmarkPose.landmarkPositions_device[13].y(),
          leftLandmarkPose.landmarkPositions_device[13].z(),
          leftLandmarkPose.landmarkPositions_device[14].x(),
          leftLandmarkPose.landmarkPositions_device[14].y(),
          leftLandmarkPose.landmarkPositions_device[14].z(),
          leftLandmarkPose.landmarkPositions_device[15].x(),
          leftLandmarkPose.landmarkPositions_device[15].y(),
          leftLandmarkPose.landmarkPositions_device[15].z(),
          leftLandmarkPose.landmarkPositions_device[16].x(),
          leftLandmarkPose.landmarkPositions_device[16].y(),
          leftLandmarkPose.landmarkPositions_device[16].z(),
          leftLandmarkPose.landmarkPositions_device[17].x(),
          leftLandmarkPose.landmarkPositions_device[17].y(),
          leftLandmarkPose.landmarkPositions_device[17].z(),
          leftLandmarkPose.landmarkPositions_device[18].x(),
          leftLandmarkPose.landmarkPositions_device[18].y(),
          leftLandmarkPose.landmarkPositions_device[18].z(),
          leftLandmarkPose.landmarkPositions_device[19].x(),
          leftLandmarkPose.landmarkPositions_device[19].y(),
          leftLandmarkPose.landmarkPositions_device[19].z(),
          leftLandmarkPose.landmarkPositions_device[20].x(),
          leftLandmarkPose.landmarkPositions_device[20].y(),
          leftLandmarkPose.landmarkPositions_device[20].z(),
          rightLandmarkPose.confidence,
          rightLandmarkPose.landmarkPositions_device[0].x(),
          rightLandmarkPose.landmarkPositions_device[0].y(),
          rightLandmarkPose.landmarkPositions_device[0].z(),
          rightLandmarkPose.landmarkPositions_device[1].x(),
          rightLandmarkPose.landmarkPositions_device[1].y(),
          rightLandmarkPose.landmarkPositions_device[1].z(),
          rightLandmarkPose.landmarkPositions_device[2].x(),
          rightLandmarkPose.landmarkPositions_device[2].y(),
          rightLandmarkPose.landmarkPositions_device[2].z(),
          rightLandmarkPose.landmarkPositions_device[3].x(),
          rightLandmarkPose.landmarkPositions_device[3].y(),
          rightLandmarkPose.landmarkPositions_device[3].z(),
          rightLandmarkPose.landmarkPositions_device[4].x(),
          rightLandmarkPose.landmarkPositions_device[4].y(),
          rightLandmarkPose.landmarkPositions_device[4].z(),
          rightLandmarkPose.landmarkPositions_device[5].x(),
          rightLandmarkPose.landmarkPositions_device[5].y(),
          rightLandmarkPose.landmarkPositions_device[5].z(),
          rightLandmarkPose.landmarkPositions_device[6].x(),
          rightLandmarkPose.landmarkPositions_device[6].y(),
          rightLandmarkPose.landmarkPositions_device[6].z(),
          rightLandmarkPose.landmarkPositions_device[7].x(),
          rightLandmarkPose.landmarkPositions_device[7].y(),
          rightLandmarkPose.landmarkPositions_device[7].z(),
          rightLandmarkPose.landmarkPositions_device[8].x(),
          rightLandmarkPose.landmarkPositions_device[8].y(),
          rightLandmarkPose.landmarkPositions_device[8].z(),
          rightLandmarkPose.landmarkPositions_device[9].x(),
          rightLandmarkPose.landmarkPositions_device[9].y(),
          rightLandmarkPose.landmarkPositions_device[9].z(),
          rightLandmarkPose.landmarkPositions_device[10].x(),
          rightLandmarkPose.landmarkPositions_device[10].y(),
          rightLandmarkPose.landmarkPositions_device[10].z(),
          rightLandmarkPose.landmarkPositions_device[11].x(),
          rightLandmarkPose.landmarkPositions_device[11].y(),
          rightLandmarkPose.landmarkPositions_device[11].z(),
          rightLandmarkPose.landmarkPositions_device[12].x(),
          rightLandmarkPose.landmarkPositions_device[12].y(),
          rightLandmarkPose.landmarkPositions_device[12].z(),
          rightLandmarkPose.landmarkPositions_device[13].x(),
          rightLandmarkPose.landmarkPositions_device[13].y(),
          rightLandmarkPose.landmarkPositions_device[13].z(),
          rightLandmarkPose.landmarkPositions_device[14].x(),
          rightLandmarkPose.landmarkPositions_device[14].y(),
          rightLandmarkPose.landmarkPositions_device[14].z(),
          rightLandmarkPose.landmarkPositions_device[15].x(),
          rightLandmarkPose.landmarkPositions_device[15].y(),
          rightLandmarkPose.landmarkPositions_device[15].z(),
          rightLandmarkPose.landmarkPositions_device[16].x(),
          rightLandmarkPose.landmarkPositions_device[16].y(),
          rightLandmarkPose.landmarkPositions_device[16].z(),
          rightLandmarkPose.landmarkPositions_device[17].x(),
          rightLandmarkPose.landmarkPositions_device[17].y(),
          rightLandmarkPose.landmarkPositions_device[17].z(),
          rightLandmarkPose.landmarkPositions_device[18].x(),
          rightLandmarkPose.landmarkPositions_device[18].y(),
          rightLandmarkPose.landmarkPositions_device[18].z(),
          rightLandmarkPose.landmarkPositions_device[19].x(),
          rightLandmarkPose.landmarkPositions_device[19].y(),
          rightLandmarkPose.landmarkPositions_device[19].z(),
          rightLandmarkPose.landmarkPositions_device[20].x(),
          rightLandmarkPose.landmarkPositions_device[20].y(),
          rightLandmarkPose.landmarkPositions_device[20].z());

      if (!read_success) {
        break;
      }
      WristAndPalmPose landmarkPose;
      landmarkPose.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      landmarkPose.leftHand = leftLandmarkPose;
      landmarkPose.rightHand = rightLandmarkPose;
      assignWristAndPalm(landmarkPose);
      landmarkPoses.push_back(landmarkPose);
    }
    std::cout << "Loaded #LandmarkPoses: " << landmarkPoses.size() << std::endl;
  } catch (std::exception& e) {
    std::cout << "Warning: failed to parse landmark file v3: " << filepath << e.what() << std::endl;
  }
  return landmarkPoses;
}

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

// Read file using V4 reader first. If it returns empty list, then try V3, V2 and V1 readers in
// order. Regardless of the naming used below, if V4 or V3 reader is used, the output will be
// containing all the landmark positions. If V2 or V1 reader is used, the output will only contain
// wrist and palm positions.
WristAndPalmPoses readWristAndPalmPoses(const std::string& filepath) {
  WristAndPalmPoses wristAndPalmPoses = readWristAndPalmPosesV4(filepath);
  if (wristAndPalmPoses.empty()) {
    wristAndPalmPoses = readWristAndPalmPosesV3(filepath);
  }
  if (wristAndPalmPoses.empty()) {
    wristAndPalmPoses = readWristAndPalmPosesV2(filepath);
  }
  if (wristAndPalmPoses.empty()) {
    wristAndPalmPoses = readWristAndPalmPosesV1(filepath);
  }
  if (wristAndPalmPoses.empty()) {
    std::cerr << "Failed to read any wrist and palm poses from file: " << filepath << std::endl;
  }
  return wristAndPalmPoses;
}
} // namespace projectaria::tools::mps
