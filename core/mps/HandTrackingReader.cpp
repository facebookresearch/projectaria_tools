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

#include <sophus/se3.hpp>
#include <Eigen/Core>

#include <array>
#include <chrono>
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

HandTrackingResults readHandTrackingResultsV2(const std::string& filepath) {
  HandTrackingResults handTrackingResults;
  try {
    io::CSVReader<155> csv(filepath);

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
        "tx_left_device_wrist",
        "ty_left_device_wrist",
        "tz_left_device_wrist",
        "qx_left_device_wrist",
        "qy_left_device_wrist",
        "qz_left_device_wrist",
        "qw_left_device_wrist",
        "tx_right_device_wrist",
        "ty_right_device_wrist",
        "tz_right_device_wrist",
        "qx_right_device_wrist",
        "qy_right_device_wrist",
        "qz_right_device_wrist",
        "qw_right_device_wrist",
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
    HandTrackingResult::OneSide leftHandTrackingResult, rightHandTrackingResult;

    while (true) {
      bool read_success = false;
      leftHandTrackingResult.wristAndPalmNormal_device =
          rightHandTrackingResult.wristAndPalmNormal_device = {
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
      leftHandTrackingResult.landmarkPositions_device.fill(Eigen::Vector3d::Zero());
      rightHandTrackingResult.landmarkPositions_device.fill(Eigen::Vector3d::Zero());
      auto& leftLandmarkPositions_device = leftHandTrackingResult.landmarkPositions_device;
      auto& rightLandmarkPositions_device = rightHandTrackingResult.landmarkPositions_device;
      std::array<Eigen::Quaterniond, 2> R_Device_Wrist;
      std::array<Eigen::Vector3d, 2> t_Device_Wrist;
      read_success = csv.read_row(
          tracking_timestamp_us,
          leftHandTrackingResult.confidence,
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].z(),
          rightHandTrackingResult.confidence,
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)]
              .x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)]
              .y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)]
              .z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].z(),
          // T_Device_Wrist fields start:
          t_Device_Wrist[0].x(),
          t_Device_Wrist[0].y(),
          t_Device_Wrist[0].z(),
          R_Device_Wrist[0].x(),
          R_Device_Wrist[0].y(),
          R_Device_Wrist[0].z(),
          R_Device_Wrist[0].w(),
          t_Device_Wrist[1].x(),
          t_Device_Wrist[1].y(),
          t_Device_Wrist[1].z(),
          R_Device_Wrist[1].x(),
          R_Device_Wrist[1].y(),
          R_Device_Wrist[1].z(),
          R_Device_Wrist[1].w(),
          // Normal vector fields start:
          leftHandTrackingResult.wristAndPalmNormal_device->palmNormal_device.x(),
          leftHandTrackingResult.wristAndPalmNormal_device->palmNormal_device.y(),
          leftHandTrackingResult.wristAndPalmNormal_device->palmNormal_device.z(),
          leftHandTrackingResult.wristAndPalmNormal_device->wristNormal_device.x(),
          leftHandTrackingResult.wristAndPalmNormal_device->wristNormal_device.y(),
          leftHandTrackingResult.wristAndPalmNormal_device->wristNormal_device.z(),
          rightHandTrackingResult.wristAndPalmNormal_device->palmNormal_device.x(),
          rightHandTrackingResult.wristAndPalmNormal_device->palmNormal_device.y(),
          rightHandTrackingResult.wristAndPalmNormal_device->palmNormal_device.z(),
          rightHandTrackingResult.wristAndPalmNormal_device->wristNormal_device.x(),
          rightHandTrackingResult.wristAndPalmNormal_device->wristNormal_device.y(),
          rightHandTrackingResult.wristAndPalmNormal_device->wristNormal_device.z());

      if (!read_success) {
        break;
      }
      leftHandTrackingResult.T_Device_Wrist =
          Sophus::SE3d(R_Device_Wrist[0].normalized(), t_Device_Wrist[0]);
      rightHandTrackingResult.T_Device_Wrist =
          Sophus::SE3d(R_Device_Wrist[1].normalized(), t_Device_Wrist[1]);
      HandTrackingResult handTrackingResult;
      handTrackingResult.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      if (leftHandTrackingResult.confidence > 0) {
        handTrackingResult.leftHand = leftHandTrackingResult;
      } else {
        handTrackingResult.leftHand = std::nullopt;
      }
      if (rightHandTrackingResult.confidence > 0) {
        handTrackingResult.rightHand = rightHandTrackingResult;
      } else {
        handTrackingResult.rightHand = std::nullopt;
      }
      handTrackingResults.emplace_back(handTrackingResult);
    }
    std::cout << "Loaded #HandTrackingResults: " << handTrackingResults.size() << std::endl;
  } catch (std::exception& e) {
    std::cout << "Warning: failed to parse hand tracking result file v2: " << filepath << e.what()
              << std::endl;
  }
  return handTrackingResults;
}

HandTrackingResults readHandTrackingResultsV1(const std::string& filepath) {
  HandTrackingResults handTrackingResults;
  try {
    io::CSVReader<143> csv(filepath);

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
        "tx_left_device_wrist",
        "ty_left_device_wrist",
        "tz_left_device_wrist",
        "qx_left_device_wrist",
        "qy_left_device_wrist",
        "qz_left_device_wrist",
        "qw_left_device_wrist",
        "tx_right_device_wrist",
        "ty_right_device_wrist",
        "tz_right_device_wrist",
        "qx_right_device_wrist",
        "qy_right_device_wrist",
        "qz_right_device_wrist",
        "qw_right_device_wrist");

    int64_t tracking_timestamp_us = 0;
    HandTrackingResult::OneSide leftHandTrackingResult, rightHandTrackingResult;

    while (true) {
      bool read_success = false;
      leftHandTrackingResult.landmarkPositions_device =
          rightHandTrackingResult.landmarkPositions_device = {
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
      auto& leftLandmarkPositions_device = leftHandTrackingResult.landmarkPositions_device;
      auto& rightLandmarkPositions_device = rightHandTrackingResult.landmarkPositions_device;
      std::array<Eigen::Quaterniond, 2> R_Device_Wrist;
      std::array<Eigen::Vector3d, 2> t_Device_Wrist;
      read_success = csv.read_row(
          tracking_timestamp_us,
          leftHandTrackingResult.confidence,
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].z(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].x(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].y(),
          leftLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].z(),
          rightHandTrackingResult.confidence,
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_FINGERTIP)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_FINGERTIP)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_FINGERTIP)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_FINGERTIP)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_FINGERTIP)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_INTERMEDIATE)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::THUMB_DISTAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_INTERMEDIATE)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::INDEX_DISTAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_PROXIMAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)]
              .x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)]
              .y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_INTERMEDIATE)]
              .z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::MIDDLE_DISTAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_PROXIMAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_INTERMEDIATE)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::RING_DISTAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_INTERMEDIATE)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PINKY_DISTAL)].z(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].x(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].y(),
          rightLandmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)].z(),
          // T_Device_Wrist fields start:
          t_Device_Wrist[0].x(),
          t_Device_Wrist[0].y(),
          t_Device_Wrist[0].z(),
          R_Device_Wrist[0].x(),
          R_Device_Wrist[0].y(),
          R_Device_Wrist[0].z(),
          R_Device_Wrist[0].w(),
          t_Device_Wrist[1].x(),
          t_Device_Wrist[1].y(),
          t_Device_Wrist[1].z(),
          R_Device_Wrist[1].x(),
          R_Device_Wrist[1].y(),
          R_Device_Wrist[1].z(),
          R_Device_Wrist[1].w());

      if (!read_success) {
        break;
      }
      leftHandTrackingResult.T_Device_Wrist =
          Sophus::SE3d(R_Device_Wrist[0].normalized(), t_Device_Wrist[0]);
      rightHandTrackingResult.T_Device_Wrist =
          Sophus::SE3d(R_Device_Wrist[1].normalized(), t_Device_Wrist[1]);
      HandTrackingResult handTrackingResult;
      handTrackingResult.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      if (leftHandTrackingResult.confidence > 0) {
        handTrackingResult.leftHand = leftHandTrackingResult;
      } else {
        handTrackingResult.leftHand = std::nullopt;
      }
      if (rightHandTrackingResult.confidence > 0) {
        handTrackingResult.rightHand = rightHandTrackingResult;
      } else {
        handTrackingResult.rightHand = std::nullopt;
      }
      handTrackingResults.emplace_back(handTrackingResult);
    }
    std::cout << "Loaded #HandTrackingResults: " << handTrackingResults.size() << std::endl;
  } catch (std::exception& e) {
    std::cout << "Warning: failed to parse hand tracking result file: " << filepath << e.what()
              << std::endl;
  }
  return handTrackingResults;
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
      if (leftWristAndPalmPose.confidence > 0) {
        wristAndPalmPose.leftHand = leftWristAndPalmPose;
      } else {
        wristAndPalmPose.leftHand = std::nullopt;
      }
      if (rightWristAndPalmPose.confidence > 0) {
        wristAndPalmPose.rightHand = rightWristAndPalmPose;
      } else {
        wristAndPalmPose.rightHand = std::nullopt;
      }
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
      if (leftWristAndPalmPose.confidence > 0) {
        wristAndPalmPose.leftHand = leftWristAndPalmPose;
      } else {
        wristAndPalmPose.leftHand = std::nullopt;
      }
      if (rightWristAndPalmPose.confidence > 0) {
        wristAndPalmPose.rightHand = rightWristAndPalmPose;
      } else {
        wristAndPalmPose.rightHand = std::nullopt;
      }
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

// Read file using V2 reader first. If it returns empty list, then try V1 reader.
HandTrackingResults readHandTrackingResults(const std::string& filepath) {
  HandTrackingResults handTrackingResults = readHandTrackingResultsV2(filepath);
  if (handTrackingResults.empty()) {
    handTrackingResults = readHandTrackingResultsV1(filepath);
  }
  if (handTrackingResults.empty()) {
    std::cerr << "Failed to read any hand tracking result from file: " << filepath << std::endl;
  }
  return handTrackingResults;
}
} // namespace projectaria::tools::mps
