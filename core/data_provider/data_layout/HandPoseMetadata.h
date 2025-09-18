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

#pragma once

#include <vrs/DataLayout.h>
#include <vrs/DataLayoutConventions.h>

namespace projectaria::tools::datalayout {

class HandPoseConfigurationLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;
  vrs::DataPieceValue<uint32_t> streamId{"stream_id"};
  // Algorithm version to distinguish different hand tracking implementations. Formatted in string
  // as "Major.minor",
  vrs::DataPieceString algorithmVersion{"algorithm_version"};

  vrs::DataPieceString algorithmName{"algorithm_name"};

  vrs::DataPieceValue<double> nominalRateHz{"nominal_rate_hz"};

  // Whether the result is limited to wrist and palm joints.
  vrs::DataPieceValue<vrs::Bool> isWristPalmOnly{"is_wrist_palm_only"};

  // Optional json string representation for UserProfile:
  // https://fburl.com/code/bqu5tqfe
  // Default user profile will be used (for visualization) if this is empty. Hand scale estimation
  // is also included here
  vrs::DataPieceString userProfile{"user_profile"};

  vrs::AutoDataLayoutEnd endLayout;
};

struct SingleHandPoseLayoutStruct : public vrs::DataLayoutStruct {
  DATA_LAYOUT_STRUCT(SingleHandPoseLayoutStruct)

  static constexpr uint8_t kHtFingerCount = 5;
  static constexpr uint8_t kHtJointAnglesCount = 22;
  static constexpr uint8_t kHtLandmarksCount = 21;

  // Hand joint angles, length = kHtJointAnglesCount (22)
  vrs::DataPieceArray<float> jointAnglesRadian{"joint_angles_radian", kHtJointAnglesCount};
  // Rotation is a quaternion stored in the order of (x, y, z, w)
  vrs::DataPieceValue<vrs::Point4Df> wristRotationXyzw{"wrist_rotation_xyzw"};
  vrs::DataPieceValue<vrs::Point3Df> wristTranslationMeterXyz{"wrist_translation_meter_xyz"};

  vrs::DataPieceValue<float> handConfidence{"hand_confidence"};
  // Finger confidences, length = kHtFingerCount (5)
  vrs::DataPieceArray<float> fingerConfidences{"finger_confidences", kHtFingerCount};
  vrs::DataPieceValue<vrs::Bool> handValid{"hand_valid"};

  //// Derived results by processing the raw results
  // 3D landmark positions in the device reference frame, length = kHtLandmarksCount (21)
  // Invalid landmarks will have 0 values
  // Positions are stored in XYZ order in meters
  vrs::DataPieceArray<vrs::Point3Df> handLandmarks3d_DeviceMeterXyz{
      "hand_landmarks_3d_device_meter_xyz",
      kHtLandmarksCount};
};

class HandPoseDataLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;

  //// Raw results returned by the hand tracker
  // Latest timestamp used from the camera to perform hand tracking
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};
  // Left Hand
  SingleHandPoseLayoutStruct leftHand{"left_hand"};
  // Right Hand
  SingleHandPoseLayoutStruct rightHand{"right_hand"};

  vrs::AutoDataLayoutEnd endLayout;
};

} // namespace projectaria::tools::datalayout
