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
#include <chrono>
#include <optional>
#include <vector>

#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace projectaria::tools::mps {

/**
 * @brief An enumeration representing the side (left or right) of the hand tracking data.
 */
enum class HANDEDNESS : uint8_t { LEFT = 0, RIGHT = 1 };

// A enum class to represent the type of hand landmarks
enum class HandLandmark : uint8_t {
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

  NUM_LANDMARKS = 21,
};

// Type for single hand 3D landmarks, (x, y, z) array in meters in device coordinate
constexpr uint32_t kNumHandLandmarks = static_cast<uint32_t>(HandLandmark::NUM_LANDMARKS);
using Landmarks = std::array<Eigen::Vector3d, kNumHandLandmarks>;

constexpr uint32_t kNumHandJointConnections = 23;
constexpr std::array<std::pair<HandLandmark, HandLandmark>, kNumHandJointConnections>
    kHandJointConnections = {
        {{HandLandmark::WRIST, HandLandmark::PINKY_PROXIMAL},
         {HandLandmark::PINKY_PROXIMAL, HandLandmark::PINKY_INTERMEDIATE},
         {HandLandmark::PINKY_INTERMEDIATE, HandLandmark::PINKY_DISTAL},
         {HandLandmark::PINKY_DISTAL, HandLandmark::PINKY_FINGERTIP},
         {HandLandmark::WRIST, HandLandmark::RING_PROXIMAL},
         {HandLandmark::RING_PROXIMAL, HandLandmark::RING_INTERMEDIATE},
         {HandLandmark::RING_INTERMEDIATE, HandLandmark::RING_DISTAL},
         {HandLandmark::RING_DISTAL, HandLandmark::RING_FINGERTIP},
         {HandLandmark::WRIST, HandLandmark::MIDDLE_PROXIMAL},
         {HandLandmark::MIDDLE_PROXIMAL, HandLandmark::MIDDLE_INTERMEDIATE},
         {HandLandmark::MIDDLE_INTERMEDIATE, HandLandmark::MIDDLE_DISTAL},
         {HandLandmark::MIDDLE_DISTAL, HandLandmark::MIDDLE_FINGERTIP},
         {HandLandmark::WRIST, HandLandmark::INDEX_PROXIMAL},
         {HandLandmark::INDEX_PROXIMAL, HandLandmark::INDEX_INTERMEDIATE},
         {HandLandmark::INDEX_INTERMEDIATE, HandLandmark::INDEX_DISTAL},
         {HandLandmark::INDEX_DISTAL, HandLandmark::INDEX_FINGERTIP},
         {HandLandmark::WRIST, HandLandmark::THUMB_INTERMEDIATE},
         {HandLandmark::THUMB_INTERMEDIATE, HandLandmark::THUMB_DISTAL},
         {HandLandmark::THUMB_DISTAL, HandLandmark::THUMB_FINGERTIP},
         {HandLandmark::THUMB_INTERMEDIATE, HandLandmark::INDEX_PROXIMAL},
         {HandLandmark::INDEX_PROXIMAL, HandLandmark::MIDDLE_PROXIMAL},
         {HandLandmark::MIDDLE_PROXIMAL, HandLandmark::RING_PROXIMAL},
         {HandLandmark::RING_PROXIMAL, HandLandmark::PINKY_PROXIMAL}}};

/**
 * @brief A struct representing wrist and palm tracking status per frame.
 */
struct WristAndPalmPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct OneSide {
    // Gen1 data fields
    double confidence;
    Eigen::Vector3d wristPosition_device;
    Eigen::Vector3d palmPosition_device;
    struct WristAndPalmNormals {
      Eigen::Vector3d palmNormal_device{0., 0., 0.};
      Eigen::Vector3d wristNormal_device{0., 0., 0.};
    };
    std::optional<WristAndPalmNormals> wristAndPalmNormal_device;
  };
  // The timestamp of the wrist and palm tracking measurement in device time domain.
  std::chrono::microseconds trackingTimestamp{};
  // Per-hand wrist and palm pose. Can be null if the hand is not tracked.
  std::optional<OneSide> leftHand, rightHand;

  constexpr const std::optional<OneSide>& operator[](HANDEDNESS handedness) const {
    return (handedness == HANDEDNESS::LEFT) ? leftHand : rightHand;
  }
};

/**
 * @brief A struct representing hand tracking result per frame.
 */
struct HandTrackingResult {
  struct OneSide {
    double confidence = 0.;
    Landmarks landmarkPositions_device;
    Sophus::SE3d T_Device_Wrist;
    struct WristAndPalmNormals {
      Eigen::Vector3d palmNormal_device{};
      Eigen::Vector3d wristNormal_device{};
    };
    std::optional<WristAndPalmNormals> wristAndPalmNormal_device;

    OneSide() {
      landmarkPositions_device.fill(Eigen::Vector3d::Zero());
      T_Device_Wrist = Sophus::SE3d();
    }

    // Helper functions to get wrist/palm_positions_device.
    [[nodiscard]] constexpr const Eigen::Vector3d& getWristPositionInDevice() const {
      return landmarkPositions_device[static_cast<uint8_t>(HandLandmark::WRIST)];
    }

    [[nodiscard]] constexpr const Eigen::Vector3d& getPalmPositionInDevice() const {
      return landmarkPositions_device[static_cast<uint8_t>(HandLandmark::PALM_CENTER)];
    }
  };

  // The timestamp of the wrist and palm tracking measurement in device time domain.
  std::chrono::microseconds trackingTimestamp{};

  // Per-hand tracking result. Can be null if the hand is not tracked.
  std::optional<OneSide> leftHand, rightHand;

  constexpr const std::optional<OneSide>& operator[](HANDEDNESS handedness) const {
    return (handedness == HANDEDNESS::LEFT) ? leftHand : rightHand;
  }
};

/**
 * @brief alias to represent a vector of `WristAndPalmPose`
 */
using WristAndPalmPoses = std::vector<WristAndPalmPose>;

/**
 * @brief alias to represent a vector of `HandTrackingResult`
 */
using HandTrackingResults = std::vector<HandTrackingResult>;
} // namespace projectaria::tools::mps
