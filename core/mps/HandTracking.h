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
#include <cinttypes>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace projectaria::tools::mps {

/**
 * @brief An enumeration representing the side (left or right) of the hand tracking data.
 */
enum class HANDEDNESS { LEFT = 0, RIGHT = 1 };

/**
 * @brief A struct representing wrist and palm tracking status per frame.
 */
struct WristAndPalmPose {
  struct OneSide {
    double confidence;
    Eigen::Vector3d wristPosition_device;
    Eigen::Vector3d palmPosition_device;
  };
  std::chrono::microseconds trackingTimestamp; /**< The timestamp of the wrist and palm tracking
                                                  measurement in device time domain */
  std::optional<OneSide> leftHand, rightHand; /**< The pose results of the
                                                        wrist and palm tracking frame */

  constexpr const std::optional<OneSide>& operator[](HANDEDNESS handedness) const {
    if (handedness == HANDEDNESS::LEFT) {
      return leftHand;
    } else {
      return rightHand;
    }
  }
};

/**
 * @brief alias to represent a vector of `WristAndPalmPose`
 */
using WristAndPalmPoses = std::vector<WristAndPalmPose>;
} // namespace projectaria::tools::mps
