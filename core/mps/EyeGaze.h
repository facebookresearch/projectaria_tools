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
#include <vector>

namespace projectaria::tools::mps {
/**
 * @brief A struct representing eye gaze direction estimate in the Central Pupil Frame (CPF).
 */
struct EyeGaze {
  std::chrono::microseconds
      trackingTimestamp; /**< The timestamp of the eye gaze sample in device time domain */

  float yaw; /**< Eye gaze yaw angle (horizontal) in radians in CPF frame. */

  float pitch; /**< Eye gaze pitch angle (vertical) in radians in CPF frame. */

  float depth; /**< Depth in meters of the 3D eye gaze point in CPF frame.  A value of 0 indicates
                  that the depth is not available. */

  float
      yaw_low; /**< [yaw_low, yaw_high] represent the confidence interval of the eye gaze yaw.  The
                  `yaw` is in the [yaw_low, yaw_high] interval but not necessarily in the middle */
  float
      yaw_high; /**< [yaw_low, yaw_high] represent the confidence interval of the eye gaze yaw.  The
                   `yaw` is in the [yaw_low, yaw_high] interval but not necessarily in the middle */

  float pitch_low; /**< [pitch_low, pitch_high] represent the confidence interval of the eye gaze
                      pitch.  The `pitch` is in the [pitch_low, pitch_high] interval but not
                      necessarily in the middle */
  float pitch_high; /**< [pitch_low, pitch_high] represent the confidence interval of the eye gaze
                       pitch.  The `pitch` is in the [pitch_low, pitch_high] interval but not
                       necessarily in the middle */
};

/**
 * @brief alias to represent a vector of `EyeGaze`
 */
using EyeGazes = std::vector<EyeGaze>;
} // namespace projectaria::tools::mps
