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
#include <string>
#include <vector>

namespace projectaria::tools::mps {

/**
 * @brief A struct representing additional fields in new model output in the Central Pupil Frame
 * (CPF).
 */
struct EyeGazeVergence {
  /* additional Vergence Fields */

  float left_yaw{}; /**< Eye gaze left yaw angle: [the angle between projected left eye gaze ray (on
                X-Z plane) and Z axis] in radians in CPF frame.*/

  float right_yaw{}; /**< Eye gaze yaw angle: [the angle between projected right eye gaze ray (on
                X-Z plane) and Z axis] in radians in CPF frame.*/
  float left_yaw_low{}; /**< [left_yaw_low, left_yaw_high] represent the confidence interval of the
                      left eye gaze yaw.  The `yaw` is in the [left_yaw_low, left_yaw_high] interval
                      but not necessarily in the middle */

  float right_yaw_low{}; /**< [right_yaw_low, right_yaw_high] represent the confidence interval of
                    the right eye gaze yaw.  The `yaw` is in the [right_yaw_low, right_yaw_high]
                    interval but not necessarily in the middle */

  float left_yaw_high{}; /**< [left_yaw_low, left_yaw_high] represent the confidence interval of the
                     left eye gaze yaw.  The `yaw` is in the [left_yaw_low, left_yaw_high] interval
                     but not necessarily in the middle. */

  float right_yaw_high{}; /**< [right_yaw_low, right_yaw_high] represent the confidence interval of
                     the right eye gaze yaw.  The `yaw` is in the [right_yaw_low, right_yaw_high]
                     interval but not necessarily in the middle. */

  float tx_left_eye{}; /**< Translation along X for left eye origin in CPF frame. */
  float ty_left_eye{}; /**< Translation along Y for left eye origin in CPF frame. */
  float tz_left_eye{}; /**< Translation along Z for left eye origin in CPF frame. */

  float tx_right_eye{}; /**< Translation along X for right eye origin in CPF frame. */
  float ty_right_eye{}; /**< Translation along X for right eye origin in CPF frame. */
  float tz_right_eye{}; /**< Translation along X for right eye origin in CPF frame. */
};

/**
 * @brief A struct representing eye gaze direction estimate in the Central Pupil Frame (CPF).
 */
struct EyeGaze {
  std::chrono::microseconds
      trackingTimestamp; /**< The timestamp of the eye gaze sample in device time domain */

  float yaw{}; /**< Eye gaze yaw angle: [the angle between projected gaze ray (on X-Z plane) and Z
                axis] in radians in CPF frame.*/

  float pitch{}; /**< Eye gaze pitch angle: [the angle between projected gaze ray (on Y-Z plane) and
                  Z axis] in radians in CPF frame. For the new model, this is a common pitch for
                  both eyes.*/

  float depth{}; /**< Depth in meters of the 3D eye gaze point in CPF frame.  A value of 0 indicates
                  that the depth is not available. */

  float
      yaw_low{}; /**< [yaw_low, yaw_high] represent the confidence interval of the eye gaze yaw. The
                  `yaw` is in the [yaw_low, yaw_high] interval but not necessarily in the middle */
  float yaw_high{}; /**< [yaw_low, yaw_high] represent the confidence interval of the eye gaze yaw.
                     The `yaw` is in the [yaw_low, yaw_high] interval but not necessarily in the
                     middle */

  float pitch_low{}; /**< [pitch_low, pitch_high] represent the confidence interval of the eye gaze
                      pitch.  The `pitch` is in the [pitch_low, pitch_high] interval but not
                      necessarily in the middle. For the new model output, this corresponds to the
                      lower bound of the pitch common to both eyes. */
  float pitch_high{}; /**< [pitch_low, pitch_high] represent the confidence interval of the eye gaze
                       pitch.  The `pitch` is in the [pitch_low, pitch_high] interval but not
                       necessarily in the middle. For the new model output, this corresponds to the
                      upper bound of the pitch common to both eyes. */

  EyeGazeVergence
      vergence{}; /**< Additional fields in new model output in the Central Pupil Frame (CPF). */

  std::string session_uid; /*unique id for the calibration session. If there are multiple
                            in-session calibrations in the recording, each segment will have a
                            different session_uid.*/
};

/**
 * @brief alias to represent a vector of `EyeGaze`
 */
using EyeGazes = std::vector<EyeGaze>;

} // namespace projectaria::tools::mps
