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
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace projectaria::tools::mps {

/**
 * @brief A struct representing additional fields in new model output in the Central Pupil Frame
 * (CPF).
 */
struct EyeGazeVergence {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Gen1 data types
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
  float ty_right_eye{}; /**< Translation along Y for right eye origin in CPF frame. */
  float tz_right_eye{}; /**< Translation along Z for right eye origin in CPF frame. */

  /*********  Gen2 data types *****/
  float left_pitch{};
  float right_pitch{};
  bool left_blink = false;
  bool right_blink = false;
  // In CPF Frame
  Eigen::Vector3f left_entrance_pupil_position_meter{Eigen::Vector3f::Zero()};
  Eigen::Vector3f right_entrance_pupil_position_meter{Eigen::Vector3f::Zero()};
  float left_pupil_diameter_meter{};
  float right_pupil_diameter_meter{};

  // Gen2 validity flags
  bool left_gaze_valid = true;
  bool right_gaze_valid = true;
  bool left_blink_valid = true;
  bool right_blink_valid = true;
  bool left_entrance_pupil_position_valid = true;
  bool right_entrance_pupil_position_valid = true;
  bool left_pupil_diameter_valid = true;
  bool right_pupil_diameter_valid = true;

  // Gen2 per-eye valid score from the ML eye tracking model, in [0, 1] with 1 = valid. The
  // per-eye gaze valid flag is this score compared against the model's default threshold. NaN
  // when the source has no score (geometric eye tracking, models without a validity output, or
  // data that predates the score). Use `applyValidScoreThreshold` for another threshold.
  float left_valid_score = std::numeric_limits<float>::quiet_NaN();
  float right_valid_score = std::numeric_limits<float>::quiet_NaN();
};

/**
 * @brief A struct representing eye gaze direction estimate in the Central Pupil Frame (CPF).
 */
struct EyeGaze {
  /****************** Gen1 data types ****************/
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

  std::string session_uid; /*unique id for the calibration session. If there are
                            multiple in-session calibrations in the recording, each segment will
                            have a different session_uid.*/

  /***************** Gen2 data types ******************/
  Eigen::Vector3f spatial_gaze_point_in_cpf{Eigen::Vector3f::Zero()};
  Eigen::Vector3f combined_gaze_origin_in_cpf{Eigen::Vector3f::Zero()};

  // Gen2 data validity flags. Default to false; the producer (MPS CSV reader, on-device
  // EyeGazePlayer, or OatmealTypes converter) is responsible for setting them to true only
  // after it has populated the corresponding field with a real value.
  bool spatial_gaze_point_valid = false;
  bool combined_gaze_valid = false;
};

/**
 * @brief alias to represent a vector of `EyeGaze`
 */
using EyeGazes = std::vector<EyeGaze>;

/**
 * A helper function to get Gaze Direction as Vector 3D given yaw and pitch values
 */
inline Eigen::Vector3d getUnitVectorFromYawPitch(float yawRads, float pitchRads) {
  float z = 1; // arbitrary
  float x = std::tan(yawRads) * z;
  float y = std::tan(pitchRads) * z;
  Eigen::Vector3d direction(x, y, z);
  return direction.normalized();
}

namespace detail {
inline void checkValidScoreThreshold(float threshold) {
  if (std::isnan(threshold) || threshold < 0.f || threshold > 1.f) {
    throw std::invalid_argument("Valid score threshold must be in [0, 1]");
  }
}
} // namespace detail

/**
 * @brief Re-derives the Gen2 eye validity flags of `eyeGaze` at `threshold` instead of the eye
 * tracking model's default.
 *
 * An eye with a valid score is valid when its score is at least `threshold`. The combined gaze
 * and the spatial gaze point then need both eyes valid. An eye without a score keeps its flag,
 * and a frame where neither eye has a score is returned unchanged. The result depends only on
 * the scores, so applying a second threshold gives the same flags as applying it alone.
 *
 * @throws std::invalid_argument unless `threshold` is in [0, 1].
 */
inline EyeGaze applyValidScoreThreshold(EyeGaze eyeGaze, float threshold) {
  detail::checkValidScoreThreshold(threshold);
  auto& vergence = eyeGaze.vergence;
  const bool hasLeftScore = !std::isnan(vergence.left_valid_score);
  const bool hasRightScore = !std::isnan(vergence.right_valid_score);
  if (!hasLeftScore && !hasRightScore) {
    return eyeGaze;
  }
  if (hasLeftScore) {
    vergence.left_gaze_valid = vergence.left_valid_score >= threshold;
  }
  if (hasRightScore) {
    vergence.right_gaze_valid = vergence.right_valid_score >= threshold;
  }
  eyeGaze.combined_gaze_valid = vergence.left_gaze_valid && vergence.right_gaze_valid;
  eyeGaze.spatial_gaze_point_valid = eyeGaze.combined_gaze_valid;
  return eyeGaze;
}

/**
 * @brief Applies `applyValidScoreThreshold` to every frame of `eyeGazes`.
 *
 * @throws std::invalid_argument unless `threshold` is in [0, 1].
 */
inline EyeGazes applyValidScoreThreshold(EyeGazes eyeGazes, float threshold) {
  detail::checkValidScoreThreshold(threshold);
  for (auto& eyeGaze : eyeGazes) {
    eyeGaze = applyValidScoreThreshold(std::move(eyeGaze), threshold);
  }
  return eyeGazes;
}

} // namespace projectaria::tools::mps
