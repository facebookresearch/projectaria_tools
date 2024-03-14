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
#include <fmt/chrono.h>
#include <fmt/format.h>

#include "EyeGaze.h"

/*
 * fmt::format() specialization for EyeGazeVergence
 */
template <>
struct fmt::formatter<projectaria::tools::mps::EyeGazeVergence> : fmt::formatter<std::string_view> {
  // Format the EyeGazeVergence object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::EyeGazeVergence& gazeVergence, FormatContext& ctx)
      const {
    constexpr double kRadsToDegs = 180.0 / M_PI;

    return format_to(
        ctx.out(),
        "EyeGazeVergence(left_yaw: {:.2f} degs, right_yaw: {:.2f} degs, left_yaw_low: {:.2f} degs, right_yaw_low: {:.2f} degs, left_yaw_high: {:.2f} degs, right_yaw_high: {:.2f} degs, tx_left_eye: {} m, ty_left_eye: {} m, tz_left_eye: {} m, tx_right_eye: {} m, ty_right_eye: {} m, tz_right_eye: {} m)",
        gazeVergence.left_yaw * kRadsToDegs,
        gazeVergence.right_yaw * kRadsToDegs,
        gazeVergence.left_yaw_low * kRadsToDegs,
        gazeVergence.right_yaw_low * kRadsToDegs,
        gazeVergence.left_yaw_high * kRadsToDegs,
        gazeVergence.right_yaw_high * kRadsToDegs,
        gazeVergence.tx_left_eye,
        gazeVergence.ty_left_eye,
        gazeVergence.tz_left_eye,
        gazeVergence.tx_right_eye,
        gazeVergence.ty_right_eye,
        gazeVergence.tz_right_eye);
  }
};

/*
 * fmt::format() specialization for EyeGaze
 */
template <>
struct fmt::formatter<projectaria::tools::mps::EyeGaze> : fmt::formatter<std::string_view> {
  // Format the EyeGaze object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::EyeGaze& gaze, FormatContext& ctx) const {
    constexpr double kRadsToDegs = 180.0 / M_PI;
    return format_to(
        ctx.out(),
        "EyeGaze(tracking_timestamp: {}, yaw: {:.2f} degs, pitch: {:.2f} degs, depth: {} m, yaw_low: {:.2f} degs, yaw_high: {:.2f} degs, pitch_low: {:.2f} degs, pitch_high: {:.2f} degs, vergence: {}, session_uid: {})",
        gaze.trackingTimestamp,
        gaze.yaw * kRadsToDegs,
        gaze.pitch * kRadsToDegs,
        gaze.depth,
        gaze.yaw_low * kRadsToDegs,
        gaze.yaw_high * kRadsToDegs,
        gaze.pitch_low * kRadsToDegs,
        gaze.pitch_high * kRadsToDegs,
        gaze.vergence,
        gaze.session_uid);
  }
};
