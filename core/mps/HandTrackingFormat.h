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

#include "HandTracking.h"
#include "format/Format.h"

/*
 * fmt::format() specialization for WristAndPalmPose::OneSide
 */
template <>
struct fmt::formatter<projectaria::tools::mps::WristAndPalmPose::OneSide>
    : fmt::formatter<std::string_view> {
  // Format the WristAndPalmPose object
  template <typename FormatContext>
  auto format(
      const projectaria::tools::mps::WristAndPalmPose::OneSide& oneSideWristAndPalmPose,
      FormatContext& ctx) const {
    return format_to(
        ctx.out(),
        "WristAndPalmPose::OneSide(confidence: {}, wrist: {}, palm: {})",
        oneSideWristAndPalmPose.confidence,
        oneSideWristAndPalmPose.wristPosition_device,
        oneSideWristAndPalmPose.palmPosition_device);
  }
};

/*
 * fmt::format() specialization for WristAndPalmPose
 */
template <>
struct fmt::formatter<projectaria::tools::mps::WristAndPalmPose>
    : fmt::formatter<std::string_view> {
  // Format the WristAndPalmPose object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::WristAndPalmPose& wristAndPalmPose, FormatContext& ctx)
      const {
    const auto& leftHandString = wristAndPalmPose.leftHand.has_value()
        ? fmt::to_string(wristAndPalmPose.leftHand.value())
        : "NONE";
    const auto& rightHandString = wristAndPalmPose.rightHand.has_value()
        ? fmt::to_string(wristAndPalmPose.rightHand.value())
        : "NONE";
    return format_to(
        ctx.out(),
        "WristAndPalmPose(tracking_timestamp: {}, left: {}, right: {})",
        wristAndPalmPose.trackingTimestamp,
        leftHandString,
        rightHandString);
  }
};
