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

#include "Trajectory.h"
#include "format/Format.h"

/*
 * fmt::format() specialization for OpenLoopTrajectoryPose
 */
template <>
struct fmt::formatter<projectaria::tools::mps::OpenLoopTrajectoryPose>
    : fmt::formatter<std::string_view> {
  // Format the OpenLoopTrajectoryPose object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::OpenLoopTrajectoryPose& pose, FormatContext& ctx)
      const {
    return fmt::format_to(
        ctx.out(),
        "OpenLoopTrajectory(tracking_timestamp: {}, utc_timestamp: {}, quality_score: {:.4f}, sessionUid: {}, T_odometry_device: {}, deviceLinearVelocity_odometry: {}, angularVelocity_device: {}, gravity_odometry: {})",
        pose.trackingTimestamp,
        pose.utcTimestamp,
        pose.qualityScore,
        pose.sessionUid,
        pose.T_odometry_device,
        pose.deviceLinearVelocity_odometry,
        pose.angularVelocity_device,
        pose.gravity_odometry);
  }
};

/*
 * fmt::format() specialization for ClosedLoopTrajectoryPose
 */
template <>
struct fmt::formatter<projectaria::tools::mps::ClosedLoopTrajectoryPose>
    : fmt::formatter<std::string_view> {
  // Format the ClosedLoopTrajectoryPose object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::ClosedLoopTrajectoryPose& pose, FormatContext& ctx)
      const {
    return fmt::format_to(
        ctx.out(),
        "ClosedLoopTrajectory(tracking_timestamp: {}, utc_timestamp: {}, quality_score: {:.4f}, graphUid: {}, T_world_device: {}, deviceLinearVelocity_device: {}, angularVelocity_device: {}, gravity_world: {})",
        pose.trackingTimestamp,
        pose.utcTimestamp,
        pose.qualityScore,
        pose.graphUid,
        pose.T_world_device,
        pose.deviceLinearVelocity_device,
        pose.angularVelocity_device,
        pose.gravity_world);
  }
};
