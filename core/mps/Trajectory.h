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

#include <sophus/se3.hpp>
#include <Eigen/Core>

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace projectaria::tools::mps {

// Shared fields for different trajectory types
// Coordinate frame, pose and dynamics information is in each specialized trajectory types
struct TrajectoryPoseBase {
  // Aria internal processor timestamp in microseconds
  std::chrono::microseconds trackingTimestamp;

  // UTC Timestamp of the device image capture
  std::chrono::nanoseconds utcTimestamp;

  // Quality score: float between [0, 1] which describes how good the pose and dynamics are, the
  // higher score the estimation has higher quality
  float qualityScore;
};

/*
 * Open loop trajectory is the odometry estimation output by the visual-inertial odometry (VIO), in
 * an arbitrary odometry coordinate frame. The estimation includes pose and dynamics (translational
 * and angular velocities).
 *
 * The open loop trajectory has good “relative” and “local” accuracy: the relative transformation
 * between two frames is accurate when the time span between two frames is short (within a few
 * minutes). However, the open loop trajectory has increased drift error accumulated over time spent
 * and travel distance. Consider using closed loop trajectory if you are looking for trajectory
 * without drift error.
 */

struct OpenLoopTrajectoryPose : TrajectoryPoseBase {
  // Unique identifier of the odometry coordinate frame. When the session_uid is the same, poses and
  // velocities are defined in the same coordinate frame.
  std::string sessionUid;

  // Transformation from this device to an arbitrary odometry coordinate frame
  Sophus::SE3d T_odometry_device;

  // Translational velocity of device coordinate frame in odometry frame
  Eigen::Vector3d deviceLinearVelocity_odometry;

  // Angular velocity of device coordinate frame in device frame
  Eigen::Vector3d angularVelocity_device;

  // Earth gravity vector in odometry frame. This vector is pointing toward the ground, and includes
  // gravitation and centrifugal forces from earth rotation.
  Eigen::Vector3d gravity_odometry;
};

/*
 * Closed loop trajectory is the pose estimation output by our mapping process, in an arbitrary
 * gravity aligned world coordinate frame. The estimation includes pose and dynamics (translational
 * and angular velocities).

 * Closed loop trajectories are fully bundle adjusted with detected loop closures, reducing the VIO
 * drift which is present in the open loop trajectories. However, due to the loop closure
 * correction, the “relative” and “local” trajectory accuracy within a short time span (i.e.
 * seconds) might be worse compared to open loop trajectories.

 * In some datasets we also share and use this format for trajectory pose ground truth from
 * simulation or Optitrack
 */

struct ClosedLoopTrajectoryPose : TrajectoryPoseBase {
  // Unique identifier of the world coordinate frame. When the graphUid is the same, poses,
  // velocities and point clouds are defined in the same coordinate frame.
  std::string graphUid;

  // Transformation from this device to world coordinate frame
  Sophus::SE3d T_world_device;

  // Translational velocity of device coordinate frame in device frame
  Eigen::Vector3d deviceLinearVelocity_device;

  // Angular velocity of device coordinate frame in device frame
  Eigen::Vector3d angularVelocity_device;

  // Earth gravity vector in world frame. This vector is pointing toward the ground, and includes
  // gravitation and centrifugal forces from earth rotation.
  Eigen::Vector3d gravity_world = {0, 0, -9.81};
};

// A Trajectory is a list of Trajectory poses
using OpenLoopTrajectory = std::vector<OpenLoopTrajectoryPose>;
using ClosedLoopTrajectory = std::vector<ClosedLoopTrajectoryPose>;

} // namespace projectaria::tools::mps
