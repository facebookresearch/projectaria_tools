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

#include "TrajectoryReaders.h"

#ifndef CSV_IO_NO_THREAD
#define CSV_IO_NO_THREAD
#endif
#include "fast-cpp-csv-parser/csv.h"

#include <array>
#include <iostream>

namespace projectaria::tools::mps {

constexpr std::array<const char*, 20> kOpenLoopTrajectoryColumns = {
    "tracking_timestamp_us",
    "utc_timestamp_ns",
    "session_uid",
    "tx_odometry_device",
    "ty_odometry_device",
    "tz_odometry_device",
    "qx_odometry_device",
    "qy_odometry_device",
    "qz_odometry_device",
    "qw_odometry_device",
    "device_linear_velocity_x_odometry",
    "device_linear_velocity_y_odometry",
    "device_linear_velocity_z_odometry",
    "angular_velocity_x_device",
    "angular_velocity_y_device",
    "angular_velocity_z_device",
    "gravity_x_odometry",
    "gravity_y_odometry",
    "gravity_z_odometry",
    "quality_score"};

OpenLoopTrajectory readOpenLoopTrajectory(const std::string& path) {
  OpenLoopTrajectory trajectory;
  try {
    io::CSVReader<kOpenLoopTrajectoryColumns.size()> csv(path);
    // Read in the CSV header
    // allow extra column for future-proof forward compatibility
    const auto readHeader = [&](auto&&... args) {
      csv.read_header(io::ignore_extra_column, args...);
    };
    std::apply(readHeader, kOpenLoopTrajectoryColumns);

    std::string session_uid;
    std::int64_t tracking_timestamp_us;
    std::int64_t utc_timestamp_ns;
    Eigen::Vector3d t_device;
    Eigen::Quaterniond q_device;
    Eigen::Vector3d gravity_odometry;
    Eigen::Vector3d linearVelocity;
    Eigen::Vector3d angularVelocity;
    float quality_score;

    while (csv.read_row(
        tracking_timestamp_us,
        utc_timestamp_ns,
        session_uid,
        t_device.x(),
        t_device.y(),
        t_device.z(),
        q_device.x(),
        q_device.y(),
        q_device.z(),
        q_device.w(),
        linearVelocity.x(),
        linearVelocity.y(),
        linearVelocity.z(),
        angularVelocity.x(),
        angularVelocity.y(),
        angularVelocity.z(),
        gravity_odometry.x(),
        gravity_odometry.y(),
        gravity_odometry.z(),
        quality_score)) {
      trajectory.emplace_back();
      auto& pose = trajectory.back();
      pose.sessionUid = session_uid;
      pose.T_odometry_device = Sophus::SE3d(q_device, t_device);
      pose.deviceLinearVelocity_odometry = linearVelocity;
      pose.angularVelocity_device = angularVelocity;
      pose.gravity_odometry = gravity_odometry;
      pose.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      pose.utcTimestamp = std::chrono::nanoseconds(utc_timestamp_ns);
      pose.qualityScore = quality_score;
    }
    std::cout << "Loaded #open loop trajectory poses records: " << trajectory.size() << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Failed to parse closed loop trajectory file: " << e.what() << std::endl;
  }
  return trajectory;
}

constexpr std::array<const char*, 20> kCloseLoopTrajectoryColumns = {
    "graph_uid",
    "tracking_timestamp_us",
    "utc_timestamp_ns",
    "tx_world_device",
    "ty_world_device",
    "tz_world_device",
    "qx_world_device",
    "qy_world_device",
    "qz_world_device",
    "qw_world_device",
    "device_linear_velocity_x_device",
    "device_linear_velocity_y_device",
    "device_linear_velocity_z_device",
    "angular_velocity_x_device",
    "angular_velocity_y_device",
    "angular_velocity_z_device",
    "gravity_x_world",
    "gravity_y_world",
    "gravity_z_world",
    "quality_score"};

ClosedLoopTrajectory readClosedLoopTrajectory(const std::string& path) {
  ClosedLoopTrajectory trajectory;
  try {
    io::CSVReader<kCloseLoopTrajectoryColumns.size()> csv(path);
    // Read in the CSV header
    // allow extra column for future-proof forward compatibility
    const auto readHeader = [&](auto&&... args) {
      csv.read_header(io::ignore_extra_column, args...);
    };
    std::apply(readHeader, kCloseLoopTrajectoryColumns);

    std::string graph_uid;
    std::int64_t tracking_timestamp_us;
    std::int64_t utc_timestamp_ns;
    Eigen::Vector3d t_device;
    Eigen::Quaterniond q_device;
    Eigen::Vector3d gravity;
    Eigen::Vector3d linearVelocity;
    Eigen::Vector3d angularVelocity;
    float quality_score;

    while (csv.read_row(
        graph_uid,
        tracking_timestamp_us,
        utc_timestamp_ns,
        t_device.x(),
        t_device.y(),
        t_device.z(),
        q_device.x(),
        q_device.y(),
        q_device.z(),
        q_device.w(),
        linearVelocity.x(),
        linearVelocity.y(),
        linearVelocity.z(),
        angularVelocity.x(),
        angularVelocity.y(),
        angularVelocity.z(),
        gravity.x(),
        gravity.y(),
        gravity.z(),
        quality_score)) {
      trajectory.emplace_back();
      auto& pose = trajectory.back();
      pose.graphUid = graph_uid;
      pose.T_world_device = Sophus::SE3d(q_device, t_device);
      pose.deviceLinearVelocity_device = linearVelocity;
      pose.angularVelocity_device = angularVelocity;
      pose.gravity_world = gravity;
      pose.trackingTimestamp = std::chrono::microseconds(tracking_timestamp_us);
      pose.utcTimestamp = std::chrono::nanoseconds(utc_timestamp_ns);
      pose.qualityScore = quality_score;
    }
    std::cout << "Loaded #closed loop trajectory poses records: " << trajectory.size() << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Failed to parse closed loop trajectory file: " << e.what() << std::endl;
  }
  return trajectory;
}

} // namespace projectaria::tools::mps
