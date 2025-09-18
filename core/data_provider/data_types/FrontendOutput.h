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
#include <string>
#include <vector>

#include <sophus/se3.hpp>

#include <data_provider/data_types/FrontendTypes.h>

namespace projectaria::tools::data_provider {

struct FrontendOutput {
  // Uid tag for the Frontend session. Every time the Frontend resets, this session uid
  // needs to be regenerated.
  FrontendSessionUid frontendSessionUid;

  // Id of this frame set
  FrameSetId frameID;

  // Center capture time of the frame set
  int64_t captureTimestampNs;

  // Unix timestamp of the frame
  int64_t unixTimestampNs;

  // Status of the Frontend, whether there is pose result available
  VioStatus status = VioStatus::INVALID;
  // VIO tracking quality: quality of the pose results
  TrackingQuality poseQuality = TrackingQuality::UNKNOWN;
  // Visual-only tracking quality
  VisualTrackingQuality visualTrackingQuality = VisualTrackingQuality::UNKNOWN;

  // online calibration estimate
  OnlineCalibState onlineCalib;
  // camera serial numbers
  std::vector<std::string> cameraSerials;

  // Gravity vector in odometry frame.
  Eigen::Vector3f gravityInOdometry = Eigen::Vector3f::Zero();

  // This frame's pose in odometry reference.
  Sophus::SE3f T_Odometry_BodyImu;

  // pose of the device frame
  Sophus::SE3f T_BodyImu_Device;

  // This frame's linear velocity in odometry reference.
  Eigen::Vector3f linearVelocityInOdometry = Eigen::Vector3f::Zero();

  // This frame's angular velocity in the body IMU frame.
  Eigen::Vector3f angularVelocityInBodyImu = Eigen::Vector3f::Zero();
};

} // namespace projectaria::tools::data_provider
