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

#include <vrs/DataLayout.h>
#include <vrs/DataLayoutConventions.h>

namespace projectaria::tools::datalayout {

class VioResultConfigurationLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;
  vrs::DataPieceValue<uint32_t> streamId{"stream_id"};

  // Algorithm version to distinguish different VIO implementations (e.g., internal,
  // external, silicon, x86, etc). Formatted in string as "Major.minor",
  vrs::DataPieceString algorithmVersion{"algorithm_version"};

  vrs::DataPieceString algorithmName{"algorithm_name"};

  // Payload version to indicate the VIO message serialization format. Formatted in string as
  // "Major.minor",
  vrs::DataPieceString messageVersion{"message_version"};

  vrs::DataPieceValue<double> nominalRateHz{"nominal_rate_hz"};

  vrs::AutoDataLayoutEnd endLayout;
};

class VioResultDataLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceString frontendSessionUuid{"frontend_session_uuid"};
  vrs::DataPieceString odometryUuid{"odometry_uuid"};
  vrs::DataPieceValue<std::int32_t> frameId{"frame_id"};

  // Latest timestamp used from the sensor (camera or IMU)
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};

  // Various status flags
  vrs::DataPieceValue<std::int8_t> status{"status"};
  vrs::DataPieceValue<std::int8_t> resetStatus{"reset_status"};
  vrs::DataPieceValue<vrs::Bool> isTrackerHealthy{"is_tracker_healthy"};
  vrs::DataPieceValue<vrs::Bool> isMapTrackingHealthy{"is_map_tracking_healthy"};

  vrs::DataPieceValue<std::int64_t> numTrackMeasurements{"num_track_measurements"};
  vrs::DataPieceValue<std::int64_t> numKeypoints{"num_keypoints"};

  // Gravity vector in the odometry frame
  vrs::DataPieceValue<vrs::Point3Dd> gravityVecInOdometry{"gravity_vec_odometry"};

  // T_Odometry_BodyImu, stored as translation and quaternion separately
  vrs::DataPieceValue<vrs::Point3Dd> txyz_Odometry_BodyImu{"txyz_odometry_bodyimu"};
  vrs::DataPieceValue<vrs::Point4Dd> qxyzw_Odometry_BodyImu{"qxyzw_odometry_bodyimu"};

  // T_BodyImu_Device, from the initial device factory calibration
  vrs::DataPieceValue<vrs::Point3Dd> txyz_BodyImu_Device{"txyz_bodyimu_device"};
  vrs::DataPieceValue<vrs::Point4Dd> qxyzw_BodyImu_Device{"qxyzw_bodyimu_device"};

  // linearVelocity_odometry
  vrs::DataPieceValue<vrs::Point3Dd> linearVelocityInOdometry{"linear_velocity_odometry"};

  // angularVelocity_bodyImu
  vrs::DataPieceValue<vrs::Point3Dd> angularVelocityInBodyImu{"angular_velocity_bodyimu"};

  // Online calibration
  vrs::DataPieceString onlineCalibState{"online_calib_state"};

  vrs::AutoDataLayoutEnd endLayout;
};

class VioHighFrequencyResultDataLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceString frontendSessionUuid{"frontend_session_uuid"};

  // Latest IMU timestamp used to estimate this pose result
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};

  // Gravity vector in the odometry frame
  vrs::DataPieceValue<vrs::Point3Dd> gravityVec_Odometry{"gravity_vec_odometry"};

  // T_Odometry_Device, stored as translation and quaternion separately
  vrs::DataPieceValue<vrs::Point3Dd> txyz_Odometry_Device{"txyz_odometry_device"};
  vrs::DataPieceValue<vrs::Point4Dd> qxyzw_Odometry_Device{"qxyzw_odometry_device"};

  // linearVelocity in odometry frame
  vrs::DataPieceValue<vrs::Point3Dd> linearVelocity_Odometry{"linear_velocity_odometry"};

  // angularVelocity in device frame
  vrs::DataPieceValue<vrs::Point3Dd> angularVelocity_Device{"angular_velocity_device"};

  // Quality score: float between [0, 1] which describes how good the pose and dynamics are.
  // qualityScore = 1: we are confident the pose and dynamics "good"
  // qualityScore = 0: we have no confidence on the pose and dynamics quality
  vrs::DataPieceValue<float> qualityScore{"quality_score"};

  vrs::AutoDataLayoutEnd endLayout;
};

} // namespace projectaria::tools::datalayout
