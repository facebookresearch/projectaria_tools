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

#include <cstdint>

#include <vrs/DataLayout.h>
#include <vrs/DataPieces.h>

// Note: The VRS stream type for Pose data is vrs::RecordableTypeId::PoseRecordableClass.

namespace datalayout {

struct PoseConfigRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceValue<std::uint32_t> streamId{"stream_id"};

  vrs::AutoDataLayoutEnd end;
};

struct PoseDataRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  // Timestamp of capturing this sample, in nanoseconds.
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};
  // Translation of IMU left sensor pose in world coordinates
  vrs::DataPieceValue<vrs::Point3Df> T_World_ImuLeft_translation{"T_Odometry_BodyImu_translation"};
  // Rotation quaternion of IMU left sensor pose in world coordinates
  vrs::DataPieceValue<vrs::Point4Df> T_World_ImuLeft_quaternion{"T_Odometry_BodyImu_quaternion"};

  vrs::AutoDataLayoutEnd endLayout;
};

} // namespace datalayout
