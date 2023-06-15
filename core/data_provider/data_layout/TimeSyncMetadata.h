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

// Note: The VRS stream type for time sync data is vrs::RecordableTypeId::TimeRecordableClass.

namespace datalayout {

struct TimeSyncConfigRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceValue<std::uint32_t> streamId{"stream_id"};

  // Sample rate for time data [Hz]
  vrs::DataPieceValue<double> sampleRateHz{"sample_rate_hz"};

  /// Timestamp mode
  vrs::DataPieceString mode{"mode"};

  vrs::AutoDataLayoutEnd endLayout;
};

struct TimeSyncDataRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  // The capture timestamp in nanoseconds using a monotonic clock, same clock that
  // is used for the VRS records timestamps
  vrs::DataPieceValue<std::int64_t> monotonicTimestampNs{"monotonic_timestamp_ns"};

  // The real time clock or wall clock in nanoseconds
  vrs::DataPieceValue<std::int64_t> realTimestampNs{"real_timestamp_ns"};

  vrs::AutoDataLayoutEnd endLayout;
};

} // namespace datalayout
