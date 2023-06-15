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

// Note: The VRS stream type for GPS data is vrs::RecordableTypeId::GpsRecordableClass.

namespace datalayout {

struct GpsConfigRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceValue<std::uint32_t> streamId{"stream_id"};

  // Sample rate [Hz]
  vrs::DataPieceValue<double> sampleRateHz{"sample_rate_hz"};

  vrs::AutoDataLayoutEnd end;
};

struct GpsDataMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  // Timestamp of capturing this sample, in nanoseconds.
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};

  // UTC time in milliseconds.
  vrs::DataPieceValue<std::int64_t> utcTimeMs{"utc_time_ms"};

  // GPS fix data: This is the calculated positional information from raw satellite data.
  // Provider is typically "gps"
  vrs::DataPieceString provider{"provider"};
  // Latitude in degrees
  vrs::DataPieceValue<float> latitude{"latitude"};
  // Longitude in degrees
  vrs::DataPieceValue<float> longitude{"longitude"};
  // Altitude in meters
  vrs::DataPieceValue<float> altitude{"altitude"};

  // Horizontal accuracy in meters
  vrs::DataPieceValue<float> accuracy{"accuracy"};

  // Speed over ground [m/s]
  vrs::DataPieceValue<float> speed{"speed"};

  // Raw data: This is the raw numerical data from each satellite. Each element is a string
  // representing the raw data sentence from one satellite. Can be parsed by any standard GPS
  // library.
  vrs::DataPieceVector<std::string> rawData{"raw_data"};

  vrs::AutoDataLayoutEnd end;
};

} // namespace datalayout
