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

  // The GPS provider, e.g. "app" or "gps"
  vrs::DataPieceString provider{"provider"};

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

  // Vertical accuracy in meters
  vrs::DataPieceValue<float> verticalAccuracy{"vertical_accuracy"};

  // Speed over ground [m/s]
  vrs::DataPieceValue<float> speed{"speed"};

  // For Aria gen1, raw data store the raw numerical data from each satellite. Each element is a
  // string representing the raw data sentence from one satellite. Can be parsed by any standard GPS
  // library. For Aria gen2, rawData is deprecated and broken down into new two fields:
  // rawMeasurements and navigationMessages.
  vrs::DataPieceVector<std::string> rawData{"raw_data"};

  // Raw satellite measurements (such as pseudo-range), only available for gen2.
  vrs::DataPieceString rawMeasurements{"raw_measurements"};

  // Navigation message (which includes Ephemeris data) stored in text format, only available for
  // gen2.
  vrs::DataPieceString navigationMessages{"navigation_messages"};

  // A list of constellations and frequency bands used for this GNSS result, valid examples include
  // but not limited to:
  // GPS_L1, GPS_L2 GPS_L5
  // GLONASS_G1, GLONASS_G2, GLONASS_G3
  // GALILEO_E1, GALILEO_E5, GALILEO_E6
  // BEIDOU_B1C, BEIDOU_B2A, BEIDOU_B3I
  // SBAS, QZSS, NAVIC
  // only available for gen2.
  vrs::DataPieceVector<std::string> constellationsEnabled{"constellations_enabled"};

  vrs::AutoDataLayoutEnd end;
};

} // namespace datalayout
