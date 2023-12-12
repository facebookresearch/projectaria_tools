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

#include "Periodic.h"

#include <data_provider/players/GpsPlayer.h>

namespace projectaria::tools::vrs_check {

struct GpsStats {
  uint64_t accurate = 0; // Fixes with accuracy equal or better than the min accuracy
  uint64_t rawMeasurement = 0; // Total number of raw measurements
  uint64_t invalidRawMeasurement = 0; // Fixes with less than kMinRawMeasurements
};

class Gps : public Periodic {
 public:
  Gps(vrs::StreamId streamId, double sampleRateHz, float minAccuracy);
  // Setup the gps player
  bool setup(vrs::RecordFileReader& reader) override;
  GpsStats getGpsStats(); // Get stats specific to GPS
  void logStats() override;
  bool getResult() override; // Pass or fail for this stream

 private:
  void processData(const data_provider::GpsData& data);
  std::unique_ptr<data_provider::GpsPlayer> gpsPlayer_;
  GpsStats gpsStats_;
  // Constants that are used when validating the GPS fixes
  static constexpr int kMinRawMeasurements = 4;
  static constexpr float kMaxPercentInvalidRawMeas = 0.01;
  static constexpr float kLatitudeMax = 90.0;
  static constexpr float kLatitudeMin = -90.0;
  static constexpr float kLongitudeMax = 180.0;
  static constexpr float kLongitudeMin = -180.0;
  static constexpr float kAltitudeMin = -1000.0; // -1km
  static constexpr float kAltitudeMax = 10000.0; // 10km
  static constexpr int64_t kUtcMsMin = 1577836800000; // 01/01/2020 00:00:00
  // Minimum number of raw measurements expected in a fix, at least 4 satellites should be
  // visible for a 3D fix so expecting a minimum of 4 raw measurements
  const float minAccuracy_; // Minimum accuracy in meters
  const float defaultPeriodUs_; // Default period to use if not specified in the stream
};

} // namespace projectaria::tools::vrs_check
