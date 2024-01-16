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

#include "Gps.h"

#include <iostream>
#include <sstream>

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:Gps"
#include <logging/Log.h>

namespace projectaria::tools::vrs_check {

Gps::Gps(vrs::StreamId streamId, double sampleRateHz, float minAccuracy)
    : Periodic(streamId, 0.0),
      minAccuracy_(minAccuracy),
      defaultPeriodUs_(1.0 / static_cast<float>(sampleRateHz) * 1e6) {}

bool Gps::setup(vrs::RecordFileReader& reader) {
  data_provider::GpsCallback callback =
      [&](const data_provider::GpsData& data, const data_provider::GpsConfigRecord&, bool) {
        processData(data);
        return true;
      };
  gpsPlayer_ = std::make_unique<data_provider::GpsPlayer>(streamId_);
  if (!gpsPlayer_) {
    XR_LOGE("Cannot create playable for {}", streamId_.getName());
    return false;
  }
  gpsPlayer_->setCallback(callback);
  reader.setStreamPlayer(streamId_, gpsPlayer_.get());

  // Parse the configuration record
  if (!reader.readFirstConfigurationRecord(streamId_)) {
    XR_LOGE("Stream {} is missing a configuration record", streamId_.getName());
    return false;
  }

  const auto& config = gpsPlayer_->getConfigRecord();

  if (config.sampleRateHz > 0) {
    periodUs_ = 1 / config.sampleRateHz * 1e6;
  } else {
    XR_LOGW(
        "Stream {} is missing period, assuming {}us ({}Hz)",
        streamId_.getName(),
        defaultPeriodUs_,
        1 / (defaultPeriodUs_ / 1e6));
    periodUs_ = defaultPeriodUs_;
  }
  setMaxDeviationFromPeriodUs();
  preprocessStream(reader);

  return true;
}

GpsStats Gps::getGpsStats() {
  std::lock_guard lock{mutex_};
  return gpsStats_;
}

void Gps::logStats() {
  std::unique_lock lock{mutex_};
  std::stringstream seqDropStr;
  for (const auto& count : stats_.consecutiveDrops) {
    seqDropStr << count.first << ":" << count.second;
    if (count.first != (--stats_.consecutiveDrops.end())->first) {
      seqDropStr << " ";
    }
  }
  std::cout << fmt::format(
                   "{}: total={} expected={} processed={} dropped={} bad={} timeError={}"
                   " accurate={} rawMeas={} invalidRawMeas={} sequentialDrops=[{}]",
                   streamId_.getName(),
                   stats_.total,
                   stats_.expected,
                   stats_.processed,
                   stats_.dropped,
                   stats_.bad,
                   stats_.timeError,
                   gpsStats_.accurate,
                   gpsStats_.rawMeasurement,
                   gpsStats_.invalidRawMeasurement,
                   seqDropStr.str())
            << std::endl;
}

bool Gps::getResult() {
  if (stats_.total == stats_.bad) {
    XR_LOGE("{}: No valid GPS fix collected", streamId_.getName());
    return false;
  } else if (gpsStats_.accurate == 0) {
    XR_LOGE("{}: No accurate GPS fix collected", streamId_.getName());
    return false;
  } else if (gpsStats_.invalidRawMeasurement > kMaxPercentInvalidRawMeas * stats_.total) {
    XR_LOGE(
        "{}: Invalid number of raw measurements {} in more than {}% of the fixes",
        streamId_.getName(),
        gpsStats_.invalidRawMeasurement,
        kMaxPercentInvalidRawMeas * 100);
    return false;
  }
  return true;
}

void Gps::processData(const data_provider::GpsData& data) {
  std::lock_guard lock{mutex_};
  if (data.captureTimestampNs < 0 || data.latitude > kLatitudeMax || data.latitude < kLatitudeMin ||
      data.longitude > kLongitudeMax || data.longitude < kLongitudeMin ||
      data.altitude > kAltitudeMax || data.altitude < kAltitudeMin || data.utcTimeMs < kUtcMsMin) {
    stats_.bad++;
  }
  if (data.accuracy <= minAccuracy_) {
    gpsStats_.accurate++;
  }
  gpsStats_.rawMeasurement += data.rawData.size();
  if (data.rawData.size() < kMinRawMeasurements) {
    gpsStats_.invalidRawMeasurement++;
  }
  processTimestamp(data.captureTimestampNs / 1e3);
}

} // namespace projectaria::tools::vrs_check
