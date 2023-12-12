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

#include "Barometer.h"

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:Barometer"
#include <logging/Log.h>

namespace projectaria::tools::vrs_check {

Barometer::Barometer(vrs::StreamId streamId, float minScore, float minTemp, float maxTemp)
    : Periodic(streamId, minScore), minTemp_(minTemp), maxTemp_(maxTemp) {}

bool Barometer::setup(vrs::RecordFileReader& reader) {
  data_provider::BarometerCallback callback = [&](const data_provider::BarometerData& data,
                                                  const data_provider::BarometerConfigRecord&,
                                                  bool) {
    processData(data);
    return true;
  };

  barometerPlayer_ = std::make_unique<data_provider::BarometerPlayer>(streamId_);
  if (!barometerPlayer_) {
    XR_LOGE("Cannot create playable for {}", streamId_.getName());
    return false;
  }
  barometerPlayer_->setCallback(callback);
  reader.setStreamPlayer(streamId_, barometerPlayer_.get());

  // Parse the configuration record
  if (!reader.readFirstConfigurationRecord(streamId_)) {
    XR_LOGE("Stream {} is missing a configuration record", streamId_.getName());
    return false;
  }

  const auto& config = barometerPlayer_->getConfigRecord();
  periodUs_ = 1 / config.sampleRate * 1e6;
  setMaxDeviationFromPeriodUs();
  preprocessStream(reader);

  return true;
}

const BarometerStats& Barometer::getBarometerStats() {
  std::lock_guard lock{mutex_};
  return barometerStats_;
}

nlohmann::json Barometer::statsToJson() {
  const BarometerStats stats = getBarometerStats();
  nlohmann::json jsonStats = Periodic::statsToJson();
  jsonStats["repeat_pressure"] = stats.repeatPressure;
  jsonStats["repeat_temp"] = stats.repeatTemp;
  jsonStats["temp_out_of_range"] = stats.tempOutOfRange;
  return jsonStats;
}

void Barometer::logStats() {
  const BarometerStats stats = getBarometerStats();
  std::cout << fmt::format(
                   "{}: repeatPressure={} repeatTemp={} tempOutOfRange={}",
                   streamId_.getName(),
                   stats.repeatPressure,
                   stats.repeatTemp,
                   stats.tempOutOfRange)
            << std::endl;
  Periodic::logStats();
}

void Barometer::processData(const data_provider::BarometerData& data) {
  std::lock_guard lock{mutex_};
  // Check for bad data
  if (data.pressure < 0 || data.captureTimestampNs < 0) {
    stats_.bad++;
  }
  // Check for repeated data
  if (data.pressure == prevPressure_) {
    barometerStats_.repeatPressure++;
  } else {
    prevPressure_ = data.pressure;
  }
  if (data.temperature == prevTemp_) {
    barometerStats_.repeatTemp++;
  } else {
    if (data.temperature < minTemp_ || data.temperature > maxTemp_) {
      barometerStats_.tempOutOfRange++;
    }
    prevTemp_ = data.temperature;
  }
  processTimestamp(data.captureTimestampNs / 1e3);
}

} // namespace projectaria::tools::vrs_check
