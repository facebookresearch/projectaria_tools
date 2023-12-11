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

#include "SensorMisalignmentStats.h"
#include "Stream.h"

#include <vrs/RecordFileReader.h>
#include <vrs/StreamId.h>

namespace projectaria::tools::vrs_check {

struct PeriodicStats : public Stats {
  uint64_t dropped = 0; // Frames dropped based on the period
  uint64_t timeError = 0; // Frames with unexpected time
  std::map<uint64_t, uint64_t> consecutiveDrops; // Number of consecutive drops of a given length
  uint64_t largestDeviationFromPeriodUs = 0; // Largest deviation from period observed in stream
  uint64_t nonMonotonic = 0; // Number of timestamps that are non-monotonic
};

struct DroppedFrame {
  uint64_t captureTimestampUs = 0;
  uint64_t expectedTimestampUs = 0;
  uint64_t deltaFromExpectedUs = 0;
  uint64_t deltaFromPreviousUs = 0;
  uint64_t periodUs = 0;
  int dropped = 0;
};

class Periodic : public Stream {
 public:
  Periodic(vrs::StreamId streamId, float minScore);
  void setMaxDeviationFromPeriodUs();
  // Get stats during processing, useful for showing processing progress
  const Stats getStats() override;
  void logStats() override;
  float getScore() override;
  void logScore() override;
  bool getResult() override; // Pass or fail for this stream
  uint32_t getPeriodUs() override {
    return periodUs_;
  }
  void logDroppedFrames(std::ofstream& csvWriter);
  static void setSensorMisalignmentStats(
      const std::map<std::string, std::unique_ptr<SensorHealthStats>>& sensorHealthStatsMap);

  static SensorMisalignmentStats* getSensorMisalignmentStats();
  nlohmann::json statsToJson() override;

 private:
  std::vector<DroppedFrame> droppedFrames_;

 protected:
  void preprocessStream(vrs::RecordFileReader& reader);
  void processTimestamp(uint64_t captureTimestampUs); // Needs to be called with the lock held
  const float minScore_; // Minimum score for pass criteria
  PeriodicStats stats_;
  uint32_t periodUs_; // Expected interval between frames
  uint64_t prevTimestampUs_ = 0;
  uint64_t maxDeviationFromPeriodUs_;
  uint64_t firstTimestampUs_ = 0;
  inline static std::unique_ptr<SensorMisalignmentStats> sensorMisalignmentStats_;
};

} // namespace projectaria::tools::vrs_check
