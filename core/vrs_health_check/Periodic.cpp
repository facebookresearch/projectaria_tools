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

#include "Periodic.h"
#include "Utils.h"

#include <fmt/format.h>
#include <format/Format.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:Periodic"
#include <logging/Log.h>

namespace projectaria::tools::vrs_check {

Periodic::Periodic(vrs::StreamId streamId, float minScore)
    : Stream(streamId), minScore_(minScore) {}

void Periodic::setMaxDeviationFromPeriodUs() {
  maxDeviationFromPeriodUs_ = 0.20 * periodUs_;
}

void Periodic::setSensorMisalignmentStats(
    const std::map<std::string, std::unique_ptr<SensorHealthStats>>& sensorHealthStatsMap) {
  sensorMisalignmentStats_ = std::make_unique<SensorMisalignmentStats>(sensorHealthStatsMap);
}

SensorMisalignmentStats* Periodic::getSensorMisalignmentStats() {
  return sensorMisalignmentStats_.get();
}

Stats Periodic::getStats() {
  std::lock_guard lock{mutex_};
  return stats_;
}

nlohmann::json Periodic::statsToJson() {
  std::unique_lock lock{mutex_};
  const PeriodicStats stats = stats_;
  lock.unlock();
  nlohmann::json jsonStats = Stream::statsToJson();
  jsonStats["dropped"] = stats.dropped;
  jsonStats["time_error"] = stats.timeError;
  jsonStats["largest_deviation_from_period_us"] = stats_.largestDeviationFromPeriodUs;
  jsonStats["non_monotonic"] = stats.nonMonotonic;
  if (!stats.consecutiveDrops.empty()) {
    nlohmann::json sequentialDropsJson;
    for (const auto& count : stats.consecutiveDrops) {
      sequentialDropsJson[std::to_string(count.first)] = count.second;
    }
    jsonStats["sequential_drops"] = sequentialDropsJson;
  }
  return jsonStats;
}

void Periodic::logStats() {
  std::unique_lock lock{mutex_};
  std::stringstream seqDropStr;
  for (const auto& count : stats_.consecutiveDrops) {
    seqDropStr << count.first << ":" << count.second;
    if (count.first != (--stats_.consecutiveDrops.end())->first) {
      seqDropStr << " ";
    }
  }
  std::cout
      << fmt::format(
             "{}: total={} expected={} processed={} dropped={} bad={} nonMonotonic={} timeError={}"
             " largestDeviationFromPeriod={}us sequentialDrops=[{}](width:count)",
             streamId_.getName(),
             stats_.total,
             stats_.expected,
             stats_.processed,
             stats_.dropped,
             stats_.bad,
             stats_.nonMonotonic,
             stats_.timeError,
             stats_.largestDeviationFromPeriodUs,
             seqDropStr.str())
      << std::endl;
}

float Periodic::getScore() {
  std::lock_guard lock{mutex_};
  if (stats_.total == 0) {
    return 0.f;
  } else if (stats_.total + stats_.dropped + stats_.bad < stats_.expected) {
    // Still got less frames than expected. Probably too many period
    // variations in the data
    return float(100.0 * double(stats_.total) / stats_.expected);
  } else {
    return float(100.0 * (1.0 - double(stats_.dropped + stats_.bad) / stats_.expected));
  }
}

void Periodic::logScore() {
  Utils::logScore(streamId_.getName(), getScore(), minScore_);
}

void Periodic::logDroppedFrames(std::ofstream& csvWriter) {
  if (droppedFrames_.empty()) {
    return;
  }
  constexpr char csvHeader[] =
      "stream,captureTimestampUs,expectedTimestampUs,deltaFromExpectedUs,deltaFromPreviousUs,periodUs,dropped,firstTimestampUs,lastTimestampUs";
  if (csvWriter.tellp() == 0) {
    csvWriter << csvHeader;
  }
  for (const auto& sample : droppedFrames_) {
    csvWriter << fmt::format(
        "\n{},{},{},{},{},{},{},{},{}",
        getStreamId().getName(),
        sample.captureTimestampUs,
        sample.expectedTimestampUs,
        sample.deltaFromExpectedUs,
        sample.deltaFromPreviousUs,
        sample.periodUs,
        sample.dropped,
        firstTimestampUs_,
        prevTimestampUs_);
  }
}

bool Periodic::getResult() {
  float score = getScore();
  if (score < minScore_) {
    XR_LOGE("{}: Score {}% is less than minimum {}%", streamId_.getName(), score, minScore_);
    return false;
  }
  return true;
}

void Periodic::preprocessStream(vrs::RecordFileReader& reader) {
  std::lock_guard lock{mutex_};
  // Stats that don't require parsing through the whole file
  stats_.total = reader.getRecordCount(streamId_, vrs::Record::Type::DATA);
  // Count from the first frame, this intentionally ignores drops that happen before the first
  // valid frame. It's hard to know whether the data source just started late or dropped the
  // initial frames.
  if (stats_.total > 0) {
    const double firstTimestampUs =
        reader.getRecordByTime(streamId_, vrs::Record::Type::DATA, 0.0)->timestamp * 1e6;
    const double lastTimestampUs =
        reader.getLastRecord(streamId_, vrs::Record::Type::DATA)->timestamp * 1e6;
    stats_.expected = uint64_t(round((lastTimestampUs - firstTimestampUs) / periodUs_)) + 1;
  }
}

void Periodic::processTimestamp(const uint64_t captureTimestampUs) {
  if (prevTimestampUs_ == 0) {
    firstTimestampUs_ = captureTimestampUs;
  } else {
    const uint64_t expectedTimeUs = prevTimestampUs_ + periodUs_;
    const int64_t deviationFromOnePeriodUs = captureTimestampUs - expectedTimeUs;
    int64_t absoluteDeviationFromOnePeriodUs = std::abs(deviationFromOnePeriodUs);
    // Project absoluteDeviationFromPeriodUs to the expected timestamp for either the frame
    // before (measurement is late) or after (measurement is early) to determine the
    // period deviation.
    const uint64_t accountForDroppedMeasurements = std::abs(
        absoluteDeviationFromOnePeriodUs -
        (int64_t)(round(absoluteDeviationFromOnePeriodUs / periodUs_) * periodUs_));

    const uint64_t projectedAbsolutePeriodDeviationUs = std::min(
        accountForDroppedMeasurements,
        uint64_t(std::abs(int64_t(accountForDroppedMeasurements - periodUs_))));

    if (captureTimestampUs <= prevTimestampUs_) {
      stats_.nonMonotonic++;
    }
    if (projectedAbsolutePeriodDeviationUs > stats_.largestDeviationFromPeriodUs) {
      stats_.largestDeviationFromPeriodUs = projectedAbsolutePeriodDeviationUs;
    }
    if (projectedAbsolutePeriodDeviationUs > maxDeviationFromPeriodUs_) {
      std::cout << fmt::format(
                       "{}: Deviation from period {}us at ts={}us is over max allowed {}us",
                       streamId_.getName(),
                       projectedAbsolutePeriodDeviationUs,
                       captureTimestampUs,
                       maxDeviationFromPeriodUs_)
                << std::endl;
    }

    // A drop happens when the inter-frame gap is 20% over the period (strict)
    if (captureTimestampUs > expectedTimeUs + 0.20 * periodUs_) {
      int dropped = int(round(double(deviationFromOnePeriodUs) / periodUs_));
      if (dropped < 1) { // If the extra gap is between 20% and 50%, dropped rounds to 0.
        dropped = 1;
      }
      stats_.dropped += dropped;
      stats_.consecutiveDrops[dropped]++;
      droppedFrames_.push_back(DroppedFrame{
          .captureTimestampUs = captureTimestampUs,
          .expectedTimestampUs = expectedTimeUs,
          .deltaFromExpectedUs = captureTimestampUs - expectedTimeUs,
          .deltaFromPreviousUs = captureTimestampUs - prevTimestampUs_,
          .periodUs = periodUs_,
          .dropped = dropped});
    } else {
      const double marginUs = 0.10 * periodUs_; // 10%
      if (captureTimestampUs < expectedTimeUs - marginUs ||
          captureTimestampUs > expectedTimeUs + marginUs) {
        // Not a drop but the frame came at the wrong time
        stats_.timeError++;
      }
    }
  }
  prevTimestampUs_ = captureTimestampUs;
  sensorMisalignmentStats_->checkMisalignment(streamId_.getName(), captureTimestampUs);
  stats_.processed++;
}

} // namespace projectaria::tools::vrs_check
