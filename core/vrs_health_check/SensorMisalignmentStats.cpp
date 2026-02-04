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

#include "SensorMisalignmentStats.h"

#include <cmath>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility> // for pair
#include <vector>

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:SensorMisalignmentStats"
#include <logging/Log.h>

#include "Logging.h"

namespace projectaria::tools::vrs_check {

namespace {

std::unordered_map<std::string, int64_t> kAlignmentToleranceUsMap = {
    {"SLAM-SLAM", 200},
    {"SLAM-RGB", 200}};

std::set<std::string> kConsideredSensors = {
    "Camera Data (SLAM) #1",
    "Camera Data (SLAM) #2",
    "RGB Camera Class #1"};

constexpr const int64_t kDeleteToleranceUs = 2e6;
constexpr const uint32_t kSamplesBacklogSize = 1000;

bool isMatchingSensorType(const std::string& sensorId, const std::string& sensorType) {
  return sensorId.find(sensorType) != std::string::npos;
}

// Find pairwise alignment type in kAlignmentToleranceUsMap by querying sensor type substrings in
// each map entry, in the two given sensor names. Return the value of the matching entry, which is
// the alignment tolerance value in microseconds
int64_t getAlignmentToleranceUs(const std::string& sensor1, const std::string& sensor2) {
  for (const auto& alignmentTolerance : kAlignmentToleranceUsMap) {
    std::vector<std::string> sensors;
    std::stringstream ss(alignmentTolerance.first);
    std::string item;
    while (std::getline(ss, item, '-')) {
      sensors.push_back(item);
    }
    if (sensors.size() != 2) {
      throw std::runtime_error("sensor size should be 2");
    }

    if ((isMatchingSensorType(sensor1, sensors[0]) && isMatchingSensorType(sensor2, sensors[1])) ||
        (isMatchingSensorType(sensor2, sensors[0]) && isMatchingSensorType(sensor1, sensors[1]))) {
      return alignmentTolerance.second;
    }
  }
  throw std::runtime_error(
      fmt::format(
          "Alignment tolerance isn't defined for sensor {} and sensor {}",
          sensor1.c_str(),
          sensor2.c_str()));
}

} // namespace

SensorHealthStats::SensorHealthStats(const std::string& sensorName, int64_t periodUs)
    : sensorName_(sensorName),
      periodUs_(periodUs),
      marginUs_(periodUs_ * 0.1) // 10% margin
{}

SensorMisalignmentStats::SensorMisalignmentStats(
    const std::map<std::string, std::unique_ptr<SensorHealthStats>>& sensorHealthStatsMap) {
  std::unordered_set<std::string> disabledSensors;
  for (const auto& sensorId : kConsideredSensors) {
    if (sensorHealthStatsMap.find(sensorId) == sensorHealthStatsMap.end()) {
      disabledSensors.insert(sensorId);
      continue;
    }
    const auto& sensorPeriodUs = sensorHealthStatsMap.at(sensorId)->getPeriodUs();
    if (sensorPeriodUs < affinityRangeUs_) {
      affinityRangeUs_ = sensorPeriodUs;
    }
  }
  // Filter out disabled sensors
  for (const auto& sensorId : disabledSensors) {
    kConsideredSensors.erase(sensorId);
  }
  // Set affinity range as half of the shortest period to ensure alignment checks only performed
  // within relevant samples
  affinityRangeUs_ /= 2;
}

void SensorMisalignmentStats::checkMisalignmentInSamplesVector(const SamplesVector& samplesVector) {
  for (const auto& sample1 : samplesVector) {
    for (const auto& sample2 : samplesVector) {
      // skip if equal or not in alphanumerical order
      if (sample1.first >= sample2.first) {
        continue;
      }

      const auto& sample1SensorTimestampUs = sample1.second;
      const auto& sample2SensorTimestampUs = sample2.second;
      std::string firstSensorId = sample1.first;
      std::string secondSensorId = sample2.first;

      // create implicitly if not there yet!
      SensorMisalignmentStatistics& stats =
          misalignmentStatisticsMap_[firstSensorId][secondSensorId];

      const int64_t misalignment = std::abs(sample1SensorTimestampUs - sample2SensorTimestampUs);

      stats.total++;
      stats.max_misalignment_us = std::max(stats.max_misalignment_us, misalignment);
      if (misalignment > getAlignmentToleranceUs(firstSensorId, secondSensorId)) {
        stats.misaligned++;
      }
    }
  }
}

// Comparing timestamps of the latest samples would complicate the misalignment check algorithm
// since pairwise sensor misalignment checks could potentially require considering different
// latest timestamps at each check. Instead, grouping samples with respect to their timestamps
// allows checking alignment for all sensors around a timepoint regardless of their period. This
// simplifies the algorithm by not worrying about dropped samples and just comparing all samples
// available at a given time
void SensorMisalignmentStats::checkMisalignment(
    const std::string& newSensorId,
    int64_t newSensorTimestampUs) {
  std::unique_lock<std::mutex> misalignmentLock(misalignmentMutex_);
  // Skip sample if it belongs to a sensor that is not considered for misalignment checks
  if (kConsideredSensors.find(newSensorId) == kConsideredSensors.end()) {
    return;
  }
  bool timeBucketFound = false;
  // Check all previously created time buckets to place the new sample in a vector to perform
  // the alignment check with
  for (auto& alignmentCheck : alignmentCheckMap_) {
    // Timestamp for the time bucket
    const auto& referenceTimestampUs = alignmentCheck.first;
    // If time bucket is older than kDeleteToleranceUs, check misalignment for all samples in the
    // samples vector and add timestamp of the bucket to the set of timestampsToDelete_ for
    // preventing increasing memory usage
    if (newSensorTimestampUs - referenceTimestampUs > kDeleteToleranceUs) {
      // Check misalignment for all samples in the bucket
      checkMisalignmentInSamplesVector(alignmentCheck.second);
      // Add bucket to the set of buckets to be deleted
      timestampsToDelete_.insert(referenceTimestampUs);
      continue;
    }
    // If new timestamp is close enough to the bucket's timestamp, add it to the bucket
    if (newSensorTimestampUs - referenceTimestampUs < affinityRangeUs_) {
      timeBucketFound = true;
      alignmentCheck.second.emplace_back(newSensorId, newSensorTimestampUs);
      break;
    }
  }
  // Discard old time buckets
  for (const auto& timestampToDelete : timestampsToDelete_) {
    alignmentCheckMap_.erase(timestampToDelete);
  }
  // Create a new time bucket with this sample's timestamp and add the sample to the list if there
  // was no matching bucket for it
  if (!timeBucketFound) {
    alignmentCheckMap_[newSensorTimestampUs].push_back({newSensorId, newSensorTimestampUs});
  }
  // Reset timestampsToDelete_
  timestampsToDelete_.clear();
  timestampsToDelete_.reserve(kSamplesBacklogSize);
}

void SensorMisalignmentStats::computeScores() {
  for (auto& alignmentCheck : alignmentCheckMap_) {
    checkMisalignmentInSamplesVector(alignmentCheck.second);
  }
  for (auto& [sensor1Id, misalignedTos] : misalignmentStatisticsMap_) {
    for (auto& misalignedTo : misalignedTos) {
      const auto& sensor2Id = misalignedTo.first;

      misalignedTo.second.score =
          100 - 100 * (float)misalignedTo.second.misaligned / (float)misalignedTo.second.total;

      XR_LOGI(
          "{} - {} alignment: {} ({} bad sets out of {})",
          sensor1Id.c_str(),
          sensor2Id.c_str(),
          misalignedTo.second.score,
          static_cast<int>(misalignedTo.second.misaligned),
          static_cast<int>(misalignedTo.second.total));
    }
  }
}

} // namespace projectaria::tools::vrs_check
