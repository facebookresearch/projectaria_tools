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

#include <stdint.h>

#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace projectaria::tools::vrs_check {

struct SensorMisalignmentStatistics {
  SensorMisalignmentStatistics() : total(0), misaligned(0), max_misalignment_us(0), score(0) {}
  int64_t total = 0;
  int64_t misaligned = 0;
  int64_t max_misalignment_us = 0;
  float score = 0.0f;
};

class SensorHealthStats {
 public:
  SensorHealthStats(const std::string& sensorName, int64_t periodUs);
  int64_t getPeriodUs() const {
    return periodUs_;
  }

 protected:
  std::string sensorName_;
  int64_t periodUs_; // Expected interval between frames
  int64_t marginUs_; // 10% of the period
  std::mutex mutex_; // Protect stats_ since it can be called async by the client
};

using SamplesVector = std::vector<std::pair<std::string, int64_t>>;

class SensorMisalignmentStats {
 public:
  explicit SensorMisalignmentStats(
      const std::map<std::string, std::unique_ptr<SensorHealthStats>>& sensorHealthStatsMap);

  void checkMisalignment(const std::string& newSensorId, int64_t newSensorTimestampUs);

  void computeScores();

  const std::
      unordered_map<std::string, std::unordered_map<std::string, SensorMisalignmentStatistics>>&
      misalignmentStatisticsMap() {
    return misalignmentStatisticsMap_;
  }

 private:
  void checkMisalignmentInSamplesVector(const SamplesVector& samplesVector);

  std::unordered_map<std::string, std::unordered_map<std::string, SensorMisalignmentStatistics>>
      misalignmentStatisticsMap_;

  std::map<int64_t, SamplesVector> alignmentCheckMap_;
  std::unordered_set<int64_t> timestampsToDelete_;
  std::mutex misalignmentMutex_;
  int64_t affinityRangeUs_ = std::numeric_limits<int64_t>::max();
};

} // namespace projectaria::tools::vrs_check
