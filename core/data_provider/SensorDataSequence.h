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

#include <iterator>
#include <queue>

#include <data_provider/DeliverQueuedOptions.h>
#include <data_provider/SensorData.h>

namespace projectaria::tools::data_provider {

struct CompareDeviceTime {
  bool operator()(const SensorData& lhs, const SensorData& rhs) {
    return lhs.getTimeNs(TimeDomain::DeviceTime) >= rhs.getTimeNs(TimeDomain::DeviceTime);
  }
};

class VrsDataProvider;

/**
 * @brief Forward iterator for a sensor data container
 */
class SensorDataIterator {
 public:
  SensorDataIterator() = default;

  SensorDataIterator(
      VrsDataProvider* provider,
      const std::priority_queue<SensorData, std::vector<SensorData>, CompareDeviceTime>& queue,
      const std::map<vrs::StreamId, int>& streamIdToNextIndex,
      const std::map<vrs::StreamId, int>& streamIdToSubsampleRate,
      int64_t endDeviceTimeNs);

  SensorDataIterator operator++();
  bool operator!=(const SensorDataIterator& other) const;
  bool operator==(const SensorDataIterator& other) const;

  const SensorData& operator*() const;

 private:
  VrsDataProvider* provider_; // non-owning pointer to vrs data provider
  // the queue stores "the next data" in each stream and orders data by device time
  std::priority_queue<SensorData, std::vector<SensorData>, CompareDeviceTime> queue_;
  // the next data index for each stream id
  std::map<vrs::StreamId, int> streamIdToNextIndex_;
  // index increments when each ++ operation is called
  std::map<vrs::StreamId, int> streamIdToSubsampleRate_;
  int64_t endDeviceTimeNs_; // ending point of the sequence
};

/**
 * @brief Interface for delivering sensor data sorted by timestamps, with iterator support
 */
class SensorDataSequence {
 public:
  /**
   * @brief Constructs the sequence
   * @param provider the provider holding the data
   * @param options options for substream selection, frame rate, etc
   */
  SensorDataSequence(VrsDataProvider* provider, const DeliverQueuedOptions& options);
  /**
   * @brief Returns the iterator representing the starting point of the sequence
   */
  SensorDataIterator begin();
  /**
   * @brief Returns iterator representing the end point of the sequence
   */
  static SensorDataIterator end();

 private:
  VrsDataProvider* provider_; // non-owning pointer to vrs data provider
  DeliverQueuedOptions options_;
};
} // namespace projectaria::tools::data_provider
