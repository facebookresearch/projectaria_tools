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

#include <data_provider/ErrorHandler.h>
#include <data_provider/SensorDataSequence.h>
#include <data_provider/VrsDataProvider.h>

namespace projectaria::tools::data_provider {

SensorDataSequence::SensorDataSequence(
    VrsDataProvider* provider,
    const DeliverQueuedOptions& options)
    : provider_(provider), options_(options) {}

SensorDataIterator SensorDataSequence::begin() {
  const auto& streamIds = options_.getActiveStreamIds();

  int64_t startDeviceTimeNs = provider_->getFirstTimeNsAllStreams(TimeDomain::DeviceTime);
  int64_t endDeviceTimeNs = provider_->getLastTimeNsAllStreams(TimeDomain::DeviceTime);

  startDeviceTimeNs += options_.getTruncateFirstDeviceTimeNs();
  endDeviceTimeNs -= options_.getTruncateLastDeviceTimeNs();

  checkAndThrow(
      startDeviceTimeNs <= endDeviceTimeNs,
      fmt::format(
          "Start time : {} should be smaller or equal to end time : {} after truncation.",
          startDeviceTimeNs,
          endDeviceTimeNs));

  std::map<vrs::StreamId, int> streamIdToNextIndex;
  std::map<vrs::StreamId, int> streamIdToSubsampleRate;

  std::priority_queue<SensorData, std::vector<SensorData>, CompareDeviceTime> queue;
  for (const auto& streamId : streamIds) {
    if (!provider_->checkStreamIsActive(streamId)) {
      continue;
    }

    int index = provider_->getIndexByTimeNs(
        streamId, startDeviceTimeNs, TimeDomain::DeviceTime, TimeQueryOptions::After);
    streamIdToNextIndex.emplace(streamId, index + options_.getSubsampleRate(streamId));
    streamIdToSubsampleRate.emplace(streamId, options_.getSubsampleRate(streamId));

    SensorData data = provider_->getSensorDataByIndex(streamId, index);
    if (data.sensorDataType() == SensorDataType::NotValid) {
      continue;
    }
    queue.push(data);
  }
  return SensorDataIterator(
      provider_, queue, streamIdToNextIndex, streamIdToSubsampleRate, endDeviceTimeNs);
}

SensorDataIterator SensorDataSequence::end() {
  return {};
}

SensorDataIterator::SensorDataIterator(
    VrsDataProvider* provider,
    const std::priority_queue<SensorData, std::vector<SensorData>, CompareDeviceTime>& queue,
    const std::map<vrs::StreamId, int>& streamIdToNextIndex,
    const std::map<vrs::StreamId, int>& streamIdToSubsampleRate,
    const int64_t endDeviceTimeNs)
    : provider_(provider),
      queue_(queue),
      streamIdToNextIndex_(streamIdToNextIndex),
      streamIdToSubsampleRate_(streamIdToSubsampleRate),
      endDeviceTimeNs_(endDeviceTimeNs) {}

SensorDataIterator SensorDataIterator::operator++() {
  while (!queue_.empty()) {
    SensorData sensorData = queue_.top();
    queue_.pop();
    vrs::StreamId streamId = sensorData.streamId();

    int nextIndex = streamIdToNextIndex_.at(streamId);
    if (nextIndex < provider_->getNumData(streamId)) {
      SensorData nextData = provider_->getSensorDataByIndex(streamId, nextIndex);
      streamIdToNextIndex_.at(streamId) += streamIdToSubsampleRate_.at(streamId);
      if (nextData.getTimeNs(TimeDomain::DeviceTime) <= endDeviceTimeNs_) {
        queue_.push(nextData);
      }
    }

    if (sensorData.sensorDataType() != SensorDataType::NotValid) {
      return *this;
    }
  }
  return *this;
}

bool SensorDataIterator::operator!=(const SensorDataIterator& other) const {
  return queue_.empty() != other.queue_.empty();
}

bool SensorDataIterator::operator==(const SensorDataIterator& other) const {
  return queue_.empty() && other.queue_.empty();
}

const SensorData& SensorDataIterator::operator*() const {
  checkAndThrow(!queue_.empty(), "empty queue, data has already been exhausted");
  return queue_.top();
}

} // namespace projectaria::tools::data_provider
