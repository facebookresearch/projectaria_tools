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

#include <data_provider/DeliverQueuedOptions.h>
#include <data_provider/ErrorHandler.h>

namespace projectaria::tools::data_provider {
namespace {
std::set<vrs::StreamId> getAllStreamIds(
    const std::map<vrs::StreamId, size_t>& streamIdToDownSampleRate) {
  std::set<vrs::StreamId> streamIds;
  for (const auto& [streamId, _] : streamIdToDownSampleRate) {
    streamIds.insert(streamId);
  }
  return streamIds;
}
} // namespace

DeliverQueuedOptions::DeliverQueuedOptions(
    const int64_t truncateFirstDeviceTimeNs,
    const int64_t truncateLastDeviceTimeNs,
    const std::map<vrs::StreamId, size_t>& streamIdToDownSampleRate)
    : SubstreamSelector(getAllStreamIds(streamIdToDownSampleRate)),
      truncateFirstDeviceTimeNs_(truncateFirstDeviceTimeNs),
      truncateLastDeviceTimeNs_(truncateLastDeviceTimeNs),
      streamIdToDownSampleRate_(streamIdToDownSampleRate) {}

int64_t DeliverQueuedOptions::getTruncateFirstDeviceTimeNs() const {
  return truncateFirstDeviceTimeNs_;
}

int64_t DeliverQueuedOptions::getTruncateLastDeviceTimeNs() const {
  return truncateLastDeviceTimeNs_;
}

size_t DeliverQueuedOptions::getSubsampleRate(const vrs::StreamId& streamId) const {
  return streamIdToDownSampleRate_.at(streamId);
}

void DeliverQueuedOptions::setTruncateFirstDeviceTimeNs(int64_t timeNs) {
  checkAndThrow(timeNs >= 0);
  truncateFirstDeviceTimeNs_ = timeNs;
}

void DeliverQueuedOptions::setTruncateLastDeviceTimeNs(int64_t timeNs) {
  checkAndThrow(timeNs >= 0);
  truncateLastDeviceTimeNs_ = timeNs;
}

void DeliverQueuedOptions::setSubsampleRate(const vrs::StreamId& streamId, size_t rate) {
  checkAndThrow(rate > 0);
  streamIdToDownSampleRate_.at(streamId) = rate;
}

} // namespace projectaria::tools::data_provider
