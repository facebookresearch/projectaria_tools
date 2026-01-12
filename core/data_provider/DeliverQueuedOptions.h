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

#include <map>

#include <vrs/StreamId.h>

#include <data_provider/SubstreamSelector.h>

namespace projectaria::tools::data_provider {
/**
 * Options for delivering sensor data of multiple streams sorted in device timestamps
 */
class DeliverQueuedOptions : public SubstreamSelector {
 public:
  DeliverQueuedOptions(
      int64_t truncateFirstDeviceTimeNs,
      int64_t truncateLastDeviceTimeNs,
      const std::map<vrs::StreamId, size_t>& streamIdToDownSampleRate);

  /** @brief Returns how many nanoseconds to skip from the beginning of the vrs recording */
  [[nodiscard]] int64_t getTruncateFirstDeviceTimeNs() const;
  /** @brief Returns how many nanoseconds to skip before the end of the vrs recording */
  [[nodiscard]] int64_t getTruncateLastDeviceTimeNs() const;
  /**
   * @brief Returns how many times the frame rate is downsampled in a stream
   * @param streamId ID of the VRS stream of interest
   */
  [[nodiscard]] size_t getSubsampleRate(const vrs::StreamId& streamId) const;
  /**
   * @brief Sets how much time to skip from the beginning of the recording
   * @param time to skip in the beginning of the recording in nanoseconds
   */
  void setTruncateFirstDeviceTimeNs(int64_t timeNs);
  /**
   * @brief Sets how much time to skip from the end of the recording
   * @param time to skip in the end of the recording in nanoseconds
   */
  void setTruncateLastDeviceTimeNs(int64_t timeNs);
  /**
   * @brief Sets how many times the frame rate is downsampled in a stream
   * i.e, after a data is played, rate - 1 data are skipped
   * @param streamId ID of the VRS stream of interest
   */
  void setSubsampleRate(const vrs::StreamId& streamId, size_t rate);

 private:
  int64_t truncateFirstDeviceTimeNs_;
  int64_t truncateLastDeviceTimeNs_;
  std::map<vrs::StreamId, size_t> streamIdToDownSampleRate_;
};
} // namespace projectaria::tools::data_provider
