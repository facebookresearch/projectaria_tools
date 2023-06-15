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

#include <data_layout/TimeSyncMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

namespace projectaria::tools::data_provider {

using TimeSyncCallback =
    std::function<bool(const vrs::CurrentRecord& r, vrs::DataLayout& dataLayout, bool verbose)>;

struct AriaTimeSyncConfigRecord {
  uint32_t streamId;
  double sampleRateHz;
  std::string mode;
};

struct TimeSyncData {
  int64_t monotonicTimestampNs;
  int64_t realTimestampNs;
};

class TimeSyncPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit TimeSyncPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  TimeSyncPlayer(const TimeSyncPlayer&) = delete;
  TimeSyncPlayer& operator=(const TimeSyncPlayer&) = delete;
  TimeSyncPlayer& operator=(TimeSyncPlayer&) = delete;
  TimeSyncPlayer(TimeSyncPlayer&&) = default;

  void setCallback(TimeSyncCallback callback) {
    callback_ = callback;
  }

  const AriaTimeSyncConfigRecord& getConfigRecord() const {
    return configRecord_;
  }

  const TimeSyncData& getDataRecord() const {
    return dataRecord_;
  }

  const vrs::StreamId& getStreamId() const {
    return streamId_;
  }

  double getNextTimestampSec() const {
    return nextTimestampSec_;
  }

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

 private:
  bool onDataLayoutRead(const vrs::CurrentRecord& r, size_t blockIndex, vrs::DataLayout& dl)
      override;

  const vrs::StreamId streamId_;
  TimeSyncCallback callback_ = [](const vrs::CurrentRecord&, vrs::DataLayout&, bool) {
    return true;
  };

  AriaTimeSyncConfigRecord configRecord_;
  TimeSyncData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};

} // namespace projectaria::tools::data_provider
