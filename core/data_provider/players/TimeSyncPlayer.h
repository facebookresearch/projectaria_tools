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

struct AriaTimeSyncConfigRecord {
  uint32_t streamId;
  double sampleRateHz;
  std::string mode;
};

enum class TimeSyncMode : uint8_t {
  TIMECODE, ///< Timecode timestamps
  TIC_SYNC, ///< TicSync timestamps
  SUBGHZ, ///< SubGHz timestamps
  UTC, ///< UTC timestamps (obtained from companion App)
  COUNT ///< Count of values in this enum type.
};

struct TimeSyncData {
  int64_t monotonicTimestampNs;
  int64_t realTimestampNs;
};

using TimeSyncCallback = std::function<
    bool(const TimeSyncData& data, const AriaTimeSyncConfigRecord& config, bool verbose)>;

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

  [[nodiscard]] const AriaTimeSyncConfigRecord& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const TimeSyncData& getDataRecord() const {
    return dataRecord_;
  }

  [[nodiscard]] const vrs::StreamId& getStreamId() const {
    return streamId_;
  }

  [[nodiscard]] double getNextTimestampSec() const {
    return nextTimestampSec_;
  }

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

 private:
  bool onDataLayoutRead(const vrs::CurrentRecord& r, size_t blockIndex, vrs::DataLayout& dl)
      override;

  const vrs::StreamId streamId_;
  TimeSyncCallback callback_ = [](const TimeSyncData&, const AriaTimeSyncConfigRecord&, bool) {
    return true;
  };

  AriaTimeSyncConfigRecord configRecord_;
  TimeSyncData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};

} // namespace projectaria::tools::data_provider
