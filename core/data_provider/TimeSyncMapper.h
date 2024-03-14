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

#include <memory>
#include <vector>

#include <vrs/MultiRecordFileReader.h>
#include <vrs/StreamId.h>

#include <players/TimeSyncPlayer.h>

namespace projectaria::tools::data_provider {
// maps between device time and timecode time
class TimeSyncMapper {
 public:
  TimeSyncMapper() = default;
  explicit TimeSyncMapper(
      const std::shared_ptr<vrs::MultiRecordFileReader>& reader,
      const std::map<TimeSyncMode, std::shared_ptr<TimeSyncPlayer>>& timesyncPlayers);

  // general function to convert between two times in TimeSyncData
  // syncTime: TimeSyncData.realTimestampNs
  // deviceTime: TimeSyncData.monotonicTimestampNs
  int64_t convertFromSyncTimeToDeviceTimeNs(const int64_t syncTimeNs, const TimeSyncMode mode)
      const;
  int64_t convertFromDeviceTimeToSyncTimeNs(const int64_t deviceTimeNs, const TimeSyncMode mode)
      const;

  // backward compatible with timecode conversion
  int64_t convertFromTimeCodeToDeviceTimeNs(const int64_t timecodeTimeNs) const;
  int64_t convertFromDeviceTimeToTimeCodeNs(const int64_t deviceTimeNs) const;

  // only support TIMECODE and TIC_SYNC mode
  bool supportsMode(const TimeSyncMode mode) const;

  std::vector<TimeSyncMode> getTimeSyncModes() const;

 private:
  std::map<TimeSyncMode, std::shared_ptr<TimeSyncPlayer>> timesyncPlayers_;
  std::map<TimeSyncMode, std::vector<TimeSyncData>> timeSyncData_;
  std::map<TimeSyncMode, std::vector<int64_t>> recordInfoTimeNs_;
  std::vector<TimeSyncMode> timeSyncModes_;
};
} // namespace projectaria::tools::data_provider
