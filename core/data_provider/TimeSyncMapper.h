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
#include <optional>
#include <vector>

#include <mutex>

#include <vrs/IndexRecord.h>
#include <vrs/MultiRecordFileReader.h>
#include <vrs/StreamId.h>

#include <players/TimeSyncPlayer.h>

namespace projectaria::tools::data_provider {
// maps between device time and timecode time
class TimeSyncMapper {
 public:
  TimeSyncMapper() = default;
  // Construction reads no record payloads. Each TimeSync stream contributes its
  // record index -- already in memory once the file is open -- and the samples
  // themselves are fetched the first time a conversion needs them.
  //
  // Reading a TimeSync stream up-front is the worst possible access order on a
  // remote file: its records are interleaved with sensor data across the whole
  // file, so walking them cold pulls (and then evicts) every cache block before
  // any sensor read can reuse it. Fetching them on demand instead lets them ride
  // along in the blocks a sequential sweep is already reading.
  TimeSyncMapper(
      const std::shared_ptr<vrs::MultiRecordFileReader>& reader,
      const std::map<TimeSyncMode, std::shared_ptr<TimeSyncPlayer>>& timesyncPlayers);

  // general function to convert between two times in TimeSyncData
  // syncTime: TimeSyncData.realTimestampNs
  // deviceTime: TimeSyncData.monotonicTimestampNs
  [[nodiscard]] int64_t convertFromSyncTimeToDeviceTimeNs(int64_t syncTimeNs, TimeSyncMode mode)
      const;
  [[nodiscard]] int64_t convertFromDeviceTimeToSyncTimeNs(int64_t deviceTimeNs, TimeSyncMode mode)
      const;

  // backward compatible with timecode conversion
  [[nodiscard]] int64_t convertFromTimeCodeToDeviceTimeNs(int64_t timecodeTimeNs) const;
  [[nodiscard]] int64_t convertFromDeviceTimeToTimeCodeNs(int64_t deviceTimeNs) const;

  // only support TIMECODE, TIC_SYNC, SUBGHZ and UTC mode
  [[nodiscard]] bool supportsMode(TimeSyncMode mode) const;

  [[nodiscard]] std::vector<TimeSyncMode> getTimeSyncModes() const;

 private:
  /// One record of a TimeSync stream: where it lives, and its sample once fetched.
  struct Sample {
    const vrs::IndexRecord::RecordInfo* recordInfo = nullptr;
    TimeSyncData data{};
    bool loaded = false;
    /// Set when the record could not be read. `data` is then meaningless and must
    /// never reach a comparison or an interpolation.
    bool readFailed = false;
  };

  /// One TimeSync stream: its index (free) plus the samples fetched so far.
  struct ModeData {
    std::shared_ptr<TimeSyncPlayer> player;
    /// One entry per data record, in stream order. Populated at construction, no I/O.
    std::vector<Sample> samples;
    /// Record timestamps from the index, in nanoseconds, parallel to `samples`.
    /// Close to, but not necessarily bit-equal to, a sample's own
    /// monotonicTimestampNs, so they locate a candidate bracket that is then
    /// confirmed against the real samples. Kept separate to stay contiguous for
    /// the binary search.
    std::vector<int64_t> indexTimeNs;
    /// Last bracket used, so a sequential sweep costs O(1) instead of a binary search.
    size_t cursor = 0;
  };

  /// Fetch (and memoize) the sample at `index`. Null if its record could not be
  /// read, which fails the conversion rather than letting a zeroed sample through.
  [[nodiscard]] const TimeSyncData* sampleAt(ModeData& modeData, size_t index) const;
  /// Index of the last sample at or before `deviceTimeNs`, confirmed against
  /// samples. Empty if a record the search needed could not be read.
  [[nodiscard]] std::optional<size_t> findBracketByDeviceTime(
      ModeData& modeData,
      int64_t deviceTimeNs) const;
  /// Same, keyed on the sync clock, which the record index does not carry.
  [[nodiscard]] std::optional<size_t> findBracketBySyncTime(ModeData& modeData, int64_t syncTimeNs)
      const;

  std::shared_ptr<vrs::MultiRecordFileReader> reader_;
  /// Guards the on-demand fetches: they mutate the samples and drive the players,
  /// and the file handler underneath expects one caller at a time.
  // std::mutex rather than tsa::mutex: this header is part of the open-source
  // CMake build, which has no thread_safety_analysis.
  // NOLINTNEXTLINE(facebook-thread-safety-analysis)
  mutable std::mutex mutex_;
  mutable std::map<TimeSyncMode, ModeData> modes_;
  /// Fixed after construction, so `supportsMode()` and `getTimeSyncModes()` answer
  /// without taking the lock -- and can be called from a conversion that holds it.
  std::vector<TimeSyncMode> timeSyncModes_;
};
} // namespace projectaria::tools::data_provider
