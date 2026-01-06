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

#include <data_provider/RecordReaderInterface.h>

namespace projectaria::tools::data_provider {

// maps between device time and timecode time
class TimestampIndexMapper {
 public:
  explicit TimestampIndexMapper(std::shared_ptr<RecordReaderInterface> interface);

  // get start and end time w.r.t. different time domain
  [[nodiscard]] int64_t getFirstTimeNs(const vrs::StreamId& streamId, const TimeDomain& timeDomain)
      const;
  [[nodiscard]] int64_t getLastTimeNs(const vrs::StreamId& streamId, const TimeDomain& timeDomain)
      const;

  // get index of data based on timestamp in a specified domain
  int getIndexByTimeNs(
      const vrs::StreamId& streamId,
      int64_t timeNs,
      const TimeDomain& timeDomain,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);

  std::vector<int64_t> getTimestampsNs(const vrs::StreamId& streamId, const TimeDomain& timeDomain);

 private:
  /* timecode mapper */
  int getIndexBeforeTimeNsNonTimeCode(
      const vrs::StreamId& streamId,
      int64_t timeNsInTimeDomain,
      const TimeDomain& timeDomain);

  int getIndexAfterTimeNsNonTimeCode(
      const vrs::StreamId& streamId,
      int64_t timeNsInTimeDomain,
      const TimeDomain& timeDomain);

  int getIndexClosestTimeNsNonTimeCode(
      const vrs::StreamId& streamId,
      int64_t timeNsInTimeDomain,
      const TimeDomain& timeDomain);

 private:
  void cacheDataRecords();
  void findTimeRange();
  void findTimedomainDeltas();

  int getIndexAfterTimeNsNonTimeCodeFromIndexBefore(
      const vrs::StreamId& streamId,
      int64_t timeNsInTimeDomain,
      const TimeDomain& timeDomain,
      int indexBefore);

  int64_t
  getTimestampByIndex(const vrs::StreamId& streamId, int index, const TimeDomain& timeDomain);

 private:
  std::shared_ptr<RecordReaderInterface> interface_;
  std::map<vrs::StreamId, std::vector<const vrs::IndexRecord::RecordInfo*>> streamIdToDataRecords_;

  std::map<vrs::StreamId, std::array<int64_t, kNumTimeDomain - 1>> streamIdToFirstTimeNs_;
  std::map<vrs::StreamId, std::array<int64_t, kNumTimeDomain - 1>> streamIdToLastTimeNs_;
  std::map<vrs::StreamId, std::array<int64_t, kNumTimeDomain - 1>> streamIdToDeltaToRecordTimeNs_;
};
} // namespace projectaria::tools::data_provider
