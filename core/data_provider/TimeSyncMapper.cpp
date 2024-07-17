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
#include <data_provider/TimeSyncMapper.h>

#define DEFAULT_LOG_CHANNEL "TimeSyncMapper"
#include <logging/Log.h>

#include <algorithm>

namespace projectaria::tools::data_provider {

TimeSyncMapper::TimeSyncMapper(
    const std::shared_ptr<vrs::MultiRecordFileReader>& reader,
    const std::map<TimeSyncMode, std::shared_ptr<TimeSyncPlayer>>& timesyncPlayers) {
  if (timesyncPlayers.size() == 0) {
    return;
  }
  timesyncPlayers_ = timesyncPlayers;
  for (const auto& [mode, player] : timesyncPlayers) {
    vrs::StreamId streamId = player->getStreamId();
    int numTimeCode = reader->getRecordCount(streamId, vrs::Record::Type::DATA);
    recordInfoTimeNs_[mode].reserve(numTimeCode);
    timeSyncData_[mode].reserve(numTimeCode);
    timeSyncModes_.push_back(mode);

    for (int index = 0; index < numTimeCode; ++index) {
      const vrs::IndexRecord::RecordInfo* recordInfo =
          reader->getRecord(streamId, vrs::Record::Type::DATA, static_cast<uint32_t>(index));
      checkAndThrow(
          recordInfo, fmt::format("getRecord failed for {}, index {}", streamId.getName(), index));
      const int errorCode = reader->readRecord(*recordInfo);
      if (errorCode != 0) {
        XR_LOGE(
            "Fail to read record {} from streamId {} with code {}",
            index,
            streamId.getNumericName(),
            errorCode);
        continue;
      }
      recordInfoTimeNs_[mode].push_back(static_cast<int64_t>(recordInfo->timestamp * 1e9));
      timeSyncData_[mode].push_back(player->getDataRecord());
    }
    recordInfoTimeNs_[mode].shrink_to_fit();
    timeSyncData_[mode].shrink_to_fit();
  }
}

int64_t TimeSyncMapper::convertFromSyncTimeToDeviceTimeNs(
    const int64_t timecodeTimeNs,
    const TimeSyncMode mode) const {
  if (!supportsMode(mode)) {
    return -1;
  }
  auto timecodeData = timeSyncData_.at(mode);

  if (timecodeTimeNs <= timecodeData.front().realTimestampNs) {
    return timecodeData.front().monotonicTimestampNs - timecodeData.front().realTimestampNs +
        timecodeTimeNs;
  }
  if (timecodeTimeNs >= timecodeData.back().realTimestampNs) {
    return timecodeData.back().monotonicTimestampNs - timecodeData.back().realTimestampNs +
        timecodeTimeNs;
  }

  TimeSyncData query;
  query.realTimestampNs = timecodeTimeNs;
  auto timecodeIter = std::upper_bound( // finds first timestamp > query
      timecodeData.begin(),
      timecodeData.end(),
      query,
      [&](const auto& lhs, const auto& rhs) { return lhs.realTimestampNs < rhs.realTimestampNs; });
  auto lastTimeCodeIter = timecodeIter - 1;
  int64_t monoTimeRight = timecodeIter->monotonicTimestampNs;
  int64_t monoTimeLeft = lastTimeCodeIter->monotonicTimestampNs;
  int64_t realTimeRight = timecodeIter->realTimestampNs;
  int64_t realTimeLeft = lastTimeCodeIter->realTimestampNs;

  double ratioRight = double(timecodeTimeNs - realTimeLeft) / double(realTimeRight - realTimeLeft);
  double ratioLeft = 1 - ratioRight;

  return static_cast<int64_t>(ratioLeft * monoTimeLeft + ratioRight * monoTimeRight);
}

int64_t TimeSyncMapper::convertFromDeviceTimeToSyncTimeNs(
    const int64_t deviceTimeNs,
    const TimeSyncMode mode) const {
  if (!supportsMode(mode)) {
    return -1;
  }
  auto timecodeData = timeSyncData_.at(mode);

  if (deviceTimeNs <= timecodeData.front().monotonicTimestampNs) {
    return timecodeData.front().realTimestampNs - timecodeData.front().monotonicTimestampNs +
        deviceTimeNs;
  }
  if (deviceTimeNs >= timecodeData.back().monotonicTimestampNs) {
    return timecodeData.back().realTimestampNs - timecodeData.back().monotonicTimestampNs +
        deviceTimeNs;
  }

  TimeSyncData query;
  query.monotonicTimestampNs = deviceTimeNs;
  auto timecodeIter = std::upper_bound( // finds first timestamp > query
      timecodeData.begin(),
      timecodeData.end(),
      query,
      [&](const auto& lhs, const auto& rhs) {
        return lhs.monotonicTimestampNs < rhs.monotonicTimestampNs;
      });
  auto lastTimeCodeIter = timecodeIter - 1;
  int64_t monoTimeRight = timecodeIter->monotonicTimestampNs;
  int64_t monoTimeLeft = lastTimeCodeIter->monotonicTimestampNs;
  int64_t realTimeRight = timecodeIter->realTimestampNs;
  int64_t realTimeLeft = lastTimeCodeIter->realTimestampNs;

  double ratioRight = double(deviceTimeNs - monoTimeLeft) / double(monoTimeRight - monoTimeLeft);
  double ratioLeft = 1 - ratioRight;

  return static_cast<int64_t>(ratioLeft * realTimeLeft + ratioRight * realTimeRight);
}

int64_t TimeSyncMapper::convertFromTimeCodeToDeviceTimeNs(const int64_t timecodeTimeNs) const {
  return convertFromSyncTimeToDeviceTimeNs(timecodeTimeNs, TimeSyncMode::TIMECODE);
}

int64_t TimeSyncMapper::convertFromDeviceTimeToTimeCodeNs(const int64_t deviceTimeNs) const {
  return convertFromDeviceTimeToSyncTimeNs(deviceTimeNs, TimeSyncMode::TIMECODE);
}

bool TimeSyncMapper::supportsMode(const TimeSyncMode mode) const {
  return (timesyncPlayers_.find(mode) != timesyncPlayers_.end()) &&
      (mode == TimeSyncMode::TIMECODE ||
       mode == TimeSyncMode::TIC_SYNC); // only support TIMECODE and TIC_SYNC mode
}

std::vector<TimeSyncMode> TimeSyncMapper::getTimeSyncModes() const {
  return timeSyncModes_;
}

} // namespace projectaria::tools::data_provider
