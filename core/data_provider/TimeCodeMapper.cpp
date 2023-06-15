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
#include <data_provider/TimeCodeMapper.h>

#define DEFAULT_LOG_CHANNEL "TimeCodeMapper"
#include <logging/Log.h>

namespace projectaria::tools::data_provider {

TimeCodeMapper::TimeCodeMapper(
    const std::shared_ptr<vrs::MultiRecordFileReader>& reader,
    const std::shared_ptr<TimeSyncPlayer>& timesyncPlayer) {
  if (timesyncPlayer == nullptr) {
    return;
  }

  vrs::StreamId streamId = timesyncPlayer->getStreamId();
  int numTimeCode = reader->getRecordCount(streamId, vrs::Record::Type::DATA);
  recordInfoTimeNsVec_.reserve(numTimeCode);
  timeCodes_.reserve(numTimeCode);
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
    recordInfoTimeNsVec_.push_back(static_cast<int64_t>(recordInfo->timestamp * 1e9));
    timeCodes_.push_back(timesyncPlayer->getDataRecord());
  }
  recordInfoTimeNsVec_.shrink_to_fit();
  timeCodes_.shrink_to_fit();
}

int64_t TimeCodeMapper::convertFromTimeCodeToDeviceTimeNs(const int64_t timecodeTimeNs) const {
  if (!supportsTimeCodeDomain()) {
    return -1;
  }

  if (timecodeTimeNs <= timeCodes_.front().realTimestampNs) {
    return timeCodes_.front().monotonicTimestampNs - timeCodes_.front().realTimestampNs +
        timecodeTimeNs;
  }
  if (timecodeTimeNs >= timeCodes_.back().realTimestampNs) {
    return timeCodes_.back().monotonicTimestampNs - timeCodes_.back().realTimestampNs +
        timecodeTimeNs;
  }

  TimeSyncData query;
  query.realTimestampNs = timecodeTimeNs;
  auto timecodeIter = std::upper_bound( // finds first timestamp > query
      timeCodes_.begin(),
      timeCodes_.end(),
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

int64_t TimeCodeMapper::convertFromDeviceTimeToTimeCodeNs(const int64_t deviceTimeNs) const {
  if (!supportsTimeCodeDomain()) {
    return -1;
  }
  if (deviceTimeNs <= timeCodes_.front().monotonicTimestampNs) {
    return timeCodes_.front().realTimestampNs - timeCodes_.front().monotonicTimestampNs +
        deviceTimeNs;
  }
  if (deviceTimeNs >= timeCodes_.back().monotonicTimestampNs) {
    return timeCodes_.back().realTimestampNs - timeCodes_.back().monotonicTimestampNs +
        deviceTimeNs;
  }

  TimeSyncData query;
  query.monotonicTimestampNs = deviceTimeNs;
  auto timecodeIter = std::upper_bound( // finds first timestamp > query
      timeCodes_.begin(),
      timeCodes_.end(),
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

bool TimeCodeMapper::supportsTimeCodeDomain() const {
  return timeCodes_.size() > 0;
}

} // namespace projectaria::tools::data_provider
