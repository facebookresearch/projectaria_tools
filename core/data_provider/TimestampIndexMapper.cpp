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
#include <data_provider/TimestampIndexMapper.h>

#define DEFAULT_LOG_CHANNEL "TimestampIndexMapper"
#include <logging/Log.h>

namespace projectaria::tools::data_provider {
TimestampIndexMapper::TimestampIndexMapper(std::shared_ptr<RecordReaderInterface> interface)
    : interface_(interface), streamIdToDataRecords_(interface_->getStreamIdToDataRecords()) {
  // find time range
  for (const auto& streamId : interface_->getStreamIds()) {
    // lambda function for finding first or last timestamp
    auto findFirstDataTimestamp = [&](int first, int last, int increment) {
      std::array<int64_t, kNumTimeDomain> timeNs;
      timeNs.fill(-1);
      for (int index = first; index != last; index += increment) {
        const vrs::IndexRecord::RecordInfo* recordInfo =
            interface_->readRecordByIndex(streamId, index);
        if (recordInfo) {
          for (auto timeDomain : std::vector<TimeDomain>{
                   TimeDomain::RecordTime, TimeDomain::DeviceTime, TimeDomain::HostTime}) {
            timeNs.at(static_cast<size_t>(timeDomain)) =
                interface_->getLastCachedSensorData(streamId).getTimeNs(timeDomain);
          }
          break;
        }
      }
      return timeNs;
    };

    int numData = interface_->getNumData(streamId);

    auto firstTimeNs = findFirstDataTimestamp(0, numData, 1);
    auto lastTimeNs = findFirstDataTimestamp(numData - 1, -1, -1);

    streamIdToFirstTimeNs_.emplace(streamId, firstTimeNs);
    streamIdToLastTimeNs_.emplace(streamId, lastTimeNs);
  }

  // find delta time between record and device/host time
  for (const auto& streamId : interface_->getStreamIds()) {
    for (auto timeDomain : {TimeDomain::DeviceTime, TimeDomain::HostTime}) {
      streamIdToDeltaToRecordTimeNs_[streamId].at(static_cast<size_t>(timeDomain)) =
          (getFirstTimeNs(streamId, TimeDomain::RecordTime) - getFirstTimeNs(streamId, timeDomain) +
           getLastTimeNs(streamId, TimeDomain::RecordTime) - getLastTimeNs(streamId, timeDomain)) /
          2;
    }
  }
}

// get start and end time w.r.t. different time domain
int64_t TimestampIndexMapper::getFirstTimeNs(
    const vrs::StreamId& streamId,
    const TimeDomain& timeDomain) const {
  checkAndThrow(
      streamIdToFirstTimeNs_.count(streamId) > 0,
      fmt::format("Cannot find streamId {}", streamId.getNumericName()));
  return streamIdToFirstTimeNs_.at(streamId).at(static_cast<size_t>(timeDomain));
}

int64_t TimestampIndexMapper::getLastTimeNs(
    const vrs::StreamId& streamId,
    const TimeDomain& timeDomain) const {
  checkAndThrow(
      streamIdToLastTimeNs_.count(streamId) > 0,
      fmt::format("Cannot find streamId {}", streamId.getNumericName()));
  return streamIdToLastTimeNs_.at(streamId).at(static_cast<size_t>(timeDomain));
}

int TimestampIndexMapper::getIndexByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNsInTimeDomain,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  interface_->setReadImageContent(streamId, false);
  int index = -1;
  switch (timeQueryOptions) {
    case TimeQueryOptions::Before:
      index = getIndexBeforeTimeNsNonTimeCode(streamId, timeNsInTimeDomain, timeDomain);
      break;
    case TimeQueryOptions::After:
      index = getIndexAfterTimeNsNonTimeCode(streamId, timeNsInTimeDomain, timeDomain);
      break;
    case TimeQueryOptions::Closest:
      index = getIndexClosestTimeNsNonTimeCode(streamId, timeNsInTimeDomain, timeDomain);
      break;
    default:
      break;
  }
  interface_->setReadImageContent(streamId, true);
  return index;
}

int TimestampIndexMapper::getIndexBeforeTimeNsNonTimeCode(
    const vrs::StreamId& streamId,
    const int64_t timeNsInTimeDomain,
    const TimeDomain& timeDomain) {
  // check if time is outside of the time range
  if (timeNsInTimeDomain < getFirstTimeNs(streamId, timeDomain)) {
    return -1;
  }
  if (timeNsInTimeDomain >= getLastTimeNs(streamId, timeDomain)) {
    return interface_->getNumData(streamId) - 1;
  }

  // step 2: predict record timestamp from input timestamp and search by record time

  // convert from host/device to record timestamp
  int64_t deltaToRecordTimeNs =
      streamIdToDeltaToRecordTimeNs_.at(streamId).at(static_cast<size_t>(timeDomain));
  int64_t estTimeNsInRecordTime = std::max(timeNsInTimeDomain + deltaToRecordTimeNs, int64_t(0));

  // search
  double estTimeSecInRecordTime = static_cast<double>(estTimeNsInRecordTime) * 1e-9;
  vrs::IndexRecord::RecordInfo queryTime(
      estTimeSecInRecordTime, 0, vrs::StreamId(), vrs::Record::Type::UNDEFINED);
  auto dataRecords = streamIdToDataRecords_.at(streamId);

  auto recordIter = std::upper_bound( // searches for earliest timestamp > query
      dataRecords.begin(),
      dataRecords.end(),
      &queryTime,
      [&](const auto& query, const auto& dataRecord) {
        // Convert both to nanoseconds in integer for comparison, to avoid precision issue when
        // comparing with double. Note that queryTimeNs has undergone int -> double -> int
        // conversion, in theory it should get back the same int64_t number as dataRecordTimeNs.
        int64_t queryTimeNs = static_cast<int64_t>(std::floor(query->timestamp * 1e9));
        int64_t dataRecordTimeNs = static_cast<int64_t>(dataRecord->timestamp * 1e9);

        return queryTimeNs < dataRecordTimeNs;
      });

  if (recordIter == dataRecords.begin()) {
    return 0;
  }
  int indexAtRecordTime = static_cast<int>(std::distance(dataRecords.begin(), recordIter - 1));
  // make sure the returned data is undamaged
  while (!interface_->readRecordByIndex(streamId, indexAtRecordTime)) {
    checkAndThrow(
        indexAtRecordTime >= 0); // since we already checked boundary this shouldn't happen
    indexAtRecordTime--;
  }

  if (timeDomain == TimeDomain::RecordTime) {
    return indexAtRecordTime;
  }

  // step 3: if timedomain is device/host, the predicted record time may not be accurate
  // thus need to reach the actual record nearby to locate the exact timestamp t[i] so that
  // t[i] <= timeNsInTimeDomain < t[i+1]

  // if current record <= timeNsTimeDomain, search forward to find t[i+1]
  // otherwise search backward to find t[i]
  bool searchForward =
      interface_->getLastCachedSensorData(streamId).getTimeNs(timeDomain) <= timeNsInTimeDomain;
  const int increment = searchForward ? 1 : -1;
  bool foundRecord = false;
  while (indexAtRecordTime >= 0 && indexAtRecordTime < dataRecords.size()) {
    if (interface_->readRecordByIndex(streamId, indexAtRecordTime)) { // if damaged data, skip
      int64_t cachedTimeNs = interface_->getLastCachedSensorData(streamId).getTimeNs(timeDomain);
      if ((cachedTimeNs > timeNsInTimeDomain) == searchForward) {
        foundRecord = true;
        break;
      }
    }
    indexAtRecordTime += increment;
  }
  if (!foundRecord) {
    return -1;
  }
  return searchForward ? indexAtRecordTime - 1 : indexAtRecordTime;
}

int64_t TimestampIndexMapper::getTimestampByIndex(
    const vrs::StreamId& streamId,
    const int index,
    const TimeDomain& timeDomain) {
  int64_t timestamp = -1;
  if (index >= 0) {
    // check if timestamp at indexBefore == equal
    if (timeDomain == TimeDomain::RecordTime) {
      timestamp =
          static_cast<int64_t>(streamIdToDataRecords_.at(streamId).at(index)->timestamp * 1e9);
    } else {
      interface_->readRecordByIndex(streamId, index);
      timestamp = interface_->getLastCachedSensorData(streamId).getTimeNs(timeDomain);
    }
  }
  return timestamp;
}

int TimestampIndexMapper::getIndexAfterTimeNsNonTimeCodeFromIndexBefore(
    const vrs::StreamId& streamId,
    const int64_t timeNsInTimeDomain,
    const TimeDomain& timeDomain,
    const int indexBefore) {
  // search forward
  int64_t indexAfter = indexBefore + 1;
  while (indexAfter < interface_->getNumData(streamId) &&
         !interface_->readRecordByIndex(streamId, indexAfter)) {
    indexAfter++;
  }
  return indexAfter >= interface_->getNumData(streamId) ? -1 : indexAfter;
}

int TimestampIndexMapper::getIndexAfterTimeNsNonTimeCode(
    const vrs::StreamId& streamId,
    const int64_t timeNsInTimeDomain,
    const TimeDomain& timeDomain) {
  // get timestamp before
  int indexBefore = getIndexBeforeTimeNsNonTimeCode(streamId, timeNsInTimeDomain, timeDomain);
  // if timestampBefore equals queried timestamp, before/after/closest corresponds to the same
  // index no need the query the next timestamp then
  if (timeNsInTimeDomain == getTimestampByIndex(streamId, indexBefore, timeDomain)) {
    return indexBefore;
  }

  return getIndexAfterTimeNsNonTimeCodeFromIndexBefore(
      streamId, timeNsInTimeDomain, timeDomain, indexBefore);
}

int TimestampIndexMapper::getIndexClosestTimeNsNonTimeCode(
    const vrs::StreamId& streamId,
    const int64_t timeNsInTimeDomain,
    const TimeDomain& timeDomain) {
  // get timestamp before
  int indexBefore = getIndexBeforeTimeNsNonTimeCode(streamId, timeNsInTimeDomain, timeDomain);

  int64_t timestampBefore = getTimestampByIndex(streamId, indexBefore, timeDomain);

  // if timestampBefore equals queried timestamp, before/after/closest corresponds to the same
  // index no need the query the next timestamp then
  if (timeNsInTimeDomain == timestampBefore) {
    return indexBefore;
  }

  // search forward
  int indexAfter = getIndexAfterTimeNsNonTimeCodeFromIndexBefore(
      streamId, timeNsInTimeDomain, timeDomain, indexBefore);

  if (indexAfter >= interface_->getNumData(streamId)) {
    return indexBefore;
  }

  // check if timestamp at indexBefore == equal
  int64_t timestampAfter = getTimestampByIndex(streamId, indexAfter, timeDomain);

  if (indexBefore >= 0 &&
      (timeNsInTimeDomain - timestampBefore <= timestampAfter - timeNsInTimeDomain)) {
    return indexBefore;
  } else {
    return indexAfter;
  }
}

std::vector<int64_t> TimestampIndexMapper::getTimestampsNs(
    const vrs::StreamId& streamId,
    const TimeDomain& timeDomain) {
  int numData = interface_->getNumData(streamId);
  std::vector<int64_t> timestampsNs(numData);
  if (timeDomain ==
      TimeDomain::RecordTime) { // separate recordTime for the fastest timestamp retrieval
    for (int index = 0; index < numData; ++index) {
      timestampsNs.at(index) = streamIdToDataRecords_.at(streamId).at(index)->timestamp * 1e9;
    }
  } else {
    interface_->setReadImageContent(streamId, false);
    for (int index = 0; index < numData; ++index) {
      interface_->readRecordByIndex(streamId, index);
      timestampsNs.at(index) = interface_->getLastCachedSensorData(streamId).getTimeNs(timeDomain);
    }
    interface_->setReadImageContent(streamId, true);
  }
  return timestampsNs;
}

} // namespace projectaria::tools::data_provider
