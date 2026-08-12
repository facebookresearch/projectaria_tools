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

namespace {
/// Linear interpolation between two clock samples, expressed as a ratio so both
/// conversion directions share one rounding behaviour.
int64_t
interpolate(int64_t fromLeft, int64_t fromRight, int64_t toLeft, int64_t toRight, int64_t from) {
  if (fromRight == fromLeft) {
    return toLeft;
  }
  const double ratioRight =
      static_cast<double>(from - fromLeft) / static_cast<double>(fromRight - fromLeft);
  const double ratioLeft = 1 - ratioRight;
  return static_cast<int64_t>(
      ratioLeft * static_cast<double>(toLeft) + ratioRight * static_cast<double>(toRight));
}
} // namespace

TimeSyncMapper::TimeSyncMapper(
    const std::shared_ptr<vrs::MultiRecordFileReader>& reader,
    const std::map<TimeSyncMode, std::shared_ptr<TimeSyncPlayer>>& timesyncPlayers)
    : reader_(reader) {
  const std::scoped_lock lock(mutex_);
  for (const auto& [mode, player] : timesyncPlayers) {
    const vrs::StreamId streamId = player->getStreamId();
    // A stream can legitimately carry zero data records -- the UTC stream ticks
    // once a minute, so a short recording has none. Such a mode is still
    // registered, and reported as supported, to keep the long-standing contract;
    // its conversions return -1.
    const int numRecords = reader->getRecordCount(streamId, vrs::Record::Type::DATA);
    ModeData modeData;
    modeData.player = player;
    modeData.samples.reserve(numRecords);
    modeData.indexTimeNs.reserve(numRecords);
    for (int index = 0; index < numRecords; ++index) {
      const vrs::IndexRecord::RecordInfo* recordInfo =
          reader->getRecord(streamId, vrs::Record::Type::DATA, static_cast<uint32_t>(index));
      if (recordInfo == nullptr) {
        continue;
      }
      modeData.samples.push_back(Sample{.recordInfo = recordInfo});
      modeData.indexTimeNs.push_back(static_cast<int64_t>(recordInfo->timestamp * 1e9));
    }
    modes_.emplace(mode, std::move(modeData));
    timeSyncModes_.push_back(mode);
  }
}

const TimeSyncData* TimeSyncMapper::sampleAt(ModeData& modeData, const size_t index) const {
  if (index >= modeData.samples.size()) {
    return nullptr;
  }
  Sample& sample = modeData.samples[index];
  if (sample.recordInfo == nullptr) {
    return nullptr;
  }
  if (!sample.loaded) {
    const int errorCode = reader_->readRecord(*sample.recordInfo);
    if (errorCode == 0) {
      sample.data = modeData.player->getDataRecord();
    } else {
      XR_LOGE(
          "Fail to read TimeSync record {} from streamId {} with code {}",
          index,
          modeData.player->getStreamId().getNumericName(),
          errorCode);
      sample.readFailed = true;
    }
    sample.loaded = true;
  }
  // A failed read leaves a zeroed sample behind. Handing it out would break the
  // ordering every search here relies on, so conversions fail instead.
  return sample.readFailed ? nullptr : &sample.data;
}

std::optional<size_t> TimeSyncMapper::findBracketByDeviceTime(
    ModeData& modeData,
    const int64_t deviceTimeNs) const {
  const size_t last = modeData.samples.size() - 1;
  // A sequential sweep keeps landing in the bracket it used last, or the next
  // one; check those before paying for a search.
  if (modeData.cursor < last) {
    const TimeSyncData* left = sampleAt(modeData, modeData.cursor);
    const TimeSyncData* right = sampleAt(modeData, modeData.cursor + 1);
    if (left == nullptr || right == nullptr) {
      return std::nullopt;
    }
    if (left->monotonicTimestampNs <= deviceTimeNs && deviceTimeNs <= right->monotonicTimestampNs) {
      return modeData.cursor;
    }
  }
  // The index timestamps track the samples' own monotonic clock closely enough
  // to land on or beside the right bracket, but they are not the same numbers,
  // so the candidate is confirmed against the samples below.
  const auto it = std::ranges::upper_bound(modeData.indexTimeNs, deviceTimeNs);
  auto index = static_cast<size_t>(std::distance(modeData.indexTimeNs.begin(), it));
  index = index == 0 ? 0 : index - 1;
  while (index > 0) {
    const TimeSyncData* sample = sampleAt(modeData, index);
    if (sample == nullptr) {
      return std::nullopt;
    }
    if (sample->monotonicTimestampNs <= deviceTimeNs) {
      break;
    }
    --index;
  }
  while (index < last) {
    const TimeSyncData* next = sampleAt(modeData, index + 1);
    if (next == nullptr) {
      return std::nullopt;
    }
    if (next->monotonicTimestampNs > deviceTimeNs) {
      break;
    }
    ++index;
  }
  modeData.cursor = index;
  return index;
}

std::optional<size_t> TimeSyncMapper::findBracketBySyncTime(
    ModeData& modeData,
    const int64_t syncTimeNs) const {
  const size_t last = modeData.samples.size() - 1;
  if (modeData.cursor < last) {
    const TimeSyncData* left = sampleAt(modeData, modeData.cursor);
    const TimeSyncData* right = sampleAt(modeData, modeData.cursor + 1);
    if (left == nullptr || right == nullptr) {
      return std::nullopt;
    }
    if (left->realTimestampNs <= syncTimeNs && syncTimeNs <= right->realTimestampNs) {
      return modeData.cursor;
    }
  }
  // The record index carries no sync-clock timestamps, so the search reads the
  // samples it probes -- O(log n) records rather than the whole stream.
  size_t low = 0;
  size_t high = last;
  while (low < high) {
    const size_t mid = low + (high - low + 1) / 2;
    const TimeSyncData* sample = sampleAt(modeData, mid);
    if (sample == nullptr) {
      return std::nullopt;
    }
    if (sample->realTimestampNs <= syncTimeNs) {
      low = mid;
    } else {
      high = mid - 1;
    }
  }
  modeData.cursor = low;
  return low;
}

int64_t TimeSyncMapper::convertFromSyncTimeToDeviceTimeNs(
    const int64_t syncTimeNs,
    const TimeSyncMode mode) const {
  if (!supportsMode(mode)) {
    return -1;
  }
  const std::scoped_lock lock(mutex_);
  const auto modeIt = modes_.find(mode);
  if (modeIt == modes_.end() || modeIt->second.samples.empty()) {
    return -1;
  }
  ModeData& modeData = modeIt->second;
  const size_t last = modeData.samples.size() - 1;

  const TimeSyncData* front = sampleAt(modeData, 0);
  const TimeSyncData* back = sampleAt(modeData, last);
  if (front == nullptr || back == nullptr) {
    return -1;
  }
  if (syncTimeNs <= front->realTimestampNs) {
    return front->monotonicTimestampNs - front->realTimestampNs + syncTimeNs;
  }
  if (syncTimeNs >= back->realTimestampNs) {
    return back->monotonicTimestampNs - back->realTimestampNs + syncTimeNs;
  }

  const std::optional<size_t> index = findBracketBySyncTime(modeData, syncTimeNs);
  if (!index.has_value()) {
    return -1;
  }
  const TimeSyncData* left = sampleAt(modeData, *index);
  const TimeSyncData* right = sampleAt(modeData, std::min(*index + 1, last));
  if (left == nullptr || right == nullptr) {
    return -1;
  }
  return interpolate(
      left->realTimestampNs,
      right->realTimestampNs,
      left->monotonicTimestampNs,
      right->monotonicTimestampNs,
      syncTimeNs);
}

int64_t TimeSyncMapper::convertFromDeviceTimeToSyncTimeNs(
    const int64_t deviceTimeNs,
    const TimeSyncMode mode) const {
  if (!supportsMode(mode)) {
    return -1;
  }
  const std::scoped_lock lock(mutex_);
  const auto modeIt = modes_.find(mode);
  if (modeIt == modes_.end() || modeIt->second.samples.empty()) {
    return -1;
  }
  ModeData& modeData = modeIt->second;
  const size_t last = modeData.samples.size() - 1;

  const TimeSyncData* front = sampleAt(modeData, 0);
  const TimeSyncData* back = sampleAt(modeData, last);
  if (front == nullptr || back == nullptr) {
    return -1;
  }
  if (deviceTimeNs <= front->monotonicTimestampNs) {
    return front->realTimestampNs - front->monotonicTimestampNs + deviceTimeNs;
  }
  if (deviceTimeNs >= back->monotonicTimestampNs) {
    return back->realTimestampNs - back->monotonicTimestampNs + deviceTimeNs;
  }

  const std::optional<size_t> index = findBracketByDeviceTime(modeData, deviceTimeNs);
  if (!index.has_value()) {
    return -1;
  }
  const TimeSyncData* left = sampleAt(modeData, *index);
  const TimeSyncData* right = sampleAt(modeData, std::min(*index + 1, last));
  if (left == nullptr || right == nullptr) {
    return -1;
  }
  return interpolate(
      left->monotonicTimestampNs,
      right->monotonicTimestampNs,
      left->realTimestampNs,
      right->realTimestampNs,
      deviceTimeNs);
}

int64_t TimeSyncMapper::convertFromTimeCodeToDeviceTimeNs(const int64_t timecodeTimeNs) const {
  return convertFromSyncTimeToDeviceTimeNs(timecodeTimeNs, TimeSyncMode::TIMECODE);
}

int64_t TimeSyncMapper::convertFromDeviceTimeToTimeCodeNs(const int64_t deviceTimeNs) const {
  return convertFromDeviceTimeToSyncTimeNs(deviceTimeNs, TimeSyncMode::TIMECODE);
}

bool TimeSyncMapper::supportsMode(const TimeSyncMode mode) const {
  return (std::ranges::find(timeSyncModes_, mode) != timeSyncModes_.end()) &&
      (mode == TimeSyncMode::TIMECODE || mode == TimeSyncMode::TIC_SYNC ||
       mode == TimeSyncMode::SUBGHZ || mode == TimeSyncMode::UTC);
}

std::vector<TimeSyncMode> TimeSyncMapper::getTimeSyncModes() const {
  return timeSyncModes_;
}

} // namespace projectaria::tools::data_provider
