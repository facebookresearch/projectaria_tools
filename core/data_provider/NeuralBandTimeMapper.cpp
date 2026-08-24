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

#include <data_provider/NeuralBandTimeMapper.h>

#define DEFAULT_LOG_CHANNEL "NeuralBandTimeMapper"
#include <logging/Log.h>

#include <algorithm>

namespace projectaria::tools::data_provider {

namespace {

// Anchor geometry, in records, at the stream's nominal 128 per second. Sized by
// measurement: under a ~2 s window averaging barely helps, because the reception
// stalls dominating the noise are themselves seconds long, and past a few
// seconds the window smooths over real drift. Decimating to every 16th record
// costs nothing measurable at 128 Hz; every 32nd degrades.
constexpr size_t kAnchorSpacingRecords = 512; // ~4 s
constexpr size_t kAnchorHalfWindowRecords = 512; // ~8 s total
constexpr size_t kAnchorDecimation = 16;
// Below this an anchor is noise, not an estimate.
constexpr size_t kMinSamplesPerAnchor = 8;
// The wristband clock steps evenly, so a short run measures the period.
constexpr size_t kPacketPeriodProbeRecords = 64;

std::optional<int64_t> medianOf(std::vector<int64_t>& values) {
  if (values.empty()) {
    return std::nullopt;
  }
  const size_t mid = values.size() / 2;
  std::ranges::nth_element(values, values.begin() + mid);
  return values[mid];
}

int64_t
interpolate(int64_t fromLeft, int64_t fromRight, int64_t toLeft, int64_t toRight, int64_t from) {
  if (fromRight == fromLeft) {
    return toLeft;
  }
  const double ratioRight =
      static_cast<double>(from - fromLeft) / static_cast<double>(fromRight - fromLeft);
  return static_cast<int64_t>(
      (1 - ratioRight) * static_cast<double>(toLeft) + ratioRight * static_cast<double>(toRight));
}

} // namespace

NeuralBandTimeMapper::NeuralBandTimeMapper(
    const std::shared_ptr<vrs::MultiRecordFileReader>& reader,
    const std::map<vrs::StreamId, std::shared_ptr<NeuralBandBatchPlayer>>& neuralBandPlayers)
    : reader_(reader) {
  const std::scoped_lock lock(mutex_);
  for (const auto& [streamId, providerPlayer] : neuralBandPlayers) {
    StreamData stream;
    stream.streamId = streamId;
    stream.player = std::make_unique<NeuralBandBatchPlayer>(streamId);
    stream.recordCount = reader->getRecordCount(streamId, vrs::Record::Type::DATA);
    stream.anchors.reserve(stream.recordCount / kAnchorSpacingRecords + 2);
    for (size_t center = 0; center < stream.recordCount; center += kAnchorSpacingRecords) {
      stream.anchors.push_back(Anchor{.centerIndex = center});
    }
    // Without this everything past the last spaced anchor would sit outside the
    // anchored span.
    if (stream.recordCount > 0 && stream.anchors.back().centerIndex != stream.recordCount - 1) {
      stream.anchors.push_back(Anchor{.centerIndex = stream.recordCount - 1});
    }
    streams_.emplace(streamId, std::move(stream));
  }
}

bool NeuralBandTimeMapper::supportsStream(const vrs::StreamId& streamId) const {
  return streams_.contains(streamId);
}

std::optional<std::pair<int64_t, int64_t>> NeuralBandTimeMapper::pairAt(
    StreamData& stream,
    const size_t recordIndex) const {
  {
    const std::scoped_lock readerLock(*readerMutex_);
    const vrs::IndexRecord::RecordInfo* record = reader_->getRecord(
        stream.streamId, vrs::Record::Type::DATA, static_cast<uint32_t>(recordIndex));
    if (record == nullptr ||
        reader_->readRecord(*record, stream.player.get(), !stream.playerAttached) != 0) {
      return std::nullopt;
    }
    stream.playerAttached = true;
  }
  const NeuralBandBatch& batch = stream.player->getDataRecord();
  // EMG is the sub-stream always present when the band is sending. Accel and
  // gyro run a few hundred microseconds ahead of it, which matters to a caller
  // but not to an offset estimate.
  if (!batch.emg.empty()) {
    return std::make_pair(batch.emg.front().wristbandTimestampNs, batch.arrivalTimestampNs);
  }
  if (!batch.accel.empty()) {
    return std::make_pair(batch.accel.front().wristbandTimestampNs, batch.arrivalTimestampNs);
  }
  return std::nullopt;
}

const NeuralBandTimeMapper::Anchor* NeuralBandTimeMapper::anchorAt(
    StreamData& stream,
    const size_t anchorIndex) const {
  if (anchorIndex >= stream.anchors.size()) {
    return nullptr;
  }
  Anchor& anchor = stream.anchors[anchorIndex];
  if (anchor.loaded) {
    return anchor.fitFailed ? nullptr : &anchor;
  }
  anchor.loaded = true;

  const std::optional<std::pair<int64_t, int64_t>> center = pairAt(stream, anchor.centerIndex);
  if (!center.has_value()) {
    anchor.fitFailed = true;
    XR_LOGE("Neural Band anchor at record {} could not be read", anchor.centerIndex);
    return nullptr;
  }

  const size_t low = anchor.centerIndex > kAnchorHalfWindowRecords
      ? anchor.centerIndex - kAnchorHalfWindowRecords
      : 0;
  const size_t high =
      std::min(anchor.centerIndex + kAnchorHalfWindowRecords, stream.recordCount - 1);
  std::vector<int64_t> offsets;
  offsets.reserve((high - low) / kAnchorDecimation + 1);
  for (size_t index = low; index <= high; index += kAnchorDecimation) {
    if (const auto pair = pairAt(stream, index)) {
      offsets.push_back(pair->second - pair->first);
    }
  }
  if (offsets.size() < kMinSamplesPerAnchor) {
    anchor.fitFailed = true;
    XR_LOGE(
        "Neural Band anchor at record {} fitted from {} records, need {}",
        anchor.centerIndex,
        offsets.size(),
        kMinSamplesPerAnchor);
    return nullptr;
  }
  const std::optional<int64_t> offsetNs = medianOf(offsets);
  if (!offsetNs.has_value()) {
    anchor.fitFailed = true;
    return nullptr;
  }
  anchor.wristbandNs = center->first;
  anchor.deviceNs = center->first + *offsetNs;
  return &anchor;
}

std::optional<double> NeuralBandTimeMapper::recordArrivalSec(
    const StreamData& stream,
    const size_t recordIndex) const {
  const vrs::IndexRecord::RecordInfo* record = reader_->getRecord(
      stream.streamId, vrs::Record::Type::DATA, static_cast<uint32_t>(recordIndex));
  if (record == nullptr) {
    return std::nullopt;
  }
  return record->timestamp;
}

size_t NeuralBandTimeMapper::recordIndexAtOrBefore(
    const StreamData& stream,
    const int64_t deviceTimeNs) const {
  const auto deviceTimeSec = static_cast<double>(deviceTimeNs) * 1e-9;
  // Held across the whole search rather than per probe: `recordArrivalSec` goes
  // to the shared reader, and an anchor fit cannot run underneath this.
  const std::scoped_lock readerLock(*readerMutex_);
  size_t low = 0;
  size_t high = stream.recordCount;
  while (low < high) {
    const size_t mid = low + (high - low) / 2;
    // A record the index cannot produce counts as after the query, never before:
    // this returns the last record at or before it, so erring early keeps that.
    const std::optional<double> arrivalSec = recordArrivalSec(stream, mid);
    if (arrivalSec.has_value() && *arrivalSec <= deviceTimeSec) {
      low = mid + 1;
    } else {
      high = mid;
    }
  }
  return low == 0 ? 0 : low - 1;
}

std::optional<size_t> NeuralBandTimeMapper::locateAnchor(
    StreamData& stream,
    const int64_t timeNs,
    const bool onWristbandClock) const {
  const size_t last = stream.anchors.size() - 1;
  const auto keyOf = [onWristbandClock](const Anchor& a) {
    return onWristbandClock ? a.wristbandNs : a.deviceNs;
  };

  // One fitted anchor puts a wristband time close enough to find its
  // neighbourhood in the record index: the seed's offset is stale by the drift
  // across the file at worst, a small number of anchor spacings.
  const Anchor* seed = anchorAt(stream, std::min(stream.cursor, last));
  if (seed == nullptr) {
    return std::nullopt;
  }
  const int64_t approxDeviceNs =
      onWristbandClock ? timeNs + (seed->deviceNs - seed->wristbandNs) : timeNs;
  const size_t recordIndex = recordIndexAtOrBefore(stream, approxDeviceNs);
  size_t index = std::min(recordIndex / kAnchorSpacingRecords, last);

  // Correct the guess against the anchors themselves; a step or two in practice.
  while (index > 0) {
    const Anchor* anchor = anchorAt(stream, index);
    if (anchor == nullptr) {
      return std::nullopt;
    }
    if (keyOf(*anchor) <= timeNs) {
      break;
    }
    --index;
  }
  while (index < last) {
    const Anchor* next = anchorAt(stream, index + 1);
    if (next == nullptr) {
      return std::nullopt;
    }
    if (keyOf(*next) > timeNs) {
      break;
    }
    ++index;
  }
  stream.cursor = index;
  return index;
}

int64_t NeuralBandTimeMapper::Mapping::apply(const int64_t fromNs) const {
  // Held at the nearest estimate rather than extrapolated: the drift it would
  // extrapolate along is smaller than the noise on the estimate itself.
  if (fromNs <= fromLeft) {
    return toLeft - fromLeft + fromNs;
  }
  if (fromNs >= fromRight) {
    return toRight - fromRight + fromNs;
  }
  return interpolate(fromLeft, fromRight, toLeft, toRight, fromNs);
}

std::optional<NeuralBandTimeMapper::Mapping> NeuralBandTimeMapper::mappingAt(
    StreamData& stream,
    const int64_t timeNs,
    const bool fromWristband) const {
  if (stream.anchors.empty()) {
    return std::nullopt;
  }
  const std::optional<size_t> index = locateAnchor(stream, timeNs, fromWristband);
  if (!index.has_value()) {
    return std::nullopt;
  }
  const Anchor* left = anchorAt(stream, *index);
  const Anchor* right = anchorAt(stream, std::min(*index + 1, stream.anchors.size() - 1));
  if (left == nullptr || right == nullptr) {
    return std::nullopt;
  }
  return fromWristband
      ? Mapping{
            .fromLeft = left->wristbandNs,
            .fromRight = right->wristbandNs,
            .toLeft = left->deviceNs,
            .toRight = right->deviceNs}
      : Mapping{
            .fromLeft = left->deviceNs,
            .fromRight = right->deviceNs,
            .toLeft = left->wristbandNs,
            .toRight = right->wristbandNs};
}

std::optional<int64_t> NeuralBandTimeMapper::convert(
    const int64_t timeNs,
    const vrs::StreamId& streamId,
    const bool fromWristband) const {
  const std::scoped_lock lock(mutex_);
  const auto it = streams_.find(streamId);
  if (it == streams_.end()) {
    return std::nullopt;
  }
  const std::optional<Mapping> mapping = mappingAt(it->second, timeNs, fromWristband);
  if (!mapping.has_value()) {
    return std::nullopt;
  }
  return mapping->apply(timeNs);
}

std::optional<int64_t> NeuralBandTimeMapper::convertFromWristbandTimeToDeviceTimeNs(
    const int64_t wristbandTimeNs,
    const vrs::StreamId& streamId) const {
  return convert(wristbandTimeNs, streamId, /*fromWristband=*/true);
}

std::optional<int64_t> NeuralBandTimeMapper::convertFromDeviceTimeToWristbandTimeNs(
    const int64_t deviceTimeNs,
    const vrs::StreamId& streamId) const {
  return convert(deviceTimeNs, streamId, /*fromWristband=*/false);
}

void NeuralBandTimeMapper::probeEmgCadence(StreamData& stream) const {
  if (stream.cadenceProbed) {
    return;
  }
  std::vector<int64_t> wristbandNs;
  const size_t probe = std::min(kPacketPeriodProbeRecords, stream.recordCount);
  wristbandNs.reserve(probe);
  size_t readable = 0;
  for (size_t index = 0; index < probe; ++index) {
    if (!pairAt(stream, index).has_value()) {
      continue;
    }
    ++readable;
    // EMG only. Accel and gyro run on their own trigger, a few hundred
    // microseconds off the EMG, which is nothing against the arrival jitter an
    // offset is fitted through but is percent of a period.
    const NeuralBandBatch& batch = stream.player->getDataRecord();
    if (batch.emg.empty()) {
      continue;
    }
    wristbandNs.push_back(batch.emg.front().wristbandTimestampNs);
    if (!stream.samplesPerPacket.has_value()) {
      // A packet's sub-samples all state its timestamp, so the leading run of
      // equal timestamps is one packet.
      int64_t count = 1;
      while (static_cast<size_t>(count) < batch.emg.size() &&
             batch.emg[count].wristbandTimestampNs == batch.emg.front().wristbandTimestampNs) {
        ++count;
      }
      stream.samplesPerPacket = count;
    }
  }
  // Only a probe that got to read the records it asked for has decided anything.
  // Memoizing a read stall would disable EMG spacing for the rest of the session.
  stream.cadenceProbed = readable >= std::min<size_t>(2, probe);
  if (wristbandNs.size() < 2) {
    return;
  }
  std::vector<int64_t> deltas;
  deltas.reserve(wristbandNs.size() - 1);
  for (size_t i = 1; i < wristbandNs.size(); ++i) {
    deltas.push_back(wristbandNs[i] - wristbandNs[i - 1]);
  }
  // A median first, not a mean: a dropped packet leaves a gap that is a real
  // multiple of the period, and averaging it in would inflate every sub-sample
  // offset. It only resolves whole microseconds though -- the wire timestamp has
  // microsecond resolution, so the true period lands between two quantised
  // spacings and no median of them can reach it. Averaging the spacings the
  // median just proved to be drop-free divides that quantisation by their count.
  const std::optional<int64_t> median = medianOf(deltas);
  if (!median.has_value() || *median <= 0) {
    return;
  }
  const int64_t medianNs = *median;
  const int64_t dropThresholdNs = medianNs + medianNs / 2;
  int64_t sumNs = 0;
  int64_t count = 0;
  for (const int64_t delta : deltas) {
    if (delta > 0 && delta < dropThresholdNs) {
      sumNs += delta;
      ++count;
    }
  }
  stream.packetPeriodNs = count > 0 ? (sumNs + count / 2) / count : medianNs;
}

std::optional<int64_t> NeuralBandTimeMapper::getPacketPeriodNs(
    const vrs::StreamId& streamId) const {
  const std::scoped_lock lock(mutex_);
  const auto it = streams_.find(streamId);
  if (it == streams_.end()) {
    return std::nullopt;
  }
  probeEmgCadence(it->second);
  return it->second.packetPeriodNs;
}

std::optional<int64_t> NeuralBandTimeMapper::getEmgSamplePeriodNs(
    const vrs::StreamId& streamId) const {
  const std::optional<int64_t> packetPeriodNs = getPacketPeriodNs(streamId);
  if (!packetPeriodNs.has_value()) {
    return std::nullopt;
  }
  const std::scoped_lock lock(mutex_);
  const auto it = streams_.find(streamId);
  if (it == streams_.end()) {
    return std::nullopt;
  }
  const std::optional<int64_t> samplesPerPacket = emgSamplesPerPacket(it->second);
  if (!samplesPerPacket.has_value()) {
    return std::nullopt;
  }
  return *packetPeriodNs / *samplesPerPacket;
}

std::optional<int64_t> NeuralBandTimeMapper::emgSamplesPerPacket(StreamData& stream) const {
  probeEmgCadence(stream);
  return stream.samplesPerPacket;
}

void NeuralBandTimeMapper::fillDeviceTimestamps(
    NeuralBandBatch& batch,
    const vrs::StreamId& streamId) const {
  // Taken before the lock: it takes the same one.
  const std::optional<int64_t> packetPeriodNs = getPacketPeriodNs(streamId);
  const std::scoped_lock lock(mutex_);
  const auto it = streams_.find(streamId);
  if (it == streams_.end()) {
    return;
  }
  StreamData& stream = it->second;
  // One bracket for the whole batch. A batch spans milliseconds and the anchors
  // seconds, so resolving it per sample would search the same pair every time.
  int64_t batchNs = 0;
  if (!batch.emg.empty()) {
    batchNs = batch.emg.front().wristbandTimestampNs;
  } else if (!batch.accel.empty()) {
    batchNs = batch.accel.front().wristbandTimestampNs;
  } else if (!batch.gyro.empty()) {
    batchNs = batch.gyro.front().wristbandTimestampNs;
  } else {
    return;
  }
  const std::optional<Mapping> mapping = mappingAt(stream, batchNs, /*fromWristband=*/true);
  if (!mapping.has_value()) {
    return;
  }
  for (auto& sample : batch.accel) {
    sample.deviceTimestampNs = mapping->apply(sample.wristbandTimestampNs);
  }
  for (auto& sample : batch.gyro) {
    sample.deviceTimestampNs = mapping->apply(sample.wristbandTimestampNs);
  }
  const std::optional<int64_t> samplesPerPacket = emgSamplesPerPacket(stream);
  if (batch.emg.empty() || !packetPeriodNs.has_value() || !samplesPerPacket.has_value()) {
    return;
  }
  size_t runStart = 0;
  while (runStart < batch.emg.size()) {
    size_t runEnd = runStart;
    while (runEnd + 1 < batch.emg.size() &&
           batch.emg[runEnd + 1].wristbandTimestampNs == batch.emg[runStart].wristbandTimestampNs) {
      ++runEnd;
    }
    for (size_t k = runStart; k <= runEnd; ++k) {
      // The wire timestamp is the packet's first sub-sample, so the rest step
      // forward from it, at the rate a full packet implies -- a short final
      // packet is fewer samples, not slower ones. Scaling before dividing keeps
      // the truncation from accumulating across the run.
      //
      // Capped at one packet: a run longer than a packet means two packets
      // stated the same wire timestamp, which the format does not allow. Holding
      // the tail on the packet's last sub-sample keeps the batch ordered, where
      // stepping past it would overtake the next packet.
      const int64_t step = std::min(static_cast<int64_t>(k - runStart), *samplesPerPacket - 1);
      const int64_t wristbandNs =
          batch.emg[k].wristbandTimestampNs + step * *packetPeriodNs / *samplesPerPacket;
      batch.emg[k].deviceTimestampNs = mapping->apply(wristbandNs);
    }
    runStart = runEnd + 1;
  }
}

} // namespace projectaria::tools::data_provider
