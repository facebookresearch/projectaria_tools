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

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <vrs/IndexRecord.h>
#include <vrs/MultiRecordFileReader.h>
#include <vrs/StreamId.h>

#include <players/NeuralBandBatchPlayer.h>

namespace projectaria::tools::data_provider {

/// Maps between the wristband clock a Neural Band batch is sampled against and
/// the device clock the rest of the recording lives on.
///
/// The clocks free-run, so the offset has to be estimated from the pair every
/// batch record carries. A single pair is a poor estimate -- reception is
/// bursty, placing it tens of milliseconds out, while the wristband clock steps
/// evenly to within a microsecond -- so the offset is fitted over a window of
/// records. Anchors sit at fixed record positions and a conversion interpolates
/// between the two bracketing it.
///
/// Anchor positions and their inputs depend only on the file, so a timestamp
/// converts identically however it was reached. A causal filter cannot have that
/// property, which is why the streaming side needs a different estimator.
class NeuralBandTimeMapper {
 public:
  NeuralBandTimeMapper() = default;
  /// Reads no record payloads: it takes each stream's record index, already in
  /// memory, and defers every fit until a conversion needs it.
  NeuralBandTimeMapper(
      const std::shared_ptr<vrs::MultiRecordFileReader>& reader,
      const std::map<vrs::StreamId, std::shared_ptr<NeuralBandBatchPlayer>>& neuralBandPlayers);

  [[nodiscard]] bool supportsStream(const vrs::StreamId& streamId) const;

  /// The lock every read of the shared reader has to hold. Owned here because
  /// the mapper reads on demand, after the provider is built.
  [[nodiscard]] const std::shared_ptr<std::mutex>& readerMutex() const {
    return readerMutex_;
  }

  /// Empty when the stream is unknown or its anchors could not be fitted.
  [[nodiscard]] std::optional<int64_t> convertFromWristbandTimeToDeviceTimeNs(
      int64_t wristbandTimeNs,
      const vrs::StreamId& streamId) const;
  [[nodiscard]] std::optional<int64_t> convertFromDeviceTimeToWristbandTimeNs(
      int64_t deviceTimeNs,
      const vrs::StreamId& streamId) const;

  /// Wristband-clock interval between consecutive records, measured from the
  /// recording. One record is one wire packet, so dividing this by a packet's
  /// sub-sample count gives the sampling period.
  [[nodiscard]] std::optional<int64_t> getPacketPeriodNs(const vrs::StreamId& streamId) const;

  /// Wristband-clock interval between consecutive EMG sub-samples, measured from
  /// the recording rather than assumed, so it follows the band's own crystal.
  [[nodiscard]] std::optional<int64_t> getEmgSamplePeriodNs(const vrs::StreamId& streamId) const;

  /// Fill in `deviceTimestampNs` on every sample, left empty where no mapping
  /// could be produced. EMG sub-samples spread forward across their packet: the
  /// band stamps a packet with the instant of its FIRST sub-sample, back-dating
  /// the acquisition counter by one channel sweep to get there.
  void fillDeviceTimestamps(NeuralBandBatch& batch, const vrs::StreamId& streamId) const;

 private:
  /// A point on both clocks, fitted from the records around `centerIndex`.
  struct Anchor {
    size_t centerIndex = 0;
    int64_t wristbandNs = 0;
    int64_t deviceNs = 0;
    bool loaded = false;
    /// Set when too few records around `centerIndex` could be read. `wristbandNs`
    /// and `deviceNs` are then meaningless and must not reach an interpolation.
    bool fitFailed = false;
  };

  struct StreamData {
    vrs::StreamId streamId;
    /// The mapper's own player, not the provider's: a fit reads records the
    /// caller never asked for, and must not overwrite what the caller is about
    /// to collect. Only the timestamps are read, and a data record states its
    /// own channel count, so this player never needs the configuration record.
    std::unique_ptr<NeuralBandBatchPlayer> player;
    bool playerAttached = false;
    size_t recordCount = 0;
    std::vector<Anchor> anchors;
    /// Last bracket used, so a sequential sweep skips the search.
    size_t cursor = 0;
    std::optional<int64_t> packetPeriodNs;
    std::optional<int64_t> samplesPerPacket;
    /// Set once a probe actually read records, so a read stall is retried rather
    /// than remembered as "this stream has no cadence".
    bool cadenceProbed = false;
  };

  /// The affine map between the two clocks over one anchor bracket. Resolved
  /// once per batch: re-deriving it per sample searches the same pair every time.
  struct Mapping {
    int64_t fromLeft = 0;
    int64_t fromRight = 0;
    int64_t toLeft = 0;
    int64_t toRight = 0;
    [[nodiscard]] int64_t apply(int64_t fromNs) const;
  };

  /// The (wristband, device) pair a single record states, or empty if it could
  /// not be read or carries no samples.
  [[nodiscard]] std::optional<std::pair<int64_t, int64_t>> pairAt(
      StreamData& stream,
      size_t recordIndex) const;
  /// Fit (and memoize) the anchor at `anchorIndex`. Null when the fit failed.
  [[nodiscard]] const Anchor* anchorAt(StreamData& stream, size_t anchorIndex) const;
  /// Read the head of the stream once for both the packet period and the
  /// sub-samples a full packet carries. Caller holds `mutex_`.
  void probeEmgCadence(StreamData& stream) const;
  /// Sub-samples a full EMG packet carries, from the first record that has any.
  /// The divisor both the reported period and the fill step from, so a short
  /// final packet cannot stretch its samples. Caller holds `mutex_`.
  [[nodiscard]] std::optional<int64_t> emgSamplesPerPacket(StreamData& stream) const;
  /// Arrival time of a record, from the in-memory index. No payload read.
  /// Empty if the index has no such record. Caller holds `readerMutex_`.
  [[nodiscard]] std::optional<double> recordArrivalSec(const StreamData& stream, size_t recordIndex)
      const;
  /// Last record arriving at or before `deviceTimeNs`. Reads the index only, so
  /// locating a query costs no payload reads and no anchor fits.
  [[nodiscard]] size_t recordIndexAtOrBefore(const StreamData& stream, int64_t deviceTimeNs) const;
  /// Last anchor at or before the query, on whichever clock `onWristbandClock`
  /// selects. Empty if an anchor it needed could not be fitted.
  [[nodiscard]] std::optional<size_t>
  locateAnchor(StreamData& stream, int64_t timeNs, bool onWristbandClock) const;
  /// The bracket around `timeNs`, fitting the anchors it needs. Empty if either
  /// fit failed. Caller holds `mutex_`.
  [[nodiscard]] std::optional<Mapping>
  mappingAt(StreamData& stream, int64_t timeNs, bool fromWristband) const;
  [[nodiscard]] std::optional<int64_t>
  convert(int64_t timeNs, const vrs::StreamId& streamId, bool fromWristband) const;

  std::shared_ptr<vrs::MultiRecordFileReader> reader_;
  std::shared_ptr<std::mutex> readerMutex_ = std::make_shared<std::mutex>();
  /// Guards the anchors and the mapper's own players. Taken before
  /// `readerMutex_` wherever both are needed.
  // NOLINTNEXTLINE(facebook-thread-safety-analysis)
  mutable std::mutex mutex_;
  mutable std::map<vrs::StreamId, StreamData> streams_;
};

} // namespace projectaria::tools::data_provider
