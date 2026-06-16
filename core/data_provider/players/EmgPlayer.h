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
#include <functional>
#include <string>
#include <vector>

#include <data_layout/EmgMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

namespace projectaria::tools::data_provider {

/**
 * @brief Emg sensor configuration type
 */
struct EmgConfiguration {
  uint32_t streamId{}; ///< @brief recorder's internal sensor stream index (not a vrs::StreamId)
  std::string sensorModel; ///< @brief sensor model name
  uint64_t deviceId{}; ///< @brief device ID for emg sensor
  double nominalRateHz{}; ///< @brief number of frames per second
  std::string description; ///< @brief description of the EMG sensor
};

/**
 * @brief A single EMG / IMU sample within an EMG IMU batch
 */
struct EmgImuSample {
  uint32_t sequenceNumber{}; ///< @brief monotonic sequence number of the sample
  int64_t timestampNs{}; ///< @brief capture timestamp of the sample in nanoseconds
  /// @brief packed binary blob of per-channel ADC readings for this sample; unpack using
  /// EmgData::channelCount and EmgData::bitsPerAdcReading
  std::string packedChannelData;
  uint32_t encoding{}; ///< @brief encoding of the packed channel data (EMG only)
};

/**
 * @brief Emg data type holding a batch of EMG, accelerometer and gyroscope samples.
 *
 * Note: channelCount, bitsPerAdcReading and samplesPerBatch describe the EMG sub-stream only; they
 * do not apply to the accelerometer or gyroscope samples.
 */
struct EmgData {
  int64_t captureTimestampNs{}; ///< @brief timestamp of capturing this batch
  uint32_t batchSequenceNumber{}; ///< @brief monotonic sequence number of the batch
  std::vector<EmgImuSample> emg; ///< @brief EMG samples in this batch
  std::vector<EmgImuSample> accel; ///< @brief accelerometer samples in this batch
  std::vector<EmgImuSample> gyro; ///< @brief gyroscope samples in this batch
  uint32_t channelCount{}; ///< @brief number of EMG channels
  uint32_t bitsPerAdcReading{}; ///< @brief number of bits per ADC reading
  uint32_t samplesPerBatch{}; ///< @brief number of samples per batch
};

using EmgCallback =
    std::function<bool(const EmgData& data, const EmgConfiguration& config, bool verbose)>;

class EmgPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit EmgPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  EmgPlayer(const EmgPlayer&) = delete;
  EmgPlayer& operator=(const EmgPlayer&) = delete;
  EmgPlayer(EmgPlayer&&) = default;
  // streamId_ is const, so a move assignment cannot be synthesized; the player is held by
  // shared_ptr and never assigned, so it is explicitly deleted.
  EmgPlayer& operator=(EmgPlayer&&) = delete;

  void setCallback(EmgCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const EmgConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const EmgData& getDataRecord() const {
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
  EmgCallback callback_ = [](const EmgData&, const EmgConfiguration&, bool) { return true; };

  EmgConfiguration configRecord_;
  EmgData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};
} // namespace projectaria::tools::data_provider
