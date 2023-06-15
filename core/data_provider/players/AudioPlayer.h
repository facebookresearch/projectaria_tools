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

#include <data_layout/AudioMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>
#include <Eigen/Core>

namespace projectaria::tools::data_provider {

using AudioCallback =
    std::function<bool(const vrs::CurrentRecord& r, std::vector<int32_t>& data, bool verbose)>;

/**
 * @brief Audio sensor data type: the audio value
 */
struct AudioData {
  std::vector<int32_t> data; ///< @brief raw data, length = nChannels * nSamples
  using ConstMapInt32 =
      Eigen::Map<const Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
  /**
   * @brief Returns a 2D Eigen::Map of the data, each row represents a time sample, each column
   * represents a channel
   */
  ConstMapInt32 reshapeAudioData(const int numSamples, const int numChannels) const {
    return ConstMapInt32(data.data(), numSamples, numChannels);
  }
};

/**
 * @brief Audio sensor configuration type
 */
struct AudioConfig {
  uint32_t streamId; ///< @brief ID of the VRS stream
  uint8_t numChannels; ///< @brief number of microphones used
  uint32_t sampleRate; ///< @brief number of timestamps per second
  uint8_t sampleFormat; ///< @brief format of the Audio data
};

/**
 * @brief Audio meta data
 */
struct AudioDataRecord {
  std::vector<int64_t> captureTimestampsNs; ///< @brief timestamps in device time domain
  uint8_t audioMuted; ///< @brief whether audio is muted
};

class AudioPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit AudioPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  AudioPlayer(const AudioPlayer&) = delete;
  AudioPlayer& operator=(const AudioPlayer&) = delete;
  AudioPlayer& operator=(AudioPlayer&&) = delete;
  AudioPlayer(AudioPlayer&&) = default;

  void setCallback(AudioCallback callback) {
    callback_ = callback;
  }

  const AudioData& getData() const {
    return data_;
  }

  const AudioConfig& getConfigRecord() const {
    return configRecord_;
  }

  const AudioDataRecord& getDataRecord() const {
    return dataRecord_;
  }

  const vrs::StreamId& getStreamId() const {
    return streamId_;
  }

  double getNextTimestampSec() const {
    return nextTimestampSec_;
  }

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

 private:
  bool onDataLayoutRead(const vrs::CurrentRecord& r, size_t blockIndex, vrs::DataLayout& dl)
      override;
  bool onAudioRead(const vrs::CurrentRecord& r, size_t blockIdx, const vrs::ContentBlock& cb)
      override;

  const vrs::StreamId streamId_;
  AudioCallback callback_ = [](const vrs::CurrentRecord&, std::vector<int32_t>&, bool) {
    return true;
  };
  AudioData data_;
  AudioConfig configRecord_;
  AudioDataRecord dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};

} // namespace projectaria::tools::data_provider
