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

#include <data_layout/AlsFlickerDetectionMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>
#include <array>
#include <cstdint>
#include <functional>
#include <string>

namespace projectaria::tools::data_provider {

// Number of frequency/magnitude pairs reported per flicker detection sample.
constexpr size_t kAlsFlickerDetectionFreqCount = 3;

/**
 * @brief ALS light flicker detection configuration type
 */
struct AlsFlickerDetectionConfiguration {
  uint32_t streamId{}; ///< ID of the VRS stream
  uint64_t deviceId{}; ///< device ID for the ALS sensor
  double nominalRateHz{}; ///< number of samples per second
  std::string sensorModel; ///< sensor model name
};

/**
 * @brief A single light flicker frequency/magnitude pair
 */
struct AlsFlickerFrequencyData {
  float frequencyHz{}; ///< detected flicker frequency in Hz
  float magnitude{}; ///< magnitude of the detected flicker
};

/**
 * @brief ALS light flicker detection data type
 */
struct AlsFlickerDetectionData {
  // Timestamp of capturing this sample.
  int64_t captureTimestampNs{};
  // Top detected flicker frequency/magnitude pairs.
  std::array<AlsFlickerFrequencyData, kAlsFlickerDetectionFreqCount> flickerData{};
};

using AlsFlickerDetectionCallback = std::function<bool(
    const AlsFlickerDetectionData& data,
    const AlsFlickerDetectionConfiguration& config,
    bool verbose)>;

class AlsFlickerDetectionPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit AlsFlickerDetectionPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  AlsFlickerDetectionPlayer(const AlsFlickerDetectionPlayer&) = delete;
  AlsFlickerDetectionPlayer& operator=(const AlsFlickerDetectionPlayer&) = delete;
  AlsFlickerDetectionPlayer(AlsFlickerDetectionPlayer&&) = default;
  AlsFlickerDetectionPlayer& operator=(AlsFlickerDetectionPlayer&&) = delete;
  ~AlsFlickerDetectionPlayer() override = default;

  void setCallback(AlsFlickerDetectionCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const AlsFlickerDetectionConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const AlsFlickerDetectionData& getDataRecord() const {
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
  AlsFlickerDetectionCallback callback_ = [](const AlsFlickerDetectionData&,
                                             const AlsFlickerDetectionConfiguration&,
                                             bool) { return true; };

  AlsFlickerDetectionConfiguration configRecord_;
  AlsFlickerDetectionData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};
} // namespace projectaria::tools::data_provider
