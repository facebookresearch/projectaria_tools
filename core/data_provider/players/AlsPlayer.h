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

#include <data_layout/AlsMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>
#include <cstdint>
#include <functional>
#include <string>

namespace projectaria::tools::data_provider {

/**
 * @brief ALS (Ambient Light Sensor) configuration type
 */
struct AlsConfiguration {
  uint32_t streamId; ///< @brief ID of the VRS stream
  uint64_t deviceId; ///< @brief device ID for ALS sensor
  double nominalRateHz; ///< @brief number of frames per second
  std::string sensorModel; ///< @brief sensor model name
};

/**
 * @brief ALS data type
 */
struct AlsData {
  // Timestamp of capturing this sample.
  int64_t captureTimestampNs;
  // Normalized channel values
  float redChannelNormalized;
  float greenChannelNormalized;
  float blueChannelNormalized;
  float uvChannelNormalized;
  float irChannelNormalized;
  float clearChannelNormalized;
  // Flux measurements in watts per square meter
  float uvFluxWattPerSquareMeter;
  float irFluxWattPerSquareMeter;
  float clearFluxWattPerSquareMeter;
  // Gain values for each channel
  int32_t gainRed;
  int32_t gainGreen;
  int32_t gainBlue;
  int32_t gainUv;
  int32_t gainIr;
  int32_t gainClear;
  // Exposure time in microseconds
  int32_t exposureTimeUs;
  // Correlated color temperature
  float cct;
  // Illuminance in lux
  float lux;
};

using AlsCallback =
    std::function<bool(const AlsData& data, const AlsConfiguration& config, bool verbose)>;

class AlsPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit AlsPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  AlsPlayer(const AlsPlayer&) = delete;
  AlsPlayer& operator=(const AlsPlayer&) = delete;
  AlsPlayer& operator=(AlsPlayer&) = delete;
  AlsPlayer(AlsPlayer&&) = default;

  void setCallback(AlsCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const AlsConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const AlsData& getDataRecord() const {
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
  AlsCallback callback_ = [](const AlsData&, const AlsConfiguration&, bool) { return true; };

  AlsConfiguration configRecord_;
  AlsData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};
} // namespace projectaria::tools::data_provider
