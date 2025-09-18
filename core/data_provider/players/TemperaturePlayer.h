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

#include <data_layout/TemperatureMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

#include <string>

namespace projectaria::tools::data_provider {

/**
 * @brief Temperature sensor configuration type
 */
struct TemperatureConfiguration {
  uint32_t streamId; ///< @brief ID of the VRS stream
  std::string sensorModel; ///< @brief sensor model name
  uint64_t deviceId; ///< @brief device ID for temperature sensor
  double nominalRateHz; ///< @brief number of frames per second
};

/**
 * @brief Temperature data type
 */
struct TemperatureData {
  // Timestamp of capturing this sample.
  int64_t captureTimestampNs;
  // Temperature measurement in Celsius
  float temperatureCelsius;
  // Name of the temperature sensor
  std::string sensorName;
};

using TemperatureCallback = std::function<
    bool(const TemperatureData& data, const TemperatureConfiguration& config, bool verbose)>;

class TemperaturePlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit TemperaturePlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  TemperaturePlayer(const TemperaturePlayer&) = delete;
  TemperaturePlayer& operator=(const TemperaturePlayer&) = delete;
  TemperaturePlayer& operator=(TemperaturePlayer&) = delete;
  TemperaturePlayer(TemperaturePlayer&&) = default;

  void setCallback(TemperatureCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const TemperatureConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const TemperatureData& getDataRecord() const {
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
  TemperatureCallback callback_ =
      [](const TemperatureData&, const TemperatureConfiguration&, bool) { return true; };

  TemperatureConfiguration configRecord_;
  TemperatureData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};
} // namespace projectaria::tools::data_provider
