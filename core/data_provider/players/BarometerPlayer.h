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

#include <data_layout/BarometerMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

namespace projectaria::tools::data_provider {

/**
 * @brief Barometer sensor configuration type
 */
struct BarometerConfigRecord {
  uint32_t streamId; ///< @brief ID of the VRS stream
  std::string sensorModelName; ///< @brief sensor model
  double sampleRate; ///< @brief number of samples per second
};

/**
 * @brief Barometer data type
 */
struct BarometerData {
  int64_t captureTimestampNs; ///< @brief the timestamp when the data is captured
  double temperature; ///< @brief temperature of the sensor in degrees Celsius
  double pressure; ///< @brief raw sensor readout of pressure in Pascal
};

using BarometerCallback = std::function<
    bool(const BarometerData& data, const BarometerConfigRecord& config, bool verbose)>;

class BarometerPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit BarometerPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  BarometerPlayer(const BarometerPlayer&) = delete;
  BarometerPlayer& operator=(const BarometerPlayer&) = delete;
  BarometerPlayer& operator=(BarometerPlayer&) = delete;
  BarometerPlayer(BarometerPlayer&&) = default;

  void setCallback(BarometerCallback callback) {
    callback_ = callback;
  }

  const BarometerConfigRecord& getConfigRecord() const {
    return configRecord_;
  }

  const BarometerData& getDataRecord() const {
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

  const vrs::StreamId streamId_;
  BarometerCallback callback_ = [](const BarometerData&, const BarometerConfigRecord&, bool) {
    return true;
  };

  BarometerConfigRecord configRecord_;
  BarometerData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};

} // namespace projectaria::tools::data_provider
