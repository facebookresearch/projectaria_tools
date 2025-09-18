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

#include <data_layout/PpgMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

namespace projectaria::tools::data_provider {

/**
 * @brief Ppg sensor configuration type
 */
struct PpgConfiguration {
  uint32_t streamId{}; ///< @brief ID of the VRS stream
  std::string sensorModel; ///< @brief sensor model name
  uint64_t deviceId{}; ///< @brief device ID for ppg sensor
  double nominalRateHz{}; ///< @brief number of frames per second
  std::string description; // TODO: check if this is needed and what is description
};

/**
 * @brief Ppg data type
 */
struct PpgData {
  // Timestamp of capturing this sample.
  int64_t captureTimestampNs{};
  // Raw sensor light exposure measurement, there is no unit for this value.
  int32_t value{};
  // LED current in mA
  float ledCurrentMa{};
  // PPG integration time in us
  float integrationTimeUs{};
};

using PpgCallback =
    std::function<bool(const PpgData& data, const PpgConfiguration& config, bool verbose)>;

class PpgPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit PpgPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  PpgPlayer(const PpgPlayer&) = delete;
  PpgPlayer& operator=(const PpgPlayer&) = delete;
  PpgPlayer& operator=(PpgPlayer&) = delete;
  PpgPlayer(PpgPlayer&&) = default;

  void setCallback(PpgCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const PpgConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const PpgData& getDataRecord() const {
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
  PpgCallback callback_ = [](const PpgData&, const PpgConfiguration&, bool) { return true; };

  PpgConfiguration configRecord_;
  PpgData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};
} // namespace projectaria::tools::data_provider
