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

#include <data_layout/BatteryStatusMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

#include <string>

namespace projectaria::tools::data_provider {

/**
 * @brief Battery status sensor configuration type
 */
struct BatteryStatusConfiguration {
  uint32_t streamId{}; ///< @brief ID of the VRS stream
  std::string sensorModel; ///< @brief sensor model name
  uint64_t deviceId{}; ///< @brief device ID for battery status sensor
  double nominalRateHz{}; ///< @brief number of frames per second
  std::string description; ///< @brief description of the battery status sensor
};

/**
 * @brief Battery charging status enum
 */
enum class ChargingStatus {
  Discharging,
  Charging,
  Full,
  Unknown,
};

/**
 * @brief Battery status data type
 */
struct BatteryStatusData {
  // Timestamp of capturing this sample.
  int64_t captureTimestampNs;
  // Battery charge status
  ChargingStatus chargingStatus;
  // Battery level in percentage. Value range [0, 100]
  uint8_t batteryLevel;
  // Temperature in Celsius
  float temperatureC;
  // Current voltage in volts
  float voltageVolt;
  // Average voltage in volts
  float voltageAvgVolt;
  // Current current in amps
  float currentAmp;
  // Average current in amps
  float currentAvgAmp;
  // Current power in watts
  float powerWatt;
  // Average power in watts
  float powerAvgWatt;
};

using BatteryStatusCallback = std::function<
    bool(const BatteryStatusData& data, const BatteryStatusConfiguration& config, bool verbose)>;

class BatteryStatusPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit BatteryStatusPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  BatteryStatusPlayer(const BatteryStatusPlayer&) = delete;
  BatteryStatusPlayer& operator=(const BatteryStatusPlayer&) = delete;
  BatteryStatusPlayer(BatteryStatusPlayer&&) = default;
  BatteryStatusPlayer& operator=(BatteryStatusPlayer&&) = delete;
  ~BatteryStatusPlayer() override = default;

  void setCallback(BatteryStatusCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const BatteryStatusConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const BatteryStatusData& getDataRecord() const {
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
  BatteryStatusCallback callback_ =
      [](const BatteryStatusData&, const BatteryStatusConfiguration&, bool) { return true; };

  BatteryStatusConfiguration configRecord_;
  BatteryStatusData dataRecord_{};

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};
} // namespace projectaria::tools::data_provider
