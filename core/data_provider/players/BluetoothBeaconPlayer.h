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

#include <data_layout/BluetoothBeaconMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

namespace projectaria::tools::data_provider {

/**
 * @brief Bluetooth sensor configuration type
 */
struct BluetoothBeaconConfigRecord {
  uint32_t streamId; ///< @brief ID of the VRS stream
  double sampleRateHz; ///< @brief number of times the device request data from bluetooth
};

/**
 * @brief Bluetooth sensor data type
 */
struct BluetoothBeaconData {
  int64_t systemTimestampNs; ///< @brief capture time of the data in host domain
  int64_t boardTimestampNs; ///< @brief capture time of the data in device domain
  int64_t boardScanRequestStartTimestampNs; ///< @brief the device time the request starts
  int64_t boardScanRequestCompleteTimestampNs; ///< @brief the device time the request starts
  std::string uniqueId; ///< @brief id of the bluetooth
  float txPower; ///< @brief the range of the bluetooth signal to transmit the beacon
  float rssi; ///< @brief bluetooth data readout in dBm
  float freqMhz; ///< @brief frequency of the data
};

using BluetoothBeaconCallback = std::function<
    bool(const BluetoothBeaconData& data, const BluetoothBeaconConfigRecord& config, bool verbose)>;

class BluetoothBeaconPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit BluetoothBeaconPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  BluetoothBeaconPlayer(const BluetoothBeaconPlayer&) = delete;
  BluetoothBeaconPlayer& operator=(const BluetoothBeaconPlayer&) = delete;
  BluetoothBeaconPlayer& operator=(BluetoothBeaconPlayer&) = delete;
  BluetoothBeaconPlayer(BluetoothBeaconPlayer&&) = default;

  void setCallback(BluetoothBeaconCallback callback) {
    callback_ = callback;
  }

  const BluetoothBeaconConfigRecord& getConfigRecord() const {
    return configRecord_;
  }

  const BluetoothBeaconData& getDataRecord() const {
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
  BluetoothBeaconCallback callback_ =
      [](const BluetoothBeaconData&, const BluetoothBeaconConfigRecord&, bool) { return true; };

  BluetoothBeaconConfigRecord configRecord_;
  BluetoothBeaconData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};

} // namespace projectaria::tools::data_provider
