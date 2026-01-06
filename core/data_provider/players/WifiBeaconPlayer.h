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

#include <data_layout/WifiBeaconMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

#include <utility>

namespace projectaria::tools::data_provider {

/**
 * @brief Wi-Fi beacon sensor configurations
 */
struct WifiBeaconConfigRecord {
  uint32_t streamId; ///< @brief ID of the VRS stream
};

/**
 * @brief Wi-Fi beacon data
 */
struct WifiBeaconData {
  int64_t systemTimestampNs; ///< @brief capture time of the data in host domain
  int64_t boardTimestampNs; ///< @brief capture time of the data in device domain
  int64_t boardScanRequestStartTimestampNs; ///< @brief the device time the request starts
  int64_t boardScanRequestCompleteTimestampNs; ///< @brief the device time the request starts
  std::string ssid; ///< @brief id of the Wi-Fi beacon
  std::string bssidMac; ///< @brief mac id of the Wi-Fi beacon
  float rssi; ///< @brief sensor readout in dBm
  float freqMhz; ///< @brief frequency of the data
  std::vector<float> rssiPerAntenna;
};

using WifiBeaconCallback = std::function<
    bool(const WifiBeaconData& data, const WifiBeaconConfigRecord& config, bool verbose)>;

class WifiBeaconPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit WifiBeaconPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  WifiBeaconPlayer(const WifiBeaconPlayer&) = delete;
  WifiBeaconPlayer& operator=(const WifiBeaconPlayer&) = delete;
  WifiBeaconPlayer& operator=(WifiBeaconPlayer&) = delete;
  WifiBeaconPlayer(WifiBeaconPlayer&&) = default;

  void setCallback(WifiBeaconCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const WifiBeaconConfigRecord& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const WifiBeaconData& getDataRecord() const {
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
  WifiBeaconCallback callback_ = [](const WifiBeaconData&, const WifiBeaconConfigRecord&, bool) {
    return true;
  };

  WifiBeaconConfigRecord configRecord_;
  WifiBeaconData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};

} // namespace projectaria::tools::data_provider
