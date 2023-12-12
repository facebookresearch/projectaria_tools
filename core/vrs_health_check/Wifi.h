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

#include "Stream.h"

#include <data_provider/players/WifiBeaconPlayer.h>

namespace projectaria::tools::vrs_check {

struct WifiStats : public Stats {
  uint64_t ssid = 0; // Number of unique SSIDs in this stream
  uint64_t bssid = 0; // Number of unique BSSIDs (MAC addresses) in this stream
  uint64_t nomap = 0; // Beacons with _nomap SSID, should be zero
  uint64_t outOfOrder = 0; // Beacons that are out of order
  std::map<float, uint64_t> totalPerFreq; // Per frequency stats
};

class Wifi : public Stream {
 public:
  explicit Wifi(vrs::StreamId streamId);
  bool setup(vrs::RecordFileReader& reader) override; // Setup the Wi-Fi player
  Stats getStats() override;
  void logStats() override;
  bool getResult() override; // Pass or fail for this stream
  uint32_t getPeriodUs() override {
    return static_cast<uint32_t>(-1); // No concept of a period for Wifi stream
  }

 private:
  void processData(const data_provider::WifiBeaconData& data);
  // Minimum number of channels required to report beacons
  static constexpr int kMinChannels = 3;
  std::unique_ptr<data_provider::WifiBeaconPlayer> wifiBeaconPlayer_;
  WifiStats stats_;
  // Set of unique BSSIDs (MAC addresses) collected in this stream
  std::set<std::string> bssid_;
  // Set of unique SSIDs collected in this stream
  std::set<std::string> ssid_;
  uint64_t prevTimestampUs_ = 0;
};

} // namespace projectaria::tools::vrs_check
