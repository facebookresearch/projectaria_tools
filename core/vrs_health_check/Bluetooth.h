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

#include <data_provider/players/BluetoothBeaconPlayer.h>

namespace projectaria::tools::vrs_check {

struct BluetoothStats : public Stats {
  uint64_t uniqueId = 0; // Number of unique identifiers in this stream
  uint64_t outOfOrder = 0; // Beacons that are out of order
  std::map<float, uint64_t> totalPerFreq; // Per frequency stats
};

class Bluetooth : public Stream {
 public:
  explicit Bluetooth(vrs::StreamId streamId);
  bool setup(vrs::RecordFileReader& reader) override; // Setup the bluetooth player
  Stats getStats() override;
  void logStats() override;
  bool getResult() override; // Pass or fail for this stream
  uint32_t getPeriodUs() override {
    return static_cast<uint32_t>(-1); // No concept of a period for Bluetooth stream
  }

 private:
  static constexpr char kNilUuid[] = "00000000-0000-0000-0000-000000000000";
  void processData(const data_provider::BluetoothBeaconData& data);
  std::unique_ptr<data_provider::BluetoothBeaconPlayer> bluetoothPlayer_;
  BluetoothStats stats_;
  std::set<std::string> uniqueId_;
  uint64_t prevTimestampUs_ = 0;
};

} // namespace projectaria::tools::vrs_check
