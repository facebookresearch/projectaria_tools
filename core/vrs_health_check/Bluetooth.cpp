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

#include "Bluetooth.h"

#include <iomanip>
#include <sstream>

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:Bluetooth"
#include <logging/Log.h>

namespace projectaria::tools::vrs_check {

Bluetooth::Bluetooth(vrs::StreamId streamId) : Stream(streamId) {}

bool Bluetooth::setup(vrs::RecordFileReader& reader) {
  data_provider::BluetoothBeaconCallback callback =
      [&](const data_provider::BluetoothBeaconData& data,
          const data_provider::BluetoothBeaconConfigRecord&,
          bool) {
        processData(data);
        return true;
      };

  bluetoothPlayer_ = std::make_unique<data_provider::BluetoothBeaconPlayer>(streamId_);
  if (!bluetoothPlayer_) {
    XR_LOGE("Cannot create playable for {}", streamId_.getName());
    return false;
  }
  bluetoothPlayer_->setCallback(callback);
  reader.setStreamPlayer(streamId_, bluetoothPlayer_.get());
  std::unique_lock lock{mutex_};
  stats_.total = reader.getRecordCount(streamId_, vrs::Record::Type::DATA);
  lock.unlock();

  return true;
}

Stats Bluetooth::getStats() {
  std::lock_guard lock{mutex_};
  return stats_;
}

void Bluetooth::logStats() {
  std::lock_guard lock{mutex_};
  std::stringstream freqStr;
  freqStr << std::fixed << std::setprecision(1);
  for (const auto& count : stats_.totalPerFreq) {
    freqStr << count.first << ":" << count.second;
    if (count.first != (--stats_.totalPerFreq.end())->first) {
      freqStr << " ";
    }
  }
  std::cout << fmt::format(
                   "{}: total={} processed={} bad={} uniqueId={} outOfOrder={} freqStats=[{}]",
                   streamId_.getName(),
                   stats_.total,
                   stats_.processed,
                   stats_.bad,
                   stats_.uniqueId,
                   stats_.outOfOrder,
                   freqStr.str())
            << std::endl;
}

bool Bluetooth::getResult() {
  bool result = true;
  if (stats_.total == stats_.bad) {
    XR_LOGE("{}: No valid beacons collected", streamId_.getName());
    result = false;
  }
  return result;
}

void Bluetooth::processData(const data_provider::BluetoothBeaconData& data) {
  std::lock_guard lock{mutex_};
  uint64_t currTimestampUs = data.boardTimestampNs / 1e3;
  if (data.boardTimestampNs < 0 || data.uniqueId.empty() || data.uniqueId == kNilUuid ||
      data.rssi >= 0.0) {
    XR_LOGE(
        "{}: Bad data timestamp={}us uniqueId={} rssi={}",
        streamId_.getName(),
        currTimestampUs,
        data.uniqueId,
        data.rssi);
    stats_.bad++;
  } else if (!data.uniqueId.empty() && data.uniqueId != kNilUuid) {
    uniqueId_.emplace(data.uniqueId);
    stats_.uniqueId = uniqueId_.size();
    stats_.totalPerFreq[data.freqMhz]++;
  }
  if (currTimestampUs < prevTimestampUs_) {
    XR_LOGW(
        "{}: Beacon received out of order beaconTs={}us prevTs={}us",
        streamId_.getName(),
        currTimestampUs,
        prevTimestampUs_);
    stats_.outOfOrder++;
  }
  prevTimestampUs_ = currTimestampUs;
  stats_.processed++;
}

} // namespace projectaria::tools::vrs_check
