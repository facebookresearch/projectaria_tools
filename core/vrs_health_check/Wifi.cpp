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

#include "Wifi.h"

#include <cctype>
#include <iomanip>
#include <sstream>

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:Wifi"
#include <logging/Log.h>

namespace projectaria::tools::vrs_check {

Wifi::Wifi(vrs::StreamId streamId) : Stream(streamId) {}

bool Wifi::setup(vrs::RecordFileReader& reader) {
  data_provider::WifiBeaconCallback callback = [&](const data_provider::WifiBeaconData& data,
                                                   const data_provider::WifiBeaconConfigRecord&,
                                                   bool) {
    processData(data);
    return true;
  };

  wifiBeaconPlayer_ = std::make_unique<data_provider::WifiBeaconPlayer>(streamId_);
  if (!wifiBeaconPlayer_) {
    XR_LOGE("Cannot create playable for {}", streamId_.getName());
    return false;
  }
  wifiBeaconPlayer_->setCallback(callback);
  reader.setStreamPlayer(streamId_, wifiBeaconPlayer_.get());

  std::unique_lock lock{mutex_};
  stats_.total = reader.getRecordCount(streamId_, vrs::Record::Type::DATA);
  lock.unlock();

  return true;
}

Stats Wifi::getStats() {
  std::lock_guard lock{mutex_};
  return stats_;
}

void Wifi::logStats() {
  std::lock_guard lock{mutex_};
  std::stringstream freqStr;
  freqStr << std::fixed << std::setprecision(1);
  for (const auto& count : stats_.totalPerFreq) {
    freqStr << count.first << ":" << count.second;
    if (count.first != (--stats_.totalPerFreq.end())->first) {
      freqStr << " ";
    }
  }
  std::cout
      << fmt::format(
             "{}: total={} processed={} bad={} ssid={} bssid={} nomap={} outOfOrder={} freqStats=[{}]",
             streamId_.getName(),
             stats_.total,
             stats_.processed,
             stats_.bad,
             stats_.ssid,
             stats_.bssid,
             stats_.nomap,
             stats_.outOfOrder,
             freqStr.str())
      << std::endl;
}

bool Wifi::getResult() {
  bool result = true;
  if (stats_.total == stats_.bad) {
    XR_LOGE("{}: No valid beacons collected", streamId_.getName());
    result = false;
  } else if (stats_.nomap > 0) {
    XR_LOGE("{}: Collected {} beacons with opt-out SSID", streamId_.getName(), stats_.nomap);
    result = false;
  } else if (stats_.totalPerFreq.size() < kMinChannels) {
    XR_LOGW(
        "{}: Collected beacons in only {} channels (min {})",
        streamId_.getName(),
        stats_.totalPerFreq.size(),
        kMinChannels);
  }
  return result;
}

namespace {
inline bool hasSuffix(const std::string& str, const std::string& suffix) {
  return str.size() >= suffix.size() &&
      str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}
} // namespace

void Wifi::processData(const data_provider::WifiBeaconData& data) {
  std::lock_guard lock{mutex_};
  std::string ssidLower = data.ssid;
  std::transform(ssidLower.begin(), ssidLower.end(), ssidLower.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  uint64_t currTimestampUs = data.boardTimestampNs / 1e3;
  if (hasSuffix(ssidLower, "_nomap") || hasSuffix(ssidLower, "_optout")) {
    XR_LOGE("{}: Opt out SSID collected {}", streamId_.getName(), data.ssid);
    stats_.nomap++;
  } else if (data.boardTimestampNs < 0 || data.bssidMac.empty() || data.rssi >= 0) {
    XR_LOGE(
        "{}: Bad data timestamp={}us uniqueId={} rssi={}",
        streamId_.getName(),
        currTimestampUs,
        data.bssidMac,
        data.rssi);
    stats_.bad++;
  } else {
    ssid_.emplace(data.ssid);
    stats_.ssid = ssid_.size();
    bssid_.emplace(data.bssidMac);
    stats_.bssid = bssid_.size();
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
