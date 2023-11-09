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

#include <cmath>

#include "BluetoothBeaconPlayer.h"

namespace projectaria::tools::data_provider {

bool BluetoothBeaconPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config =
        getExpectedLayout<datalayout::BluetoothBeaconConfigRecordMetadata>(dl, blockIndex);
    configRecord_.streamId = config.streamId.get();
    configRecord_.sampleRateHz = config.sampleRateHz.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::BluetoothBeaconDataMetadata>(dl, blockIndex);
    dataRecord_.systemTimestampNs = data.systemTimestampNs.get();
    dataRecord_.boardTimestampNs = data.boardTimestampNs.get();
    dataRecord_.boardScanRequestStartTimestampNs = data.boardScanRequestStartTimestampNs.get();
    dataRecord_.boardScanRequestCompleteTimestampNs =
        data.boardScanRequestCompleteTimestampNs.get();
    dataRecord_.uniqueId = data.uniqueId.get();
    dataRecord_.txPower = data.txPower.get();
    dataRecord_.rssi = data.rssi.get();
    dataRecord_.freqMhz = data.freqMhz.get();
    nextTimestampSec_ = std::nextafter(r.timestamp, std::numeric_limits<double>::max());
    callback_(dataRecord_, configRecord_, verbose_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
