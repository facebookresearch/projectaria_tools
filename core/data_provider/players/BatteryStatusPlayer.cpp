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

#include "BatteryStatusPlayer.h"

namespace projectaria::tools::data_provider {

bool BatteryStatusPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config = getExpectedLayout<datalayout::BatteryStatusConfigRecordMetadata>(dl, blockIndex);
    configRecord_.streamId = config.streamId.get();
    configRecord_.sensorModel = config.sensorModel.get();
    configRecord_.deviceId = config.deviceId.get();
    configRecord_.nominalRateHz = config.nominalRateHz.get();
    configRecord_.description = config.description.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::BatteryStatusDataMetadata>(dl, blockIndex);
    dataRecord_.captureTimestampNs = data.captureTimestampNs.get();
    dataRecord_.chargingStatus = static_cast<ChargingStatus>(data.chargingStatus.get());
    dataRecord_.batteryLevel = data.batteryLevel.get();
    dataRecord_.temperatureC = data.temperatureC.get();
    dataRecord_.voltageVolt = data.voltageNowVolt.get();
    dataRecord_.voltageAvgVolt = data.voltageAvgVolt.get();
    dataRecord_.currentAmp = data.currentNowAmp.get();
    dataRecord_.currentAvgAmp = data.currentAvgAmp.get();
    dataRecord_.powerWatt = data.powerNowWatt.get();
    dataRecord_.powerAvgWatt = data.powerAvgWatt.get();
    nextTimestampSec_ = std::nextafter(r.timestamp, std::numeric_limits<double>::max());
    callback_(dataRecord_, configRecord_, verbose_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
