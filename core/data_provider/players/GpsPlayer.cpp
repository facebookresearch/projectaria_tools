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

#include "GpsPlayer.h"

namespace projectaria::tools::data_provider {

bool GpsPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config = getExpectedLayout<datalayout::GpsConfigRecordMetadata>(dl, blockIndex);
    configRecord_.streamId = config.streamId.get();
    configRecord_.sampleRateHz = config.sampleRateHz.get();
    configRecord_.provider = config.provider.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::GpsDataMetadata>(dl, blockIndex);
    dataRecord_.captureTimestampNs = data.captureTimestampNs.get();
    dataRecord_.utcTimeMs = data.utcTimeMs.get();
    dataRecord_.provider = data.provider.get();
    dataRecord_.latitude = data.latitude.get();
    dataRecord_.longitude = data.longitude.get();
    dataRecord_.altitude = data.altitude.get();
    dataRecord_.accuracy = data.accuracy.get();
    dataRecord_.verticalAccuracy = data.verticalAccuracy.get();
    dataRecord_.speed = data.speed.get();
    data.rawData.get(dataRecord_.rawData);
    data.rawMeasurements.get(dataRecord_.rawMeasurements);
    data.navigationMessages.get(dataRecord_.navigationMessages);
    data.constellationsEnabled.get(dataRecord_.constellationsEnabled);
    nextTimestampSec_ = std::nextafter(r.timestamp, std::numeric_limits<double>::max());
    callback_(dataRecord_, configRecord_, verbose_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
