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

#include <data_provider/players/AlsFlickerDetectionPlayer.h>

namespace projectaria::tools::data_provider {
bool AlsFlickerDetectionPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config =
        getExpectedLayout<datalayout::AlsFlickerDetectionConfigurationLayout>(dl, blockIndex);
    configRecord_.streamId = config.streamId.get();
    configRecord_.deviceId = config.deviceId.get();
    configRecord_.nominalRateHz = config.nominalRate.get();
    configRecord_.sensorModel = config.sensorModel.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::AlsFlickerDetectionDataLayout>(dl, blockIndex);
    dataRecord_.captureTimestampNs = data.captureTimestampNs.get();
    dataRecord_.flickerData[0].frequencyHz = data.flickerFrequencyHz0.get();
    dataRecord_.flickerData[0].magnitude = data.flickerMagnitude0.get();
    dataRecord_.flickerData[1].frequencyHz = data.flickerFrequencyHz1.get();
    dataRecord_.flickerData[1].magnitude = data.flickerMagnitude1.get();
    dataRecord_.flickerData[2].frequencyHz = data.flickerFrequencyHz2.get();
    dataRecord_.flickerData[2].magnitude = data.flickerMagnitude2.get();
    // The user-defined callback function set via setCallback will be invoked here
    callback_(dataRecord_, configRecord_, verbose_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
