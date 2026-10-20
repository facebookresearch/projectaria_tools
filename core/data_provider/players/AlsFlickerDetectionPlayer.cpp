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

#include <limits>

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
    if (!config.nominalRate.get(configRecord_.nominalRateHz)) {
      configRecord_.nominalRateHz = std::numeric_limits<double>::quiet_NaN();
    }
    configRecord_.sensorModel = config.sensorModel.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::AlsFlickerDetectionDataLayout>(dl, blockIndex);
    if (!data.captureTimestampNs.get(dataRecord_.captureTimestampNs)) {
      dataRecord_.captureTimestampNs = -1;
    }
    const auto readPair = [](const auto& frequency, const auto& magnitude, auto& pair) {
      const bool hasFrequency = frequency.get(pair.frequencyHz);
      const bool hasMagnitude = magnitude.get(pair.magnitude);
      // Failed getters write zero. Invalidate the pair afterward: a missing magnitude next to a
      // NaN frequency would otherwise look like a legal unused slot, and a missing frequency
      // would read as a 0 Hz detection.
      if (!hasFrequency || !hasMagnitude) {
        pair.magnitude = std::numeric_limits<float>::quiet_NaN();
      }
    };
    readPair(data.flickerFrequencyHz0, data.flickerMagnitude0, dataRecord_.flickerData[0]);
    readPair(data.flickerFrequencyHz1, data.flickerMagnitude1, dataRecord_.flickerData[1]);
    readPair(data.flickerFrequencyHz2, data.flickerMagnitude2, dataRecord_.flickerData[2]);
    // The user-defined callback function set via setCallback will be invoked here
    callback_(dataRecord_, configRecord_, verbose_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
