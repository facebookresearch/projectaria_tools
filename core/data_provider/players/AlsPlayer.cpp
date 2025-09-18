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

#include <data_provider/players/AlsPlayer.h>

namespace projectaria::tools::data_provider {
bool AlsPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config = getExpectedLayout<datalayout::AlsConfigurationLayout>(dl, blockIndex);
    configRecord_.streamId = config.streamId.get();
    configRecord_.deviceId = config.deviceId.get();
    configRecord_.nominalRateHz = config.nominalRate.get();
    configRecord_.sensorModel = config.sensorModel.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::AlsDataLayout>(dl, blockIndex);
    dataRecord_.captureTimestampNs = data.captureTimestampNs.get();
    dataRecord_.redChannelNormalized = data.redChannelNormalized.get();
    dataRecord_.greenChannelNormalized = data.greenChannelNormalized.get();
    dataRecord_.blueChannelNormalized = data.blueChannelNormalized.get();
    dataRecord_.uvChannelNormalized = data.uvChannelNormalized.get();
    dataRecord_.irChannelNormalized = data.irChannelNormalized.get();
    dataRecord_.clearChannelNormalized = data.clearChannelNormalized.get();
    dataRecord_.uvFluxWattPerSquareMeter = data.uvFluxWattPerSquareMeter.get();
    dataRecord_.irFluxWattPerSquareMeter = data.irFluxWattPerSquareMeter.get();
    dataRecord_.clearFluxWattPerSquareMeter = data.clearFluxWattPerSquareMeter.get();
    dataRecord_.gainRed = data.gainRed.get();
    dataRecord_.gainGreen = data.gainGreen.get();
    dataRecord_.gainBlue = data.gainBlue.get();
    dataRecord_.gainUv = data.gainUv.get();
    dataRecord_.gainIr = data.gainIr.get();
    dataRecord_.gainClear = data.gainClear.get();
    dataRecord_.exposureTimeUs = data.exposureTimeUs.get();
    dataRecord_.cct = data.cct.get();
    dataRecord_.lux = data.lux.get();
    // The user-defined callback function set via setCallback will be invoked here
    callback_(dataRecord_, configRecord_, verbose_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
