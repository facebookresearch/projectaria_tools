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

#include <data_provider/players/EmgPlayer.h>

#include <algorithm>

#define DEFAULT_LOG_CHANNEL "EmgPlayer"
#include <logging/Log.h>

namespace projectaria::tools::data_provider {
namespace {
// Zip the parallel per-sample vectors read from the data layout into EmgImuSample structs.
// The DataPieceVector fields are read into the local vectors via the out-param get() before being
// passed here, and `channels` is consumed (moved) into the samples. `encodings` is only populated
// for EMG samples; for accel / gyro it is empty and the encoding field defaults to 0. Mirrors the
// diagnostics emitted by oatmeal::EmgImuBatchPlayable::readSamples.
std::vector<EmgImuSample> readSamples(
    const std::vector<uint32_t>& sequenceNumbers,
    const std::vector<uint64_t>& timestampsNs,
    std::vector<std::string>& channels,
    const std::vector<uint32_t>& encodings) {
  if (sequenceNumbers.size() != timestampsNs.size() || sequenceNumbers.size() != channels.size()) {
    XR_LOGE(
        "EMG IMU sample arrays have mismatched sizes (sequence_numbers={}, timestamps={}, channels={}); truncating to the smallest",
        sequenceNumbers.size(),
        timestampsNs.size(),
        channels.size());
  }
  const size_t sampleCount =
      std::min({sequenceNumbers.size(), timestampsNs.size(), channels.size()});
  if (!encodings.empty() && encodings.size() != sampleCount) {
    XR_LOGE(
        "EMG encodings array size ({}) does not match the sample count ({}); missing encodings default to 0",
        encodings.size(),
        sampleCount);
  }
  std::vector<EmgImuSample> samples;
  samples.reserve(sampleCount);
  for (size_t i = 0; i < sampleCount; ++i) {
    samples.emplace_back(
        EmgImuSample{
            sequenceNumbers[i],
            static_cast<int64_t>(timestampsNs[i]),
            std::move(channels[i]),
            i < encodings.size() ? encodings[i] : 0});
  }
  return samples;
}
} // namespace

bool EmgPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config = getExpectedLayout<datalayout::EmgConfigurationLayout>(dl, blockIndex);
    configRecord_.streamId = config.streamId.get();
    configRecord_.sensorModel = config.sensorModel.get();
    configRecord_.deviceId = config.deviceId.get();
    configRecord_.nominalRateHz = config.nominalRateHz.get();
    configRecord_.description = config.description.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::EmgDataLayout>(dl, blockIndex);
    dataRecord_.captureTimestampNs = data.captureTimestampNs.get();
    dataRecord_.batchSequenceNumber = data.batchSequenceNumber.get();

    std::vector<uint32_t> emgSequenceNumbers;
    std::vector<uint64_t> emgTimestampsNs;
    std::vector<std::string> emgChannels;
    std::vector<uint32_t> emgEncodings;
    data.emgSequenceNumbers.get(emgSequenceNumbers);
    data.emgTimestampsNs.get(emgTimestampsNs);
    data.emgChannels.get(emgChannels);
    data.emgEncodings.get(emgEncodings);
    dataRecord_.emg = readSamples(emgSequenceNumbers, emgTimestampsNs, emgChannels, emgEncodings);

    std::vector<uint32_t> accelSequenceNumbers;
    std::vector<uint64_t> accelTimestampsNs;
    std::vector<std::string> accelChannels;
    data.accelSequenceNumbers.get(accelSequenceNumbers);
    data.accelTimestampsNs.get(accelTimestampsNs);
    data.accelChannels.get(accelChannels);
    dataRecord_.accel = readSamples(accelSequenceNumbers, accelTimestampsNs, accelChannels, {});

    std::vector<uint32_t> gyroSequenceNumbers;
    std::vector<uint64_t> gyroTimestampsNs;
    std::vector<std::string> gyroChannels;
    data.gyroSequenceNumbers.get(gyroSequenceNumbers);
    data.gyroTimestampsNs.get(gyroTimestampsNs);
    data.gyroChannels.get(gyroChannels);
    dataRecord_.gyro = readSamples(gyroSequenceNumbers, gyroTimestampsNs, gyroChannels, {});

    dataRecord_.channelCount = data.channelCount.get();
    dataRecord_.bitsPerAdcReading = data.bitsPerAdcReading.get();
    dataRecord_.samplesPerBatch = data.samplesPerBatch.get();
    // The user-defined callback function set via setCallback will be invoked here
    callback_(dataRecord_, configRecord_, verbose_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
