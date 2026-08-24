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

#include <data_provider/players/NeuralBandBatchPlayer.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <numbers>
#include <string>
#include <vector>

#define DEFAULT_LOG_CHANNEL "NeuralBandBatchPlayer"
#include <logging/Log.h>

namespace projectaria::tools::data_provider {
namespace {

// Guard latches. Each `atomic_flag` fires once per process to prevent log
// spam from a systematic firmware regression. Not observable outside logs.
std::atomic_flag emgSizeMismatchFired = ATOMIC_FLAG_INIT;
std::atomic_flag imuSizeMismatchFired = ATOMIC_FLAG_INIT;

constexpr size_t kNeuralBandImuChannelsPerSample = 3;
constexpr size_t kNeuralBandImuBytesPerSample = kNeuralBandImuChannelsPerSample * sizeof(int16_t);
// LSM6DSV32X datasheet sensitivities at ±8g / ±2000dps. Note the gyro spec
// (0.070 dps/LSB) is *not* 2000/32768 (~0.061); the theoretical ratio would
// under-scale gyro by ~15%. Wire-side calibration overrides these when present.
constexpr float kAccelGPerLsb = 0.000244f;
constexpr float kGyroDpsPerLsb = 0.070f;
constexpr float kGravityMSec2 = 9.80665f;
constexpr float kPiF = std::numbers::pi_v<float>;
constexpr float kDegToRad = kPiF / 180.0f;
constexpr float kAccelLsbToMSec2 = kAccelGPerLsb * kGravityMSec2;
constexpr float kGyroLsbToRadSec = kGyroDpsPerLsb * kDegToRad;
constexpr int64_t kMicrosToNanos = 1'000;

int16_t readLeInt16(const uint8_t* bytes) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8));
}

uint16_t readLeUint16(const uint8_t* bytes) {
  return static_cast<uint16_t>(
      static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8));
}

// Cast so non-monotonic input produces a negative delta rather than uint wrap.
std::vector<int64_t> toInt64Vec(const std::vector<uint64_t>& in) {
  std::vector<int64_t> out;
  out.reserve(in.size());
  for (const uint64_t v : in) {
    out.push_back(static_cast<int64_t>(v));
  }
  return out;
}

template <typename SampleT, typename AssignAxes>
void decodeImu3AxisSamples(
    const std::vector<int64_t>& wireTimestampsUs,
    const std::vector<std::string>& channelBlobs,
    float lsbToUnit,
    const char* streamTag,
    AssignAxes assign,
    std::vector<SampleT>& out) {
  out.clear();
  if (wireTimestampsUs.size() != channelBlobs.size() && !imuSizeMismatchFired.test_and_set()) {
    XR_LOGW(
        "Neural Band {} wire-timestamp vs channel-blob array size mismatch ({} vs {}); truncating to smaller.",
        streamTag,
        wireTimestampsUs.size(),
        channelBlobs.size());
  }
  const size_t packetCount = std::min(wireTimestampsUs.size(), channelBlobs.size());
  out.reserve(packetCount);
  for (size_t p = 0; p < packetCount; ++p) {
    const std::string& blob = channelBlobs[p];
    if (blob.size() != kNeuralBandImuBytesPerSample) {
      XR_LOGE(
          "Skipping malformed Neural Band {} blob (size {} != expected {}).",
          streamTag,
          blob.size(),
          kNeuralBandImuBytesPerSample);
      continue;
    }
    const auto* bytes = reinterpret_cast<const uint8_t*>(blob.data());
    SampleT s;
    s.wristbandTimestampNs = wireTimestampsUs[p] * kMicrosToNanos;
    assign(
        s,
        std::array<float, 3>{
            static_cast<float>(readLeInt16(bytes)) * lsbToUnit,
            static_cast<float>(readLeInt16(bytes + 2)) * lsbToUnit,
            static_cast<float>(readLeInt16(bytes + 4)) * lsbToUnit,
        });
    out.push_back(s);
  }
}

} // namespace

void expandEmgPacketsToSubSamples(NeuralBandBatch& batch, uint32_t timeStepsPerPacket) {
  if (batch.emg.empty() || timeStepsPerPacket == 0) {
    return;
  }
  std::vector<NeuralBandEmgSample> expanded;
  expanded.reserve(batch.emg.size() * timeStepsPerPacket);
  for (const auto& packet : batch.emg) {
    for (uint32_t k = 0; k < timeStepsPerPacket; ++k) {
      NeuralBandEmgSample sub;
      sub.wristbandTimestampNs = packet.wristbandTimestampNs;
      const size_t rowStart = static_cast<size_t>(k) * batch.emgChannelCount;
      const size_t rowEnd = rowStart + batch.emgChannelCount;
      if (rowEnd <= packet.channelValues.size()) {
        sub.channelValues.assign(
            packet.channelValues.begin() + rowStart, packet.channelValues.begin() + rowEnd);
      }
      expanded.push_back(std::move(sub));
    }
  }
  batch.emg = std::move(expanded);
}

void decodeEmgSamples(
    const std::vector<int64_t>& wireTimestampsUs,
    const std::vector<std::string>& channelBlobs,
    const std::vector<uint32_t>& encodings,
    uint32_t timeStepsPerPacket,
    uint32_t emgChannelCount,
    std::vector<NeuralBandEmgSample>& out) {
  out.clear();
  out.reserve(channelBlobs.size());
  if ((wireTimestampsUs.size() != channelBlobs.size() ||
       wireTimestampsUs.size() != encodings.size()) &&
      !emgSizeMismatchFired.test_and_set()) {
    XR_LOGW(
        "Neural Band EMG wire-timestamp vs channel-blob vs encoding array size mismatch ({} vs {} vs {}); truncating to smallest.",
        wireTimestampsUs.size(),
        channelBlobs.size(),
        encodings.size());
  }
  const size_t emgBytesPerPacket =
      static_cast<size_t>(timeStepsPerPacket) * emgChannelCount * sizeof(uint16_t);
  for (size_t p = 0; p < channelBlobs.size() && p < wireTimestampsUs.size(); ++p) {
    const uint32_t encoding = p < encodings.size() ? encodings[p] : 0;
    if (encoding != 0) {
      XR_LOGE("Skipping Neural Band EMG packet with non-zero encoding {}.", encoding);
      continue;
    }
    const std::string& blob = channelBlobs[p];
    if (blob.size() != emgBytesPerPacket) {
      XR_LOGE(
          "Skipping malformed Neural Band EMG blob (size {} != expected {}).",
          blob.size(),
          emgBytesPerPacket);
      continue;
    }
    NeuralBandEmgSample packetSample;
    packetSample.wristbandTimestampNs = wireTimestampsUs[p] * kMicrosToNanos;
    packetSample.channelValues.reserve(static_cast<size_t>(timeStepsPerPacket) * emgChannelCount);
    const auto* bytes = reinterpret_cast<const uint8_t*>(blob.data());
    for (uint32_t t = 0; t < timeStepsPerPacket; ++t) {
      for (uint32_t c = 0; c < emgChannelCount; ++c) {
        const size_t off = (static_cast<size_t>(t) * emgChannelCount + c) * sizeof(uint16_t);
        packetSample.channelValues.push_back(readLeUint16(bytes + off));
      }
    }
    out.push_back(std::move(packetSample));
  }
}

void decodeAccelSamples(
    const std::vector<int64_t>& wireTimestampsUs,
    const std::vector<std::string>& channelBlobs,
    float lsbToMSec2,
    std::vector<NeuralBandAccelSample>& out) {
  decodeImu3AxisSamples(
      wireTimestampsUs,
      channelBlobs,
      lsbToMSec2,
      "accel",
      [](NeuralBandAccelSample& s, const std::array<float, 3>& xyz) { s.accelMSec2 = xyz; },
      out);
}

void decodeGyroSamples(
    const std::vector<int64_t>& wireTimestampsUs,
    const std::vector<std::string>& channelBlobs,
    float lsbToRadSec,
    std::vector<NeuralBandGyroSample>& out) {
  decodeImu3AxisSamples(
      wireTimestampsUs,
      channelBlobs,
      lsbToRadSec,
      "gyro",
      [](NeuralBandGyroSample& s, const std::array<float, 3>& xyz) { s.gyroRadSec = xyz; },
      out);
}

bool NeuralBandBatchPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config =
        getExpectedLayout<datalayout::NeuralBandBatchConfigurationLayout>(dl, blockIndex);
    configRecord_.streamId = config.streamId.get();
    configRecord_.sensorModel = config.sensorModel.get();
    configRecord_.deviceId = config.deviceId.get();
    std::string newJson = config.emgCalibrationParamsJson.get();
    if (newJson != configRecord_.emgCalibrationParamsJson) {
      configRecord_.emgCalibrationParamsJson = std::move(newJson);
      configRecord_.emgCalibration = calibration::NeuralBandEmgCalibration::fromParamsJson(
          configRecord_.emgCalibrationParamsJson);
      configRecord_.imuCalibration = calibration::NeuralBandImuCalibration::fromParamsJson(
          configRecord_.emgCalibrationParamsJson);
    }
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& layout = getExpectedLayout<datalayout::NeuralBandBatchDataLayout>(dl, blockIndex);
    dataRecord_.arrivalTimestampNs = layout.captureTimestampNs.get();
    dataRecord_.batchSequenceNumber = layout.batchSequenceNumber.get();
    dataRecord_.emgChannelCount = layout.channelCount.get();
    dataRecord_.emgBitsPerAdcReading = layout.bitsPerAdcReading.get();
    const uint32_t timeStepsPerPacket = layout.samplesPerBatch.get();

    std::vector<uint64_t> emgTimestampsUsRaw;
    std::vector<std::string> emgChannels;
    std::vector<uint32_t> emgEncodings;
    layout.emgTimestampsNs.get(emgTimestampsUsRaw);
    layout.emgChannels.get(emgChannels);
    layout.emgEncodings.get(emgEncodings);

    std::vector<uint64_t> accelTimestampsUsRaw;
    std::vector<std::string> accelChannels;
    layout.accelTimestampsNs.get(accelTimestampsUsRaw);
    layout.accelChannels.get(accelChannels);

    std::vector<uint64_t> gyroTimestampsUsRaw;
    std::vector<std::string> gyroChannels;
    layout.gyroTimestampsNs.get(gyroTimestampsUsRaw);
    layout.gyroChannels.get(gyroChannels);

    const std::vector<int64_t> emgWireUs = toInt64Vec(emgTimestampsUsRaw);
    const std::vector<int64_t> accelWireUs = toInt64Vec(accelTimestampsUsRaw);
    const std::vector<int64_t> gyroWireUs = toInt64Vec(gyroTimestampsUsRaw);

    decodeEmgSamples(
        emgWireUs,
        emgChannels,
        emgEncodings,
        timeStepsPerPacket,
        dataRecord_.emgChannelCount,
        dataRecord_.emg);

    const float accelLsbToMSec2 = configRecord_.imuCalibration
        ? configRecord_.imuCalibration->getAccelScalingFactor() * kGravityMSec2
        : kAccelLsbToMSec2;
    const float gyroLsbToRadSec = configRecord_.imuCalibration
        ? configRecord_.imuCalibration->getGyroScalingFactor() * kDegToRad
        : kGyroLsbToRadSec;

    decodeAccelSamples(accelWireUs, accelChannels, accelLsbToMSec2, dataRecord_.accel);
    decodeGyroSamples(gyroWireUs, gyroChannels, gyroLsbToRadSec, dataRecord_.gyro);

    expandEmgPacketsToSubSamples(dataRecord_, timeStepsPerPacket);

    callback_(dataRecord_, configRecord_, verbose_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
