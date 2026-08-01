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

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <calibration/NeuralBandEmgCalibration.h>
#include <calibration/NeuralBandImuCalibration.h>
#include <data_layout/NeuralBandBatchMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

namespace projectaria::tools::data_provider {

struct NeuralBandEmgSample {
  int64_t captureTimestampNs{};
  std::vector<uint16_t> channelValues;
};

struct NeuralBandAccelSample {
  int64_t captureTimestampNs{};
  std::array<float, 3> accelMSec2{};
};

struct NeuralBandGyroSample {
  int64_t captureTimestampNs{};
  std::array<float, 3> gyroRadSec{};
};

struct NeuralBandBatch {
  int64_t captureTimestampNs{};
  uint32_t batchSequenceNumber{};
  uint32_t emgChannelCount{};
  uint32_t emgBitsPerAdcReading{};
  std::vector<NeuralBandEmgSample> emg;
  std::vector<NeuralBandAccelSample> accel;
  std::vector<NeuralBandGyroSample> gyro;
};

// Sample rate is not stored here; infer from the delta between consecutive
// sample timestamps.
struct NeuralBandBatchConfiguration {
  uint32_t streamId{};
  std::string sensorModel;
  uint64_t deviceId{};
  std::string emgCalibrationParamsJson;
  std::optional<calibration::NeuralBandEmgCalibration> emgCalibration;
  std::optional<calibration::NeuralBandImuCalibration> imuCalibration;
};

using NeuralBandBatchCallback = std::function<
    bool(const NeuralBandBatch& batch, const NeuralBandBatchConfiguration& config, bool verbose)>;

class NeuralBandBatchPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit NeuralBandBatchPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  NeuralBandBatchPlayer(const NeuralBandBatchPlayer&) = delete;
  NeuralBandBatchPlayer& operator=(const NeuralBandBatchPlayer&) = delete;
  NeuralBandBatchPlayer(NeuralBandBatchPlayer&&) = default;
  NeuralBandBatchPlayer& operator=(NeuralBandBatchPlayer&&) = delete;

  void setCallback(NeuralBandBatchCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const NeuralBandBatchConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const NeuralBandBatch& getDataRecord() const {
    return dataRecord_;
  }

  [[nodiscard]] const vrs::StreamId& getStreamId() const {
    return streamId_;
  }

  [[nodiscard]] double getNextTimestampSec() const {
    return nextTimestampSec_;
  }

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

 private:
  bool onDataLayoutRead(const vrs::CurrentRecord& r, size_t blockIndex, vrs::DataLayout& dl)
      override;

  const vrs::StreamId streamId_;
  NeuralBandBatchCallback callback_ =
      [](const NeuralBandBatch&, const NeuralBandBatchConfiguration&, bool) { return true; };
  NeuralBandBatchConfiguration configRecord_;
  NeuralBandBatch dataRecord_;
  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};

void rebaseWristbandTimestamps(NeuralBandBatch& batch);

int64_t deriveBatchWideEmgPeriodNs(
    const std::vector<int64_t>& wireTimestampsUs,
    uint32_t timeStepsPerPacket);

void synthesizeEmgSubSampleTimestamps(
    NeuralBandBatch& batch,
    uint32_t timeStepsPerPacket,
    int64_t periodNs);

void decodeEmgSamples(
    const std::vector<int64_t>& wireTimestampsUs,
    const std::vector<std::string>& channelBlobs,
    const std::vector<uint32_t>& encodings,
    uint32_t timeStepsPerPacket,
    uint32_t emgChannelCount,
    std::vector<NeuralBandEmgSample>& out);

void decodeAccelSamples(
    const std::vector<int64_t>& wireTimestampsUs,
    const std::vector<std::string>& channelBlobs,
    float lsbToMSec2,
    std::vector<NeuralBandAccelSample>& out);

void decodeGyroSamples(
    const std::vector<int64_t>& wireTimestampsUs,
    const std::vector<std::string>& channelBlobs,
    float lsbToRadSec,
    std::vector<NeuralBandGyroSample>& out);

} // namespace projectaria::tools::data_provider
