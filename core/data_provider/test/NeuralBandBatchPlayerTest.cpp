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

#include <data_provider/VrsDataProvider.h>
#include <data_provider/data_layout/MotionSensorMetadata.h>
#include <data_provider/test/NeuralBandBatchFixture.h>

#include <vrs/DataSource.h>
#include <vrs/RecordFileWriter.h>
#include <vrs/RecordFormat.h>
#include <vrs/Recordable.h>
#include <vrs/os/Utils.h>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <string>
#include <vector>

namespace projectaria::tools::data_provider {

using test::NeuralBandBatchFixtureSpec;
using test::TempFileGuard;
using test::writeSyntheticNeuralBandBatchVrs;

namespace {

std::vector<std::vector<uint16_t>> collectChannelValues(const NeuralBandBatch& batch) {
  std::vector<std::vector<uint16_t>> rows;
  rows.reserve(batch.emg.size());
  for (const auto& sample : batch.emg) {
    rows.push_back(sample.channelValues);
  }
  return rows;
}

std::vector<int64_t> collectWristbandTimes(const NeuralBandBatch& batch) {
  std::vector<int64_t> times;
  times.reserve(batch.emg.size());
  for (const auto& sample : batch.emg) {
    times.push_back(sample.wristbandTimestampNs);
  }
  return times;
}

// Sample-major uint16, little-endian — matches EMG wire format.
std::string packEmgBlobLittleEndian(const std::vector<uint16_t>& counts) {
  std::string blob;
  blob.reserve(counts.size() * sizeof(uint16_t));
  for (const uint16_t v : counts) {
    blob.push_back(static_cast<char>(v & 0xFF));
    blob.push_back(static_cast<char>((v >> 8) & 0xFF));
  }
  return blob;
}

// 3x LE int16 = 6-byte blob — matches accel/gyro wire format.
std::string packImuBlobLittleEndian(int16_t x, int16_t y, int16_t z) {
  std::string blob;
  blob.resize(6);
  const auto write = [&](size_t off, int16_t v) {
    const auto u = static_cast<uint16_t>(v);
    blob[off] = static_cast<char>(u & 0xFF);
    blob[off + 1] = static_cast<char>((u >> 8) & 0xFF);
  };
  write(0, x);
  write(2, y);
  write(4, z);
  return blob;
}

// Independently derived from spec so an impl math error can't mirror-match here.
// LSM6DSV32X datasheet sensitivities — see NeuralBandBatchPlayer.cpp constants comment.
constexpr float kAccelLsbToMSec2 = 0.000244f * 9.80665f;
constexpr float kGyroLsbToRadSec = 0.070f * (std::numbers::pi_v<float> / 180.0f);

} // namespace

TEST(NeuralBandBatchPlayerTest, ExpandEmgPacketsToSubSamples_SplitsBlobIntoRows) {
  NeuralBandBatch batch;
  batch.emgChannelCount = 2;
  NeuralBandEmgSample packet;
  packet.wristbandTimestampNs = 750'000'000;
  // Row-major [3 timesteps, 2 channels]: rows are {10,20}, {30,40}, {50,60}.
  packet.channelValues = {10, 20, 30, 40, 50, 60};
  batch.emg.push_back(std::move(packet));

  expandEmgPacketsToSubSamples(batch, /*timeStepsPerPacket=*/3);

  const std::vector<std::vector<uint16_t>> expectedRows{{10, 20}, {30, 40}, {50, 60}};
  EXPECT_EQ(collectChannelValues(batch), expectedRows);
  const std::vector<int64_t> expectedTimes{750'000'000, 750'000'000, 750'000'000};
  EXPECT_EQ(collectWristbandTimes(batch), expectedTimes);
}

TEST(NeuralBandBatchPlayerTest, ExpandEmgPacketsToSubSamples_PacketsKeepTheirOwnWireTime) {
  NeuralBandBatch batch;
  batch.emgChannelCount = 1;
  for (const auto& [ts, blob] : std::vector<std::pair<int64_t, std::vector<uint16_t>>>{
           {100'000'000, {1, 2}}, {200'000'000, {3, 4}}, {300'000'000, {5, 6}}}) {
    NeuralBandEmgSample packet;
    packet.wristbandTimestampNs = ts;
    packet.channelValues = blob;
    batch.emg.push_back(std::move(packet));
  }

  expandEmgPacketsToSubSamples(batch, /*timeStepsPerPacket=*/2);

  const std::vector<std::vector<uint16_t>> expectedRows{{1}, {2}, {3}, {4}, {5}, {6}};
  EXPECT_EQ(collectChannelValues(batch), expectedRows);
  const std::vector<int64_t> expectedTimes{
      100'000'000, 100'000'000, 200'000'000, 200'000'000, 300'000'000, 300'000'000};
  EXPECT_EQ(collectWristbandTimes(batch), expectedTimes);
}

TEST(NeuralBandBatchPlayerTest, ExpandEmgPacketsToSubSamples_LeavesDeviceTimeUnset) {
  NeuralBandBatch batch;
  batch.emgChannelCount = 1;
  NeuralBandEmgSample packet;
  packet.wristbandTimestampNs = 1'000;
  packet.channelValues = {7, 8};
  batch.emg.push_back(std::move(packet));

  expandEmgPacketsToSubSamples(batch, /*timeStepsPerPacket=*/2);

  ASSERT_EQ(batch.emg.size(), 2u);
  EXPECT_FALSE(batch.emg[0].deviceTimestampNs.has_value());
  EXPECT_FALSE(batch.emg[1].deviceTimestampNs.has_value());
}

TEST(NeuralBandBatchPlayerTest, ExpandEmgPacketsToSubSamples_EmptyBatchIsNoOp) {
  NeuralBandBatch batch;
  batch.emgChannelCount = 2;
  expandEmgPacketsToSubSamples(batch, 8);
  EXPECT_TRUE(batch.emg.empty());
}

TEST(NeuralBandBatchPlayerTest, ExpandEmgPacketsToSubSamples_ZeroTimeStepsLeavesPacketsAsIs) {
  NeuralBandBatch batch;
  batch.emgChannelCount = 2;
  NeuralBandEmgSample packet;
  packet.channelValues = {10, 20};
  batch.emg.push_back(std::move(packet));

  expandEmgPacketsToSubSamples(batch, 0);

  const std::vector<std::vector<uint16_t>> unchanged{{10, 20}};
  EXPECT_EQ(collectChannelValues(batch), unchanged);
}

TEST(NeuralBandBatchPlayerTest, DecodeAccel_WellFormedBlobProducesScaledXyz) {
  const std::vector<int64_t> tsUs{500'000};
  const std::vector<std::string> blobs{packImuBlobLittleEndian(100, -200, 300)};
  std::vector<NeuralBandAccelSample> out;

  decodeAccelSamples(tsUs, blobs, kAccelLsbToMSec2, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].wristbandTimestampNs, 500'000'000);
  EXPECT_FLOAT_EQ(out[0].accelMSec2[0], 100.0f * kAccelLsbToMSec2);
  EXPECT_FLOAT_EQ(out[0].accelMSec2[1], -200.0f * kAccelLsbToMSec2);
  EXPECT_FLOAT_EQ(out[0].accelMSec2[2], 300.0f * kAccelLsbToMSec2);
}

TEST(NeuralBandBatchPlayerTest, DecodeAccel_MalformedBlobSkipped) {
  const std::vector<int64_t> tsUs{100, 200, 300};
  const std::vector<std::string> blobs{
      packImuBlobLittleEndian(1, 2, 3),
      std::string(5, '\0'), // wrong size
      packImuBlobLittleEndian(4, 5, 6),
  };
  std::vector<NeuralBandAccelSample> out;

  decodeAccelSamples(tsUs, blobs, kAccelLsbToMSec2, out);

  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].wristbandTimestampNs, 100'000);
  EXPECT_EQ(out[1].wristbandTimestampNs, 300'000);
}

TEST(NeuralBandBatchPlayerTest, DecodeAccel_BoundaryInt16Values) {
  const std::vector<int64_t> tsUs{0};
  const std::vector<std::string> blobs{packImuBlobLittleEndian(INT16_MIN, INT16_MAX, 0)};
  std::vector<NeuralBandAccelSample> out;

  decodeAccelSamples(tsUs, blobs, kAccelLsbToMSec2, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].accelMSec2[0], static_cast<float>(INT16_MIN) * kAccelLsbToMSec2);
  EXPECT_FLOAT_EQ(out[0].accelMSec2[1], static_cast<float>(INT16_MAX) * kAccelLsbToMSec2);
  EXPECT_FLOAT_EQ(out[0].accelMSec2[2], 0.0f);
}

TEST(NeuralBandBatchPlayerTest, DecodeAccel_UsesProvidedScaleFactor) {
  const std::vector<int64_t> tsUs{0};
  const std::vector<std::string> blobs{packImuBlobLittleEndian(1000, 0, 0)};
  std::vector<NeuralBandAccelSample> out;
  constexpr float kCustomScale = 0.5f;

  decodeAccelSamples(tsUs, blobs, kCustomScale, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].accelMSec2[0], 1000.0f * kCustomScale);
}

TEST(NeuralBandBatchPlayerTest, DecodeGyro_WellFormedBlobProducesScaledXyz) {
  const std::vector<int64_t> tsUs{700'000};
  const std::vector<std::string> blobs{packImuBlobLittleEndian(50, -75, 125)};
  std::vector<NeuralBandGyroSample> out;

  decodeGyroSamples(tsUs, blobs, kGyroLsbToRadSec, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].wristbandTimestampNs, 700'000'000);
  EXPECT_FLOAT_EQ(out[0].gyroRadSec[0], 50.0f * kGyroLsbToRadSec);
  EXPECT_FLOAT_EQ(out[0].gyroRadSec[1], -75.0f * kGyroLsbToRadSec);
  EXPECT_FLOAT_EQ(out[0].gyroRadSec[2], 125.0f * kGyroLsbToRadSec);
}

TEST(NeuralBandBatchPlayerTest, DecodeGyro_MalformedBlobSkipped) {
  const std::vector<int64_t> tsUs{10, 20};
  const std::vector<std::string> blobs{
      std::string(7, '\0'),
      packImuBlobLittleEndian(1, 2, 3),
  };
  std::vector<NeuralBandGyroSample> out;

  decodeGyroSamples(tsUs, blobs, kGyroLsbToRadSec, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].wristbandTimestampNs, 20'000);
}

// Full-pipeline tests write a synthetic VRS fixture and open it via
// createVrsDataProvider — they exercise the wiring only reachable when a
// record traverses the whole reader path.
namespace {

// Assertions below assume:
//   - 3 EMG packets at wire us {100_000, 200_000, 300_000} (uniform 100 ms cadence)
//   - batchCaptureNs = 300_000_000 (last packet anchor lines up on capture => zero rebase offset)
//   - timeStepsPerPacket = 4 => derived period = 200_000 * 1000 / ((3-1)*4) = 25_000_000 ns
constexpr int64_t kFixtureBatchCaptureNs = 300'000'000;
constexpr uint32_t kFixtureBatchSeq = 42;
constexpr uint32_t kFixtureChannelCount = 2;
constexpr uint32_t kFixtureBitsPerAdcReading = 12;
constexpr uint32_t kFixtureTimeStepsPerPacket = 4;
constexpr uint32_t kFixtureNumEmgPackets = 3;

// For packet p, time-step t, channel c: raw count = p*100 + t*10 + c.
// Row-major flat layout within a packet's blob is [t, c].
constexpr uint16_t encodeEmgCount(uint32_t p, uint32_t t, uint32_t c) {
  return static_cast<uint16_t>(p * 100 + t * 10 + c);
}

NeuralBandBatchFixtureSpec makeBackwardCompatSpec() {
  NeuralBandBatchFixtureSpec spec;
  spec.captureNs = kFixtureBatchCaptureNs;
  spec.batchSequence = kFixtureBatchSeq;
  spec.channelCount = kFixtureChannelCount;
  spec.bitsPerAdcReading = kFixtureBitsPerAdcReading;
  spec.timeStepsPerPacket = kFixtureTimeStepsPerPacket;
  spec.emgTimestampsUs = {100'000, 200'000, 300'000};
  spec.emgBlobs.reserve(kFixtureNumEmgPackets);
  for (uint32_t p = 0; p < kFixtureNumEmgPackets; ++p) {
    std::vector<uint16_t> counts;
    counts.reserve(kFixtureTimeStepsPerPacket * kFixtureChannelCount);
    for (uint32_t t = 0; t < kFixtureTimeStepsPerPacket; ++t) {
      for (uint32_t c = 0; c < kFixtureChannelCount; ++c) {
        counts.push_back(encodeEmgCount(p, t, c));
      }
    }
    spec.emgBlobs.push_back(packEmgBlobLittleEndian(counts));
  }
  spec.emgEncodings.assign(kFixtureNumEmgPackets, 0);
  spec.emgSequenceNumbers = {1, 2, 3};
  return spec;
}

} // namespace

TEST(NeuralBandBatchPlayerTest, EmgCalibrationParamsJson_ParsedIntoOptionalOnFullPipeline) {
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_neural_band_calib");
  TempFileGuard guard{vrsPath};
  NeuralBandBatchFixtureSpec spec = makeBackwardCompatSpec();
  spec.emgCalibrationParamsJson = R"({
    "analog_gain": 30.0,
    "daq_range_max": 8191,
    "daq_vref_pos": 1.6,
    "daq_vref_neg": 0.0,
    "daq_vref_dc": 0.8,
    "emg_adc_chip": 3,
    "emg_truncation_bits_transmitted_per_sample": 13,
    "emg_truncation_dropped_lsb": 3
  })";
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, spec));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  const auto config = provider->getNeuralBandBatchConfiguration(*streamId);
  EXPECT_FALSE(config.emgCalibrationParamsJson.empty());
  ASSERT_TRUE(config.emgCalibration.has_value());
  EXPECT_TRUE(config.emgCalibration->isTruncated());
  EXPECT_EQ(config.emgCalibration->getStreamedBitWidth(), 13u);
  EXPECT_EQ(config.emgCalibration->getDroppedLsb(), 3u);
  EXPECT_EQ(config.emgCalibration->getAdcChipCode(), 3u);
  EXPECT_FLOAT_EQ(config.emgCalibration->getAnalogGain(), 30.0f);
}

TEST(NeuralBandBatchPlayerTest, EmgCalibrationParamsJson_EmptyStringLeavesOptionalNullopt) {
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_neural_band_no_calib");
  TempFileGuard guard{vrsPath};
  const NeuralBandBatchFixtureSpec spec = makeBackwardCompatSpec(); // empty calib string
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, spec));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  const auto config = provider->getNeuralBandBatchConfiguration(*streamId);
  EXPECT_TRUE(config.emgCalibrationParamsJson.empty());
  EXPECT_FALSE(config.emgCalibration.has_value());
  EXPECT_FALSE(config.imuCalibration.has_value());
}

TEST(NeuralBandBatchPlayerTest, ImuCalibrationParamsJson_ParsedIntoOptionalOnFullPipeline) {
  const std::string vrsPath = vrs::os::getUniquePath(
      vrs::os::getTempFolder() + "aria_gen2_unit_test_neural_band_imu_calib");
  TempFileGuard guard{vrsPath};
  NeuralBandBatchFixtureSpec spec = makeBackwardCompatSpec();
  // Non-default gyro scaling factor (0.07 dps/LSB, LSM6DSV32X ±2000dps) — different from the
  // hardcoded fallback (2000/32768 ≈ 0.061); asserts the reader honors the wire value.
  spec.emgCalibrationParamsJson = R"({
    "imu_accel_scaling_factor": 0.000244,
    "imu_gyro_scaling_factor": 0.07,
    "imu_calibration_applied": 2,
    "imu_calibration": {
      "sigfigs": 6,
      "offline_accel_offset": [0, 0, 0],
      "offline_gyro_offset": [0, 0, 0],
      "online_accel_offset": [0, 0, 0],
      "online_gyro_offset": [0, 0, 0],
      "accel_cross_axis_rect_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1],
      "gyro_cross_axis_rect_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1],
      "gyro_linear_g_rect_matrix": [0, 0, 0, 0, 0, 0, 0, 0, 0]
    }
  })";
  // Inject a single accel+gyro packet each so we can assert the wire scaling factor is used.
  spec.accelTimestampsUs = {300'000};
  spec.accelBlobs = {packImuBlobLittleEndian(1000, 0, 0)};
  spec.accelSequenceNumbers = {1};
  spec.gyroTimestampsUs = {300'000};
  spec.gyroBlobs = {packImuBlobLittleEndian(1000, 0, 0)};
  spec.gyroSequenceNumbers = {1};

  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, spec));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  const auto config = provider->getNeuralBandBatchConfiguration(*streamId);
  ASSERT_TRUE(config.imuCalibration.has_value());
  EXPECT_FLOAT_EQ(config.imuCalibration->getAccelScalingFactor(), 0.000244f);
  EXPECT_FLOAT_EQ(config.imuCalibration->getGyroScalingFactor(), 0.07f);
  EXPECT_EQ(
      config.imuCalibration->getCalibrationApplied(),
      projectaria::tools::calibration::NeuralBandImuCalibrationApplied::GyroOnlineBias);
  EXPECT_EQ(config.imuCalibration->getSigfigs(), 6u);

  const NeuralBandBatch batch = provider->getNeuralBandBatchByIndex(*streamId, 0);
  ASSERT_EQ(batch.accel.size(), 1u);
  ASSERT_EQ(batch.gyro.size(), 1u);
  constexpr float kGravity = 9.80665f;
  constexpr float kDegToRad = std::numbers::pi_v<float> / 180.0f;
  EXPECT_FLOAT_EQ(batch.accel[0].accelMSec2[0], 1000.0f * 0.000244f * kGravity);
  EXPECT_FLOAT_EQ(batch.gyro[0].gyroRadSec[0], 1000.0f * 0.07f * kDegToRad);
}

TEST(NeuralBandBatchPlayerTest, ImuCalibrationParamsJson_FallsBackToHardcodedWhenAbsent) {
  const std::string vrsPath = vrs::os::getUniquePath(
      vrs::os::getTempFolder() + "aria_gen2_unit_test_neural_band_imu_fallback");
  TempFileGuard guard{vrsPath};
  NeuralBandBatchFixtureSpec spec = makeBackwardCompatSpec();
  spec.accelTimestampsUs = {300'000};
  spec.accelBlobs = {packImuBlobLittleEndian(1000, 0, 0)};
  spec.accelSequenceNumbers = {1};
  spec.gyroTimestampsUs = {300'000};
  spec.gyroBlobs = {packImuBlobLittleEndian(1000, 0, 0)};
  spec.gyroSequenceNumbers = {1};

  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, spec));
  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  const auto config = provider->getNeuralBandBatchConfiguration(*streamId);
  EXPECT_FALSE(config.imuCalibration.has_value());

  const NeuralBandBatch batch = provider->getNeuralBandBatchByIndex(*streamId, 0);
  ASSERT_EQ(batch.accel.size(), 1u);
  ASSERT_EQ(batch.gyro.size(), 1u);
  EXPECT_FLOAT_EQ(batch.accel[0].accelMSec2[0], 1000.0f * kAccelLsbToMSec2);
  EXPECT_FLOAT_EQ(batch.gyro[0].gyroRadSec[0], 1000.0f * kGyroLsbToRadSec);
}

TEST(NeuralBandBatchPlayerTest, BackwardCompat_ReadsSyntheticGen2FixtureViaFullPipeline) {
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_sequence_with_emg");
  TempFileGuard guard{vrsPath};
  const NeuralBandBatchFixtureSpec spec = makeBackwardCompatSpec();
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, spec));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);

  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value()) << "Test fixture must contain an 'emg' stream.";

  const NeuralBandBatch batch = provider->getNeuralBandBatchByIndex(*streamId, 0);

  EXPECT_EQ(batch.arrivalTimestampNs, kFixtureBatchCaptureNs);
  EXPECT_EQ(batch.batchSequenceNumber, kFixtureBatchSeq);
  EXPECT_EQ(batch.emgChannelCount, kFixtureChannelCount);
  EXPECT_EQ(batch.emgBitsPerAdcReading, kFixtureBitsPerAdcReading);

  ASSERT_EQ(batch.emg.size(), kFixtureNumEmgPackets * kFixtureTimeStepsPerPacket);
  EXPECT_EQ(batch.emg[0].channelValues.size(), batch.emgChannelCount);

  const std::vector<uint16_t> expectedFirstRow{encodeEmgCount(0, 0, 0), encodeEmgCount(0, 0, 1)};
  EXPECT_EQ(batch.emg.front().channelValues, expectedFirstRow);

  const std::vector<uint16_t> expectedLastRow{encodeEmgCount(2, 3, 0), encodeEmgCount(2, 3, 1)};
  EXPECT_EQ(batch.emg.back().channelValues, expectedLastRow);

  // Wire us reaches the samples as ns; first and last packet.
  EXPECT_EQ(batch.emg.front().wristbandTimestampNs, 100'000'000);
  EXPECT_EQ(batch.emg.back().wristbandTimestampNs, 300'000'000);
  EXPECT_FALSE(batch.emg.front().deviceTimestampNs.has_value());

  EXPECT_TRUE(batch.accel.empty());
  EXPECT_TRUE(batch.gyro.empty());
}

TEST(NeuralBandBatchPlayerTest, Integration_MalformedEmgPacketSkippedSurvivorsIntact) {
  constexpr int64_t kBatchCaptureNs = 300'000'000;
  constexpr uint32_t kTimeStepsPerPacket = 2;
  constexpr uint32_t kChannelCount = 1;
  const size_t expectedBlobBytes =
      static_cast<size_t>(kTimeStepsPerPacket) * kChannelCount * sizeof(uint16_t);

  NeuralBandBatchFixtureSpec spec;
  spec.captureNs = kBatchCaptureNs;
  spec.batchSequence = 9;
  spec.channelCount = kChannelCount;
  spec.bitsPerAdcReading = 16;
  spec.timeStepsPerPacket = kTimeStepsPerPacket;
  // 3 wire ts at 100/200/300 kus, span 200_000 us => 200_000_000 ns.
  //   all-wire period    = 200_000_000 / ((3-1) * timeSteps) = 50_000_000
  //   decoded-only period = 200_000_000 / ((2-1) * timeSteps) = 100_000_000
  // Assertions below assume the all-wire period.
  spec.emgTimestampsUs = {100'000, 200'000, 300'000};
  spec.emgBlobs = {
      packEmgBlobLittleEndian({0x0001, 0x0002}),
      std::string(expectedBlobBytes - 1, '\0'), // malformed
      packEmgBlobLittleEndian({0x0005, 0x0006}),
  };
  spec.emgEncodings = {0, 0, 0};
  spec.emgSequenceNumbers = {1, 2, 3};

  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_malformed_emg");
  TempFileGuard guard{vrsPath};
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, spec));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());
  const NeuralBandBatch batch = provider->getNeuralBandBatchByIndex(*streamId, 0);

  ASSERT_EQ(batch.emg.size(), 2u * kTimeStepsPerPacket);

  // Packet 1 is the malformed one.
  const std::vector<int64_t> expectedTimes{100'000'000, 100'000'000, 300'000'000, 300'000'000};
  EXPECT_EQ(collectWristbandTimes(batch), expectedTimes);

  const std::vector<uint16_t> row0Packet0{0x0001};
  const std::vector<uint16_t> row1Packet0{0x0002};
  const std::vector<uint16_t> row0Packet2{0x0005};
  const std::vector<uint16_t> row1Packet2{0x0006};
  EXPECT_EQ(batch.emg[0].channelValues, row0Packet0);
  EXPECT_EQ(batch.emg[1].channelValues, row1Packet0);
  EXPECT_EQ(batch.emg[2].channelValues, row0Packet2);
  EXPECT_EQ(batch.emg[3].channelValues, row1Packet2);
}

} // namespace projectaria::tools::data_provider
