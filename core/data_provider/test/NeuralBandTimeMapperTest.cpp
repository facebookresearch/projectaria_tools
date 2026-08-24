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

#include <data_provider/VrsDataProvider.h>
#include <data_provider/test/NeuralBandBatchFixture.h>

#include <vrs/os/Utils.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::data_provider::test;

namespace {

constexpr uint32_t kChannelCount = 2;
constexpr uint32_t kTimeStepsPerPacket = 4;
constexpr int64_t kPacketPeriodUs = 8'000;
constexpr int64_t kSamplePeriodNs = kPacketPeriodUs * 1'000 / kTimeStepsPerPacket;
/// Several times the mapper's anchor spacing, so a file has anchors to interpolate between.
constexpr size_t kNumRecords = 2'000;
constexpr int64_t kWristbandStartUs = 5'000'000;
/// Held constant so the expected conversion is exact; jitter is opt-in per test.
constexpr int64_t kTrueOffsetNs = 30'000'000'000;

int64_t wristbandNsAtRecord(size_t index) {
  return (kWristbandStartUs + static_cast<int64_t>(index) * kPacketPeriodUs) * 1'000;
}

int64_t noJitterNs(size_t /*index*/) {
  return 0;
}

/// Stands in for burst delivery: tens of milliseconds either side, and not
/// periodic in the mapper's decimation step, so a window has to average rather
/// than land on one phase of a repeating pattern.
int64_t burstJitterNs(size_t index) {
  const auto scrambled = static_cast<uint32_t>(index * 2654435761U);
  return static_cast<int64_t>(scrambled % 64'000'001U) - 32'000'000;
}

std::string packBlob(uint16_t seed) {
  std::string blob;
  blob.reserve(kTimeStepsPerPacket * kChannelCount * sizeof(uint16_t));
  for (uint32_t i = 0; i < kTimeStepsPerPacket * kChannelCount; ++i) {
    const auto value = static_cast<uint16_t>(seed + i);
    blob.push_back(static_cast<char>(value & 0xFF));
    blob.push_back(static_cast<char>((value >> 8) & 0xFF));
  }
  return blob;
}

/// A recording of `kNumRecords` single-packet batches, one packet per record as
/// the band sends them. `imuLeadUs` gives accel and gyro a wire time of their own
/// ahead of the EMG, the way a separate trigger does; 0 leaves them out.
std::vector<NeuralBandBatchFixtureSpec> makeSpecs(
    int64_t (*arrivalJitterNs)(size_t) = noJitterNs,
    int64_t imuLeadUs = 0) {
  std::vector<NeuralBandBatchFixtureSpec> specs;
  specs.reserve(kNumRecords);
  for (size_t index = 0; index < kNumRecords; ++index) {
    const int64_t wristbandUs = kWristbandStartUs + static_cast<int64_t>(index) * kPacketPeriodUs;
    NeuralBandBatchFixtureSpec spec;
    spec.captureNs = wristbandUs * 1'000 + kTrueOffsetNs + arrivalJitterNs(index);
    spec.batchSequence = static_cast<uint32_t>(index);
    spec.channelCount = kChannelCount;
    spec.bitsPerAdcReading = 16;
    spec.timeStepsPerPacket = kTimeStepsPerPacket;
    spec.emgTimestampsUs = {static_cast<uint64_t>(wristbandUs)};
    spec.emgBlobs = {packBlob(static_cast<uint16_t>(index))};
    spec.emgEncodings = {0};
    spec.emgSequenceNumbers = {static_cast<uint32_t>(index)};
    if (imuLeadUs != 0) {
      const auto imuUs = static_cast<uint64_t>(wristbandUs - imuLeadUs);
      spec.accelTimestampsUs = {imuUs};
      spec.accelBlobs = {std::string(6, '\0')};
      spec.accelSequenceNumbers = {spec.batchSequence};
      spec.gyroTimestampsUs = {imuUs};
      spec.gyroBlobs = {std::string(6, '\0')};
      spec.gyroSequenceNumbers = {spec.batchSequence};
    }
    specs.push_back(std::move(spec));
  }
  return specs;
}

std::vector<std::optional<int64_t>> deviceTimestampsOf(const NeuralBandBatch& batch) {
  std::vector<std::optional<int64_t>> times;
  times.reserve(batch.emg.size());
  for (const auto& sample : batch.emg) {
    times.push_back(sample.deviceTimestampNs);
  }
  return times;
}

} // namespace

TEST(NeuralBandTimeMapper, MeasuresSamplePeriodFromTheRecording) {
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_emg_mapper");
  TempFileGuard guard{vrsPath};
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, makeSpecs()));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  EXPECT_EQ(
      provider->getNeuralBandEmgSamplePeriodNs(*streamId), std::optional<int64_t>(kSamplePeriodNs));
}

TEST(NeuralBandTimeMapper, MeasuresSamplePeriodWhenTheFirstRecordCarriesNoEmg) {
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_emg_mapper");
  TempFileGuard guard{vrsPath};
  auto specs = makeSpecs(noJitterNs, /*imuLeadUs=*/300);
  specs.front().emgTimestampsUs.clear();
  specs.front().emgBlobs.clear();
  specs.front().emgEncodings.clear();
  specs.front().emgSequenceNumbers.clear();
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, specs));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  EXPECT_EQ(
      provider->getNeuralBandEmgSamplePeriodNs(*streamId), std::optional<int64_t>(kSamplePeriodNs));
}

TEST(NeuralBandTimeMapper, RecoversAConstantOffsetAndRoundTrips) {
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_emg_mapper");
  TempFileGuard guard{vrsPath};
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, makeSpecs()));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  const int64_t wristbandNs = wristbandNsAtRecord(500);
  const auto deviceNs = provider->convertFromWristbandTimeToDeviceTimeNs(wristbandNs, *streamId);
  ASSERT_TRUE(deviceNs.has_value());
  EXPECT_EQ(*deviceNs, wristbandNs + kTrueOffsetNs);
  EXPECT_EQ(
      provider->convertFromDeviceTimeToWristbandTimeNs(*deviceNs, *streamId),
      std::optional<int64_t>(wristbandNs));
}

// The tolerance sits four times below the +/-32 ms a single record is off by, so
// the window has to be averaging to pass, and several times above the ~3 ms
// standard error of a median over the ~64 records it samples, so it does not
// flake. Do not tighten it without redoing that arithmetic.
TEST(NeuralBandTimeMapper, AveragesOutArrivalJitter) {
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_emg_mapper");
  TempFileGuard guard{vrsPath};
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, makeSpecs(burstJitterNs)));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  const int64_t wristbandNs = wristbandNsAtRecord(1'000);
  const auto deviceNs = provider->convertFromWristbandTimeToDeviceTimeNs(wristbandNs, *streamId);
  ASSERT_TRUE(deviceNs.has_value());
  EXPECT_NEAR(
      static_cast<double>(*deviceNs - wristbandNs),
      static_cast<double>(kTrueOffsetNs),
      8'000'000.0);
}

// The reason the estimator is windowed rather than causal: a reader has to answer
// the same question the same way however it got there.
TEST(NeuralBandTimeMapper, IsIndependentOfAccessOrder) {
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_emg_mapper");
  TempFileGuard guard{vrsPath};
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, makeSpecs(burstJitterNs)));

  // Two readers of one file: a second file would take the next recordable
  // instance id, and so a stream id the label mapper does not know.
  auto coldProvider = createVrsDataProvider(vrsPath);
  auto warmProvider = createVrsDataProvider(vrsPath);
  ASSERT_NE(coldProvider, nullptr);
  ASSERT_NE(warmProvider, nullptr);
  const auto streamId = coldProvider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  for (int index = 0; index < 1'500; index += 7) {
    warmProvider->getNeuralBandBatchByIndex(*streamId, index);
  }

  const NeuralBandBatch coldBatch = coldProvider->getNeuralBandBatchByIndex(*streamId, 1'500);
  const NeuralBandBatch warmBatch = warmProvider->getNeuralBandBatchByIndex(*streamId, 1'500);

  EXPECT_EQ(deviceTimestampsOf(coldBatch), deviceTimestampsOf(warmBatch));
}

TEST(NeuralBandTimeMapper, SpreadsSubSamplesForwardAcrossTheirPacket) {
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_emg_mapper");
  TempFileGuard guard{vrsPath};
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, makeSpecs()));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  const NeuralBandBatch batch = provider->getNeuralBandBatchByIndex(*streamId, 800);
  ASSERT_EQ(batch.emg.size(), kTimeStepsPerPacket);

  // The wire timestamp is the packet's first sub-sample; the rest step forward.
  std::vector<int64_t> deviceNs;
  for (const auto& sample : batch.emg) {
    ASSERT_TRUE(sample.deviceTimestampNs.has_value());
    deviceNs.push_back(*sample.deviceTimestampNs);
  }
  EXPECT_EQ(deviceNs.front(), batch.emg.front().wristbandTimestampNs + kTrueOffsetNs);
  for (size_t i = 1; i < deviceNs.size(); ++i) {
    EXPECT_EQ(deviceNs[i] - deviceNs[i - 1], kSamplePeriodNs) << "step " << i;
  }
}

TEST(NeuralBandTimeMapper, AccelAndGyroTakeTheirOwnWireTime) {
  const int64_t imuLeadUs = kPacketPeriodUs / 4;
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_emg_mapper_imu");
  TempFileGuard guard{vrsPath};
  ASSERT_NO_FATAL_FAILURE(
      writeSyntheticNeuralBandBatchVrs(vrsPath, makeSpecs(noJitterNs, imuLeadUs)));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);
  const auto streamId = provider->getStreamIdFromLabel("emg");
  ASSERT_TRUE(streamId.has_value());

  const NeuralBandBatch batch = provider->getNeuralBandBatchByIndex(*streamId, 800);
  ASSERT_EQ(batch.accel.size(), 1u);
  ASSERT_EQ(batch.gyro.size(), 1u);
  ASSERT_TRUE(batch.accel.front().deviceTimestampNs.has_value());

  EXPECT_EQ(
      batch.emg.front().wristbandTimestampNs - batch.accel.front().wristbandTimestampNs,
      imuLeadUs * 1'000);
  EXPECT_EQ(
      *batch.accel.front().deviceTimestampNs,
      batch.accel.front().wristbandTimestampNs + kTrueOffsetNs);
  EXPECT_EQ(batch.gyro.front().deviceTimestampNs, batch.accel.front().deviceTimestampNs);
}

TEST(NeuralBandTimeMapper, UnknownStreamConvertsToNothing) {
  const std::string vrsPath =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "aria_gen2_unit_test_emg_mapper");
  TempFileGuard guard{vrsPath};
  ASSERT_NO_FATAL_FAILURE(writeSyntheticNeuralBandBatchVrs(vrsPath, makeSpecs()));

  auto provider = createVrsDataProvider(vrsPath);
  ASSERT_NE(provider, nullptr);

  const vrs::StreamId absent(vrs::RecordableTypeId::SlamImuData, 1);
  EXPECT_FALSE(provider->convertFromWristbandTimeToDeviceTimeNs(1'000, absent).has_value());
  EXPECT_FALSE(provider->getNeuralBandEmgSamplePeriodNs(absent).has_value());
}
