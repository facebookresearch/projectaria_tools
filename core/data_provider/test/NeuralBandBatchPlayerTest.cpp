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
namespace {

// Populates captureTimestampNs with the wristband boot clock in ns (µs * 1000),
// matching the post-decode / pre-rebase state. Channel data left empty;
// timestamp tests don't read it.
NeuralBandBatch makeBatchWithWirePackets(
    int64_t captureNs,
    const std::vector<int64_t>& emgWireUs,
    const std::vector<int64_t>& accelWireUs,
    const std::vector<int64_t>& gyroWireUs) {
  NeuralBandBatch batch;
  batch.captureTimestampNs = captureNs;
  for (const int64_t t : emgWireUs) {
    NeuralBandEmgSample s;
    s.captureTimestampNs = t * 1000;
    batch.emg.push_back(std::move(s));
  }
  for (const int64_t t : accelWireUs) {
    NeuralBandAccelSample s;
    s.captureTimestampNs = t * 1000;
    batch.accel.push_back(s);
  }
  for (const int64_t t : gyroWireUs) {
    NeuralBandGyroSample s;
    s.captureTimestampNs = t * 1000;
    batch.gyro.push_back(s);
  }
  return batch;
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
constexpr float kAccelLsbToMSec2 = (8.0f / 32768.0f) * 9.80665f;
constexpr float kGyroLsbToRadSec = (2000.0f / 32768.0f) * (std::numbers::pi_v<float> / 180.0f);

} // namespace

TEST(NeuralBandBatchPlayerTest, RebaseWristbandTimestamps_AnchorsLastSampleToBatchCapture) {
  auto batch = makeBatchWithWirePackets(
      /*captureNs=*/1'000'000'000,
      /*emgWireUs=*/{500'000, 504'000, 508'000},
      /*accelWireUs=*/{500'000, 508'000},
      /*gyroWireUs=*/{500'000, 508'000});

  rebaseWristbandTimestamps(batch);

  EXPECT_EQ(batch.emg.back().captureTimestampNs, 1'000'000'000);
  EXPECT_EQ(batch.accel.back().captureTimestampNs, 1'000'000'000);
  EXPECT_EQ(batch.gyro.back().captureTimestampNs, 1'000'000'000);
  EXPECT_EQ(batch.emg.front().captureTimestampNs, 992'000'000);
  EXPECT_EQ(batch.emg[1].captureTimestampNs, 996'000'000);
}

TEST(NeuralBandBatchPlayerTest, RebaseWristbandTimestamps_EmptySubStreamIsNoOp) {
  NeuralBandBatch batch;
  batch.captureTimestampNs = 500'000'000;
  rebaseWristbandTimestamps(batch);
  EXPECT_TRUE(batch.emg.empty());
  EXPECT_TRUE(batch.accel.empty());
  EXPECT_TRUE(batch.gyro.empty());
}

TEST(NeuralBandBatchPlayerTest, RebaseWristbandTimestamps_SinglePacketAnchorsExactly) {
  auto batch = makeBatchWithWirePackets(
      /*captureNs=*/2'000'000'000,
      /*emgWireUs=*/{750'000},
      /*accelWireUs=*/{},
      /*gyroWireUs=*/{});
  rebaseWristbandTimestamps(batch);
  EXPECT_EQ(batch.emg.front().captureTimestampNs, 2'000'000'000);
}

TEST(NeuralBandBatchPlayerTest, DeriveEmgPeriod_UniformCadence) {
  // 3 packets * 4us spacing * 8 sub-samples => 8_000_000 / (2*8) = 500_000 ns.
  const int64_t period = deriveBatchWideEmgPeriodNs({500'000, 504'000, 508'000}, 8);
  EXPECT_EQ(period, 500'000);
}

TEST(NeuralBandBatchPlayerTest, DeriveEmgPeriod_SinglePacketReturnsZero) {
  const int64_t period = deriveBatchWideEmgPeriodNs({500'000}, 8);
  EXPECT_EQ(period, 0);
}

TEST(NeuralBandBatchPlayerTest, DeriveEmgPeriod_EmptyInputReturnsZero) {
  const int64_t period = deriveBatchWideEmgPeriodNs({}, 8);
  EXPECT_EQ(period, 0);
}

TEST(NeuralBandBatchPlayerTest, DeriveEmgPeriod_NonMonotonicWireGivesNegativePeriod) {
  const int64_t period = deriveBatchWideEmgPeriodNs({500'000, 490'000, 480'000}, 8);
  EXPECT_LT(period, 0);
}

TEST(NeuralBandBatchPlayerTest, DeriveEmgPeriod_ZeroDeltaAcrossMultiplePacketsReturnsZero) {
  const int64_t period = deriveBatchWideEmgPeriodNs({500'000, 500'000, 500'000}, 8);
  EXPECT_EQ(period, 0);
}

TEST(NeuralBandBatchPlayerTest, DeriveEmgPeriod_ZeroTimeStepsPerPacketReturnsZero) {
  const int64_t period = deriveBatchWideEmgPeriodNs({500'000, 508'000}, 0);
  EXPECT_EQ(period, 0);
}

TEST(NeuralBandBatchPlayerTest, SynthesizeEmgSubSampleTimestamps_ThreePacketsEightStepsMonotonic) {
  NeuralBandBatch batch;
  batch.captureTimestampNs = 1'000'000'000;
  batch.emgChannelCount = 2;
  constexpr uint32_t timeSteps = 8;
  constexpr int64_t period = 500'000;
  // 500 us cadence * 8 sub-samples = 4 ms per packet spacing (post-rebase ns).
  const std::vector<int64_t> packetTsNs{992'000'000, 996'000'000, 1'000'000'000};
  for (const int64_t ts : packetTsNs) {
    NeuralBandEmgSample s;
    s.captureTimestampNs = ts;
    s.channelValues.assign(timeSteps * batch.emgChannelCount, 0);
    batch.emg.push_back(std::move(s));
  }

  synthesizeEmgSubSampleTimestamps(batch, timeSteps, period);

  EXPECT_EQ(batch.emg.size(), packetTsNs.size() * timeSteps);
  EXPECT_EQ(batch.emg[timeSteps - 1].captureTimestampNs, packetTsNs[0]);
  EXPECT_EQ(batch.emg[2 * timeSteps - 1].captureTimestampNs, packetTsNs[1]);
  EXPECT_EQ(batch.emg.back().captureTimestampNs, packetTsNs[2]);
  EXPECT_EQ(batch.emg.front().captureTimestampNs, packetTsNs[0] - 7 * period);
  for (size_t i = 1; i < batch.emg.size(); ++i) {
    EXPECT_LT(batch.emg[i - 1].captureTimestampNs, batch.emg[i].captureTimestampNs)
        << "sub-sample " << i << " not > sub-sample " << (i - 1);
  }
}

TEST(
    NeuralBandBatchPlayerTest,
    SynthesizeEmgSubSampleTimestamps_PeriodZeroCollapsesToStepFunction) {
  NeuralBandBatch batch;
  batch.captureTimestampNs = 1'000'000'000;
  batch.emgChannelCount = 1;
  constexpr uint32_t timeSteps = 4;
  const std::vector<int64_t> packetTsNs{500'000'000, 1'000'000'000};
  for (const int64_t ts : packetTsNs) {
    NeuralBandEmgSample s;
    s.captureTimestampNs = ts;
    s.channelValues.assign(timeSteps, 0);
    batch.emg.push_back(std::move(s));
  }

  synthesizeEmgSubSampleTimestamps(batch, timeSteps, 0);

  ASSERT_EQ(batch.emg.size(), packetTsNs.size() * timeSteps);
  for (uint32_t k = 0; k < timeSteps; ++k) {
    EXPECT_EQ(batch.emg[k].captureTimestampNs, packetTsNs[0]);
    EXPECT_EQ(batch.emg[timeSteps + k].captureTimestampNs, packetTsNs[1]);
  }
}

TEST(NeuralBandBatchPlayerTest, SynthesizeEmgSubSampleTimestamps_EmptyBatchIsNoOp) {
  NeuralBandBatch batch;
  batch.emgChannelCount = 2;
  synthesizeEmgSubSampleTimestamps(batch, 8, 500'000);
  EXPECT_TRUE(batch.emg.empty());
}

TEST(
    NeuralBandBatchPlayerTest,
    SynthesizeEmgSubSampleTimestamps_ExpandsPerPacketChannelValuesRowwise) {
  NeuralBandBatch batch;
  batch.captureTimestampNs = 1'000;
  batch.emgChannelCount = 2;
  constexpr uint32_t timeSteps = 3;
  NeuralBandEmgSample packet;
  packet.captureTimestampNs = 1'000;
  // Row-major [3 timesteps, 2 channels]: rows are {10,20}, {30,40}, {50,60}.
  packet.channelValues = {10, 20, 30, 40, 50, 60};
  batch.emg.push_back(std::move(packet));

  synthesizeEmgSubSampleTimestamps(batch, timeSteps, 0);

  ASSERT_EQ(batch.emg.size(), timeSteps);
  const std::vector<uint16_t> expectedRow0{10, 20};
  const std::vector<uint16_t> expectedRow1{30, 40};
  const std::vector<uint16_t> expectedRow2{50, 60};
  EXPECT_EQ(batch.emg[0].channelValues, expectedRow0);
  EXPECT_EQ(batch.emg[1].channelValues, expectedRow1);
  EXPECT_EQ(batch.emg[2].channelValues, expectedRow2);
}

TEST(NeuralBandBatchPlayerTest, DecodeAccel_WellFormedBlobProducesScaledXyz) {
  const std::vector<int64_t> tsUs{500'000};
  const std::vector<std::string> blobs{packImuBlobLittleEndian(100, -200, 300)};
  std::vector<NeuralBandAccelSample> out;

  decodeAccelSamples(tsUs, blobs, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].captureTimestampNs, 500'000'000);
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

  decodeAccelSamples(tsUs, blobs, out);

  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].captureTimestampNs, 100'000);
  EXPECT_EQ(out[1].captureTimestampNs, 300'000);
}

TEST(NeuralBandBatchPlayerTest, DecodeAccel_BoundaryInt16Values) {
  const std::vector<int64_t> tsUs{0};
  const std::vector<std::string> blobs{packImuBlobLittleEndian(INT16_MIN, INT16_MAX, 0)};
  std::vector<NeuralBandAccelSample> out;

  decodeAccelSamples(tsUs, blobs, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].accelMSec2[0], static_cast<float>(INT16_MIN) * kAccelLsbToMSec2);
  EXPECT_FLOAT_EQ(out[0].accelMSec2[1], static_cast<float>(INT16_MAX) * kAccelLsbToMSec2);
  EXPECT_FLOAT_EQ(out[0].accelMSec2[2], 0.0f);
}

TEST(NeuralBandBatchPlayerTest, DecodeGyro_WellFormedBlobProducesScaledXyz) {
  const std::vector<int64_t> tsUs{700'000};
  const std::vector<std::string> blobs{packImuBlobLittleEndian(50, -75, 125)};
  std::vector<NeuralBandGyroSample> out;

  decodeGyroSamples(tsUs, blobs, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].captureTimestampNs, 700'000'000);
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

  decodeGyroSamples(tsUs, blobs, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].captureTimestampNs, 20'000);
}

// Full-pipeline tests write a synthetic VRS fixture and open it via
// createVrsDataProvider — they exercise the wiring only reachable when a
// record traverses the whole reader path.
namespace {

// Must match the factory's dispatch flavor tag.
constexpr const char* kNeuralBandBatchFlavorForTest = "device/oatmeal/emg_imu_batch";

struct NeuralBandBatchFixtureSpec {
  int64_t captureNs{};
  uint32_t batchSequence{};
  uint32_t channelCount{};
  uint32_t bitsPerAdcReading{};
  uint32_t timeStepsPerPacket{};
  std::vector<uint64_t> emgTimestampsUs;
  std::vector<std::string> emgBlobs;
  std::vector<uint32_t> emgEncodings;
  std::vector<uint32_t> emgSequenceNumbers;
  std::vector<uint64_t> accelTimestampsUs;
  std::vector<std::string> accelBlobs;
  std::vector<uint32_t> accelSequenceNumbers;
  std::vector<uint64_t> gyroTimestampsUs;
  std::vector<std::string> gyroBlobs;
  std::vector<uint32_t> gyroSequenceNumbers;
};

// `spec` is captured by reference — must outlive the recordable.
class NeuralBandBatchRecordable : public vrs::Recordable {
 public:
  explicit NeuralBandBatchRecordable(const NeuralBandBatchFixtureSpec& spec)
      : vrs::Recordable(vrs::RecordableTypeId::ImuRecordableClass, kNeuralBandBatchFlavorForTest),
        spec_(spec) {
    addRecordFormat(vrs::Record::Type::CONFIGURATION, 1, config_.getContentBlock(), {&config_});
    addRecordFormat(vrs::Record::Type::DATA, 1, data_.getContentBlock(), {&data_});
  }

  const vrs::Record* createConfigurationRecord() override {
    config_.streamId.set(0);
    config_.sensorModel.stage("wristband-test");
    config_.deviceId.set(0xC0FFEEULL);
    config_.nominalRateHz.set(1000.0);
    config_.description.stage("");
    return createRecord(0.0, vrs::Record::Type::CONFIGURATION, 1, vrs::DataSource(config_));
  }

  const vrs::Record* createStateRecord() override {
    return createRecord(0.0, vrs::Record::Type::STATE, 1);
  }

  const vrs::Record* writeDataRecord() {
    data_.captureTimestampNs.set(spec_.captureNs);
    data_.batchSequenceNumber.set(spec_.batchSequence);
    data_.emgSampleCount.set(static_cast<uint32_t>(spec_.emgTimestampsUs.size()));
    data_.accelSampleCount.set(static_cast<uint32_t>(spec_.accelTimestampsUs.size()));
    data_.gyroSampleCount.set(static_cast<uint32_t>(spec_.gyroTimestampsUs.size()));
    data_.channelCount.set(spec_.channelCount);
    data_.bitsPerAdcReading.set(spec_.bitsPerAdcReading);
    data_.samplesPerBatch.set(spec_.timeStepsPerPacket);
    if (!spec_.emgTimestampsUs.empty()) {
      data_.emgTimestampsNs.stage(spec_.emgTimestampsUs);
      data_.emgChannels.stage(spec_.emgBlobs);
      data_.emgEncodings.stage(spec_.emgEncodings);
      data_.emgSequenceNumbers.stage(spec_.emgSequenceNumbers);
    }
    if (!spec_.accelTimestampsUs.empty()) {
      data_.accelTimestampsNs.stage(spec_.accelTimestampsUs);
      data_.accelChannels.stage(spec_.accelBlobs);
      data_.accelSequenceNumbers.stage(spec_.accelSequenceNumbers);
    }
    if (!spec_.gyroTimestampsUs.empty()) {
      data_.gyroTimestampsNs.stage(spec_.gyroTimestampsUs);
      data_.gyroChannels.stage(spec_.gyroBlobs);
      data_.gyroSequenceNumbers.stage(spec_.gyroSequenceNumbers);
    }
    return createRecord(1.0, vrs::Record::Type::DATA, 1, vrs::DataSource(data_));
  }

 private:
  const NeuralBandBatchFixtureSpec& spec_;
  ::datalayout::NeuralBandBatchConfigurationLayout config_;
  ::datalayout::NeuralBandBatchDataLayout data_;
};

// Satisfies factory's "at least one image or IMU player" invariant.
class PlaceholderImuRecordable : public vrs::Recordable {
 public:
  PlaceholderImuRecordable()
      : vrs::Recordable(vrs::RecordableTypeId::SlamImuData, "device/oatmeal") {
    addRecordFormat(vrs::Record::Type::CONFIGURATION, 1, config_.getContentBlock(), {&config_});
    addRecordFormat(vrs::Record::Type::DATA, 1, data_.getContentBlock(), {&data_});
  }

  const vrs::Record* createConfigurationRecord() override {
    return createRecord(0.0, vrs::Record::Type::CONFIGURATION, 1, vrs::DataSource(config_));
  }

  const vrs::Record* createStateRecord() override {
    return createRecord(0.0, vrs::Record::Type::STATE, 1);
  }

  // No writeDataRecord() — factory only needs the stream to exist.

 private:
  ::datalayout::MotionSensorConfigRecordMetadata config_;
  ::datalayout::MotionSensorDataRecordMetadata data_;
};

// RAII so an early ASSERT can't leak the temp file.
struct TempFileGuard {
  std::string path;
  ~TempFileGuard() {
    if (!path.empty()) {
      vrs::os::remove(path);
    }
  }
};

void writeSyntheticNeuralBandBatchVrs(
    const std::string& path,
    const NeuralBandBatchFixtureSpec& spec) {
  vrs::RecordFileWriter writer;
  // Tags loadDeviceVersion() as Gen2 so the "emg" label is present.
  writer.setTag("device_type", "Oatmeal");

  NeuralBandBatchRecordable emg(spec);
  PlaceholderImuRecordable imu;
  writer.addRecordable(&emg);
  writer.addRecordable(&imu);

  emg.createConfigurationRecord();
  emg.createStateRecord();
  ASSERT_NE(emg.writeDataRecord(), nullptr);

  imu.createConfigurationRecord();
  imu.createStateRecord();

  ASSERT_EQ(writer.writeToFile(path), 0);
}

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

  EXPECT_EQ(batch.captureTimestampNs, kFixtureBatchCaptureNs);
  EXPECT_EQ(batch.batchSequenceNumber, kFixtureBatchSeq);
  EXPECT_EQ(batch.emgChannelCount, kFixtureChannelCount);
  EXPECT_EQ(batch.emgBitsPerAdcReading, kFixtureBitsPerAdcReading);

  ASSERT_EQ(batch.emg.size(), kFixtureNumEmgPackets * kFixtureTimeStepsPerPacket);
  EXPECT_EQ(batch.emg[0].channelValues.size(), batch.emgChannelCount);

  const std::vector<uint16_t> expectedFirstRow{encodeEmgCount(0, 0, 0), encodeEmgCount(0, 0, 1)};
  EXPECT_EQ(batch.emg.front().channelValues, expectedFirstRow);

  const std::vector<uint16_t> expectedLastRow{encodeEmgCount(2, 3, 0), encodeEmgCount(2, 3, 1)};
  EXPECT_EQ(batch.emg.back().channelValues, expectedLastRow);

  // See kFixtureBatchCaptureNs comment for the period math.
  EXPECT_EQ(batch.emg.back().captureTimestampNs, kFixtureBatchCaptureNs);
  constexpr int64_t kExpectedPeriodNs = 25'000'000;
  constexpr int64_t kExpectedPacket0AnchorNs = 100'000'000;
  EXPECT_EQ(
      batch.emg.front().captureTimestampNs,
      kExpectedPacket0AnchorNs -
          static_cast<int64_t>(kFixtureTimeStepsPerPacket - 1) * kExpectedPeriodNs);

  EXPECT_TRUE(batch.accel.empty());
  EXPECT_TRUE(batch.gyro.empty());
}

// Guards against a regression that would move period derivation to after
// decodeEmgSamples (using only surviving packets rather than all wire).
TEST(NeuralBandBatchPlayerTest, Integration_MalformedEmgPacketSkippedButPeriodDerivedFromAllWire) {
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

  // Post-rebase: packet-2 anchor => kBatchCaptureNs; packet-0 => kBatchCaptureNs - 200_000 us.
  constexpr int64_t kAllWirePeriodNs = 50'000'000;
  constexpr int64_t kPacket0AnchorNs = kBatchCaptureNs - 200'000'000;
  EXPECT_EQ(batch.emg[0].captureTimestampNs, kPacket0AnchorNs - kAllWirePeriodNs);
  EXPECT_EQ(batch.emg[1].captureTimestampNs, kPacket0AnchorNs);
  EXPECT_EQ(batch.emg[2].captureTimestampNs, kBatchCaptureNs - kAllWirePeriodNs);
  EXPECT_EQ(batch.emg[3].captureTimestampNs, kBatchCaptureNs);

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
