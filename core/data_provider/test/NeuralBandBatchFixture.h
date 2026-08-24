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

#include <data_provider/data_layout/MotionSensorMetadata.h>
#include <data_provider/data_layout/NeuralBandBatchMetadata.h>

#include <vrs/DataSource.h>
#include <vrs/RecordFileWriter.h>
#include <vrs/RecordFormat.h>
#include <vrs/Recordable.h>
#include <vrs/os/Utils.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

/// Writes synthetic Neural Band VRS files, so the reader path can be exercised
/// end to end without a device recording.
namespace projectaria::tools::data_provider::test {

/// Must match the factory's dispatch flavor tag.
inline constexpr const char* kNeuralBandBatchFlavorForTest = "device/oatmeal/emg_imu_batch";

/// One data record's worth of wire content.
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
  std::string emgCalibrationParamsJson;
};

class NeuralBandBatchRecordable : public vrs::Recordable {
 public:
  NeuralBandBatchRecordable()
      : vrs::Recordable(vrs::RecordableTypeId::ImuRecordableClass, kNeuralBandBatchFlavorForTest) {
    addRecordFormat(vrs::Record::Type::CONFIGURATION, 1, config_.getContentBlock(), {&config_});
    addRecordFormat(vrs::Record::Type::DATA, 1, data_.getContentBlock(), {&data_});
  }

  const vrs::Record* createConfigurationRecord() override {
    config_.streamId.set(0);
    config_.sensorModel.stage("wristband-test");
    config_.deviceId.set(0xC0FFEEULL);
    config_.nominalRateHz.set(1000.0);
    config_.description.stage("");
    config_.emgCalibrationParamsJson.stage(calibrationParamsJson_);
    return createRecord(0.0, vrs::Record::Type::CONFIGURATION, 1, vrs::DataSource(config_));
  }

  const vrs::Record* createStateRecord() override {
    return createRecord(0.0, vrs::Record::Type::STATE, 1);
  }

  void setCalibrationParamsJson(const std::string& json) {
    calibrationParamsJson_ = json;
  }

  const vrs::Record* writeDataRecord(const NeuralBandBatchFixtureSpec& spec, double recordTimeSec) {
    data_.captureTimestampNs.set(spec.captureNs);
    data_.batchSequenceNumber.set(spec.batchSequence);
    data_.emgSampleCount.set(static_cast<uint32_t>(spec.emgTimestampsUs.size()));
    data_.accelSampleCount.set(static_cast<uint32_t>(spec.accelTimestampsUs.size()));
    data_.gyroSampleCount.set(static_cast<uint32_t>(spec.gyroTimestampsUs.size()));
    data_.channelCount.set(spec.channelCount);
    data_.bitsPerAdcReading.set(spec.bitsPerAdcReading);
    data_.samplesPerBatch.set(spec.timeStepsPerPacket);
    if (!spec.emgTimestampsUs.empty()) {
      data_.emgTimestampsNs.stage(spec.emgTimestampsUs);
      data_.emgChannels.stage(spec.emgBlobs);
      data_.emgEncodings.stage(spec.emgEncodings);
      data_.emgSequenceNumbers.stage(spec.emgSequenceNumbers);
    }
    if (!spec.accelTimestampsUs.empty()) {
      data_.accelTimestampsNs.stage(spec.accelTimestampsUs);
      data_.accelChannels.stage(spec.accelBlobs);
      data_.accelSequenceNumbers.stage(spec.accelSequenceNumbers);
    }
    if (!spec.gyroTimestampsUs.empty()) {
      data_.gyroTimestampsNs.stage(spec.gyroTimestampsUs);
      data_.gyroChannels.stage(spec.gyroBlobs);
      data_.gyroSequenceNumbers.stage(spec.gyroSequenceNumbers);
    }
    return createRecord(recordTimeSec, vrs::Record::Type::DATA, 1, vrs::DataSource(data_));
  }

 private:
  std::string calibrationParamsJson_;
  ::datalayout::NeuralBandBatchConfigurationLayout config_;
  ::datalayout::NeuralBandBatchDataLayout data_;
};

/// Satisfies the factory's "at least one image or IMU player" invariant.
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

 private:
  ::datalayout::MotionSensorConfigRecordMetadata config_;
  ::datalayout::MotionSensorDataRecordMetadata data_;
};

/// RAII so an early ASSERT cannot leak the temp file.
struct TempFileGuard {
  std::string path;
  ~TempFileGuard() {
    if (!path.empty()) {
      vrs::os::remove(path);
    }
  }
};

/// VRS needs record timestamps increasing; nothing under test reads them.
inline constexpr double kFixtureRecordSpacingSec = 0.01;

inline void writeSyntheticNeuralBandBatchVrs(
    const std::string& path,
    const std::vector<NeuralBandBatchFixtureSpec>& specs) {
  vrs::RecordFileWriter writer;
  // Tags loadDeviceVersion() as Gen2 so the "emg" label is present.
  writer.setTag("device_type", "Oatmeal");

  NeuralBandBatchRecordable emg;
  PlaceholderImuRecordable imu;
  if (!specs.empty()) {
    emg.setCalibrationParamsJson(specs.front().emgCalibrationParamsJson);
  }
  writer.addRecordable(&emg);
  writer.addRecordable(&imu);

  emg.createConfigurationRecord();
  emg.createStateRecord();
  for (size_t index = 0; index < specs.size(); ++index) {
    ASSERT_NE(
        emg.writeDataRecord(
            specs[index], static_cast<double>(index + 1) * kFixtureRecordSpacingSec),
        nullptr);
  }

  imu.createConfigurationRecord();
  imu.createStateRecord();

  ASSERT_EQ(writer.writeToFile(path), 0);
}

inline void writeSyntheticNeuralBandBatchVrs(
    const std::string& path,
    const NeuralBandBatchFixtureSpec& spec) {
  writeSyntheticNeuralBandBatchVrs(path, std::vector<NeuralBandBatchFixtureSpec>{spec});
}

} // namespace projectaria::tools::data_provider::test
