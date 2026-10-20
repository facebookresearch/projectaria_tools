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

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <vrs/DataSource.h>
#include <vrs/RecordFileReader.h>
#include <vrs/RecordFileWriter.h>
#include <vrs/Recordable.h>
#include <vrs/os/Utils.h>

namespace {

using projectaria::tools::data_provider::AlsFlickerDetectionConfiguration;
using projectaria::tools::data_provider::AlsFlickerDetectionData;
using projectaria::tools::data_provider::AlsFlickerDetectionPlayer;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
const std::array<std::string, 8> kRequiredFields = {
    "capture_timestamp_ns",
    "flicker_frequency_hz_0",
    "flicker_magnitude_0",
    "flicker_frequency_hz_1",
    "flicker_magnitude_1",
    "flicker_frequency_hz_2",
    "flicker_magnitude_2",
    "nominal_rate"};

// Test-owned layouts leave fields out of the written record, so the test doesn't depend on an
// internal writer.
class FlickerDataLayout : public vrs::ManualDataLayout {
 public:
  explicit FlickerDataLayout(const std::string& omitted) {
    if (omitted != kRequiredFields[0]) {
      auto piece = std::make_unique<vrs::DataPieceValue<int64_t>>(kRequiredFields[0]);
      timestamp = piece.get();
      add(std::move(piece));
    }
    for (size_t i = 0; i < values.size(); ++i) {
      if (omitted != kRequiredFields[i + 1]) {
        auto piece = std::make_unique<vrs::DataPieceValue<float>>(kRequiredFields[i + 1]);
        values[i] = piece.get();
        add(std::move(piece));
      }
    }
    endLayout();
  }

  void set(const AlsFlickerDetectionData& sample) {
    if (timestamp != nullptr) {
      timestamp->set(sample.captureTimestampNs);
    }
    for (size_t i = 0; i < sample.flickerData.size(); ++i) {
      if (values[2 * i] != nullptr) {
        values[2 * i]->set(sample.flickerData[i].frequencyHz);
      }
      if (values[2 * i + 1] != nullptr) {
        values[2 * i + 1]->set(sample.flickerData[i].magnitude);
      }
    }
  }

 private:
  vrs::DataPieceValue<int64_t>* timestamp = nullptr;
  std::array<vrs::DataPieceValue<float>*, 6> values{};
};

class FlickerConfigLayout : public vrs::ManualDataLayout {
 public:
  explicit FlickerConfigLayout(bool omitRate) {
    add(std::make_unique<vrs::DataPieceValue<uint32_t>>("stream_id"));
    if (!omitRate) {
      add(std::make_unique<vrs::DataPieceValue<double>>("nominal_rate"));
    }
    endLayout();
    findDataPieceValue<uint32_t>("stream_id")->set(7);
    if (!omitRate) {
      findDataPieceValue<double>("nominal_rate")->set(10.0);
    }
  }
};

class FlickerRecordable : public vrs::Recordable {
 public:
  explicit FlickerRecordable(const std::string& omitted)
      : vrs::Recordable(vrs::RecordableTypeId::AlsFlickerDetectionRecordableClass, "test"),
        config_(false),
        missingRateConfig_(true),
        data_(""),
        missingFieldData_(omitted) {
    addRecordFormat(vrs::Record::Type::CONFIGURATION, 1, config_.getContentBlock(), {&config_});
    addRecordFormat(
        vrs::Record::Type::CONFIGURATION,
        2,
        missingRateConfig_.getContentBlock(),
        {&missingRateConfig_});
    addRecordFormat(vrs::Record::Type::DATA, 1, data_.getContentBlock(), {&data_});
    addRecordFormat(
        vrs::Record::Type::DATA, 2, missingFieldData_.getContentBlock(), {&missingFieldData_});
  }

  const vrs::Record* createConfigurationRecord() override {
    return writeConfiguration(0.0, false);
  }

  const vrs::Record* createStateRecord() override {
    return createRecord(0.0, vrs::Record::Type::STATE, 1);
  }

  const vrs::Record* writeConfiguration(double time, bool missingRate) {
    auto& layout = missingRate ? missingRateConfig_ : config_;
    return createRecord(
        time, vrs::Record::Type::CONFIGURATION, missingRate ? 2 : 1, vrs::DataSource(layout));
  }

  void writeData(double time, const AlsFlickerDetectionData& sample, bool missingField) {
    auto& layout = missingField ? missingFieldData_ : data_;
    layout.set(sample);
    createRecord(time, vrs::Record::Type::DATA, missingField ? 2 : 1, vrs::DataSource(layout));
  }

 private:
  FlickerConfigLayout config_;
  FlickerConfigLayout missingRateConfig_;
  FlickerDataLayout data_;
  FlickerDataLayout missingFieldData_;
};

class AlsFlickerDetectionPlayerTest : public testing::Test {
 protected:
  void TearDown() override {
    if (!path_.empty()) {
      vrs::os::remove(path_);
    }
  }

  void read(
      const AlsFlickerDetectionData& sample,
      const std::string& omitted = "",
      bool validFirst = false) {
    path_ = vrs::os::getUniquePath(vrs::os::getTempFolder() + "als_flicker_player_test");
    vrs::RecordFileWriter writer;
    FlickerRecordable recordable{omitted};
    writer.addRecordable(&recordable);
    recordable.writeConfiguration(0.0, !validFirst && omitted == "nominal_rate");
    recordable.createStateRecord();
    if (validFirst) {
      recordable.writeData(1.0, sample, false);
      recordable.writeConfiguration(2.0, omitted == "nominal_rate");
    }
    recordable.writeData(3.0, sample, !omitted.empty());
    ASSERT_EQ(writer.writeToFile(path_), 0);

    vrs::RecordFileReader reader;
    ASSERT_EQ(reader.openFile(path_), 0);
    const auto streams = reader.getStreams();
    ASSERT_EQ(streams.size(), 1u);
    const auto stream = *streams.begin();
    AlsFlickerDetectionPlayer player{stream};
    player.setVerbose(true);
    player.setCallback([this](const auto& data, const auto& config, bool verbose) {
      samples_.push_back(data);
      configurations_.push_back(config);
      EXPECT_TRUE(verbose);
      return true;
    });
    reader.setStreamPlayer(stream, &player);
    ASSERT_EQ(reader.readAllRecords(), 0);
    reader.closeFile();
  }

  static void expectSample(
      const AlsFlickerDetectionData& actual,
      const AlsFlickerDetectionData& expected) {
    EXPECT_EQ(actual.captureTimestampNs, expected.captureTimestampNs);
    for (size_t i = 0; i < expected.flickerData.size(); ++i) {
      SCOPED_TRACE(i);
      if (std::isnan(expected.flickerData[i].frequencyHz)) {
        EXPECT_TRUE(std::isnan(actual.flickerData[i].frequencyHz));
      } else {
        EXPECT_FLOAT_EQ(actual.flickerData[i].frequencyHz, expected.flickerData[i].frequencyHz);
      }
      EXPECT_FLOAT_EQ(actual.flickerData[i].magnitude, expected.flickerData[i].magnitude);
    }
  }

  std::vector<AlsFlickerDetectionData> samples_;
  std::vector<AlsFlickerDetectionConfiguration> configurations_;

 private:
  std::string path_;
};

class MissingFlickerFieldTest : public AlsFlickerDetectionPlayerTest,
                                public testing::WithParamInterface<std::string> {
 protected:
  void checkMissing(bool validFirst) {
    const AlsFlickerDetectionData expected{
        .captureTimestampNs = 42,
        .flickerData = {{
            {.frequencyHz = 50, .magnitude = 0},
            {.frequencyHz = 100, .magnitude = 0},
            {.frequencyHz = 150, .magnitude = 0},
        }}};
    read(expected, GetParam(), validFirst);
    ASSERT_EQ(samples_.size(), validFirst ? 2u : 1u);
    ASSERT_EQ(configurations_.size(), samples_.size());
    if (validFirst) {
      expectSample(samples_.front(), expected);
      EXPECT_DOUBLE_EQ(configurations_.front().nominalRateHz, 10.0);
    }
    const auto& actual = samples_.back();
    EXPECT_EQ(actual.captureTimestampNs, GetParam() == "capture_timestamp_ns" ? -1 : 42);
    for (size_t i = 0; i < actual.flickerData.size(); ++i) {
      SCOPED_TRACE(i);
      if (GetParam() == kRequiredFields[2 * i + 1] || GetParam() == kRequiredFields[2 * i + 2]) {
        // A missing frequency or magnitude must not decode as a legal detection or an unused slot.
        EXPECT_TRUE(std::isnan(actual.flickerData[i].magnitude));
      } else {
        EXPECT_FLOAT_EQ(actual.flickerData[i].frequencyHz, expected.flickerData[i].frequencyHz);
        EXPECT_FLOAT_EQ(actual.flickerData[i].magnitude, expected.flickerData[i].magnitude);
      }
    }
    if (GetParam() == "nominal_rate") {
      EXPECT_TRUE(std::isnan(configurations_.back().nominalRateHz));
    } else {
      EXPECT_DOUBLE_EQ(configurations_.back().nominalRateHz, 10.0);
    }
  }
};

TEST_P(MissingFlickerFieldTest, MissingFieldIsInvalid) {
  checkMissing(false);
}

TEST_P(MissingFlickerFieldTest, ValidThenMissingFieldDoesNotKeepPreviousValue) {
  checkMissing(true);
}

INSTANTIATE_TEST_SUITE_P(
    RequiredScalars,
    MissingFlickerFieldTest,
    testing::ValuesIn(kRequiredFields),
    [](const testing::TestParamInfo<std::string>& info) { return info.param; });

TEST_F(AlsFlickerDetectionPlayerTest, ExplicitZerosAndInformationalConfigOmissionsSurviveDecode) {
  const AlsFlickerDetectionData expected{};
  read(expected);
  ASSERT_EQ(samples_.size(), 1u);
  expectSample(samples_.front(), expected);
  ASSERT_EQ(configurations_.size(), 1u);
  EXPECT_EQ(configurations_.front().streamId, 7u);
  EXPECT_DOUBLE_EQ(configurations_.front().nominalRateHz, 10.0);
  EXPECT_EQ(configurations_.front().deviceId, 0u);
  EXPECT_TRUE(configurations_.front().sensorModel.empty());
}

TEST_F(AlsFlickerDetectionPlayerTest, DcDetectionAndUnusedSlotSurviveDecode) {
  const AlsFlickerDetectionData expected{
      .captureTimestampNs = 123,
      .flickerData = {{
          {.frequencyHz = 0, .magnitude = 2},
          {.frequencyHz = kNaN, .magnitude = 0},
          {.frequencyHz = 120, .magnitude = 3},
      }}};
  read(expected);
  ASSERT_EQ(samples_.size(), 1u);
  expectSample(samples_.front(), expected);
}

TEST_F(AlsFlickerDetectionPlayerTest, AllUnusedSlotsSurviveDecode) {
  const AlsFlickerDetectionData expected{
      .captureTimestampNs = 0,
      .flickerData = {{
          {.frequencyHz = kNaN, .magnitude = 0},
          {.frequencyHz = kNaN, .magnitude = 0},
          {.frequencyHz = kNaN, .magnitude = 0},
      }}};
  read(expected);
  ASSERT_EQ(samples_.size(), 1u);
  expectSample(samples_.front(), expected);
}

} // namespace
