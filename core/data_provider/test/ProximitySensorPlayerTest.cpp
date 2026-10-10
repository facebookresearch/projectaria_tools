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

#include <data_layout/ProximitySensorMetadata.h>
#include <data_provider/players/ProximitySensorPlayer.h>

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

using projectaria::tools::data_provider::ProximitySensorConfiguration;
using projectaria::tools::data_provider::ProximitySensorData;
using projectaria::tools::data_provider::ProximitySensorPlayer;

struct TempFileGuard {
  std::string path;
  ~TempFileGuard() {
    if (!path.empty()) {
      vrs::os::remove(path);
    }
  }
};

class ProximitySensorRecordable : public vrs::Recordable {
 public:
  ProximitySensorRecordable()
      : vrs::Recordable(vrs::RecordableTypeId::ProximitySensorRecordableClass, "test") {
    addRecordFormat(
        vrs::Record::Type::CONFIGURATION,
        datalayout::ProximitySensorConfigurationLayout::kVersion,
        configLayout_.getContentBlock(),
        {&configLayout_});
    addRecordFormat(
        vrs::Record::Type::DATA,
        datalayout::ProximitySensorDataLayout::kVersion,
        dataLayout_.getContentBlock(),
        {&dataLayout_});
  }

  const vrs::Record* createConfigurationRecord() override {
    configLayout_.streamId.set(7);
    return createRecord(
        0.0,
        vrs::Record::Type::CONFIGURATION,
        datalayout::ProximitySensorConfigurationLayout::kVersion,
        vrs::DataSource(configLayout_));
  }

  const vrs::Record* createStateRecord() override {
    return createRecord(0.0, vrs::Record::Type::STATE, 1);
  }

  void createDataRecord(double recordTimeSec, int64_t captureTimestampNs, std::string value) {
    dataLayout_.captureTimestampNs.set(captureTimestampNs);
    dataLayout_.proximityValue.stage(std::move(value));
    createRecord(
        recordTimeSec,
        vrs::Record::Type::DATA,
        datalayout::ProximitySensorDataLayout::kVersion,
        vrs::DataSource(dataLayout_));
  }

 private:
  datalayout::ProximitySensorConfigurationLayout configLayout_;
  datalayout::ProximitySensorDataLayout dataLayout_;
};

TEST(ProximitySensorPlayerTest, ReadsConfigurationAndData) {
  const std::string path =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "proximity_sensor_player_test");
  TempFileGuard guard{path};

  vrs::RecordFileWriter writer;
  ProximitySensorRecordable recordable;
  writer.addRecordable(&recordable);
  recordable.createConfigurationRecord();
  recordable.createStateRecord();
  recordable.createDataRecord(1.0, 1'000'000'000, "near");
  recordable.createDataRecord(2.0, 2'000'000'000, "invalid");
  ASSERT_EQ(writer.writeToFile(path), 0);

  vrs::RecordFileReader reader;
  ASSERT_EQ(reader.openFile(path), 0);
  const auto streams = reader.getStreams();
  ASSERT_EQ(streams.size(), 1u);

  const vrs::StreamId streamId = *streams.begin();
  ProximitySensorPlayer player{streamId};
  std::vector<ProximitySensorData> samples;
  std::vector<ProximitySensorConfiguration> configurations;
  player.setCallback(
      [&samples, &configurations](
          const ProximitySensorData& data, const ProximitySensorConfiguration& config, bool) {
        samples.push_back(data);
        configurations.push_back(config);
        return true;
      });
  reader.setStreamPlayer(streamId, &player);
  ASSERT_EQ(reader.readAllRecords(), 0);
  reader.closeFile();

  ASSERT_EQ(samples.size(), 2u);
  EXPECT_EQ(samples[0].captureTimestampNs, 1'000'000'000);
  EXPECT_EQ(samples[0].proximityValue, "near");
  EXPECT_EQ(samples[1].captureTimestampNs, 2'000'000'000);
  EXPECT_EQ(samples[1].proximityValue, "invalid");
  ASSERT_EQ(configurations.size(), 2u);
  EXPECT_EQ(configurations[0].streamId, 7u);
  EXPECT_EQ(configurations[1].streamId, 7u);
  EXPECT_GT(player.getNextTimestampSec(), 2.0);
}

} // namespace
