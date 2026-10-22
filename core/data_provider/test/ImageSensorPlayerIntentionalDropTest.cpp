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

#include <data_provider/players/ImageSensorPlayer.h>

#include <vrs/DataLayout.h>
#include <vrs/DataPieces.h>
#include <vrs/DataSource.h>
#include <vrs/RecordFileReader.h>
#include <vrs/RecordFileWriter.h>
#include <vrs/Recordable.h>
#include <vrs/os/Utils.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

using projectaria::tools::data_provider::ImageDataRecord;
using projectaria::tools::data_provider::ImageSensorPlayer;

namespace {

struct TempFileGuard {
  std::string path;
  ~TempFileGuard() {
    if (!path.empty()) {
      vrs::os::remove(path);
    }
  }
};

// Image data record layout of a recording that does not report withheld frames
class LayoutWithoutIntentionalDropCount : public vrs::AutoDataLayout {
 public:
  vrs::DataPieceValue<std::uint64_t> frameNumber{"frame_number"};
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};
  vrs::AutoDataLayoutEnd end;
};

// Image data record layout of a recording that reports withheld frames
class LayoutWithIntentionalDropCount : public vrs::AutoDataLayout {
 public:
  vrs::DataPieceValue<std::uint64_t> frameNumber{"frame_number"};
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};
  vrs::DataPieceValue<std::uint16_t> intentionalDropCount{"intentional_drop_count"};
  vrs::AutoDataLayoutEnd end;
};

template <class DataLayoutT>
class ImageStreamRecordable : public vrs::Recordable {
 public:
  ImageStreamRecordable()
      : vrs::Recordable(vrs::RecordableTypeId::RgbCameraRecordableClass, "test/image") {
    addRecordFormat(vrs::Record::Type::DATA, 1, data_.getContentBlock(), {&data_});
  }

  const vrs::Record* createConfigurationRecord() override {
    return createRecord(0.0, vrs::Record::Type::CONFIGURATION, 1);
  }

  const vrs::Record* createStateRecord() override {
    return createRecord(0.0, vrs::Record::Type::STATE, 1);
  }

  DataLayoutT& data() {
    return data_;
  }

  void writeDataRecord(uint64_t frameNumber) {
    data_.frameNumber.set(frameNumber);
    data_.captureTimestampNs.set(static_cast<int64_t>(frameNumber) * 1'000'000);
    createRecord(1.0 + frameNumber, vrs::Record::Type::DATA, 1, vrs::DataSource(data_));
  }

 private:
  DataLayoutT data_;
};

template <class DataLayoutT>
std::string writeRecords(ImageStreamRecordable<DataLayoutT>& recordable) {
  const std::string path = vrs::os::getUniquePath(
      vrs::os::getTempFolder() + "image_sensor_player_intentional_drop_test");
  vrs::RecordFileWriter writer;
  writer.addRecordable(&recordable);
  EXPECT_EQ(writer.writeToFile(path), 0);
  return path;
}

// Writes one data record per count, each reporting that many withheld frames
std::string writeRecordsWithIntentionalDropCounts(const std::vector<uint16_t>& counts) {
  ImageStreamRecordable<LayoutWithIntentionalDropCount> recordable;
  recordable.createConfigurationRecord();
  recordable.createStateRecord();
  for (size_t index = 0; index < counts.size(); ++index) {
    recordable.data().intentionalDropCount.set(counts[index]);
    recordable.writeDataRecord(index);
  }
  return writeRecords(recordable);
}

std::string writeRecordWithoutIntentionalDropCount() {
  ImageStreamRecordable<LayoutWithoutIntentionalDropCount> recordable;
  recordable.createConfigurationRecord();
  recordable.createStateRecord();
  recordable.writeDataRecord(0);
  return writeRecords(recordable);
}

// Reads every data record of the file's only stream with ImageSensorPlayer
std::vector<ImageDataRecord> readDataRecords(const std::string& path) {
  vrs::RecordFileReader reader;
  EXPECT_EQ(reader.openFile(path), 0);
  const auto streamIds = reader.getStreams();
  EXPECT_EQ(streamIds.size(), 1u);
  std::vector<ImageDataRecord> records;
  if (streamIds.empty()) {
    return records;
  }
  const vrs::StreamId streamId = *streamIds.begin();
  ImageSensorPlayer player(streamId);
  reader.setStreamPlayer(streamId, &player);
  for (const auto* record : reader.getIndex(streamId)) {
    if (record->recordType == vrs::Record::Type::DATA) {
      EXPECT_EQ(reader.readRecord(*record), 0);
      records.push_back(player.getDataRecord());
    }
  }
  reader.closeFile();
  return records;
}

} // namespace

// A recorded 0 means no frame was withheld, which differs from a recording that does not report
TEST(ImageSensorPlayerIntentionalDropCount, FollowsTheCountOfEachRecord) {
  const TempFileGuard file{writeRecordsWithIntentionalDropCounts({3, 0})};

  const std::vector<ImageDataRecord> records = readDataRecords(file.path);

  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].intentionalDropCount, std::optional<uint16_t>(3));
  EXPECT_EQ(records[1].intentionalDropCount, std::optional<uint16_t>(0));
}

TEST(ImageSensorPlayerIntentionalDropCount, IsNulloptWhenTheRecordsDoNotHaveTheField) {
  const TempFileGuard file{writeRecordWithoutIntentionalDropCount()};

  const std::vector<ImageDataRecord> records = readDataRecords(file.path);

  ASSERT_EQ(records.size(), 1u);
  EXPECT_EQ(records[0].frameNumber, 0u);
  EXPECT_EQ(records[0].intentionalDropCount, std::nullopt);
}
