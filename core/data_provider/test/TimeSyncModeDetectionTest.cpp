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

#include <data_provider/RecordReaderInterface.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::data_provider;
using projectaria::tools::calibration::DeviceVersion;

namespace {

using PlayerMap = std::map<TimeSyncMode, std::shared_ptr<TimeSyncPlayer>>;
using TagMap = std::map<std::string, std::string>;

/// A TimeSync stream, identified only by mode: detection looks at which modes are
/// present, never at the records behind them.
PlayerMap players(const std::vector<TimeSyncMode>& modes) {
  PlayerMap map;
  uint16_t instanceId = 1;
  for (const TimeSyncMode mode : modes) {
    map.emplace(
        mode,
        std::make_shared<TimeSyncPlayer>(
            vrs::StreamId(vrs::RecordableTypeId::TimeRecordableClass, instanceId++)));
  }
  return map;
}

/// Gen 2 file metadata. `subghzMode` empty means the field is absent, which is what
/// recordings made before the OS added it look like.
TagMap gen2Tags(const std::string& subghzMode = "") {
  const std::string recording = subghzMode.empty()
      ? R"({"profile":"profile8"})"
      : R"({"profile":"profile8","subghz_mode":")" + subghzMode + R"("})";
  return {{"metadata", R"({"version":1,"recording":)" + recording + "}"}};
}

} // namespace

// A receiver logs a SubGHz mapping stream, so it is recognisable from the streams
// alone -- which is the only signal recordings predating `subghz_mode` carry.
TEST(TimeSyncModeDetection, Gen2ReceiverWithoutMetadataField) {
  EXPECT_EQ(
      determineTimeSyncMode(
          gen2Tags(), DeviceVersion::Gen2, players({TimeSyncMode::SUBGHZ, TimeSyncMode::UTC})),
      MetadataTimeSyncMode::SubGhz);
}

TEST(TimeSyncModeDetection, Gen2ReceiverWithMetadataField) {
  EXPECT_EQ(
      determineTimeSyncMode(
          gen2Tags("receiver"),
          DeviceVersion::Gen2,
          players({TimeSyncMode::SUBGHZ, TimeSyncMode::UTC})),
      MetadataTimeSyncMode::SubGhz);
}

// A broadcaster is the clock reference and logs no mapping stream of its own, so
// only the metadata field can identify it.
TEST(TimeSyncModeDetection, Gen2BroadcasterNeedsMetadataField) {
  EXPECT_EQ(
      determineTimeSyncMode(
          gen2Tags("broadcaster"), DeviceVersion::Gen2, players({TimeSyncMode::UTC})),
      MetadataTimeSyncMode::SubGhz);
}

// SubGHz off is not the same as "no time sync": the UTC stream is still usable, and
// reporting NotEnabled here is what previously caused it to be discarded.
TEST(TimeSyncModeDetection, Gen2SubGhzOffStillReportsUtc) {
  EXPECT_EQ(
      determineTimeSyncMode(gen2Tags(), DeviceVersion::Gen2, players({TimeSyncMode::UTC})),
      MetadataTimeSyncMode::Utc);
}

TEST(TimeSyncModeDetection, Gen2NoTimeSyncStreams) {
  EXPECT_EQ(
      determineTimeSyncMode(gen2Tags(), DeviceVersion::Gen2, players({})),
      MetadataTimeSyncMode::NotEnabled);
}

TEST(TimeSyncModeDetection, MissingOrMalformedMetadataFallsBackToStreams) {
  EXPECT_EQ(
      determineTimeSyncMode(TagMap{}, DeviceVersion::Gen2, players({TimeSyncMode::UTC})),
      MetadataTimeSyncMode::NotEnabled);
  EXPECT_EQ(
      determineTimeSyncMode(
          TagMap{{"metadata", "not json"}}, DeviceVersion::Gen2, players({TimeSyncMode::UTC})),
      MetadataTimeSyncMode::NotEnabled);
}

TEST(TimeSyncModeDetection, Gen1TicSyncRoles) {
  EXPECT_EQ(
      determineTimeSyncMode(
          {{"metadata", R"({"ticsync_mode":"client"})"}}, DeviceVersion::Gen1, players({})),
      MetadataTimeSyncMode::TicSyncClient);
  EXPECT_EQ(
      determineTimeSyncMode(
          {{"metadata", R"({"ticsync_mode":"server"})"}}, DeviceVersion::Gen1, players({})),
      MetadataTimeSyncMode::TicSyncServer);
  // An unrecognised value short-circuits rather than falling through to the
  // NTP / timecode / SubGHz / UTC checks.
  EXPECT_EQ(
      determineTimeSyncMode(
          {{"metadata", R"({"ticsync_mode":"whatever"})"}},
          DeviceVersion::Gen1,
          players({TimeSyncMode::UTC})),
      MetadataTimeSyncMode::NotEnabled);
}

TEST(TimeSyncModeDetection, Gen1FlagsAndStreamFallback) {
  EXPECT_EQ(
      determineTimeSyncMode(
          {{"metadata", R"({"ntp_time_enabled":true})"}}, DeviceVersion::Gen1, players({})),
      MetadataTimeSyncMode::Ntp);
  EXPECT_EQ(
      determineTimeSyncMode(
          {{"metadata", R"({"timecode_enabled":true})"}}, DeviceVersion::Gen1, players({})),
      MetadataTimeSyncMode::Timecode);
  EXPECT_EQ(
      determineTimeSyncMode(
          {{"metadata", "{}"}}, DeviceVersion::Gen1, players({TimeSyncMode::SUBGHZ})),
      MetadataTimeSyncMode::SubGhz);
  EXPECT_EQ(
      determineTimeSyncMode(
          {{"metadata", "{}"}}, DeviceVersion::Gen1, players({TimeSyncMode::UTC})),
      MetadataTimeSyncMode::Utc);
}

// The field's value decides, not merely its presence: an "off"-like or unknown
// spelling must not classify the file as SubGhz, and falls through to the streams.
TEST(TimeSyncModeDetection, Gen2UnknownSubGhzModeValueFallsThroughToStreams) {
  EXPECT_EQ(
      determineTimeSyncMode(gen2Tags("off"), DeviceVersion::Gen2, players({TimeSyncMode::UTC})),
      MetadataTimeSyncMode::Utc);
  EXPECT_EQ(
      determineTimeSyncMode(gen2Tags("off"), DeviceVersion::Gen2, players({TimeSyncMode::SUBGHZ})),
      MetadataTimeSyncMode::SubGhz);
  EXPECT_EQ(
      determineTimeSyncMode(gen2Tags("off"), DeviceVersion::Gen2, players({})),
      MetadataTimeSyncMode::NotEnabled);
}
