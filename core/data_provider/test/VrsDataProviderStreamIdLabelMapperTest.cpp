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

#include <data_provider/StreamIdLabelMapper.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::calibration;

#define STRING(x) #x
#define GEN1_STRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"
#define GEN2_STRING(x) std::string(STRING(x)) + "aria_gen2_unit_test_sequence.vrs"

static const std::string ariaGen1TestDataPath = GEN1_STRING(TEST_FOLDER);
static const std::string ariaGen2TestDataPath = GEN2_STRING(TEST_FOLDER_GEN2);

TEST(VrsDataProvider, StreamIdLabelMapper_Gen1) {
  auto mapper = getAriaStreamIdLabelMapper(DeviceVersion::Gen1);

  // Valid labels
  EXPECT_TRUE(mapper->getStreamIdFromLabel("camera-slam-left"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("camera-slam-right"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("imu-left"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("imu-right"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("mic"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("mag0"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("baro0"));
  // Invalid labels
  EXPECT_FALSE(mapper->getStreamIdFromLabel("camera-non-exist"));
  EXPECT_FALSE(mapper->getStreamIdFromLabel("slam-front-left"));
}

TEST(VrsDataProvider, StreamIdLabelMapper_Gen2) {
  auto reader = std::make_shared<vrs::MultiRecordFileReader>();
  // Successful open should return 0
  EXPECT_FALSE(reader->open({ariaGen2TestDataPath}));

  // For Gen2, if no reader provided, expect throw
  EXPECT_THROW(getAriaStreamIdLabelMapper(DeviceVersion::Gen2), std::runtime_error);

  auto mapper = getAriaStreamIdLabelMapper(DeviceVersion::Gen2, reader);

  // Valid labels
  EXPECT_TRUE(mapper->getStreamIdFromLabel("slam-front-left"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("slam-front-right"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("slam-side-left"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("slam-side-right"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("imu-left"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("imu-right"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("mic"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("mag0"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("baro0"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("ppg"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("als"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("temperature"));
  EXPECT_TRUE(mapper->getStreamIdFromLabel("eyegaze"));
  // Invalid labels
  EXPECT_FALSE(mapper->getStreamIdFromLabel("camera-non-exist"));
  EXPECT_FALSE(mapper->getStreamIdFromLabel("camera-front-left"));

  // MPV labels
  const auto maybeVioStream = mapper->getStreamIdFromLabel("vio");
  const std::string expectedVioStreamName = "371-2";
  EXPECT_TRUE(maybeVioStream);
  EXPECT_EQ(maybeVioStream->getNumericName(), expectedVioStreamName);

  const auto maybeVioHighFreqStream = mapper->getStreamIdFromLabel("vio_high_frequency");
  const std::string expectedVioHighFreqStreamName = "371-3";
  EXPECT_TRUE(maybeVioHighFreqStream);
  EXPECT_EQ(maybeVioHighFreqStream->getNumericName(), expectedVioHighFreqStreamName);
}
