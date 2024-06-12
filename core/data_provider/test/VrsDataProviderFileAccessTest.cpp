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

#include <gtest/gtest.h>

using namespace projectaria::tools::data_provider;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"

static const std::string ariaTestDataPath = XSTRING(TEST_FOLDER);

#define DEFAULT_LOG_CHANNEL "TestLog"
#include <logging/Checks.h>
#include <logging/Log.h>

namespace {
constexpr const uint32_t kFileTagsNum = 25;
}

TEST(VrsDataProvider, getFileTags) {
  auto provider = createVrsDataProvider(ariaTestDataPath);
  auto fileTags = provider->getFileTags();
  EXPECT_EQ(fileTags.size(), kFileTagsNum);
}

TEST(VrsDataProvider, getMetadata) {
  auto provider = createVrsDataProvider(ariaTestDataPath);
  EXPECT_EQ(provider->getTimeSyncMode(), MetadataTimeSyncMode::NotEnabled);
  const auto& maybeMetadata = provider->getMetadata();
  EXPECT_TRUE(maybeMetadata.has_value());
  const auto& metadata = maybeMetadata.value();

  EXPECT_EQ(metadata.deviceSerial, "1WM093701M1276");
  EXPECT_EQ(metadata.recordingProfile, "profile9");
  EXPECT_EQ(metadata.sharedSessionId, "");
  EXPECT_EQ(metadata.filename, "d3c61c3a-18ec-460e-a35c-cec9579494ca.vrs");
  EXPECT_EQ(metadata.timeSyncMode, MetadataTimeSyncMode::NotEnabled);
  EXPECT_EQ(metadata.deviceId, "35ec1d5b-689d-4531-a0c9-1c8ff42d2e89");
  EXPECT_EQ(metadata.startTimeEpochSec, 1649265055);
}
