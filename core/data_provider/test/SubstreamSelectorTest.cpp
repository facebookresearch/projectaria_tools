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

#include <data_provider/SubstreamSelector.h>
#include <data_provider/VrsDataProvider.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::data_provider;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"

static const std::string ariaTestDataPath = XSTRING(TEST_FOLDER);

TEST(SubStreamSelector, activeStreams) {
  auto provider = createVrsDataProvider(ariaTestDataPath);
  std::set<vrs::StreamId> streamIds = provider->getAllStreams();
  SubstreamSelector selector(streamIds);

  EXPECT_TRUE(streamIds.size() > 0);
  vrs::StreamId streamId = *streamIds.begin();

  selector.deactivateStreamAll();
  EXPECT_EQ(selector.getActiveStreamIds().size(), 0);

  selector.activateStream(streamId);
  std::set<vrs::StreamId> activeStreamsSelector = selector.getActiveStreamIds();

  std::set<vrs::StreamId> activeStreamsExpected{streamId};
  EXPECT_EQ(activeStreamsSelector, activeStreamsExpected);
}

TEST(SubStreamSelector, activeStreamsByTypes) {
  auto provider = createVrsDataProvider(ariaTestDataPath);
  std::set<vrs::StreamId> streamIds = provider->getAllStreams();
  SubstreamSelector selector(streamIds);

  const auto typeIds = selector.getTypeIds();
  EXPECT_TRUE(typeIds.size() > 0);
  const auto typeId = *typeIds.begin();

  selector.deactivateStreamAll();
  EXPECT_EQ(selector.getActiveStreamIds().size(), 0);

  selector.activateStream(typeId);
  std::set<vrs::StreamId> activeStreamsSelector = selector.getActiveStreamIds();

  std::set<vrs::StreamId> activeStreamsExpected = selector.getStreamIds(typeId);
  EXPECT_EQ(activeStreamsSelector, activeStreamsExpected);
}

TEST(SubStreamSelector, activeAllStreams) {
  auto provider = createVrsDataProvider(ariaTestDataPath);
  std::set<vrs::StreamId> streamIds = provider->getAllStreams();
  SubstreamSelector selector(streamIds);

  selector.deactivateStreamAll();
  EXPECT_EQ(selector.getActiveStreamIds().size(), 0);

  selector.activateStreamAll();
  std::set<vrs::StreamId> activeStreamsSelector = selector.getActiveStreamIds();

  EXPECT_EQ(streamIds, activeStreamsSelector);
}
