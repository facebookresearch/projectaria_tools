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

// test query by timestamp within vrs time range
void checkInBound(std::shared_ptr<VrsDataProvider> provider, const vrs::StreamId& streamId) {
  for (int t = 0; t < static_cast<int>(kNumTimeDomain); ++t) {
    TimeDomain timeDomain = static_cast<TimeDomain>(t);
    if (!provider->supportsTimeDomain(streamId, timeDomain)) {
      continue;
    }

    size_t numData = provider->getNumData(streamId);
    if (numData < 3) {
      continue;
    }

    int64_t timestamp0 = provider->getSensorDataByIndex(streamId, 0).getTimeNs(timeDomain);
    int64_t timestamp1 = provider->getSensorDataByIndex(streamId, 1).getTimeNs(timeDomain);
    int64_t timestamp2 = provider->getSensorDataByIndex(streamId, 2).getTimeNs(timeDomain);

    // query timestamp slightly before than existing timestamp
    // returns last data if option = Before
    // returns current data if option = Cloest or After
    {
      int64_t timestampQuery = timestamp1 - 1;
      const auto sensorDataBefore = provider->getSensorDataByTimeNs(
          streamId, timestampQuery, timeDomain, TimeQueryOptions::Before);
      EXPECT_EQ(sensorDataBefore.getTimeNs(timeDomain), timestamp0);
      const auto sensorDataClosest = provider->getSensorDataByTimeNs(
          streamId, timestampQuery, timeDomain, TimeQueryOptions::Closest);
      EXPECT_EQ(sensorDataClosest.getTimeNs(timeDomain), timestamp1);
      const auto sensorDataAfter = provider->getSensorDataByTimeNs(
          streamId, timestampQuery, timeDomain, TimeQueryOptions::After);
      EXPECT_EQ(sensorDataAfter.getTimeNs(timeDomain), timestamp1);
    }

    // query timestamp at existing timestamp
    // returns current data
    {
      int64_t timestampQuery = timestamp1;
      const auto sensorDataBefore = provider->getSensorDataByTimeNs(
          streamId, timestampQuery, timeDomain, TimeQueryOptions::Before);
      EXPECT_EQ(sensorDataBefore.getTimeNs(timeDomain), timestamp1);
      const auto sensorDataClosest = provider->getSensorDataByTimeNs(
          streamId, timestampQuery, timeDomain, TimeQueryOptions::Closest);
      EXPECT_EQ(sensorDataClosest.getTimeNs(timeDomain), timestamp1);
      const auto sensorDataAfter = provider->getSensorDataByTimeNs(
          streamId, timestampQuery, timeDomain, TimeQueryOptions::After);
      EXPECT_EQ(sensorDataAfter.getTimeNs(timeDomain), timestamp1);
    }

    // query timestamp slightly before than existing timestamp
    // returns next data if option = After
    // returns current data if option = Cloest or Before
    {
      int64_t timestampQuery = timestamp1 + 1;
      const auto sensorDataBefore = provider->getSensorDataByTimeNs(
          streamId, timestampQuery, timeDomain, TimeQueryOptions::Before);
      EXPECT_EQ(sensorDataBefore.getTimeNs(timeDomain), timestamp1);
      const auto sensorDataClosest = provider->getSensorDataByTimeNs(
          streamId, timestampQuery, timeDomain, TimeQueryOptions::Closest);
      EXPECT_EQ(sensorDataClosest.getTimeNs(timeDomain), timestamp1);
      const auto sensorDataAfter = provider->getSensorDataByTimeNs(
          streamId, timestampQuery, timeDomain, TimeQueryOptions::After);
      EXPECT_EQ(sensorDataAfter.getTimeNs(timeDomain), timestamp2);
    }
  }
}

// test query by timestamp out side vrs time range
void checkOutOfBound(std::shared_ptr<VrsDataProvider> provider, const vrs::StreamId& streamId) {
  for (int t = 0; t < static_cast<int>(kNumTimeDomain); ++t) {
    TimeDomain timeDomain = static_cast<TimeDomain>(t);
    if (!provider->supportsTimeDomain(streamId, timeDomain)) {
      continue;
    }

    size_t numData = provider->getNumData(streamId);
    if (numData < 2) {
      continue;
    }

    // test time range
    int64_t firstTimestamp = provider->getFirstTimeNs(streamId, timeDomain);
    int64_t lastTimestamp = provider->getLastTimeNs(streamId, timeDomain);
    EXPECT_GE(lastTimestamp, firstTimestamp);

    // query timestamp = first timestamp - 1ns
    {
      const auto sensorDataBefore = provider->getSensorDataByTimeNs(
          streamId, firstTimestamp - 1, timeDomain, TimeQueryOptions::Before);
      EXPECT_EQ(sensorDataBefore.sensorDataType(), SensorDataType::NotValid);

      const auto sensorDataClosest = provider->getSensorDataByTimeNs(
          streamId, firstTimestamp - 1, timeDomain, TimeQueryOptions::Closest);
      EXPECT_EQ(sensorDataClosest.getTimeNs(timeDomain), firstTimestamp);

      const auto sensorDataAfter = provider->getSensorDataByTimeNs(
          streamId, firstTimestamp - 1, timeDomain, TimeQueryOptions::After);
      EXPECT_EQ(sensorDataAfter.getTimeNs(timeDomain), firstTimestamp);
    }

    // query timestamp = last timestamp + 1ns
    {
      const auto sensorDataBefore = provider->getSensorDataByTimeNs(
          streamId, lastTimestamp + 1, timeDomain, TimeQueryOptions::Before);
      EXPECT_EQ(sensorDataBefore.getTimeNs(timeDomain), lastTimestamp);

      const auto sensorDataClosest = provider->getSensorDataByTimeNs(
          streamId, lastTimestamp + 1, timeDomain, TimeQueryOptions::Closest);
      EXPECT_EQ(sensorDataClosest.getTimeNs(timeDomain), lastTimestamp);

      const auto sensorDataAfter = provider->getSensorDataByTimeNs(
          streamId, lastTimestamp + 1, timeDomain, TimeQueryOptions::After);
      EXPECT_EQ(sensorDataAfter.sensorDataType(), SensorDataType::NotValid);
    }
  }
}

TEST(VrsDataProvider, getDataByTimeNsInBound) {
  auto provider = createVrsDataProvider(ariaTestDataPath);

  const auto streamIds = provider->getAllStreams();
  for (const auto streamId : streamIds) {
    checkInBound(provider, streamId);
  }
}

TEST(VrsDataProvider, getDataByTimeNsOutOfBound) {
  auto provider = createVrsDataProvider(ariaTestDataPath);

  const auto streamIds = provider->getAllStreams();
  for (const auto streamId : streamIds) {
    checkOutOfBound(provider, streamId);
  }
}

TEST(VrsDataProvider, multiThreadGetDataByTimeNsInBound) {
  auto provider = createVrsDataProvider(ariaTestDataPath);

  const auto streamIds = provider->getAllStreams();
  std::vector<std::thread> threads;
  for (const auto streamId : streamIds) {
    threads.emplace_back(
        std::thread([&provider, streamId]() { checkInBound(provider, streamId); }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

TEST(VrsDataProvider, multiThreadGetDataByTimeNsOutOfBound) {
  auto provider = createVrsDataProvider(ariaTestDataPath);

  const auto streamIds = provider->getAllStreams();
  std::vector<std::thread> threads;
  for (const auto streamId : streamIds) {
    threads.emplace_back(
        std::thread([&provider, streamId]() { checkOutOfBound(provider, streamId); }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}
