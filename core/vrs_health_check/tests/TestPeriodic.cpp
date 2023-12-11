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

#include "Periodic.h"
#include "tests/TestingUtils.h"

#include <gtest/gtest.h>
#include <unordered_map>

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:PeriodicTest"
#include "logging/Log.h"

namespace projectaria::tools::vrs_check {

// Num data samples to simulate in each test case
const size_t numSamples = 1000;

const uint32_t defaultPeriodUs = 1000;

const float minScore = 100.0;

// We need to pass a RecordFileReader into MockPeriodic::setup()
vrs::RecordFileReader recordFileReader = vrs::RecordFileReader();

/* Expected values for PeriodicStats for perfect data.
This map is used as a starting point for generating expected
output stats for each test case.
*/
std::unordered_map<std::string, uint64_t> expected = {
    {"bad", 0},
    {"dropped", 0},
    {"expected", numSamples},
    {"largest_deviation_from_period_us", 0},
    {"non_monotonic", 0},
    {"processed", numSamples},
    {"time_error", 0},
    {"total", numSamples},
};

TEST(TestVrsHealthCheckPeriodic, PerfectData) {
  /*
  Test the case when we have perfect data coming in, i.e.
  no dropped samples, timestamps are EXACTLY what we would expect
  based upon the specified period.
  */
  MockPeriodic mockPeriodic(
      vrs::StreamId(vrs::RecordableTypeId::UnitTestRecordableClass, 0),
      minScore,
      defaultPeriodUs,
      numSamples);
  mockPeriodic.setup(recordFileReader);

  // Simulate processing timestamps from a stream
  for (size_t sampleIdx = 0; sampleIdx < numSamples; sampleIdx++) {
    const uint64_t currentTimeStampUs = sampleIdx * defaultPeriodUs;
    mockPeriodic.processData(currentTimeStampUs);
  }

  nlohmann::json stats = mockPeriodic.statsToJson();

  // Ensure the json stats output from Periodic match our expectations.
  for (const auto& kv : stats.items()) {
    EXPECT_EQ(expected[kv.key()], kv.value());
  }

  // Verify perfect score
  EXPECT_EQ(mockPeriodic.getScore(), 100.0);

  // Verify result is SUCCESS
  EXPECT_TRUE(mockPeriodic.getResult());
}

TEST(TestVrsHealthCheckPeriodic, DroppedSamples) {
  /*
  Test the case when we drop some data samples.
  */
  const size_t numSamplesToDrop = 2;
  std::unordered_map<std::string, uint64_t> expectedDropped = expected;
  expectedDropped["dropped"] = numSamplesToDrop;
  expectedDropped["processed"] = numSamples - numSamplesToDrop;

  MockPeriodic mockPeriodic(
      vrs::StreamId(vrs::RecordableTypeId::UnitTestRecordableClass, 0),
      minScore,
      defaultPeriodUs,
      numSamples);
  mockPeriodic.setup(recordFileReader);

  // Simulate processing timestamps from a stream
  for (size_t sampleIdx = 0; sampleIdx < numSamples; sampleIdx++) {
    if (size_t(numSamples / 2) < sampleIdx &&
        sampleIdx <= size_t(numSamples / 2) + numSamplesToDrop) {
      // dropped sample...
      continue;
    }
    const uint64_t currentTimeStampUs = sampleIdx * defaultPeriodUs;
    mockPeriodic.processData(currentTimeStampUs);
  }

  nlohmann::json stats = mockPeriodic.statsToJson();

  // Ensure the json stats output from Periodic match our expectations.
  for (const auto& kv : stats.items()) {
    if (kv.key() == "sequential_drops") {
      // sequential_drops has to be parsed as it's a picojson object...
      size_t sequentialDropsCount = 0;
      for (const auto& it : kv.value().items()) {
        // We should only have one drop of length numSamplesToDrop
        EXPECT_EQ(it.key(), std::to_string(numSamplesToDrop));
        EXPECT_EQ(it.value(), 1);
        sequentialDropsCount++;
      }
      EXPECT_EQ(sequentialDropsCount, 1);
    } else {
      EXPECT_EQ(expectedDropped[kv.key()], kv.value());
    }
  }

  // Verify score
  EXPECT_NE(mockPeriodic.getScore(), 100.0 * (1.0 - ((float)numSamplesToDrop / numSamples)));

  // Verify result is FAIL (as score < 100)
  EXPECT_FALSE(mockPeriodic.getResult());
}

TEST(TestVrsHealthCheckPeriodic, DeviationFromPeriod) {
  /*
  Test the case when a sample has a timestamp that deviates
  from the expected value. Note that this should register a
  time error as well.
  */
  const uint64_t deviationFromPeriod = 200;
  std::unordered_map<std::string, uint64_t> expectedPeriodDeviation = expected;
  expectedPeriodDeviation["largest_deviation_from_period_us"] = deviationFromPeriod;
  expectedPeriodDeviation["time_error"] = 1;

  MockPeriodic mockPeriodic(
      vrs::StreamId(vrs::RecordableTypeId::UnitTestRecordableClass, 0),
      minScore,
      defaultPeriodUs,
      numSamples);
  mockPeriodic.setup(recordFileReader);

  // Simulate processing timestamps from a stream
  for (size_t sampleIdx = 0; sampleIdx < numSamples; sampleIdx++) {
    uint64_t currentTimeStampUs = sampleIdx * defaultPeriodUs;
    // Last sample comes in late...
    if (sampleIdx == numSamples - 1) {
      currentTimeStampUs += deviationFromPeriod;
    }
    mockPeriodic.processData(currentTimeStampUs);
  }

  nlohmann::json stats = mockPeriodic.statsToJson();

  // Ensure the json stats output from Periodic match our expectations.
  for (const auto& kv : stats.items()) {
    EXPECT_EQ(expectedPeriodDeviation[kv.key()], kv.value());
  }

  // Verify score
  EXPECT_EQ(mockPeriodic.getScore(), 100.0);

  // Verify result is SUCCESS
  EXPECT_TRUE(mockPeriodic.getResult());
}

TEST(TestVrsHealthCheckPeriodic, NonMonotonic) {
  /*
  Test the case when we have a sequence of timestamps for
  data samples that are non-monotonic.

  This will have knock on effects. There will be:
    - Deviations from the expected period
    - Time errors
    - Dropped samples
  */
  const size_t numNonMonotonicSamples = 5;
  std::unordered_map<std::string, uint64_t> expectedNonMonotonic = expected;
  expectedNonMonotonic["non_monotonic"] = numNonMonotonicSamples;
  expectedNonMonotonic["largest_deviation_from_period_us"] = numNonMonotonicSamples;
  expectedNonMonotonic["time_error"] = numNonMonotonicSamples;
  expectedNonMonotonic["dropped"] = numNonMonotonicSamples;

  MockPeriodic mockPeriodic(
      vrs::StreamId(vrs::RecordableTypeId::UnitTestRecordableClass, 0),
      minScore,
      defaultPeriodUs,
      numSamples);
  mockPeriodic.setup(recordFileReader);

  // Simulate processing timestamps from a stream
  uint64_t previousTimeStampUs = 0;
  uint64_t currentTimeStampUs = 0;
  for (size_t sampleIdx = 0; sampleIdx < numSamples; sampleIdx++) {
    if (size_t(numSamples / 2) < sampleIdx &&
        sampleIdx <= size_t(numSamples / 2) + numNonMonotonicSamples) {
      currentTimeStampUs = previousTimeStampUs - 1;
    } else {
      currentTimeStampUs = sampleIdx * defaultPeriodUs;
    }
    mockPeriodic.processData(currentTimeStampUs);
    previousTimeStampUs = currentTimeStampUs;
  }

  nlohmann::json stats = mockPeriodic.statsToJson();

  // Ensure the json stats output from Periodic match our expectations.
  for (const auto& kv : stats.items()) {
    if (kv.key() == "sequential_drops") {
      // sequential_drops has to be parsed as it's a picojson object...
      size_t sequentialDropsCount = 0;
      for (const auto& it : kv.value().items()) {
        // We should only have one drop of length numSamplesToDrop
        EXPECT_EQ(it.key(), std::to_string(numNonMonotonicSamples));
        EXPECT_EQ(it.value(), 1);
        sequentialDropsCount++;
      }
      EXPECT_EQ(sequentialDropsCount, 1);
    } else {
      EXPECT_EQ(expectedNonMonotonic[kv.key()], kv.value());
    }
  }

  // Verify score
  EXPECT_NE(mockPeriodic.getScore(), 100.0 * (1.0 - ((float)numNonMonotonicSamples / numSamples)));

  // Verify result is FAIL
  EXPECT_FALSE(mockPeriodic.getResult());
}

} // namespace projectaria::tools::vrs_check
