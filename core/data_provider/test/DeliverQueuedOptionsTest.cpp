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

#include <data_provider/DeliverQueuedOptions.h>

#include <gtest/gtest.h>

#include <map>
#include <stdexcept>

using namespace projectaria::tools::data_provider;

namespace {

const vrs::StreamId kStream1{vrs::RecordableTypeId::SlamCameraData, 1};
const vrs::StreamId kStream2{vrs::RecordableTypeId::SlamCameraData, 2};
const vrs::StreamId kUnknownStream{vrs::RecordableTypeId::RgbCameraRecordableClass, 99};

DeliverQueuedOptions makeDefaultOptions() {
  std::map<vrs::StreamId, size_t> rates = {{kStream1, 1}, {kStream2, 3}};
  return DeliverQueuedOptions(100, 200, rates);
}

} // namespace

TEST(DeliverQueuedOptionsTest, TruncateTimeSettersEnforceNonNegative) {
  auto opts = makeDefaultOptions();

  // Zero is the boundary: must be accepted for both first and last
  opts.setTruncateFirstDeviceTimeNs(0);
  EXPECT_EQ(opts.getTruncateFirstDeviceTimeNs(), 0);
  opts.setTruncateLastDeviceTimeNs(0);
  EXPECT_EQ(opts.getTruncateLastDeviceTimeNs(), 0);

  // Positive values accepted, update state independently
  opts.setTruncateFirstDeviceTimeNs(500);
  opts.setTruncateLastDeviceTimeNs(999);
  EXPECT_EQ(opts.getTruncateFirstDeviceTimeNs(), 500);
  EXPECT_EQ(opts.getTruncateLastDeviceTimeNs(), 999);

  // Negative values rejected, state preserved from last valid set
  EXPECT_THROW(opts.setTruncateFirstDeviceTimeNs(-1), std::runtime_error);
  EXPECT_EQ(opts.getTruncateFirstDeviceTimeNs(), 500);
  EXPECT_THROW(opts.setTruncateLastDeviceTimeNs(-1), std::runtime_error);
  EXPECT_EQ(opts.getTruncateLastDeviceTimeNs(), 999);
}

TEST(DeliverQueuedOptionsTest, SetSubsampleRateEnforcesPositiveRate) {
  auto opts = makeDefaultOptions();

  // Valid rate accepted and applied to the correct stream
  opts.setSubsampleRate(kStream1, 5);
  EXPECT_EQ(opts.getSubsampleRate(kStream1), 5);
  // Other stream must remain unaffected
  EXPECT_EQ(opts.getSubsampleRate(kStream2), 3);

  // Zero rate rejected, state preserved from last valid set
  EXPECT_THROW(opts.setSubsampleRate(kStream1, 0), std::runtime_error);
  EXPECT_EQ(opts.getSubsampleRate(kStream1), 5);
}

TEST(DeliverQueuedOptionsTest, SetSubsampleRateThrowsForUnknownStream) {
  auto opts = makeDefaultOptions();
  EXPECT_THROW(opts.setSubsampleRate(kUnknownStream, 2), std::out_of_range);
}
