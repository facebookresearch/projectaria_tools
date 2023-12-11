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

#include "tests/TestingUtils.h"

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:TestingUtils"
#include "logging/Log.h"

namespace projectaria::tools::vrs_check {

MockPeriodic::MockPeriodic(
    vrs::StreamId streamId,
    float minScore,
    uint32_t defaultPeriodUs,
    size_t numSamples)
    : Periodic(streamId, minScore), defaultPeriodUs_(defaultPeriodUs), numSamples_(numSamples) {}

bool MockPeriodic::setup(vrs::RecordFileReader& /*reader*/) {
  periodUs_ = defaultPeriodUs_;
  Periodic::setSensorMisalignmentStats({});

  // Immitate preprocessStream() method
  stats_.expected = numSamples_;
  stats_.total = numSamples_;
  XR_LOGI(
      "{}: frames expected={} actual={} missing={}",
      streamId_.getName(),
      stats_.expected,
      stats_.total,
      stats_.expected - stats_.total);

  return true;
}

void MockPeriodic::processData(uint64_t captureTimestampUs) {
  processTimestamp(captureTimestampUs);
}

} // namespace projectaria::tools::vrs_check
