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

#pragma once

#include "Periodic.h"

namespace projectaria::tools::vrs_check {

// Periodic class is abstract so we need this minimal derived class
// to test the basic functionality.
class MockPeriodic : public Periodic {
 public:
  MockPeriodic(vrs::StreamId streamId, float minScore, uint32_t defaultPeriodUs, size_t numSamples);
  bool setup(vrs::RecordFileReader& reader) override; // Setup the camera player
  void processData(uint64_t captureTimestampUs);

 protected:
  const uint32_t defaultPeriodUs_;
  const size_t numSamples_;
};

} // namespace projectaria::tools::vrs_check
