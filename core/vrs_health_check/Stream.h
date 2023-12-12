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

#include "nlohmann/json.hpp"

#include <vrs/RecordFileReader.h>
#include <vrs/StreamId.h>
#include <cmath>
#include <cstdint>

namespace projectaria::tools::vrs_check {

struct Stats {
  uint64_t total = 0; // Total data points in this stream
  uint64_t expected = 0; // Expected data points based on the period
  uint64_t processed = 0; // Processed data points at the moment
  uint64_t bad = 0; // Data points with invalid data
};

class Stream {
 public:
  explicit Stream(const vrs::StreamId streamId) : streamId_(streamId) {}
  virtual ~Stream() {}
  // Setup the motion player
  virtual bool setup(vrs::RecordFileReader& reader) = 0;
  // Get stats during processing, useful for showing processing progress
  virtual Stats getStats() = 0;
  virtual void logStats() = 0;
  //! "Score" is an integrity check (all expected records are present),
  //! not a quality check (I like the values I got).
  virtual float getScore() {
    return std::nanf("");
  }
  virtual void logScore() {}
  // Stream type and index
  vrs::StreamId getStreamId() const {
    return streamId_;
  }
  virtual uint32_t getPeriodUs() = 0;
  virtual bool getResult() = 0; // Pass or fail for this stream
  // Convert stats to JSON
  virtual nlohmann::json statsToJson() {
    nlohmann::json jsonStats;
    const Stats stats = getStats();
    jsonStats["total"] = stats.total;
    jsonStats["expected"] = stats.expected;
    jsonStats["processed"] = stats.processed;
    jsonStats["bad"] = stats.bad;
    return jsonStats;
  }

 protected:
  std::mutex mutex_; // Protect stats_ since it can be called async by the client
  const vrs::StreamId streamId_;
};

} // namespace projectaria::tools::vrs_check
