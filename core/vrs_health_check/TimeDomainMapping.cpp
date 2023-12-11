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

#include "TimeDomainMapping.h"

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:TimeDomainMapping"
#include <logging/Log.h>

namespace projectaria::tools::vrs_check {

TimeDomainMapping::TimeDomainMapping(vrs::StreamId streamId, float minScore)
    : Periodic(streamId, minScore) {}

bool TimeDomainMapping::setup(vrs::RecordFileReader& reader) {
  data_provider::TimeSyncCallback callback = [&](const data_provider::TimeSyncData& data,
                                                 const data_provider::AriaTimeSyncConfigRecord&,
                                                 bool) {
    processData(data);
    return true;
  };

  timeSyncPlayer_ = std::make_unique<data_provider::TimeSyncPlayer>(streamId_);
  if (!timeSyncPlayer_) {
    XR_LOGE("Cannot create playable for {}", streamId_.getName());
    return false;
  }
  timeSyncPlayer_->setCallback(callback);
  reader.setStreamPlayer(streamId_, timeSyncPlayer_.get());

  // Parse the configuration record
  if (!reader.readFirstConfigurationRecord(streamId_)) {
    XR_LOGE("Stream {} is missing a configuration record", streamId_.getName());
    return false;
  }
  const auto& config = timeSyncPlayer_->getConfigRecord();

  if (config.sampleRateHz > 0) {
    periodUs_ = 1 / config.sampleRateHz * 1e6;
    setMaxDeviationFromPeriodUs();
  } else {
    XR_LOGE("Stream {} is missing period", streamId_.getName());
    return false;
  }
  preprocessStream(reader);

  return true;
}

void TimeDomainMapping::processData(const data_provider::TimeSyncData& data) {
  std::lock_guard lock{mutex_};
  if (data.monotonicTimestampNs < 0 || data.realTimestampNs < 0) {
    stats_.bad++;
  }
  processTimestamp(data.monotonicTimestampNs / 1e3);
}

} // namespace projectaria::tools::vrs_check
