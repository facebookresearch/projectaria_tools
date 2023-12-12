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

#include <data_provider/players/TimeSyncPlayer.h>

namespace projectaria::tools::vrs_check {

class TimeDomainMapping : public Periodic {
 public:
  TimeDomainMapping(vrs::StreamId streamId, float minScore);
  // Setup the TimeDomainMapping player
  bool setup(vrs::RecordFileReader& reader) override;

 private:
  void processData(const data_provider::TimeSyncData& data);
  std::unique_ptr<data_provider::TimeSyncPlayer> timeSyncPlayer_;
};

} // namespace projectaria::tools::vrs_check
