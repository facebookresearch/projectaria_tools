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

#include <data_provider/players/BarometerPlayer.h>

namespace projectaria::tools::vrs_check {

struct BarometerStats {
  uint64_t repeatPressure = 0; // Repeated pressure data
  uint64_t repeatTemp = 0; // Repeated temperature data
  uint64_t tempOutOfRange =
      0; // Temperature readings that will compromise the function of other sensors
};

class Barometer : public Periodic {
 public:
  Barometer(vrs::StreamId streamId, float minScore, float minTemp, float maxTemp);
  bool setup(vrs::RecordFileReader& reader) override; // Setup the barometer player
  const BarometerStats& getBarometerStats(); // Get stats specific to barometer sensors
  void logStats() override;
  nlohmann::json statsToJson() override;

 private:
  void processData(const data_provider::BarometerData& data);
  std::unique_ptr<data_provider::BarometerPlayer> barometerPlayer_;
  BarometerStats barometerStats_;
  double prevPressure_ = 0.0;
  double prevTemp_ = 0.0;
  float minTemp_;
  float maxTemp_;
};

} // namespace projectaria::tools::vrs_check
