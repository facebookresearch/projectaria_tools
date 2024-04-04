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

#include <optional>

#include <data_provider/players/MotionSensorPlayer.h>

namespace projectaria::tools::vrs_check {

struct MotionStats {
  uint64_t repeatAccel = 0;
  uint64_t repeatGyro = 0;
  uint64_t repeatMag = 0;
  uint64_t longestContRepeatAccel = 0;
  uint64_t longestContRepeatGyro = 0;
  uint64_t zeroAccel = 0;
  uint64_t zeroGyro = 0;
  uint64_t zeroMag = 0;
  uint64_t nonPhysicalAccel = 0;
  float maxObservedRotationAccel_radPerS2_ = 0;
  uint64_t numNonPhysicalRotationAccel_ = 0;
  uint64_t longestImuSkipUs = 0; // Longest sequence observed without valid IMU samples
};

struct MotionConfig {
  bool hasAccel = false;
  bool hasGyro = false;
  bool hasMag = false;
};

class Motion : public Periodic {
 public:
  Motion(
      vrs::StreamId streamId,
      float minScore,
      float maxImuSkipUs,
      float physicalAccelThreshold,
      float maxNonPhysicalAccel,
      float maxAllowedRotationAccel_radPerS2,
      float defaultPeriodUs);
  bool setup(vrs::RecordFileReader& reader) override; // Setup the camera player
  MotionStats getMotionStats(); // Get stats specific to motion sensors
  void logStats() override;
  bool getResult() override; // Pass or fail for this stream
  nlohmann::json statsToJson() override;

 protected:
  void processData(const data_provider::MotionData& data);
  std::unique_ptr<data_provider::MotionSensorPlayer> motionSensorPlayer_;
  MotionStats motionStats_;
  MotionConfig motionConfig_;
  std::array<float, 3> prevAccel_{0.f};
  std::array<float, 3> prevGyro_{0.f};
  std::optional<int64_t> prevGyroTimeStampNs;
  std::array<float, 3> prevMag_{0.f};
  uint64_t contRepeatAccel_ = 0;
  uint64_t contRepeatGyro_ = 0;
  const float defaultPeriodUs_; // Default period to use if not specified in the stream
  const float maxImuSkipUs_; // Max sequence allowed without valid IMU samples
  const float
      physicalAccelThreshold_; // Threshold for change in IMU acceleration between two consecutive
                               // measurements which can be deemed to be physical
  const float maxNonPhysicalAccel_; // Max proportion [0->1) of non-physical IMU acceleration
                                    // measurements allowed

  const float maxAllowedRotationAccel_radPerS2_; // maximum rotation accel allowed
  float maxObservedRotationAccel_radPerS2_ = 0; // maximum rotation accel observed
  int64_t numNonPhysicalRotationAccel_ = 0; // number of exessive acceleration rotation samples
                                            // (larger than maxAllowedRotationAccel_radPerS2)

  bool checkMaxImuSkip_ = true;
  bool preprocess_ = false;
  std::optional<int64_t> lastValidTimestampNs_;
};

} // namespace projectaria::tools::vrs_check
