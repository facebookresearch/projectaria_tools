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

#include <chrono>
#include <vector>

#include <calibration/CameraCalibration.h>
#include <calibration/ImuMagnetometerCalibration.h>

namespace projectaria::tools::mps {

struct OnlineCalibration {
  // Aria internal processor timestamp in microseconds
  std::chrono::microseconds trackingTimestamp;

  // UTC Timestamp of the device image capture
  std::chrono::nanoseconds utcTimestamp;

  // Online estimated camera calibrations
  std::vector<calibration::CameraCalibration> cameraCalibs;

  // Online estimated IMU calibrations
  std::vector<calibration::ImuCalibration> imuCalibs;
};

using OnlineCalibrations = std::vector<OnlineCalibration>;

} // namespace projectaria::tools::mps
