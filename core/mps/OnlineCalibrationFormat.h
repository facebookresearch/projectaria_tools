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
#include <fmt/chrono.h>

#include "OnlineCalibration.h"
#include "calibration/CameraCalibrationFormat.h"
#include "calibration/ImuMagnetometerCalibrationFormat.h"
#include "format/Format.h"

/*
 * fmt::format() specialization for OnlineCalibration
 */
template <>
struct fmt::formatter<projectaria::tools::mps::OnlineCalibration>
    : fmt::formatter<std::string_view> {
  // Format the OnlineCalibration object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::OnlineCalibration& calib, FormatContext& ctx) const {
    std::stringstream camCalibsStr;
    for (const auto& camCalib : calib.cameraCalibs) {
      camCalibsStr << fmt::to_string(camCalib) << ", ";
    }
    std::stringstream imuCalibsStr;
    for (const auto& imuCalib : calib.imuCalibs) {
      imuCalibsStr << fmt::to_string(imuCalib) << ", ";
    }
    return format_to(
        ctx.out(),
        "OnlineCalibration(tracking_timestamp: {}, utc_timestamp: {}, cam_calibs: [{}], imu_calibs: [{}])",
        calib.trackingTimestamp,
        calib.utcTimestamp,
        camCalibsStr.str(),
        imuCalibsStr.str());
  }
};
