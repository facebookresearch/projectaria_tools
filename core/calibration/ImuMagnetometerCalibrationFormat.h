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

#include <fmt/format.h>
#include <format/Format.h>
#include "ImuMagnetometerCalibration.h"

/*
 * fmt::format() specialization for IMUCalibration
 */
template <>
struct fmt::formatter<projectaria::tools::calibration::ImuCalibration> {
  // No parse function needed

  // Format the ImuCalibrattion object
  template <typename FormatContext>
  auto format(const projectaria::tools::calibration::ImuCalibration& imuCalib, FormatContext& ctx) {
    return format_to(
        ctx.out(),
        "ImuCalibration(label: {}, T_Device_Imu: {})",
        imuCalib.getLabel(),
        imuCalib.getT_Device_Imu());
  }
};
