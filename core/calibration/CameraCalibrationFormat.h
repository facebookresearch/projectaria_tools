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

#include "CameraCalibration.h"
#include "camera_projections/CameraProjectionFormat.h"
#include "format/Format.h"

/*
 * fmt::format() specialization for CameraCalibration
 */
template <>
struct fmt::formatter<projectaria::tools::calibration::CameraCalibration>
    : fmt::formatter<std::string_view> {
  // Format the CameraCalibration object
  template <typename FormatContext>
  auto format(
      const projectaria::tools::calibration::CameraCalibration& camCalib,
      FormatContext& ctx) const {
    return fmt::format_to(
        ctx.out(),
        "CameraCalibration(label: {}, model name: {}, principal point: {}, focal length: {}, projection params: {}, image size (w,h): {}, T_Device_Camera:{}, serialNumber:{}, TimeOffsetSec_Device_Camera:{})",
        camCalib.getLabel(),
        camCalib.modelName(),
        camCalib.getPrincipalPoint(),
        camCalib.getFocalLengths(),
        camCalib.projectionParams(),
        camCalib.getImageSize(),
        camCalib.getT_Device_Camera(),
        camCalib.getSerialNumber(),
        camCalib.getTimeOffsetSecDeviceCamera());
  }
};
