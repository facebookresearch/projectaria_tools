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

#include "StaticCameraCalibration.h"
#include "format/Format.h"

/*
 * fmt::format() specialization for StaticCameraCalibration
 */
template <>
struct fmt::formatter<projectaria::tools::mps::StaticCameraCalibration>
    : fmt::formatter<std::string_view> {
  // Format the StaticCameraCalibration object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::StaticCameraCalibration& calib, FormatContext& ctx)
      const {
    return fmt::format_to(
        ctx.out(),
        "StaticCameraCalibration( cameraUid: {}, graphUid: {}, T_world_cam: {}, width: {}, height: {}, intrinsicsType: {}, intrinsics: {}, startFrameIdx: {}, endFrameIdx: {}, quality: {} )",
        calib.cameraUid,
        calib.graphUid,
        calib.T_world_cam,
        calib.width,
        calib.height,
        calib.intrinsicsType,
        calib.intrinsics,
        // TODO: std::optional support in fmt is coming in fmt 10.0.0, until then we print -1
        // https://github.com/fmtlib/fmt/blob/10.0.0/include/fmt/std.h#9
        calib.startFrameIdx.value_or(-1),
        calib.endFrameIdx.value_or(-1),
        calib.quality);
  }
};
