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
#include <fmt/format.h>

#include "MpsDataPathsProvider.h"

/*
 * fmt::format() specialization for MpsEyegazeDataPaths
 */
template <>
struct fmt::formatter<projectaria::tools::mps::MpsEyegazeDataPaths> {
  // No parse function needed

  // Format the MpsEyegazeDataPaths object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::MpsEyegazeDataPaths& paths, FormatContext& ctx) {
    return format_to(
        ctx.out(),
        "MPS Eyegaze Data Paths\n--generalEyegaze: {}\n--personalizedEyegaze: {}\n--summary: {}",
        paths.generalEyegaze,
        paths.personalizedEyegaze,
        paths.summary);
  }
};

/*
 * fmt::format() specialization for MpsSlamDataPaths
 */
template <>
struct fmt::formatter<projectaria::tools::mps::MpsSlamDataPaths> {
  // No parse function needed

  // Format the MpsSlamDataPaths object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::MpsSlamDataPaths& paths, FormatContext& ctx) {
    return format_to(
        ctx.out(),
        "MPS SLAM Data Paths\n--closedLoopTrajectory: {}\n--openLoopTrajectory: {}\n--semidensePoints: {}\n--semidenseObservations: {}\n--onlineCalibration: {}\n--summary: {}",
        paths.closedLoopTrajectory,
        paths.openLoopTrajectory,
        paths.semidensePoints,
        paths.semidenseObservations,
        paths.onlineCalibration,
        paths.summary);
  }
};

/*
 * fmt::format() specialization for MpsDataPaths
 */
template <>
struct fmt::formatter<projectaria::tools::mps::MpsDataPaths> {
  // No parse function needed

  // Format the MpsDataPaths object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::MpsDataPaths& paths, FormatContext& ctx) {
    return format_to(
        ctx.out(),
        "MPS Data Paths\n{}\n{}",
        fmt::to_string(paths.slam),
        fmt::to_string(paths.eyegaze));
  }
};
