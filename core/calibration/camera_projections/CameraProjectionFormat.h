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
#include <string_view>
#include "CameraProjection.h"

/*
 * fmt::format() specialization for ModelType
 */
template <>
struct fmt::formatter<projectaria::tools::calibration::CameraProjection::ModelType>
    : fmt::formatter<string_view> {
  // No parse function needed

  // Format the ModelType object
  template <typename FormatContext>
  auto format(
      const projectaria::tools::calibration::CameraProjection::ModelType& modelType,
      FormatContext& ctx) const {
    using mt = projectaria::tools::calibration::CameraProjection::ModelType;
    auto to_string = [&](const mt& value) {
      switch (value) {
        case mt::Linear:
          return "Linear";
        case mt::Spherical:
          return "Spherical";
        case mt::KannalaBrandtK3:
          return "KannalaBrandtK3";
        case mt::Fisheye624:
          return "Fisheye624";
        default:
          return "Unknown";
      }
    };
    return formatter<string_view>::format(to_string(modelType), ctx);
  }
};
