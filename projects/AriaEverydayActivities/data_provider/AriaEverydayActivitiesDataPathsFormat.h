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

#include <mps/MpsDataPathsFormat.h>
#include "AriaEverydayActivitiesDataPathsProvider.h"

/*
 * fmt::format() specialization for AriaEverydayActivitiesDataPaths
 */
template <>
struct fmt::formatter<projectaria::dataset::aea::AriaEverydayActivitiesDataPaths>
    : fmt::formatter<std::string_view> {
  // Format the AriaEverydayActivitiesDataPaths object
  template <typename FormatContext>
  auto format(
      const projectaria::dataset::aea::AriaEverydayActivitiesDataPaths& paths,
      FormatContext& ctx) const {
    return fmt::format_to(
        ctx.out(),
        "AEA Data Paths\n--ariaVrs: {}\n--speech: {}\n--metadata: {}\n{}",
        paths.ariaVrs,
        paths.speech,
        paths.metadata,
        fmt::to_string(paths.mps));
  }
};
