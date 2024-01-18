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

#include "GlobalPointCloud.h"
#include "format/Format.h"

/*
 * fmt::format() specialization for GlobalPointPosition
 */
template <>
struct fmt::formatter<projectaria::tools::mps::GlobalPointPosition>
    : fmt::formatter<std::string_view> {
  // Format the Point object
  template <typename FormatContext>
  auto format(const projectaria::tools::mps::GlobalPointPosition& point, FormatContext& ctx) const {
    return format_to(
        ctx.out(),
        "GlobalPointPosition(uid = {}, graphUid = {}, position_world = {}, inverseDistanceStd = {:.4f}, distanceStd = {:.4f})",
        point.uid,
        point.graphUid,
        point.position_world,
        point.inverseDistanceStd,
        point.distanceStd);
  }
};
