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

#include <iostream>
#include <vector>

namespace projectaria::tools::mps {

/// Filter a GlobalPointCloud given threshold for
///  - inverse distance standard deviation
///  - standard deviation of the distance estimate
inline GlobalPointCloud filterPointsFromConfidence(
    const GlobalPointCloud& in,
    const float threshold_invdep = 0.001,
    const float threshold_dep = 0.05) {
  GlobalPointCloud out;
  out.reserve(in.size());

  for (const auto& point : in) {
    if (point.inverseDistanceStd > threshold_invdep) {
      continue;
    }
    if (point.distanceStd > threshold_dep) {
      continue;
    }
    out.emplace_back(point);
  }
  out.shrink_to_fit();
  std::cout << "PointCloud filtering:\n"
            << " - kept: " << out.size() << " points \n"
            << " - kept: " << 100 * out.size() / static_cast<float>(in.size() + 1) << " %"
            << " of the original size" << std::endl;
  return out;
}

} // namespace projectaria::tools::mps
