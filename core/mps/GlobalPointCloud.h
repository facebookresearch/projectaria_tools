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

#include <Eigen/Core>

#include <string>
#include <vector>

namespace projectaria::tools::mps {

//
// Point cloud data
//

struct GlobalPointPosition {
  // A unique identifier of this point within this map
  uint32_t uid;

  // Unique identifier of the world coordinate frame
  std::string graphUid;

  // The position of this point relative the world coordinate frame
  Eigen::Vector3d position_world;

  // Standard deviation of the inverse distance estimate
  float inverseDistanceStd;

  // Standard deviation of distance estimate
  float distanceStd;
};

using GlobalPointCloud = std::vector<GlobalPointPosition>;

} // namespace projectaria::tools::mps
