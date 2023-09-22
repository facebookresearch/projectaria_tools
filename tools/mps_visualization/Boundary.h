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

#include <logging/Checks.h>

namespace projectaria::tools::mps {

class Boundary {
 public:
  Boundary(
      const ClosedLoopTrajectory& singleSessionTraj,
      const std::vector<std::vector<Eigen::Vector3f>>& fullTrajs) {
    XR_CHECK(!singleSessionTraj.empty());
    for (const auto& pose : singleSessionTraj) {
      const auto p = pose.T_world_device.translation().cast<float>();
      updateBoundary(p);
    }
    for (const auto& traj : fullTrajs) {
      for (const auto& p : traj) {
        updateBoundary(p);
      }
    }
  }

  float maxX() const {
    return maxX_;
  }
  float maxY() const {
    return maxY_;
  }
  float maxZ() const {
    return maxZ_;
  }
  float minX() const {
    return minX_;
  }
  float minY() const {
    return minY_;
  }
  float minZ() const {
    return minZ_;
  }

  std::vector<float> getDilatedBoundary(const float dilationMeter = 10.0) const {
    return {
        maxX_ + dilationMeter,
        maxY_ + dilationMeter,
        maxZ_ + dilationMeter,
        minX_ - dilationMeter,
        minY_ - dilationMeter,
        minZ_ - dilationMeter};
  }

 private:
  void updateBoundary(const Eigen::Vector3f& p) {
    maxX_ = std::max(maxX_, p(0));
    maxY_ = std::max(maxY_, p(1));
    maxZ_ = std::max(maxZ_, p(2));
    minX_ = std::min(minX_, p(0));
    minY_ = std::min(minY_, p(1));
    minZ_ = std::min(minZ_, p(2));
  }

  float maxX_ = std::numeric_limits<float>::lowest();
  float maxY_ = std::numeric_limits<float>::lowest();
  float maxZ_ = std::numeric_limits<float>::lowest();
  float minX_ = std::numeric_limits<float>::max();
  float minY_ = std::numeric_limits<float>::max();
  float minZ_ = std::numeric_limits<float>::max();
};

} // namespace projectaria::tools::mps
