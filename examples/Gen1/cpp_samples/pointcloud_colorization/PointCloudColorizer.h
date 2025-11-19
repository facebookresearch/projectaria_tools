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
#include "PointObservation.h"
#include "Trajectory.h"

#include <data_provider/VrsDataProvider.h>

#include <vector>

namespace {
// Generic Abstract base class for all GlobalPointCloud colorizers
class PointCloudColorizer {
 public:
  // NOLINTNEXTLINE(clang-diagnostic-unused-member-function)
  explicit PointCloudColorizer(
      const projectaria::tools::mps::ClosedLoopTrajectory& trajectory,
      const projectaria::tools::mps::GlobalPointCloud& globalPointCloud,
      const projectaria::tools::mps::PointObservations& pointObservations,
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider)
      : globalPointCloud_(globalPointCloud),
        trajectory_(trajectory),
        pointObservations_(pointObservations),
        dataProvider_(dataProvider) {}

  virtual ~PointCloudColorizer() = default;

  /// Main colorize function (must compute a color (gray or RGB) for all possible point in
  /// GlobalPointCloud)
  virtual void Colorize() = 0;

  /// Return gray colors in range [0, 255]
  /// @return empty optional if no gray colorization
  virtual std::optional<std::vector<float>> getGray() {
    return {};
  }

  /// Return RGB colors in range [0, 255]
  /// @return empty optional if no RGB colorization
  virtual std::optional<std::vector<Eigen::Vector3f>> getRGB() {
    return {};
  }

 protected:
  const projectaria::tools::mps::GlobalPointCloud& globalPointCloud_;
  const projectaria::tools::mps::ClosedLoopTrajectory& trajectory_;
  const projectaria::tools::mps::PointObservations& pointObservations_;
  std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider_;
};
} // namespace
