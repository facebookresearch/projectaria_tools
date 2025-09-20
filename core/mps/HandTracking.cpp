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

#include "HandTracking.h"
#include <sophus/interpolate.hpp>
#include <algorithm>
#include <stdexcept>

namespace projectaria::tools::mps {

Eigen::Vector3d estimatePalmNormal(const Landmarks& landmarks, HANDEDNESS handedness) {
  constexpr auto kWristIndex = static_cast<uint8_t>(HandLandmark::WRIST);
  constexpr auto kIndexProximalIndex = static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL);
  constexpr auto kPinkyProximalIndex = static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL);

  const Eigen::Vector3d& wristLocation = landmarks[kWristIndex];
  const Eigen::Vector3d& indexProximalLocation = landmarks[kIndexProximalIndex];
  const Eigen::Vector3d& pinkyProximalLocation = landmarks[kPinkyProximalIndex];

  // Calculate palm normal using cross product
  Eigen::Vector3d palmNormal = (pinkyProximalLocation - wristLocation)
                                   .cross(indexProximalLocation - wristLocation)
                                   .normalized();

  // For right hand, flip the normal
  if (handedness == HANDEDNESS::RIGHT) {
    palmNormal *= -1.0;
  }

  return palmNormal;
}

} // namespace projectaria::tools::mps
