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

#include <deque>

#include <Eigen/Core>

struct EyeGazeVisualizationData {
  // Last current valid EyeGaze recording {yaw, pitch [rad]}
  Eigen::Vector2d lastYawPitch;
  // A rolling buffer history of EyeGaze yaw, pitch recordings
  std::deque<Eigen::Vector2d> yawPitchHistory;
};
