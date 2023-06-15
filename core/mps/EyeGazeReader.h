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

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <chrono>
#include <filesystem>
#include <map>
#include <optional>
#include <string>

#include "EyeGaze.h"
#include "calibration/DeviceCalibration.h"

namespace projectaria::tools::mps {

/*
 Read EyeGaze data from an MPS file.
*/
EyeGazes readEyeGaze(const std::string& path);

/*
 Given the yaw and pitch angles of the eye gaze and a depth, return the gaze 3D point in CPF frame.
*/
inline Eigen::Vector3d getEyeGazePointAtDepth(float yawRads, float pitchRads, float depthM) {
  return Eigen::Vector3d(tan(yawRads) * depthM, tan(pitchRads) * depthM, depthM);
}

} // namespace projectaria::tools::mps
