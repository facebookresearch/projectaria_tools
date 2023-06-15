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

#include <optional>
#include <vector>

namespace projectaria::tools::mps {

//
// Static camera intrinsic calibration and extrinsics in the world frame
//

struct StaticCameraCalibration {
  // Static camera's unique identifier, currently we are using path of the video file
  std::string cameraUid;

  // UID of the world coordinate frame
  std::string graphUid;

  // Static camera's pose in world frame
  Sophus::SE3d T_world_cam;

  // image size
  int width, height;

  // intrinsics type string
  // currently the only intrinsics type supported is "KANNALABRANDTK3", a.k.a OpenCV fisheye model
  std::string intrinsicsType;

  // cam intrinsic calibration params
  Eigen::Matrix<float, 8, 1> intrinsics;

  // The start and end frame number from the video when the camera is stationary and camera pose
  // result is applicable. Not available, when the pose is applicable to the whole video
  std::optional<int> startFrameIdx, endFrameIdx;
};

using StaticCameraCalibrations = std::vector<StaticCameraCalibration>;

} // namespace projectaria::tools::mps
