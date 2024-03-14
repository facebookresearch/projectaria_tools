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

const float IPD_meters = 0.063;

namespace projectaria::tools::mps {

/*
 Read EyeGaze data from an MPS file.
*/
EyeGazes readEyeGaze(const std::string& path);

// Get Gaze Direction as Vector 3D given yaw and pitch values
inline Eigen::Vector3d getUnitVectorFromYawPitch(float yawRads, float pitchRads) {
  float z = 1; // arbitrary
  float x = std::tan(yawRads) * z;
  float y = std::tan(pitchRads) * z;
  Eigen::Vector3d direction(x, y, z);
  return direction.normalized();
}

/*
 Given the yaw and pitch angles of the eye gaze and a depth, return the gaze 3D point in CPF frame.
*/
inline Eigen::Vector3d getEyeGazePointAtDepth(float yawRads, float pitchRads, float depthM) {
  if (depthM >= 0.0) {
    Eigen::Vector3d unitVec = getUnitVectorFromYawPitch(yawRads, pitchRads);
    return unitVec * depthM;
  } else {
    return Eigen::Vector3d::Zero();
  }
}

// get 3D point of interesection of left and right gaze rays
inline Eigen::Vector3d
getGazeIntersectionPoint(float leftYawRads, float rightYawRads, float pitchRads) {
  float halfIPD = IPD_meters / 2.0;
  float intersectionX = halfIPD * (std::tan(leftYawRads) + std::tan(rightYawRads)) /
      (std::tan(rightYawRads) - std::tan(leftYawRads));
  float intersectionZ = IPD_meters / (std::tan(rightYawRads) - std::tan(leftYawRads));
  float intersectionY = intersectionZ * std::tan(pitchRads);
  return {intersectionX, intersectionY, intersectionZ};
}

// Compute depth and combined gaze angles (yaw and pitch) given left and right yaw and pitch angles
// combined pitch is the same as input pitch
inline std::tuple<float, float, float>
computeDepthAndCombinedGazeDirection(float leftYawRads, float rightYawRads, float pitchRads) {
  Eigen::Vector3d intersection = getGazeIntersectionPoint(leftYawRads, rightYawRads, pitchRads);
  float depthM = intersection.norm();
  float combinedYawRads = std::atan(intersection.x() / intersection.z());
  return std::make_tuple(depthM, combinedYawRads, pitchRads);
}

// get left and right gaze direction
inline std::tuple<Eigen::Vector3d, Eigen::Vector3d>
getGazeVectors(float leftYawRads, float rightYawRads, float pitchRads) {
  Eigen::Vector3d gazePoint = getGazeIntersectionPoint(leftYawRads, rightYawRads, pitchRads);
  Eigen::Vector3d leftOrigin = Eigen::Vector3d(IPD_meters / 2.0, 0, 0);
  Eigen::Vector3d rightOrigin = Eigen::Vector3d(-IPD_meters / 2.0, 0, 0);
  // left direction is from left origin to gaze point
  Eigen::Vector3d leftDirection = gazePoint - leftOrigin;
  // right direction is from right origin to gaze point
  Eigen::Vector3d rightDirection = gazePoint - rightOrigin;
  return std::make_tuple(leftDirection.normalized(), rightDirection.normalized());
}

} // namespace projectaria::tools::mps
