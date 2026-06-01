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

// get 3D point of intersection of left and right gaze rays
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

// Distance at which `getGazeVergencePoint` plants its far-gaze fallback when the input geometry
// is degenerate. Exposed because downstream consumers may want to suppress points at this
// distance.
constexpr double kVergenceFarFallbackDistanceM = 10.0;

// Vergence point of the two eye gaze rays in the frame of the supplied origins (typically CPF).
// Generalizes `getGazeIntersectionPoint` to the case where each eye has an independent pitch
// and the inter-pupillary distance is not the hardcoded 63 mm. Each per-eye direction is built
// from its (yaw, pitch) using the standard polar convention
// (x = sin(yaw)·cos(pitch), y = sin(pitch), z = cos(yaw)·cos(pitch)), then the midpoint of the
// shortest segment connecting the two skew rays is returned.
//
// Returns `(vergencePoint, isReal)`. `isReal` is true when the result is the geometric
// closest-approach midpoint. It is false when the inputs are degenerate (near-parallel rays,
// divergent rays, skew rays whose closest points are too far apart, or convergence beyond a
// reliable depth) — in that case the returned point is a synthetic forward fallback at
// `kVergenceFarFallbackDistanceM` along the fused direction so downstream consumers always
// receive a forward, physically plausible point but can choose to ignore it.
std::pair<Eigen::Vector3d, bool> getGazeVergencePoint(
    const Eigen::Vector3d& leftEyeOrigin,
    float leftYawRads,
    float leftPitchRads,
    const Eigen::Vector3d& rightEyeOrigin,
    float rightYawRads,
    float rightPitchRads);

} // namespace projectaria::tools::mps
