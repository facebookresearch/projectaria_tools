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

#include "MyCustomCameraModel.h"

Eigen::Vector2d MyCustomCameraModel::projectWithRadialDistortion(
    const Eigen::Vector3d& pointOptical,
    const Eigen::Vector4d& intrinsics) const {
  Eigen::Vector2d pixelsDistorted = project(pointOptical, intrinsics);
  return applyRadialDistortion(intrinsics, pixelsDistorted);
}

Eigen::Vector2d MyCustomCameraModel::applyRadialDistortion(
    const Eigen::Vector4d& intrinsics,
    const Eigen::Vector2d& pixelUndistorted) const {
  double x = pixelUndistorted[0];
  double y = pixelUndistorted[1];
  double fx = intrinsics[0];
  double fy = intrinsics[1];
  double cx = intrinsics[2];
  double cy = intrinsics[3];

  double xd = (x - cx) / fx;
  double yd = (y - cy) / fy;
  double r2 = xd * xd + yd * yd;
  double dist = (k1_ * r2 + k2_ * r2 * r2);
  double xx = x + (x - cx) * dist;
  double yy = y + (y - cy) * dist;
  return {xx, yy};
}

Eigen::Vector2d MyCustomCameraModel::getDistortionCoefficients() const {
  return {k1_, k2_};
}
