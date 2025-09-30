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

#include "Distort.h"
#include "image/utility/Distort.h"

namespace projectaria::tools::calibration {

image::ManagedImageVariant distortByCalibration(
    const image::ImageVariant& srcVariant,
    const CameraCalibration& dstCalib,
    const CameraCalibration& srcCalib,
    const image::InterpolationMethod method) {
  auto inverseWarp =
      [&dstCalib, &srcCalib](const Eigen::Vector2f& dstPixel) -> std::optional<Eigen::Vector2f> {
    Eigen::Vector3d rayDir = dstCalib.unprojectNoChecks(dstPixel.template cast<double>());
    std::optional<Eigen::Vector2d> maybeSrcPixel = srcCalib.project(rayDir);
    if (!maybeSrcPixel) {
      return {};
    } else {
      return maybeSrcPixel->template cast<float>();
    }
  };
  return distortImageVariant(srcVariant, inverseWarp, dstCalib.getImageSize(), method);
}

// version with rotation
image::ManagedImageVariant distortByCalibrationAndApplyRotation(
    const image::ImageVariant& srcVariant,
    const CameraCalibration& dstCalib,
    const CameraCalibration& srcCalib,
    const Sophus::SO3d& so3_srcCalib_dstCalib,
    const image::InterpolationMethod method) {
  auto inverseWarp = [&dstCalib, &srcCalib, &so3_srcCalib_dstCalib](
                         const Eigen::Vector2f& dstPixel) -> std::optional<Eigen::Vector2f> {
    Eigen::Vector3d rayDir = dstCalib.unprojectNoChecks(dstPixel.template cast<double>());
    // apply rotation
    rayDir = so3_srcCalib_dstCalib * rayDir;
    std::optional<Eigen::Vector2d> maybeSrcPixel = srcCalib.project(rayDir);
    if (!maybeSrcPixel) {
      return {};
    } else {
      return maybeSrcPixel->template cast<float>();
    }
  };
  return distortImageVariant(srcVariant, inverseWarp, dstCalib.getImageSize(), method);
}

image::ManagedImageVariant distortDepthByCalibration(
    const image::ImageVariant& srcVariant,
    const CameraCalibration& dstCalib,
    const CameraCalibration& srcCalib) {
  return distortByCalibration(srcVariant, dstCalib, srcCalib, image::InterpolationMethod::Bilinear);
}

image::ManagedImageVariant distortLabelMaskByCalibration(
    const image::ImageVariant& srcVariant,
    const CameraCalibration& dstCalib,
    const CameraCalibration& srcCalib) {
  return distortByCalibration(
      srcVariant, dstCalib, srcCalib, image::InterpolationMethod::NearestNeighbor);
}

} // namespace projectaria::tools::calibration
