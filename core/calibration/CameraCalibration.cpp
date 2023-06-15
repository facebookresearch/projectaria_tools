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

#include <calibration/CameraCalibration.h>

namespace projectaria::tools::calibration {
CameraCalibration::CameraCalibration(
    const std::string& label,
    const CameraProjection::ModelType& projectionModelType,
    const Eigen::VectorXd& projectionParams,
    const Sophus::SE3d& T_Device_Camera,
    const int imageWidth,
    const int imageHeight,
    const std::optional<double> maybeValidRadius,
    const double maxSolidAngle)
    : label_(label),
      projectionModel_(projectionModelType, projectionParams),
      T_Device_Camera_(T_Device_Camera),
      imageWidth_(imageWidth),
      imageHeight_(imageHeight),
      maybeValidRadius_(maybeValidRadius),
      maxSolidAngle_(maxSolidAngle) {}

std::string CameraCalibration::getLabel() const {
  return label_;
}

Sophus::SE3d CameraCalibration::getT_Device_Camera() const {
  return T_Device_Camera_;
}

Eigen::Vector2i CameraCalibration::getImageSize() const {
  return Eigen::Vector2i(imageWidth_, imageHeight_);
}

namespace {
// A helper function to determine if a pixel is within valid mask in a camera
inline bool checkPixelValidInMask(
    const Eigen::Vector2d& cameraPixel,
    const Eigen::Vector2d& principalPoint,
    const std::optional<double> validRadius) {
  return !validRadius ||
      (cameraPixel - principalPoint).squaredNorm() <= validRadius.value() * validRadius.value();
}

inline bool
checkPixelInImageBound(const Eigen::Vector2d& cameraPixel, int imageWidth, int imageHeight) {
  // pixel coord convention note:
  // pixel (x, y) in discrete coord space means in continuous space the pixel's center is at (x, y)
  // thus the valid image sensor area in the continuous space is
  //   [-0.5, imageWidth - 0.5] x [-0.5, imageHeight - 0.5]
  Eigen::AlignedBox2d imgBound(
      Eigen::Vector2d(-0.5, -0.5), Eigen::Vector2d(imageWidth - 0.5, imageHeight - 0.5));
  return imgBound.contains(cameraPixel);
}

inline bool checkPointInVisibleCone(const Eigen::Vector3d& worldPoint, double maxSolidAngle) {
  double solidAngle = std::atan2(worldPoint.head(2).norm(), worldPoint.z());
  return solidAngle <= maxSolidAngle;
}
} // namespace

bool CameraCalibration::isVisible(const Eigen::Vector2d& cameraPixel) const {
  return checkPixelInImageBound(cameraPixel, imageWidth_, imageHeight_) &&
      checkPixelValidInMask(cameraPixel, projectionModel_.getPrincipalPoint(), maybeValidRadius_);
}

CameraProjection::ModelType CameraCalibration::modelName() const {
  return projectionModel_.modelName();
}

Eigen::VectorXd CameraCalibration::projectionParams() const {
  return projectionModel_.projectionParams();
}

Eigen::Vector2d CameraCalibration::getPrincipalPoint() const {
  return projectionModel_.getPrincipalPoint();
}

Eigen::Vector2d CameraCalibration::getFocalLengths() const {
  return projectionModel_.getFocalLengths();
}

Eigen::Vector2d CameraCalibration::projectNoChecks(const Eigen::Vector3d& pointInCamera) const {
  return projectionModel_.project(pointInCamera);
}

std::optional<Eigen::Vector2d> CameraCalibration::project(
    const Eigen::Vector3d& pointInCamera) const {
  // check point is in front of camera
  if (checkPointInVisibleCone(pointInCamera, maxSolidAngle_)) {
    const Eigen::Vector2d cameraPixel = projectNoChecks(pointInCamera);
    if (isVisible(cameraPixel)) {
      return cameraPixel;
    }
  }
  return {};
}

Eigen::Vector3d CameraCalibration::unprojectNoChecks(const Eigen::Vector2d& cameraPixel) const {
  return projectionModel_.unproject(cameraPixel);
}

std::optional<Eigen::Vector3d> CameraCalibration::unproject(
    const Eigen::Vector2d& cameraPixel) const {
  if (isVisible(cameraPixel)) {
    return unprojectNoChecks(cameraPixel);
  }
  return {};
}

CameraCalibration CameraCalibration::rescale(
    const Eigen::Vector2i& newResolution,
    const double scale, // scaling factor
    const Eigen::Vector2d& originOffset) const {
  CameraCalibration camCalib(*this);

  // Rescale camera model, currently only supporting isometric scaling on X and Y
  camCalib.projectionModel_.subtractFromOrigin(originOffset.x(), originOffset.y());
  camCalib.projectionModel_.scaleParams(scale);

  // Scale mask.
  if (camCalib.maybeValidRadius_) {
    camCalib.maybeValidRadius_.value() *= scale;
  }

  // Update resolution
  camCalib.imageWidth_ = newResolution.x();
  camCalib.imageHeight_ = newResolution.y();
  return camCalib;
}

CameraCalibration getLinearCameraCalibration(
    const int imageWidth,
    const int imageHeight,
    const double focalLength,
    const std::string& label) {
  CameraProjection::ModelType type = CameraProjection::ModelType::Linear;
  Eigen::VectorXd projectionParams(4);
  projectionParams << focalLength, focalLength, double(imageWidth - 1) / 2.0,
      double(imageHeight - 1) / 2.0;
  return CameraCalibration(
      label, type, projectionParams, Sophus::SE3d{}, imageWidth, imageHeight, std::nullopt, M_PI);
}

CameraCalibration getSphericalCameraCalibration(
    const int imageWidth,
    const int imageHeight,
    const double focalLength,
    const std::string& label) {
  CameraProjection::ModelType type = CameraProjection::ModelType::Spherical;
  Eigen::VectorXd projectionParams(4);
  projectionParams << focalLength, focalLength, double(imageWidth - 1) / 2.0,
      double(imageHeight - 1) / 2.0;
  return CameraCalibration(
      label, type, projectionParams, Sophus::SE3d{}, imageWidth, imageHeight, std::nullopt, M_PI);
}

} // namespace projectaria::tools::calibration
