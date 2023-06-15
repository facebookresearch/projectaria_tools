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

#include <calibration/camera_projections/CameraProjection.h>

namespace projectaria::tools::calibration {
CameraProjection::ProjectionVariant getProjectionVariant(const CameraProjection::ModelType& type) {
  switch (type) {
    case CameraProjection::ModelType::Linear:
      return LinearProjection{};
    case CameraProjection::ModelType::Spherical:
      return SphericalProjection{};
    case CameraProjection::ModelType::KannalaBrandtK3:
      return KannalaBrandtK3Projection{};
    case CameraProjection::ModelType::Fisheye624:
      return Fisheye624{};
  }
}

CameraProjection::CameraProjection(const ModelType& type, const Eigen::VectorXd& projectionParams)
    : modelName_(type),
      projectionParams_(projectionParams),
      projectionVariant_(getProjectionVariant(type)) {}

CameraProjection::ModelType CameraProjection::modelName() const {
  return modelName_;
}

Eigen::VectorXd CameraProjection::projectionParams() const {
  return projectionParams_;
}

Eigen::Vector2d CameraProjection::getFocalLengths() const {
  return std::visit(
      [this](auto&& projection) {
        using T = std::decay_t<decltype(projection)>;
        int focalXIdx = T::kFocalXIdx;
        int focalYIdx = T::kFocalYIdx;
        return Eigen::Vector2d{projectionParams_(focalXIdx), projectionParams_(focalYIdx)};
      },
      projectionVariant_);
}

Eigen::Vector2d CameraProjection::getPrincipalPoint() const {
  return std::visit(
      [this](auto&& projection) {
        using T = std::decay_t<decltype(projection)>;
        int principalPointColIdx = T::kPrincipalPointColIdx;
        int principalPointRowIdx = T::kPrincipalPointRowIdx;
        return Eigen::Vector2d{
            projectionParams_(principalPointColIdx), projectionParams_(principalPointRowIdx)};
      },
      projectionVariant_);
}

Eigen::Vector2d CameraProjection::project(const Eigen::Vector3d& pointInCamera) const {
  return std::visit(
      [&](auto&& projection) {
        using T = std::decay_t<decltype(projection)>;
        return T::project(pointInCamera, projectionParams_);
      },
      projectionVariant_);
}

Eigen::Vector3d CameraProjection::unproject(const Eigen::Vector2d& cameraPixel) const {
  return std::visit(
      [&](auto&& projection) {
        using T = std::decay_t<decltype(projection)>;
        return T::unproject(cameraPixel, projectionParams_);
      },
      projectionVariant_);
}

void CameraProjection::scaleParams(double scale) {
  return std::visit(
      [&](auto&& projection) {
        using T = std::decay_t<decltype(projection)>;
        return T::scaleParams(scale, projectionParams_);
      },
      projectionVariant_);
}

void CameraProjection::subtractFromOrigin(double offsetU, double offsetV) {
  return std::visit(
      [&](auto&& projection) {
        using T = std::decay_t<decltype(projection)>;
        return T::subtractFromOrigin(offsetU, offsetV, projectionParams_);
      },
      projectionVariant_);
}

} // namespace projectaria::tools::calibration
