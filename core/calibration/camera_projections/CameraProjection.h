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

#include <map>
#include <string>
#include <variant>

#include <calibration/camera_projections/FisheyeRadTanThinPrism.h>
#include <calibration/camera_projections/KannalaBrandtK3.h>
#include <calibration/camera_projections/Linear.h>
#include <calibration/camera_projections/Spherical.h>

#include <Eigen/Core>

namespace projectaria::tools::calibration {

/**
 * @brief A struct to represent a camera projection instance, which is basically camera intrinsics.
 * This struct stores the intrinsics parameters internally.
 */
struct CameraProjection {
  /**
   * @brief Enum that represents the type of camera projection model. See Linear.h, Spherical.h,
   * KannalaBrandtK3.h and FisheyeRadTanThinPrism.h for details
   */
  enum class ModelType {
    Linear, /**< Linear pinhole projection, unit plane points and camera pixels are linearly related
             */
    Spherical, /**< Spherical projection, linear in angular space */
    KannalaBrandtK3, /**< Spherical + polynomial radial distortion up to 9-th order */
    Fisheye624, /**< Spherical + polynomial radial distortion up to 11-th order + tangential
                distortion */
  };
  /**
   * @brief Default constructor, creates an empty CameraProjection instance.
   */
  CameraProjection() = default;

  /**
   * @brief Constructor with a list of parameters for CameraProjection.
   * @param type The type of projection model, e.g. ModelType::Linear.
   * @param projectionParams The projection parameters.
   */
  CameraProjection(const ModelType& type, const Eigen::VectorXd& projectionParams);

  ModelType modelName() const;
  Eigen::VectorXd projectionParams() const;

  /**
   * @brief projects a 3d world point in the camera space to a 2d pixel in the image space. No
   * checks performed in this process.
   */
  Eigen::Vector2d project(const Eigen::Vector3d& pointInCamera) const;

  /**
   * @brief unprojects a 2d pixel in the image space to a 3d world point in homogenous coordinate.
   * No checks performed in this process.
   */
  Eigen::Vector3d unproject(const Eigen::Vector2d& cameraPixel) const;

  /**
   * @brief returns principal point location as {cx, cy}
   */
  Eigen::Vector2d getPrincipalPoint() const;
  /**
   * @brief returns focal lengths as {fx, fy}
   */
  Eigen::Vector2d getFocalLengths() const;

  /**
   * @brief scales the projection parameters as the image scales without the offset changing
   */
  void scaleParams(double scale);
  /**
   * @brief translates the origin by (offsetU, offsetV)
   */
  void subtractFromOrigin(double offsetU, double offsetV);

  using ProjectionVariant =
      std::variant<LinearProjection, SphericalProjection, KannalaBrandtK3Projection, Fisheye624>;

 private:
  ModelType modelName_;
  Eigen::VectorXd projectionParams_;
  ProjectionVariant projectionVariant_;
};
} // namespace projectaria::tools::calibration
