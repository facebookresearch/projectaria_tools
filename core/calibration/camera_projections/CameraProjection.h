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

#include <calibration/utility/NullRef.h>
#include <Eigen/Core>

namespace projectaria::tools::calibration {

/**
 * @brief A templated struct to represent a camera projection instance, which is basically camera
 * intrinsics. This templated struct stores the intrinsics parameters internally.
 */
template <typename Scalar>
struct CameraProjectionTemplated {
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
    Fisheye62, /**< Spherical + polynomial radial distortion up to 11-th order */
  };
  /**
   * @brief Default constructor, creates an empty CameraProjectionTemplated instance.
   */
  CameraProjectionTemplated() = default;

  /**
   * @brief Default copy constructor.
   */
  CameraProjectionTemplated(const CameraProjectionTemplated&) = default;

  /**
   * @brief Default move constructor.
   */
  CameraProjectionTemplated(CameraProjectionTemplated&&) = default;

  /**
   * @brief Default copy assignment.
   */
  CameraProjectionTemplated& operator=(const CameraProjectionTemplated&) = default;

  /**
   * @brief Default move assignment.
   */
  CameraProjectionTemplated& operator=(CameraProjectionTemplated&&) = default;

  /**
   * @brief Constructor with a list of parameters for CameraProjectionTemplated.
   * @param type The type of projection model, e.g. ModelType::Linear.
   * @param projectionParams The projection parameters.
   */
  CameraProjectionTemplated(
      const ModelType& type,
      const Eigen::Vector<Scalar, Eigen::Dynamic>& projectionParams);

  ModelType modelName() const;
  const Eigen::Vector<Scalar, Eigen::Dynamic>& projectionParams() const;
  Eigen::Vector<Scalar, Eigen::Dynamic>& projectionParamsMut();
  [[nodiscard]] int numParameters() const; // return number of parameters
  [[nodiscard]] int numProjectionParameters() const; // return number of projection parameters
  [[nodiscard]] int numDistortionParameters() const; // return number of distortion parameters

  /**
   * @brief projects a 3d world point in the camera space to a 2d pixel in the image space. No
   * checks performed in this process.
   *
   * @param pointInCamera The 3D point in camera space to be projected.
   * @param jacobianWrtPoint Optional, if not null, will store the Jacobian of the projection
   * with respect to the point in camera space.
   * @param jacobianWrtParams Optional, if not null, will store the Jacobian of the projection
   * with respect to the projection parameters.
   *
   * @return The 2D pixel coordinates of the projected point in the image space.
   */
  Eigen::Vector2<Scalar> project(
      const Eigen::Vector3<Scalar>& pointInCamera,
      Eigen::Ref<Eigen::Matrix<Scalar, 2, 3>> jacobianWrtPoint = NullRef(),
      Eigen::Ref<Eigen::Matrix<Scalar, 2, Eigen::Dynamic>> jacobianWrtParams = NullRef()) const;

  /**
   * @brief unprojects a 2d pixel in the image space to a 3d world point in homogenous coordinate.
   * No checks performed in this process.
   */
  Eigen::Vector3<Scalar> unproject(const Eigen::Vector2<Scalar>& cameraPixel) const;

  /**
   * @brief returns principal point location as {cx, cy}
   */
  Eigen::Vector2<Scalar> getPrincipalPoint() const;
  /**
   * @brief returns focal lengths as {fx, fy}
   */
  Eigen::Vector2<Scalar> getFocalLengths() const;

  /**
   * @brief scales the projection parameters as the image scales without the offset changing
   */
  void scaleParams(Scalar scale);
  /**
   * @brief translates the origin by (offsetU, offsetV)
   */
  void subtractFromOrigin(Scalar offsetU, Scalar offsetV);

  using ProjectionVariant = std::variant<
      LinearProjection,
      SphericalProjection,
      KannalaBrandtK3Projection,
      Fisheye624,
      Fisheye62>;

  /**
   * @brief Casts the CameraProjectionTemplated to another scalar type.
   * @tparam OtherScalar The target scalar type for casting.
   * @return A new CameraProjectionTemplated instance with the specified scalar type.
   */
  template <typename OtherScalar>
  CameraProjectionTemplated<OtherScalar> cast() const;

 private:
  ModelType modelName_;
  Eigen::Vector<Scalar, Eigen::Dynamic> projectionParams_;
  ProjectionVariant projectionVariant_;
};
using CameraProjection = CameraProjectionTemplated<double>;

} // namespace projectaria::tools::calibration
