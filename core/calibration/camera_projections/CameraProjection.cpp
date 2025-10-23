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
#include <stdexcept>

namespace projectaria::tools::calibration {
template <typename Scalar>
typename CameraProjectionTemplated<Scalar>::ProjectionVariant getProjectionVariant(
    const typename CameraProjectionTemplated<Scalar>::ModelType& type) {
  switch (type) {
    case CameraProjectionTemplated<Scalar>::ModelType::Linear:
      return LinearProjection{};
    case CameraProjectionTemplated<Scalar>::ModelType::Spherical:
      return SphericalProjection{};
    case CameraProjectionTemplated<Scalar>::ModelType::KannalaBrandtK3:
      return KannalaBrandtK3Projection{};
    case CameraProjectionTemplated<Scalar>::ModelType::Fisheye624:
      return Fisheye624{};
    case CameraProjectionTemplated<Scalar>::ModelType::Fisheye62:
      return Fisheye62{};
  }
  throw std::runtime_error("Unrecognized camera model.");
}

template <typename Scalar>
CameraProjectionTemplated<Scalar>::CameraProjectionTemplated(
    const ModelType& type,
    const Eigen::Vector<Scalar, Eigen::Dynamic>& projectionParams)
    : modelName_(type),
      projectionParams_(projectionParams),
      projectionVariant_(getProjectionVariant<Scalar>(type)) {}

template <typename Scalar>
typename CameraProjectionTemplated<Scalar>::ModelType CameraProjectionTemplated<Scalar>::modelName()
    const {
  return modelName_;
}

template <typename Scalar>
const Eigen::Vector<Scalar, Eigen::Dynamic>& CameraProjectionTemplated<Scalar>::projectionParams() 
    const {
  return projectionParams_;
}

template <typename Scalar>
Eigen::Vector<Scalar, Eigen::Dynamic>& CameraProjectionTemplated<Scalar>::projectionParamsMut() {
  return projectionParams_;
}

template <typename Scalar>
int CameraProjectionTemplated<Scalar>::numParameters() const {
  return std::visit(
      [this](auto&& projection) -> int {
        using T = std::decay_t<decltype(projection)>;
        return T::kNumParams;
      },
      projectionVariant_);
}

template <typename Scalar>
int CameraProjectionTemplated<Scalar>::numProjectionParameters() const {
  return std::visit(
      [this](auto&& projection) -> int {
        using T = std::decay_t<decltype(projection)>;
        return T::kNumParams - T::kNumDistortionParams;
      },
      projectionVariant_);
}

template <typename Scalar>
int CameraProjectionTemplated<Scalar>::numDistortionParameters() const {
  return std::visit(
      [this](auto&& projection) -> int {
        using T = std::decay_t<decltype(projection)>;
        return T::kNumDistortionParams;
      },
      projectionVariant_);
}

template <typename Scalar>
Eigen::Vector<Scalar, 2> CameraProjectionTemplated<Scalar>::getFocalLengths() const {
  return std::visit(
      [this](auto&& projection) -> Eigen::Vector<Scalar, 2> {
        using T = std::decay_t<decltype(projection)>;
        int focalXIdx = T::kFocalXIdx;
        int focalYIdx = T::kFocalYIdx;
        return {projectionParams_(focalXIdx), projectionParams_(focalYIdx)};
      },
      projectionVariant_);
}

template <typename Scalar>
Eigen::Vector<Scalar, 2> CameraProjectionTemplated<Scalar>::getPrincipalPoint() const {
  return std::visit(
      [this](auto&& projection) -> Eigen::Vector<Scalar, 2> {
        using T = std::decay_t<decltype(projection)>;
        int principalPointColIdx = T::kPrincipalPointColIdx;
        int principalPointRowIdx = T::kPrincipalPointRowIdx;
        return {projectionParams_(principalPointColIdx), projectionParams_(principalPointRowIdx)};
      },
      projectionVariant_);
}

template <typename Scalar>
Eigen::Vector<Scalar, 2> CameraProjectionTemplated<Scalar>::project(
    const Eigen::Vector<Scalar, 3>& pointInCamera,
    Eigen::Matrix<Scalar, 2, 3>* jacobianWrtPoint,
    Eigen::Matrix<Scalar, 2, Eigen::Dynamic>* jacobianWrtParams) const {
  return std::visit(
      [&](auto&& projection) {
        using T = std::decay_t<decltype(projection)>;
        return T::project(pointInCamera, projectionParams_, jacobianWrtPoint, jacobianWrtParams);
      },
      projectionVariant_);
}

template <typename Scalar>
Eigen::Vector<Scalar, 3> CameraProjectionTemplated<Scalar>::unproject(
    const Eigen::Vector<Scalar, 2>& cameraPixel) const {
  return std::visit(
      [&](auto&& projection) {
        using T = std::decay_t<decltype(projection)>;
        return T::unproject(cameraPixel, projectionParams_);
      },
      projectionVariant_);
}

template <typename Scalar>
void CameraProjectionTemplated<Scalar>::scaleParams(Scalar scale) {
  return std::visit(
      [&](auto&& projection) {
        using T = std::decay_t<decltype(projection)>;
        return T::scaleParams(scale, projectionParams_);
      },
      projectionVariant_);
}

template <typename Scalar>
void CameraProjectionTemplated<Scalar>::subtractFromOrigin(Scalar offsetU, Scalar offsetV) {
  return std::visit(
      [&](auto&& projection) {
        using T = std::decay_t<decltype(projection)>;
        return T::subtractFromOrigin(offsetU, offsetV, projectionParams_);
      },
      projectionVariant_);
}

template <typename Scalar>
template <typename OtherScalar>
[[nodiscard]] CameraProjectionTemplated<OtherScalar> CameraProjectionTemplated<Scalar>::cast()
    const {
  Eigen::Vector<OtherScalar, Eigen::Dynamic> castedParams =
      projectionParams_.template cast<OtherScalar>();
  auto castedModelName =
      static_cast<typename CameraProjectionTemplated<OtherScalar>::ModelType>(modelName_);
  return CameraProjectionTemplated<OtherScalar>(castedModelName, castedParams);
}

template struct CameraProjectionTemplated<double>;
template struct CameraProjectionTemplated<float>;
template CameraProjectionTemplated<float> CameraProjectionTemplated<double>::cast<float>() const;
template CameraProjectionTemplated<double> CameraProjectionTemplated<float>::cast<double>() const;
} // namespace projectaria::tools::calibration
