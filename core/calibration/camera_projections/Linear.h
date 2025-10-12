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

#include <cmath>
#include <string_view>

#include <calibration/camera_projections/Common.h>

namespace projectaria::tools::calibration {

// Pinhole model
//
// parameters = fx, fy, cx, cy
// x = u * fx + cx
// y = v * fy + cy
class LinearProjection {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kNumParams = 4;
  static constexpr std::string_view kName = "Linear";
  static constexpr std::string_view kDescription = "fx, fy, cx, cy";
  static constexpr int kNumDistortionParams = 0;
  static constexpr int kFocalXIdx = 0;
  static constexpr int kFocalYIdx = 1;
  static constexpr int kPrincipalPointColIdx = 2;
  static constexpr int kPrincipalPointRowIdx = 3;
  static constexpr bool kIsFisheye = false;
  static constexpr bool kHasAnalyticalProjection = true;

  template <class D, class DP, class DJ = Eigen::Matrix<typename D::Scalar, 2, 3>>
  static Eigen::Matrix<typename D::Scalar, 2, 1> project(
      const Eigen::MatrixBase<D>& pointOptical,
      const Eigen::MatrixBase<DP>& params,
      Eigen::MatrixBase<DJ>* d_point = nullptr) {
    validateProjectInput<D, DP, kNumParams>();
    using T = typename D::Scalar;

    SOPHUS_ENSURE(pointOptical.z() != T(0), "z(%) must not be zero.", pointOptical.z());

    // Focal length and principal point
    const Eigen::Matrix<T, 2, 1> ff = params.template head<2>();
    const Eigen::Matrix<T, 2, 1> pp = params.template segment<2>(2);
    const Eigen::Matrix<T, 2, 1> px =
        ff.cwiseProduct(pointOptical.template head<2>()) / pointOptical(2) + pp;

    if (d_point) {
      const T oneOverZ = T(1) / pointOptical(2);

      (*d_point)(0, 0) = ff(0) * oneOverZ;
      (*d_point)(0, 1) = static_cast<T>(0.0);
      (*d_point)(0, 2) = -(*d_point)(0, 0) * pointOptical(0) * oneOverZ;
      (*d_point)(1, 0) = static_cast<T>(0.0);
      (*d_point)(1, 1) = ff(1) * oneOverZ;
      (*d_point)(1, 2) = -(*d_point)(1, 1) * pointOptical(1) * oneOverZ;
    }

    return px;
  }

  template <typename D, typename DP>
  static Eigen::Matrix<typename D::Scalar, 3, 1> unproject(
      const Eigen::MatrixBase<D>& uvPixel,
      const Eigen::MatrixBase<DP>& params) {
    validateUnprojectInput<D, DP, kNumParams>();
    using T = typename D::Scalar;

    // Unprojection
    const T fu = params[0];
    const T fv = params[1];
    const T u0 = params[2];
    const T v0 = params[3];

    const T un = (uvPixel(0) - u0) / fu;
    const T vn = (uvPixel(1) - v0) / fv;
    return Eigen::Matrix<T, 3, 1>(un, vn, T(1.0));
  }

  // Return scaled parameters to cope with scaled image.
  template <class Scalar, class DP>
  static void scaleParams(Scalar s, Eigen::DenseBase<DP>& params) {
    validateScaleParamInput<Scalar, DP, kNumParams>();

    using T = typename DP::Scalar;
    const typename DP::Scalar& scale = s;
    params[0] *= scale;
    params[1] *= scale;
    params[2] = scale * (params[2] + 0.5) - T(0.5);
    params[3] = scale * (params[3] + 0.5) - T(0.5);
    // Distortion coefficients aren't effected by scale
  }

  // Subtract values from optical center, useful when sensor plane is cropped
  template <typename DP>
  static void
  subtractFromOrigin(typename DP::Scalar u, typename DP::Scalar v, Eigen::DenseBase<DP>& params) {
    params[2] -= u;
    params[3] -= v;
  }
};

} // namespace projectaria::tools::calibration
