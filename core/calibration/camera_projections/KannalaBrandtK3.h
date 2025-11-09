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

#include <calibration/camera_projections/Common.h>

namespace projectaria::tools::calibration {

// Kannala and Brandt Like 'Generic' Projection Model
// http://cs.iupui.edu/~tuceryan/pdf-repository/Kannala2006.pdf
// https://april.eecs.umich.edu/wiki/Camera_suite
// NOTE, our implementation presents some important differences wrt the original paper:
// - k1 in eq(6) in the paper is fixed to 1.0, so k0 here is k2 in the paper
//   (for the same reason we have only x4 k parameters instead of x5 in the paper, for order
//   theta^9)
//
// In this projection model the points behind the camera are projected in a way that the
// optimization cost function is continuous, therefore the optimization problem can be nicely
// solved. This option should be used during calibration.
//
// parameters = fx, fy, cx, cy, kb0, kb1, kb2, kb3
class KannalaBrandtK3Projection {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kNumParams = 8;
  static constexpr char kName[] = "KannalaBrandtK3";
  static constexpr char kDescription[] = "fx, fy, cx, cy, kb0, kb1, kb2, kb3";
  static constexpr int kNumDistortionParams = 4;
  static constexpr int kFocalXIdx = 0;
  static constexpr int kFocalYIdx = 1;
  static constexpr int kPrincipalPointColIdx = 2;
  static constexpr int kPrincipalPointRowIdx = 3;
  static constexpr bool kIsFisheye = true;
  static constexpr bool kHasAnalyticalProjection = true;

  template <
      class D,
      class DP,
      class DJ1 = Eigen::Matrix<typename D::Scalar, 2, 3>,
      class DJ2 = Eigen::Matrix<typename D::Scalar, 2, kNumParams>>
  static Eigen::Matrix<typename D::Scalar, 2, 1> project(
      const Eigen::MatrixBase<D>& pointOptical,
      const Eigen::MatrixBase<DP>& params,
      Eigen::MatrixBase<DJ1>* d_point = nullptr,
      Eigen::MatrixBase<DJ2>* d_params = nullptr) {
    validateProjectInput<D, DP, kNumParams>();

    using T = typename D::Scalar;

    SOPHUS_ENSURE(pointOptical.z() != T(0), "z(%) must not be zero.", pointOptical.z());

    // Focal length and principal point
    const Eigen::Matrix<T, 2, 1> ff = params.template head<2>();
    const Eigen::Matrix<T, 2, 1> pp = params.template segment<2>(2);

    const T k0 = params[4];
    const T k1 = params[5];
    const T k2 = params[6];
    const T k3 = params[7];

    const T radiusSquared = pointOptical(0) * pointOptical(0) + pointOptical(1) * pointOptical(1);
    using std::atan2;
    using std::sqrt;

    if (radiusSquared > Sophus::Constants<T>::epsilon()) {
      const T radius = sqrt(radiusSquared);
      const T radiusInverse = T(1.0) / radius;
      const T theta = atan2(radius, pointOptical(2));
      const T theta2 = theta * theta;
      const T theta4 = theta2 * theta2;
      const T theta6 = theta4 * theta2;
      const T theta8 = theta4 * theta4;
      const T rDistorted = theta * (T(1.0) + k0 * theta2 + k1 * theta4 + k2 * theta6 + k3 * theta8);
      const T scaling = rDistorted * radiusInverse;

      if (d_point) {
        const T xSquared = pointOptical(0) * pointOptical(0);
        const T ySquared = pointOptical(1) * pointOptical(1);
        const T normSquared = pointOptical(2) * pointOptical(2) + radiusSquared;
        const T rDistortedDerivative = T(1.0) + T(3.0) * k0 * theta2 + T(5.0) * k1 * theta4 +
            T(7.0) * k2 * theta6 + T(9.0) * k3 * theta8;
        const T x13 = pointOptical(2) * rDistortedDerivative / normSquared - scaling;
        const T rDistortedDerivativeNormalized = rDistortedDerivative / normSquared;
        const T x20 =
            pointOptical(2) * rDistortedDerivative / (normSquared)-radiusInverse * rDistorted;

        (*d_point)(0, 0) = xSquared / radiusSquared * x20 + scaling;
        (*d_point)(0, 1) = pointOptical(1) * x13 * pointOptical(0) / radiusSquared;
        (*d_point)(0, 2) = -pointOptical(0) * rDistortedDerivativeNormalized;
        (*d_point)(1, 0) = (*d_point)(0, 1);
        (*d_point)(1, 1) = ySquared / radiusSquared * x20 + scaling;
        (*d_point)(1, 2) = -pointOptical(1) * rDistortedDerivativeNormalized;

        // toDenseMatrix() is needed for CUDA to explicitly know the matrix dimensions
        (*d_point) = ff.asDiagonal().toDenseMatrix() * (*d_point);
      }

      if (d_params) {
        const T xScaled = pointOptical[0] * params[0] * radiusInverse;
        const T yScaled = pointOptical[1] * params[1] * radiusInverse;

        const T theta3 = theta * theta2;
        const T theta5 = theta3 * theta2;
        const T theta7 = theta5 * theta2;
        const T theta9 = theta7 * theta2;

        (*d_params)(0, 0) = pointOptical[0] * scaling;
        (*d_params)(0, 1) = T(0.0);
        (*d_params)(0, 2) = T(1.0);
        (*d_params)(0, 3) = T(0.0);
        (*d_params)(0, 4) = xScaled * theta3;
        (*d_params)(0, 5) = xScaled * theta5;
        (*d_params)(0, 6) = xScaled * theta7;
        (*d_params)(0, 7) = xScaled * theta9;
        (*d_params)(1, 0) = T(0.0);
        (*d_params)(1, 1) = pointOptical[1] * scaling;
        (*d_params)(1, 2) = T(0.0);
        (*d_params)(1, 3) = T(1.0);
        (*d_params)(1, 4) = yScaled * theta3;
        (*d_params)(1, 5) = yScaled * theta5;
        (*d_params)(1, 6) = yScaled * theta7;
        (*d_params)(1, 7) = yScaled * theta9;
      }

      const Eigen::Matrix<T, 2, 1> px =
          scaling * ff.cwiseProduct(pointOptical.template head<2>()) + pp;

      return px;
    } else {
      const T zInv = T(1.0) / pointOptical[2];
      const Eigen::Matrix<T, 2, 1> pointUnitPlane = pointOptical.template head<2>() * zInv;

      // linearize r around radius=0
      if (d_point) {
        // clang-format off
          (*d_point) << ff.x() * zInv, T(0.0), -ff.x() * pointUnitPlane(0) * zInv,
                      T(0.0), ff.y() * zInv, -ff.y() * pointUnitPlane(1) * zInv;
        // clang-format on
      }
      if (d_params) {
        (*d_params)(0, 0) = pointUnitPlane(0);
        (*d_params)(0, 1) = T(0.0);
        (*d_params)(0, 2) = T(1.0);
        (*d_params)(0, 3) = T(0.0);

        (*d_params)(1, 0) = T(0.0);
        (*d_params)(1, 1) = pointUnitPlane(1);
        (*d_params)(1, 2) = T(0.0);
        (*d_params)(1, 3) = T(1.0);
        (*d_params).template rightCols<4>().setZero();
      }
      const Eigen::Matrix<T, 2, 1> px = ff.cwiseProduct(pointUnitPlane) + pp;

      return px;
    }
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

    const T k0 = params[4];
    const T k1 = params[5];
    const T k2 = params[6];
    const T k3 = params[7];

    const T un = (uvPixel(0) - u0) / fu;
    const T vn = (uvPixel(1) - v0) / fv;
    const T rth2 = un * un + vn * vn;

    if (rth2 < Sophus::Constants<T>::epsilon() * Sophus::Constants<T>::epsilon()) {
      return Eigen::Matrix<T, 3, 1>(un, vn, T(1.0));
    }

    using std::sqrt;
    const T rth = sqrt(rth2);

    // Use Newtons method to solve for theta.
    T th = CameraNewtonsMethod::initTheta(rth);
    for (int i = 0; i < CameraNewtonsMethod::kMaxIterations; ++i) {
      const T th2 = th * th;
      const T th4 = th2 * th2;
      const T th6 = th4 * th2;
      const T th8 = th4 * th4;

      const T thd = th * (T(1.0) + k0 * th2 + k1 * th4 + k2 * th6 + k3 * th8);

      const T d_thd_wtr_th =
          T(1.0) + T(3.0) * k0 * th2 + T(5.0) * k1 * th4 + T(7.0) * k2 * th6 + T(9.0) * k3 * th8;

      const T step = (thd - rth) / d_thd_wtr_th;
      th -= step;
      if (CameraNewtonsMethod::hasConverged(step)) {
        break;
      }
    }

    using std::tan;
    T radiusUndistorted = tan(th);

    if (radiusUndistorted < T(0.0)) {
      return Eigen::Matrix<T, 3, 1>(
          -radiusUndistorted * un / rth, -radiusUndistorted * vn / rth, T(-1.0));
    }
    return Eigen::Matrix<T, 3, 1>(
        radiusUndistorted * un / rth, radiusUndistorted * vn / rth, T(1.0));
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
