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

#include <calibration/camera_projections/Common.h>

#include <limits>
#include <string_view>

namespace projectaria::tools::calibration {

// Model for fisheye cameras with radial, cubic, and rd4-th distortion terms.
// Specifically, the model is:
//
// proj = K * uvDistorted + [cu;cv];
//
// where:
//   K = [fu  s;
//        0  fv]
//
//   uvDistorted = [x_r] + cubicDistortion + rd4thDistortion
//                 [y_r]
//
//   a = x/z, b = y/z, r = (a^2+b^2)^(1/2)
//   th = atan(r)
//   cosPhi = a/r, sinPhi = b/r
//   [x_r]  = (th+ k0 * th^3 + k1* th^5 + ...) [cosPhi]
//   [y_r]                                     [sinPhi]
//
//   the number of terms in the series is determined by the template parameter numK.
//
//   cubicDistortion = [cubic_x; cubic_y]
//   cubic_x = p0 * xr^2 + p2 * yr^2 + p4 * xr*yr + p6 * xr^3 + p8 * yr^3 + p10 * yr^2*xr
//             + p12 * yr*xr^2
//   cubic_y = p1 * xr^2 + p3 * yr^2 + p5 * xr*yr + p7 * xr^3 + p9 * yr^3 + p11 * yr^2*xr
//
//   rd4thDistortion = [s0 * rd4; s1 * rd4], where rd4 = (x_r^2 + y_r^2)^2
//
//   parameter order: [fu fv cu cv s [k_0: k_{numK-1}] [p0...p12] s0 s1]
//
// This model is symmetric such that points represented in inverse depth project to similar
// points when moving from positive depth to negative depth. i.e. the point (1,1,1) and (-1,-1,-1)
// project to the same pixel.
template <int numK>
class FisheyeCubicPlusRd4Projection {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kNumCubicParams = 13;
  static constexpr int kNumRd4Params = 2;
  static constexpr int kNumDistortionParams = numK + kNumCubicParams + kNumRd4Params;
  static constexpr int kNumParams = 5 + kNumDistortionParams;
  static constexpr std::string_view kName = "FisheyeCubicPlusRd4";
  static constexpr std::string_view kDescription =
      "fu, fv, cu, cv, s, k0, k1, k2, k3, k4, k5, p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, "
      "p11, p12, s0, s1";
  static constexpr int kFocalXIdx = 0;
  static constexpr int kFocalYIdx = 1;
  static constexpr int kPrincipalPointColIdx = 2;
  static constexpr int kPrincipalPointRowIdx = 3;
  static constexpr int kSkewIdx = 4;
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
      Eigen::MatrixBase<DJ2>* d_param = nullptr) {
    using T = typename D::Scalar;

    validateProjectInput<D, DP, kNumParams>();
    static_assert(numK >= 0, "nonnegative number required");

    using std::atan;
    using std::sqrt;

    const T inv_z = static_cast<T>(1.0) / pointOptical[2];
    const Eigen::Matrix<T, 2, 1> ab = pointOptical.template head<2>() * inv_z;
    const Eigen::Matrix<T, 2, 1> ab_squared = ab.array().square().matrix();
    const T r_sq = ab_squared(0) + ab_squared(1);
    const T r = sqrt(r_sq);
    const T th = atan(r);
    const T thetaSq = th * th;

    T th_radial = static_cast<T>(1.0);
    T theta2is = thetaSq;
    for (int i = 0; i < numK; ++i) {
      th_radial += theta2is * params(startK + i);
      theta2is *= thetaSq;
    }

    // th/r, using the limit for small values
    const T th_divr = (r < std::numeric_limits<T>::epsilon()) ? static_cast<T>(1.0) : th / r;
    const T th_radial_divr = th_radial * th_divr;

    const Eigen::Matrix<T, 2, 1> xr_yr = th_radial_divr * ab;
    const Monomials<T> m(xr_yr);

    const Eigen::Matrix<T, 2, 1> uvDistorted = xr_yr + distortion(m, params);

    const Eigen::Matrix<T, 2, 2> K = intrinsicMatrix(params);

    if (d_param || d_point) {
      const Eigen::Matrix<T, 2, 2> duvDistorted_dxryr = compute_duvDistorted_dxryr(m, params);

      if (d_point) {
        Eigen::Matrix<T, 2, 2> duvDistorted_dab;
        if (r_sq < std::numeric_limits<T>::epsilon()) {
          duvDistorted_dab = duvDistorted_dxryr;
        } else {
          T dthD_dth = static_cast<T>(1.0);
          T theta2i = thetaSq;
          for (int i = 0; i < numK; ++i) {
            dthD_dth += T(2 * i + 3) * params[startK + i] * theta2i;
            theta2i *= thetaSq;
          }

          const T w1 = dthD_dth / (r_sq + r_sq * r_sq);
          const T w2 = th_radial_divr / r_sq;
          const T ab10 = ab[0] * ab[1];
          Eigen::Matrix<T, 2, 2> temp1;
          temp1(0, 0) = w1 * ab_squared[0] + w2 * ab_squared[1];
          temp1(0, 1) = (w1 - w2) * ab10;
          temp1(1, 0) = temp1(0, 1);
          temp1(1, 1) = w1 * ab_squared[1] + w2 * ab_squared[0];
          duvDistorted_dab.noalias() = duvDistorted_dxryr * temp1;
        }

        d_point->template leftCols<2>() = K * duvDistorted_dab * inv_z;
        d_point->template rightCols<1>().noalias() = -d_point->template leftCols<2>() * ab;
      }

      if (d_param) {
        d_param->setZero();
        (*d_param)(0, kFocalXIdx) = uvDistorted(0);
        (*d_param)(1, kFocalYIdx) = uvDistorted(1);
        d_param->template middleCols<2>(kPrincipalPointColIdx).setIdentity();
        (*d_param)(0, kSkewIdx) = uvDistorted(1);

        if (numK > 0) {
          const Eigen::Matrix<T, 2, 1> temp = K * (th_divr * (duvDistorted_dxryr * ab));
          T theta2i = thetaSq;
          for (int i = 0; i < numK; ++i) {
            d_param->col(startK + i) = theta2i * temp;
            theta2i *= thetaSq;
          }
        }

        d_param->template middleCols<2>(startP) = m.xx * K;
        d_param->template middleCols<2>(startP + 2) = m.yy * K;
        d_param->template middleCols<2>(startP + 4) = m.xy * K;
        d_param->template middleCols<2>(startP + 6) = m.xxx * K;
        d_param->template middleCols<2>(startP + 8) = m.yyy * K;
        d_param->template middleCols<2>(startP + 10) = m.xyy * K;
        d_param->col(startP + 12) = m.xxy * K.col(0);
        d_param->template middleCols<2>(startS) = m.rd4 * K;
      }
    }

    return K * uvDistorted + params.template segment<2>(kPrincipalPointColIdx);
  }

  template <typename D, typename DP>
  static Eigen::Matrix<typename D::Scalar, 3, 1> unproject(
      const Eigen::MatrixBase<D>& p,
      const Eigen::MatrixBase<DP>& params) {
    validateUnprojectInput<D, DP, kNumParams>();

    using T = typename D::Scalar;
    using std::tan;

    const Eigen::Matrix<T, 2, 1> centered = p - params.template segment<2>(kPrincipalPointColIdx);
    Eigen::Matrix<T, 2, 1> uvDistorted;
    uvDistorted(1) = centered(1) / params[kFocalYIdx];
    uvDistorted(0) = (centered(0) - params[kSkewIdx] * uvDistorted(1)) / params[kFocalXIdx];

    const Eigen::Matrix<T, 2, 1> xr_yr = compute_xr_yr_from_uvDistorted(uvDistorted, params);

    const T xr_yrNorm = xr_yr.norm();
    if (xr_yrNorm == T(0.0)) {
      return Eigen::Matrix<T, 3, 1>::UnitZ();
    }

    const T theta = getThetaFromNorm_xr_yr(xr_yrNorm, params);

    Eigen::Matrix<T, 3, 1> point3dEst;
    point3dEst.template head<2>() = tan(theta) / xr_yrNorm * xr_yr;
    point3dEst(2) = static_cast<T>(1.0);
    return point3dEst;
  }

  // Return scaled parameters to cope with scaled image.
  template <typename Scalar, typename DP>
  static void scaleParams(Scalar s, Eigen::DenseBase<DP>& params) {
    validateScaleParamInput<Scalar, DP, kNumParams>();

    using T = typename DP::Scalar;
    const T scale = s;
    constexpr T kHalf = T(0.5);
    params[kFocalXIdx] *= scale;
    params[kFocalYIdx] *= scale;
    params[kSkewIdx] *= scale;
    params[kPrincipalPointColIdx] = scale * (params[kPrincipalPointColIdx] + kHalf) - kHalf;
    params[kPrincipalPointRowIdx] = scale * (params[kPrincipalPointRowIdx] + kHalf) - kHalf;
    // Distortion coefficients aren't effected by scale
  }

  // Subtract values from optical center, useful when sensor plane is cropped
  template <typename DP>
  static void
  subtractFromOrigin(typename DP::Scalar u, typename DP::Scalar v, Eigen::DenseBase<DP>& params) {
    params[kPrincipalPointColIdx] -= u;
    params[kPrincipalPointRowIdx] -= v;
  }

 private:
  // monomials of [x_r; y_r] used by the cubic and rd4 terms
  template <typename T>
  struct Monomials {
    explicit Monomials(const Eigen::Matrix<T, 2, 1>& xr_yr)
        : x(xr_yr(0)),
          y(xr_yr(1)),
          xx(x * x),
          yy(y * y),
          xy(x * y),
          xxx(xx * x),
          yyy(yy * y),
          xyy(x * yy),
          xxy(xx * y),
          rSq(xx + yy),
          rd4(rSq * rSq) {}

    T x;
    T y;
    T xx;
    T yy;
    T xy;
    T xxx;
    T yyy;
    T xyy;
    T xxy;
    T rSq;
    T rd4;
  };

  template <typename DP>
  static Eigen::Matrix<typename DP::Scalar, 2, 2> intrinsicMatrix(
      const Eigen::MatrixBase<DP>& params) {
    using T = typename DP::Scalar;
    Eigen::Matrix<T, 2, 2> K;
    K << params[kFocalXIdx], params[kSkewIdx], T(0), params[kFocalYIdx];
    return K;
  }

  // the cubic + rd4 part of uvDistorted
  template <typename T, typename DP>
  static Eigen::Matrix<T, 2, 1> distortion(
      const Monomials<T>& m,
      const Eigen::MatrixBase<DP>& params) {
    Eigen::Matrix<T, 2, 1> d = m.xx * params.template segment<2>(startP) +
        m.yy * params.template segment<2>(startP + 2) +
        m.xy * params.template segment<2>(startP + 4) +
        m.xxx * params.template segment<2>(startP + 6) +
        m.yyy * params.template segment<2>(startP + 8) +
        m.xyy * params.template segment<2>(startP + 10) +
        m.rd4 * params.template segment<2>(startS);
    d(0) += m.xxy * params(startP + 12);
    return d;
  }

  // Jacobian of uvDistorted wrt the vector [x_r;y_r]
  template <typename T, typename DP>
  static Eigen::Matrix<T, 2, 2> compute_duvDistorted_dxryr(
      const Monomials<T>& m,
      const Eigen::MatrixBase<DP>& params) {
    Eigen::Matrix<T, 2, 2> J = Eigen::Matrix<T, 2, 2>::Identity();
    // xr^2
    J.col(0) += T(2) * m.x * params.template segment<2>(startP);
    // yr^2
    J.col(1) += T(2) * m.y * params.template segment<2>(startP + 2);
    // xr*yr
    J.col(0) += m.y * params.template segment<2>(startP + 4);
    J.col(1) += m.x * params.template segment<2>(startP + 4);
    // xr^3
    J.col(0) += T(3) * m.xx * params.template segment<2>(startP + 6);
    // yr^3
    J.col(1) += T(3) * m.yy * params.template segment<2>(startP + 8);
    // yr^2*xr
    J.col(0) += m.yy * params.template segment<2>(startP + 10);
    J.col(1) += T(2) * m.xy * params.template segment<2>(startP + 10);
    // xr^2*yr
    J(0, 0) += T(2) * m.xy * params(startP + 12);
    J(0, 1) += m.xx * params(startP + 12);
    // rd4
    J.col(0) += T(4) * m.rSq * m.x * params.template segment<2>(startS);
    J.col(1) += T(4) * m.rSq * m.y * params.template segment<2>(startS);
    return J;
  }

  // Newton iterations to find [x_r; y_r] from uvDistorted
  template <typename T, typename DP>
  static Eigen::Matrix<T, 2, 1> compute_xr_yr_from_uvDistorted(
      const Eigen::Matrix<T, 2, 1>& uvDistorted,
      const Eigen::MatrixBase<DP>& params) {
    Eigen::Matrix<T, 2, 1> xr_yr = uvDistorted;

    for (int j = 0; j < CameraNewtonsMethod::kMaxIterations; ++j) {
      const Monomials<T> m(xr_yr);
      const Eigen::Matrix<T, 2, 1> uvDistorted_est = xr_yr + distortion(m, params);
      const Eigen::Matrix<T, 2, 2> duvDistorted_dxryr = compute_duvDistorted_dxryr(m, params);

      // duvDistorted_dxryr is close to identity for reasonable distortion, so the analytical
      // inverse is safe here
      const Eigen::Matrix<T, 2, 1> correction =
          duvDistorted_dxryr.inverse() * (uvDistorted - uvDistorted_est);

      xr_yr += correction;

      if (CameraNewtonsMethod::hasConverged(correction)) {
        break;
      }
    }
    return xr_yr;
  }

  // Newton iterations to find theta from the norm of [x_r; y_r]
  template <typename T, typename DP>
  static T getThetaFromNorm_xr_yr(const T th_radialDesired, const Eigen::MatrixBase<DP>& params) {
    T th = th_radialDesired;

    using std::abs;

    for (int j = 0; j < CameraNewtonsMethod::kMaxIterations; ++j) {
      const T thetaSq = th * th;

      T th_radial = T(1);
      T dthD_dth = T(1);
      T theta2is = thetaSq;
      for (int i = 0; i < numK; ++i) {
        th_radial += theta2is * params(startK + i);
        dthD_dth += T(2 * i + 3) * params[startK + i] * theta2is;
        theta2is *= thetaSq;
      }
      th_radial *= th;

      T step;
      if (abs(dthD_dth) > Sophus::Constants<T>::epsilon()) {
        step = (th_radialDesired - th_radial) / dthD_dth;
      } else {
        // if derivative is close to zero, apply small correction in the appropriate direction
        // to avoid numerical explosions
        step = (th_radialDesired - th_radial) * dthD_dth > 0.0
            ? T(10) * Sophus::Constants<T>::epsilon()
            : T(-10) * Sophus::Constants<T>::epsilon();
      }

      th += step;

      if (CameraNewtonsMethod::hasConverged(step)) {
        break;
      }

      // revert to within 180 degrees FOV to avoid numerical overflow
      if (abs(th) >= Sophus::Constants<T>::pi() / 2.0) {
        th = T(0.999) * Sophus::Constants<T>::pi() / 2.0;
      }
    }

    return th;
  }

  // indices within the params vector where each group of parameters begins
  static constexpr int startK = kSkewIdx + 1;
  static constexpr int startP = startK + numK;
  static constexpr int startS = startP + kNumCubicParams;
};

// 6 radial, 13 cubic and 2 rd4 distortion parameters, two focal lengths and skew
using FisheyeCubicPlusRd4 = FisheyeCubicPlusRd4Projection<6>;

} // namespace projectaria::tools::calibration
