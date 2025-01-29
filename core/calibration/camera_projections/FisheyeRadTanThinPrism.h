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

namespace projectaria::tools::calibration {

// Model for fisheye cameras with radial, tangential, and thin-prism distortion.
// Specifically, the model is:
//
// uvDistorted = [x_r]  + tangentialDistortion  + thinPrismDistortion
//               [y_r]
// proj = diag(fu,fv) * uvDistorted + [cu;cv];
//
// where:
//   a = x/z, b = y/z, r = (a^2+b^2)^(1/2)
//   th = atan(r)
//   cosPhi = a/r, sinPhi = b/r
//   [x_r]  = (th+ k0 * th^3 + k1* th^5 + ...) [cosPhi]
//   [y_r]                                     [sinPhi]
//
//   the number of terms in the series is determined by the template parameter numK.
//
//   tangentialDistortion = [(2 x_r^2 + rd^2)*p_0 + 2*x_r*y_r*p_1]
//                          [(2 y_r^2 + rd^2)*p_1 + 2*x_r*y_r*p_0]
//
//   where rd^2 = x_r^2 + y_r^2
//
//   thinPrismDistortion = [s0 * rd^2 + s1 rd^4]
//                         [s2 * rd^2 + s3 rd^4]
//
//   when useSingleFocalLength==true, the model uses fx=fy, (i.e has one parameter less)
//
// This model is symmetric such that points represented in inverse depth project to similar
// points when moving from positive depth to negative depth. i.e. the point (1,1,1) and (-1,-1,-1)
// project to the same pixel. That was a requirement for VIO and since we didn't have any camera
// lens with FoV > 180 deg we decided to go for this symmetry. As a result if a point at the border
// of the image sensor (i.e. 0px ,0px) would unproject to a point with z < 0 (which can happen even
// for lenses that are FoV<180 deg, because the polynomial we use to approximate distortion is only
// calibrated in the part of the image where we have measurements and does whatever in the part we
// don't have measurements) it might unproject to a wrong 3d bearing!
template <int numK, bool useTangential, bool useThinPrism, bool useSingleFocalLength>
class FisheyeRadTanThinPrism {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kNumParams =
      (4 - useSingleFocalLength) + numK + 2 * useTangential + 4 * useThinPrism;
  static const char kName[];
  static const char kDescription[];
  static constexpr int kNumDistortionParams = numK + 2 * useTangential + 4 * useThinPrism;
  static constexpr int kFocalXIdx = 0;
  static constexpr int kFocalYIdx = 1 - useSingleFocalLength;
  static constexpr int kPrincipalPointColIdx = 2 - useSingleFocalLength;
  static constexpr int kPrincipalPointRowIdx = 3 - useSingleFocalLength;
  static constexpr bool kIsFisheye = true;
  static constexpr bool kHasAnalyticalProjection = true;

  template <class D, class DP, class DJ = Eigen::Matrix<typename D::Scalar, 2, 3>>
  static Eigen::Matrix<typename D::Scalar, 2, 1> project(
      const Eigen::MatrixBase<D>& pointOptical,
      const Eigen::MatrixBase<DP>& params,
      Eigen::MatrixBase<DJ>* d_point = nullptr) {
    using T = typename D::Scalar;

    validateProjectInput<D, DP, kNumParams>();

    // make sure we have nonnegative orders:
    static_assert(numK >= 0, "nonnegative number required");

    // the parameter vector has the following interpretation:
    // params = [f_u {f_v} c_u c_v [k_0: k_{numK-1}]  {p_0 p_1} {s_0 s_1 s_2 s_3}]

    // -----------------------------------------------------------------------
    // projection computations begin
    // compute [a;b] = [x/z; y/z]
    // make sure the point is not on the image plane

    using std::atan;
    using std::sqrt;

    const T inv_z = static_cast<T>(1.0) / pointOptical[2];
    const Eigen::Matrix<T, 2, 1> ab = pointOptical.template head<2>() * inv_z;
    // compute the squares of the elements of ab
    const Eigen::Matrix<T, 2, 1> ab_squared = ab.array().square().matrix();
    // these will be used in multiple computations
    const T r_sq = ab_squared(0) + ab_squared(1);
    const T r = sqrt(r_sq);
    const T th = atan(r);
    const T thetaSq = th * th;

    T th_radial = static_cast<T>(1.0);
    // compute the theta polynomial
    T theta2is = thetaSq;
    for (int i = 0; i < numK; ++i) {
      th_radial += theta2is * params(startK + i);
      theta2is *= thetaSq;
    }

    // compute th/r, using the limit for small values
    const T th_divr = (r < std::numeric_limits<T>::epsilon()) ? static_cast<T>(1.0) : th / r;

    // the distorted coordinates -- except for focal length and principal point
    // start with the radial term:
    const Eigen::Matrix<T, 2, 1> xr_yr = (th_radial * th_divr) * ab;
    const T xr_yr_squaredNorm = xr_yr.squaredNorm();

    // start computing the output: first the radially-distorted terms, then add more as needed
    Eigen::Matrix<T, 2, 1> uvDistorted = xr_yr;

    if (useTangential) {
      const T temp = T(2) * xr_yr.dot(params.template segment<2>(startP));
      uvDistorted.noalias() +=
          temp * xr_yr + xr_yr_squaredNorm * params.template segment<2>(startP);
    }

    Eigen::Matrix<T, 2, 1> radialPowers2And4;
    if (useThinPrism) {
      radialPowers2And4(0) = xr_yr_squaredNorm;
      radialPowers2And4(1) = xr_yr_squaredNorm * xr_yr_squaredNorm;
      uvDistorted(0) += params.template segment<2>(startS).dot(radialPowers2And4);
      uvDistorted(1) += params.template segment<2>(startS + 2).dot(radialPowers2And4);
    }

    // Maybe compute point jacobian
    if (d_point) {
      Eigen::Matrix<T, 2, 2> duvDistorted_dxryr;
      compute_duvDistorted_dxryr(xr_yr, xr_yr_squaredNorm, params, duvDistorted_dxryr);

      // compute jacobian wrt point
      Eigen::Matrix<T, 2, 2> duvDistorted_dab;
      if (r == static_cast<T>(0.0)) {
        duvDistorted_dab.setIdentity();
      } else {
        T dthD_dth = static_cast<T>(1.0);
        T theta2i = thetaSq;
        for (size_t i = 0; i < numK; ++i) {
          dthD_dth += T(2 * i + 3) * params[startK + i] * theta2i;
          theta2i *= thetaSq;
        }

        const T w1 = dthD_dth / (r_sq + r_sq * r_sq);
        const T w2 = th_radial * th_divr / r_sq;
        const T ab10 = ab[0] * ab[1];
        Eigen::Matrix<T, 2, 2> temp1;
        temp1(0, 0) = w1 * ab_squared[0] + w2 * ab_squared[1];
        temp1(0, 1) = (w1 - w2) * ab10;
        temp1(1, 0) = temp1(0, 1);
        temp1(1, 1) = w1 * ab_squared[1] + w2 * ab_squared[0];
        duvDistorted_dab.noalias() = duvDistorted_dxryr * temp1;
      }

      // compute the derivative of the projection wrt the point:
      if (useSingleFocalLength) {
        d_point->template leftCols<2>() = params[0] * inv_z * duvDistorted_dab;
      } else {
        d_point->template leftCols<2>() =
            params.template head<2>().asDiagonal() * duvDistorted_dab * inv_z;
      }
      d_point->template rightCols<1>().noalias() = -d_point->template leftCols<2>() * ab;
    }

    // compute the return value
    if (useSingleFocalLength) {
      return params[0] * uvDistorted + params.template segment<2>(kPrincipalPointColIdx);
    } else {
      return uvDistorted.cwiseProduct(params.template head<2>()) +
          params.template segment<2>(kPrincipalPointColIdx);
    }
  }

  // Return scaled parameters to cope with scaled image.
  template <typename Scalar, typename DP>
  static void scaleParams(Scalar s, Eigen::DenseBase<DP>& params) {
    validateScaleParamInput<Scalar, DP, kNumParams>();

    using T = typename DP::Scalar;
    const T scale = s;
    constexpr T kHalf = T(0.5);
    params[kFocalXIdx] *= scale;
    if (!useSingleFocalLength) {
      params[kFocalYIdx] *= scale;
    }
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

  template <typename D, typename DP>
  static Eigen::Matrix<typename D::Scalar, 3, 1> unproject(
      const Eigen::MatrixBase<D>& p,
      const Eigen::MatrixBase<DP>& params) {
    validateUnprojectInput<D, DP, kNumParams>();

    using T = typename D::Scalar;

    using std::tan;

    // get uvDistorted:
    Eigen::Matrix<T, 2, 1> uvDistorted;
    if (useSingleFocalLength) {
      uvDistorted = (p - params.template segment<2>(kPrincipalPointColIdx)) / params[0];
    } else {
      uvDistorted = (p - params.template segment<2>(kPrincipalPointColIdx))
                        .cwiseQuotient(params.template head<2>());
    }

    // get xr_yr from uvDistorted
    Eigen::Matrix<T, 2, 1> xr_yr = compute_xr_yr_from_uvDistorted(uvDistorted, params);

    // early exit if point is in the center of the image
    const T xr_yrNorm = xr_yr.norm();
    if (xr_yrNorm == T(0.0)) {
      return Eigen::Matrix<T, 3, 1>::UnitZ();
    }

    // otherwise, find theta
    T theta = getThetaFromNorm_xr_yr(xr_yrNorm, params);

    // get the point coordinates:
    Eigen::Matrix<T, 3, 1> point3dEst;
    point3dEst.template head<2>() = tan(theta) / xr_yrNorm * xr_yr;
    point3dEst(2) = static_cast<T>(1.0);

    return point3dEst;
  }

 private:
  // helper function to compute the vector [x_r; y_r] from uvDistorted
  template <typename T, typename DP>
  inline static Eigen::Matrix<T, 2, 1> compute_xr_yr_from_uvDistorted(
      const Eigen::Matrix<T, 2, 1>& uvDistorted,
      const Eigen::MatrixBase<DP>& params) {
    // early exit if we're not using any tangential/ thin prism distortions
    if (!useTangential && !useThinPrism) {
      return uvDistorted;
    }

    // initial guess:
    Eigen::Matrix<T, 2, 1> xr_yr = uvDistorted;

    // do Newton iterations to find xr_yr
    for (int j = 0; j < CameraNewtonsMethod::kMaxIterations; ++j) {
      // compute the estimated uvDistorted
      Eigen::Matrix<T, 2, 1> uvDistorted_est = xr_yr;
      const T xr_yr_squaredNorm = xr_yr.squaredNorm();

      if (useTangential) {
        const T temp = T(2) * xr_yr.dot(params.template segment<2>(startP));
        uvDistorted_est.noalias() +=
            temp * xr_yr + xr_yr_squaredNorm * params.template segment<2>(startP);
      }

      if (useThinPrism) {
        Eigen::Matrix<T, 2, 1> radialPowers2And4;
        radialPowers2And4(0) = xr_yr_squaredNorm;
        radialPowers2And4(1) = xr_yr_squaredNorm * xr_yr_squaredNorm;
        uvDistorted_est(0) += params.template segment<2>(startS).dot(radialPowers2And4);
        uvDistorted_est(1) += params.template segment<2>(startS + 2).dot(radialPowers2And4);
      }

      // compute the derivative of uvDistorted wrt xr_yr
      Eigen::Matrix<T, 2, 2> duvDistorted_dxryr;
      compute_duvDistorted_dxryr(xr_yr, xr_yr_squaredNorm, params, duvDistorted_dxryr);

      // compute correction:
      // note: the matrix duvDistorted_dxryr will be close to identity (for reasonable values
      // of tangential/thin prism distortions), so using an analytical inverse here is safe
      Eigen::Matrix<T, 2, 1> correction;
      correction.noalias() = duvDistorted_dxryr.inverse() * (uvDistorted - uvDistorted_est);

      xr_yr += correction;

      if (CameraNewtonsMethod::hasConverged(correction)) {
        break;
      }
    }
    return xr_yr;
  }

  // helper function to compute the angle theta from the norm of the vector [x_r; y_r]
  template <typename T, typename D>
  inline static T getThetaFromNorm_xr_yr(
      const T th_radialDesired,
      const Eigen::MatrixBase<D>& params) {
    // initial guess
    T th = th_radialDesired;

    using std::abs;

    for (int j = 0; j < CameraNewtonsMethod::kMaxIterations; ++j) {
      const T thetaSq = th * th;

      T th_radial = T(1);
      T dthD_dth = T(1);
      // compute the theta polynomial and its derivative wrt theta
      T theta2is = thetaSq;
      for (int i = 0; i < numK; ++i) {
        th_radial += theta2is * params(startK + i);
        dthD_dth += T(2 * i + 3) * params[startK + i] * theta2is;
        theta2is *= thetaSq;
      }
      th_radial *= th;

      // compute the correction:
      T step;
      // make sure we don't divide by zero:
      if (abs(dthD_dth) > Sophus::Constants<T>::epsilon()) {
        step = (th_radialDesired - th_radial) / dthD_dth;
      } else {
        // if derivative is close to zero, apply small correction in the appropriate direction
        // to avoid numerical explosions
        step = (th_radialDesired - th_radial) * dthD_dth > 0.0
            ? T(10) * Sophus::Constants<T>::epsilon()
            : T(-10) * Sophus::Constants<T>::epsilon();
      }

      // apply correction
      th += step;

      if (CameraNewtonsMethod::hasConverged(step)) {
        break;
      }

      // revert to within 180 degrees FOV to avoid numerical overflow
      if (abs(th) >= Sophus::Constants<T>::pi() / 2.0) {
        // the exact value we choose here is not really important, we'll iterate again over it.
        th = T(0.999) * Sophus::Constants<T>::pi() / 2.0;
      }
    }

    return th;
  }

  // helper function, computes the Jacobian of uvDistorted wrt the vector [x_r;y_r]
  template <typename D>
  inline static void compute_duvDistorted_dxryr(
      const Eigen::Matrix<typename D::Scalar, 2, 1>& xr_yr,
      const typename D::Scalar xr_yr_squaredNorm,
      const Eigen::MatrixBase<D>& params,
      Eigen::Matrix<typename D::Scalar, 2, 2>& duvDistorted_dxryr) {
    using T = typename D::Scalar;
    if (useTangential) {
      duvDistorted_dxryr(0, 0) =
          T(1) + T(6) * xr_yr(0) * params(startP) + T(2) * xr_yr(1) * params(startP + 1);
      const T offdiag = T(2) * (xr_yr(0) * params(startP + 1) + xr_yr(1) * params(startP));
      duvDistorted_dxryr(0, 1) = offdiag;
      duvDistorted_dxryr(1, 0) = offdiag;
      duvDistorted_dxryr(1, 1) =
          T(1) + T(6) * xr_yr(1) * params(startP + 1) + T(2) * xr_yr(0) * params(startP);
    } else {
      duvDistorted_dxryr.setIdentity();
    }

    if (useThinPrism) {
      const T temp1 = T(2) * (params(startS) + T(2) * params(startS + 1) * xr_yr_squaredNorm);
      duvDistorted_dxryr(0, 0) += xr_yr(0) * temp1;
      duvDistorted_dxryr(0, 1) += xr_yr(1) * temp1;

      const T temp2 = T(2) * (params(startS + 2) + T(2) * params(startS + 3) * xr_yr_squaredNorm);
      duvDistorted_dxryr(1, 0) += xr_yr(0) * temp2;
      duvDistorted_dxryr(1, 1) += xr_yr(1) * temp2;
    }
  }

  // these are the indices within the params vector where each of the parameters begins
  static constexpr int startK = kPrincipalPointRowIdx + 1;
  static constexpr int startP = startK + numK;
  static constexpr int startS = startP + 2 * useTangential;
};

// define a model with 6 radial, 2 tangential, 4 thin-prism distortion parameters, and single focal
// length
using Fisheye624 = FisheyeRadTanThinPrism<6, true, true, true>;

// We need explicit template instantiation + definition in a separate cpp (ProjectionConstants.cpp)
// in order to have static const string compile under both clang C++17 and C++11. However, that
// will cause redefinition error under MSVC. Therefore we bypass it with macro.
// clang on windows will define _MSC_VER so we need to check both _MSC_VER and __clang__.
#if !defined(_MSC_VER) || defined(__clang__)
template <>
const char Fisheye624::kName[];

template <>
const char Fisheye624::kDescription[];

template <>
const char FisheyeRadTanThinPrism<6, true, false, true>::kName[];

template <>
const char FisheyeRadTanThinPrism<6, true, false, true>::kDescription[];
#endif // !defined(_MSC_VER) || defined(__clang__)

} // namespace projectaria::tools::calibration
