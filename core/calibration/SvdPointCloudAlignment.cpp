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

#include <calibration/SvdPointCloudAlignment.h>

#include <Eigen/Dense>

namespace projectaria::tools::calibration {

namespace {

Eigen::Vector3d computePointMean(const std::vector<Eigen::Vector3d>& points) {
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>> mat(
      points[0].data(), 3, static_cast<Eigen::Index>(points.size()));
  return mat.rowwise().mean();
}

Eigen::Matrix3d computePointCovariance(
    const std::vector<Eigen::Vector3d>& points_A,
    const std::vector<Eigen::Vector3d>& points_B,
    const Eigen::Vector3d& mean_A,
    const Eigen::Vector3d& mean_B) {
  const auto n = static_cast<Eigen::Index>(points_A.size());
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>> matA(points_A[0].data(), 3, n);
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>> matB(points_B[0].data(), 3, n);
  return (matA.colwise() - mean_A) * (matB.colwise() - mean_B).transpose();
}

std::optional<Eigen::Matrix3d> computeRotationClosedForm(const Eigen::Matrix3d& pointCovariance) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(pointCovariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // If rank(sigma) in {0, 1} the problem is under-constrained (collinear points).
  if (svd.singularValues()[0] < 1e-7 || svd.singularValues()[1] * 1e9 < svd.singularValues()[0]) {
    return std::nullopt;
  }

  Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

  // Handle reflection case
  if (R.determinant() < 0) {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1;
    R = V * svd.matrixU().transpose();
  }

  if (!std::isfinite(R.determinant())) {
    return std::nullopt;
  }

  return R;
}

} // namespace

std::optional<Sophus::SE3d> alignPointCloudsToRigidTransform(
    const std::vector<Eigen::Vector3d>& points_A,
    const std::vector<Eigen::Vector3d>& points_B) {
  if (points_A.size() != points_B.size() || points_A.size() < 3) {
    return std::nullopt;
  }

  const Eigen::Vector3d mean_A = computePointMean(points_A);
  const Eigen::Vector3d mean_B = computePointMean(points_B);

  if (!std::isfinite(mean_A.dot(mean_B))) {
    return std::nullopt;
  }

  const Eigen::Matrix3d H = computePointCovariance(points_A, points_B, mean_A, mean_B);
  const auto maybeR = computeRotationClosedForm(H);
  if (!maybeR.has_value()) {
    return std::nullopt;
  }

  const Eigen::Matrix3d& R = maybeR.value();
  const Eigen::Vector3d t = mean_B - R * mean_A;

  if (!std::isfinite(t.norm())) {
    return std::nullopt;
  }

  return Sophus::SE3d(Sophus::makeRotationMatrix(R), t);
}

} // namespace projectaria::tools::calibration
