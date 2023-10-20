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

#include <pangolin/pangolin.h>

namespace pangolin_helpers {
template <typename T>
inline void glDrawFrustum(const Eigen::Matrix<T, 3, 3>& Kinv, int w, int h, GLfloat scale) {
  pangolin::glDrawFrustum(
      (GLfloat)Kinv(0, 2),
      (GLfloat)Kinv(1, 2),
      (GLfloat)Kinv(0, 0),
      (GLfloat)Kinv(1, 1),
      w,
      h,
      scale);
}

template <typename T>
inline void glDrawFrustum(
    const Eigen::Matrix<T, 3, 3>& Kinv,
    int w,
    int h,
    const Eigen::Matrix<T, 4, 4>& T_wf,
    T scale) {
  pangolin::glSetFrameOfReference(T_wf);
  pangolin_helpers::glDrawFrustum(Kinv, w, h, scale);
  pangolin::glUnsetFrameOfReference();
}

inline Eigen::MatrixXf getPositionBounds(const Eigen::MatrixXf& points) {
  Eigen::MatrixXf bounds(3, 2);
  bounds.col(0) = points.rowwise().minCoeff().cast<float>();
  bounds.col(1) = points.rowwise().maxCoeff().cast<float>();
  return bounds;
}

inline void centerViewOnMap(
    pangolin::OpenGlRenderState& glcam,
    const std::vector<Eigen::Vector3f>& points,
    const float focalLength,
    const int windowWidth) {
  if (points.empty()) {
    return;
  }
  // Map input point to an Eigen3X matrix
  using Matrix3X = Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::RowMajor>;
  using MatrixCRef = Eigen::Map<const Matrix3X>;
  MatrixCRef pointsEigen(points[0].data(), 3, points.size());

  const Eigen::Vector3f center = getPositionBounds(pointsEigen).rowwise().mean();
  const Eigen::Vector3f gdir{0.0, 0.0, -9.81};
  Eigen::Vector3f nonParallelVec(1, 0, 0);
  if (std::abs(nonParallelVec.dot(gdir)) > 0.99) {
    nonParallelVec << 0, 0, 1;
  }
  const Eigen::Vector3f perpendicularVec = nonParallelVec.cross(gdir);
  const Eigen::Vector3f viewDir = gdir + 2 * perpendicularVec;
  const Eigen::MatrixXf krFromCenter = pointsEigen.colwise() - center;
  const Eigen::VectorXf krDirectionsFromCenter = krFromCenter.colwise().norm();
  const Eigen::VectorXf distances = krDirectionsFromCenter * focalLength / (windowWidth / 2);
  const float distance = distances.maxCoeff();
  const Eigen::Vector3f eye = center - distance * viewDir;
  glcam.SetModelViewMatrix(pangolin::ModelViewLookAtRDF(
      eye.x(),
      eye.y(),
      eye.z(),
      center.x(),
      center.y(),
      center.z(),
      -gdir.x(),
      -gdir.y(),
      -gdir.z()));
}

inline void updateIndividualFlagsByCentralFlag(
    pangolin::Var<bool>& centralFlag,
    std::vector<pangolin::Var<bool>>& individualFlags) {
  // if central flag is the same as last, nothing change
  if (!centralFlag.GuiChanged()) {
    return;
  }
  // all individual flags match updated central flag
  for (auto& flag : individualFlags) {
    flag = centralFlag;
  }
}

} // namespace pangolin_helpers
