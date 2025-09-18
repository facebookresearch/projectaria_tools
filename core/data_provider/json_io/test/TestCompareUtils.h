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

#include <algorithm>

#include <sophus/se3.hpp>
#include <Eigen/Core>

// Helper comparison functions
inline bool isClose(float lhs, float rhs, float tol = 1e-5) {
  return std::abs(lhs - rhs) <= tol;
}

inline bool isClose(const Eigen::MatrixXf& lhs, const Eigen::MatrixXf& rhs, float tol = 1e-7) {
  return lhs.isApprox(rhs, tol);
}

inline bool isClose(const Sophus::SE3f& lhs, const Sophus::SE3f& rhs, float tol = 1e-5) {
  return isClose(lhs.translation(), rhs.translation(), tol) &&
      isClose(lhs.unit_quaternion().coeffs(), rhs.unit_quaternion().coeffs(), tol);
}

template <typename T>
inline bool isClose(const std::vector<T>& lhs, const std::vector<T>& rhs, float tol = 1e-5) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (int i = 0; i < lhs.size(); ++i) {
    if (!isClose(lhs[i], rhs[i], tol)) {
      return false;
    }
  }
  return true;
}
