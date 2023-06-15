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
#include <Eigen/Core>

namespace ceres {
template <typename T, int N>
struct Jet;
}

namespace projectaria::tools::calibration {

template <typename T>
inline double IgnoreJetInfinitesimal(const T& j) {
  static_assert(
      std::is_same<decltype(j.a), double>::value || std::is_same<decltype(j.a), float>::value,
      "T should be a ceres jet");
  return j.a;
}

inline double IgnoreJetInfinitesimal(double j) {
  return j;
}

inline float IgnoreJetInfinitesimal(float j) {
  return j;
}

template <typename Derived>
inline Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
IgnoreEigenJetInfinitesimal(const Eigen::MatrixBase<Derived>& mat) {
  return mat.unaryExpr([](const typename Derived::Scalar& v) { return IgnoreJetInfinitesimal(v); });
}
} // namespace projectaria::tools::calibration
