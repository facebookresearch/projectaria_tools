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

#include <mps/HandTracking.h>
#include <sophus/se3.hpp>
#include <vrs/DataPieceTypes.h>
#include <Eigen/Core>

namespace projectaria::tools::data_provider {
template <typename T>
void populatePoint3D(
    T* dest,
    const typename std::conditional<std::is_same<T, float>::value, vrs::Point3Df, vrs::Point3Dd>::
        type& src) {
  dest[0] = src[0];
  dest[1] = src[1];
  dest[2] = src[2];
}

template <typename T>
void populatePoint4D(
    T* dest,
    const typename std::conditional<std::is_same<T, float>::value, vrs::Point4Df, vrs::Point4Dd>::
        type& src) {
  dest[0] = src[0];
  dest[1] = src[1];
  dest[2] = src[2];
  dest[3] = src[3];
}

template <typename TIn, typename TOut = TIn, size_t N>
auto mapToEigenVector(const vrs::PointND<TIn, N>& point) {
  Eigen::Map<const Eigen::Matrix<TIn, N, 1>> eigenMap(point.dim);

  if constexpr (std::is_same_v<TIn, TOut>) {
    return eigenMap;
  } else {
    // different types → return a cast‐expression
    return eigenMap.template cast<TOut>();
  }
}

template <typename TIn, typename TOut = TIn>
inline Sophus::SE3<TOut> populateToSE3(
    const vrs::PointND<TIn, 4>& rotation,
    const vrs::PointND<TIn, 3>& translation) {
  Eigen::Matrix<TIn, 3, 1> eigenTranslation = {translation[0], translation[1], translation[2]};
  // xyzw -> eigen::wxyz
  Eigen::Quaternion<TIn> eigenRotation = {rotation[3], rotation[0], rotation[1], rotation[2]};

  if constexpr (std::is_same_v<TIn, TOut>) {
    return Sophus::SE3<TIn>(eigenRotation, eigenTranslation);
  } else {
    return Sophus::SE3<TOut>(
        eigenRotation.template cast<TOut>(), eigenTranslation.template cast<TOut>());
  }
}

template <typename T, int N>
void populateFromEigenVectorToPtr(const Eigen::Matrix<T, N, 1>& src, T* dest) {
  for (int i = 0; i < N; ++i) {
    dest[i] = src[i];
  }
}
} // namespace projectaria::tools::data_provider
