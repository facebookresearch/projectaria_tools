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

#include <fmt/format.h>
#include <mps/HandTracking.h>
#include <sophus/se3.hpp>
#include <vrs/DataPieceTypes.h>
#include <Eigen/Core>

#include <optional>
#include <string_view>

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

/// Builds an SE3 from a VRS quaternion/translation pair, or returns nullopt when
/// the stored values are not a pose Sophus can represent. On-device trackers
/// write NaN or zeroed poses for records they could not solve, and Sophus::SO3
/// aborts the process rather than throwing when handed one, so callers must
/// treat a stored pose as optional.
template <typename TIn, typename TOut = TIn>
inline std::optional<Sophus::SE3<TOut>> populateToSE3(
    const vrs::PointND<TIn, 4>& rotation,
    const vrs::PointND<TIn, 3>& translation) {
  const Eigen::Matrix<TOut, 3, 1> eigenTranslation{
      static_cast<TOut>(translation[0]),
      static_cast<TOut>(translation[1]),
      static_cast<TOut>(translation[2])};
  // xyzw -> eigen::wxyz
  const Eigen::Quaternion<TOut> eigenRotation{
      static_cast<TOut>(rotation[3]),
      static_cast<TOut>(rotation[0]),
      static_cast<TOut>(rotation[1]),
      static_cast<TOut>(rotation[2])};

  // Mirrors the SOPHUS_ENSURE in SO3Base::normalize(); a NaN norm also fails it.
  if (!eigenRotation.coeffs().allFinite() || !eigenTranslation.allFinite() ||
      eigenRotation.norm() < Sophus::Constants<TOut>::epsilon()) {
    return {};
  }
  return Sophus::SE3<TOut>{eigenRotation, eigenTranslation};
}

/// populateToSE3 plus the diagnostic every player prints when a record's stored
/// pose is unusable, so callers only keep the early return.
template <typename TIn, typename TOut = TIn>
inline std::optional<Sophus::SE3<TOut>> populateToSE3OrLog(
    const vrs::PointND<TIn, 4>& rotation,
    const vrs::PointND<TIn, 3>& translation,
    std::string_view fieldName,
    std::string_view layoutName) {
  std::optional<Sophus::SE3<TOut>> pose = populateToSE3<TIn, TOut>(rotation, translation);
  if (!pose) {
    fmt::print("Unrepresentable {} in {} data layout!", fieldName, layoutName);
  }
  return pose;
}

template <typename T, int N>
void populateFromEigenVectorToPtr(const Eigen::Matrix<T, N, 1>& src, T* dest) {
  for (int i = 0; i < N; ++i) {
    dest[i] = src[i];
  }
}
} // namespace projectaria::tools::data_provider
