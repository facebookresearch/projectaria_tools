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
#include <sophus/se3.hpp>

/*
 * fmt::format() specialization for Eigen Matrix
 */
template <typename SCALAR, int ROWS, int COLS, int Options, int MaxRows, int MaxCols>
struct fmt::formatter<Eigen::Matrix<SCALAR, ROWS, COLS, Options, MaxRows, MaxCols>>
    : fmt::formatter<std::string_view> {
  // Format the Eigen::Matrix object
  template <typename FormatContext>
  auto format(
      const Eigen::Matrix<SCALAR, ROWS, COLS, Options, MaxRows, MaxCols>& mat,
      FormatContext& ctx) const {
    std::stringstream ss;
    if constexpr (COLS == 1) {
      ss << mat.format(kVectorFormat);
    } else {
      ss << mat.format(kMatrixFormat);
    }

    return format_to(ctx.out(), "{}", ss.str());
  }

  const Eigen::IOFormat kMatrixFormat = Eigen::IOFormat(
      Eigen::StreamPrecision,
      Eigen::DontAlignCols, // flags
      ", ", // coeffSeparator
      ", ", // rowSeparator
      "[", // rowPrefix
      "]", // rowSuffix
      "[", // matPrefix
      "]"); // matSuffix;

  // When printing Vectors, don't use the row prefix/suffix
  const Eigen::IOFormat kVectorFormat = Eigen::IOFormat(
      Eigen::StreamPrecision,
      Eigen::DontAlignCols, // flags
      ", ", // coeffSeparator
      ", ", // rowSeparator
      "", // rowPrefix
      "", // rowSuffix
      "[", // matPrefix
      "]"); // matSuffix
};

/*
 * fmt::format() specialization for Sophus SE3
 */
template <typename SCALAR>
struct fmt::formatter<Sophus::SE3<SCALAR>> : fmt::formatter<std::string_view> {
  // Format the Sophus::SE3 object
  template <typename FormatContext>
  auto format(const Sophus::SE3<SCALAR>& se3, FormatContext& ctx) const {
    return format_to(
        ctx.out(),
        "(translation:{}, quaternion(x,y,z,w):{})",
        se3.translation(),
        se3.unit_quaternion().coeffs());
  }
};
