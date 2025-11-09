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

namespace projectaria::tools::calibration {

/// Get an Eigen reference pointing to a nullptr.
template <
    typename Scalar,
    int Rows = Eigen::Dynamic,
    int Columns = Eigen::Dynamic,
    int StorageOrder = Eigen::ColMajor>
constexpr Eigen::Ref<Eigen::Matrix<Scalar, Rows, Columns, StorageOrder>> nullRef(
    const int rows = Rows != Eigen::Dynamic ? Rows : 0,
    const int columns = Columns != Eigen::Dynamic ? Columns : 0) {
  return Eigen::Map<Eigen::Matrix<Scalar, Rows, Columns, StorageOrder>>(nullptr, rows, columns);
}

/// Test if an Eigen Reference is pointing to a nullptr.
template <typename Base>
constexpr bool isNull(const Eigen::Ref<Base>& ref) {
  return ref.data() == nullptr;
}

/**
 * A helper struct that allows direct assignments of NullRefs as function default
 * arguments.
 */
struct NullRef {
  /// The default constructor for general usecases.
  NullRef() = default;

  /// There might be edge cases where a dynamic sized NullRef is required. This
  /// provides the option, while it should generally not be used.
  NullRef(const int numRows, const int numCols) : numRows_(numRows), numCols_(numCols) {}

  /// Fixed size matrices:
  template <typename Scalar, int R, int C, int O, int MR, int MC>
  operator Eigen::Ref<Eigen::Matrix<Scalar, R, C, O, MR, MC>>() {
    return nullRef<Scalar, R, C, O>();
  }
  template <typename Scalar, int R, int C, int O, int MR, int MC>
  operator Eigen::Ref<const Eigen::Matrix<Scalar, R, C, O, MR, MC>>() {
    return nullRef<Scalar, R, C, O>();
  }

  /// Dynamic Matrices:
  template <typename Scalar, int O, int MR, int MC>
  operator Eigen::Ref<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, O, MR, MC>>() {
    return nullRef<Scalar>(numRows_, numCols_);
  }
  template <typename Scalar, int O, int MR, int MC>
  operator Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, O, MR, MC>>() {
    return nullRef<Scalar>(numRows_, numCols_);
  }

 private:
  const int numRows_ = 0;
  const int numCols_ = 0;
};

} // namespace projectaria::tools::calibration
