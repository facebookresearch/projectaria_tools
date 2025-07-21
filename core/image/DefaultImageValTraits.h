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

#include <limits>
#include <type_traits>

#include <Eigen/Core>

namespace projectaria::tools::image {
template <class T, class Enable = void>
struct DefaultImageValTraits {
  static constexpr int maxValue = 0;
  static constexpr bool isEigen = false;
  using Scalar = T;
  static const size_t channel = 1;
};

template <class T>
struct DefaultImageValTraits<T, typename std::enable_if<std::is_integral<T>::value>::type> {
  static constexpr int maxValue = (sizeof(T) >= sizeof(int))
      ? std::numeric_limits<int>::max()
      : static_cast<int>(std::numeric_limits<T>::max());
  static constexpr bool isEigen = false;
  using Scalar = T;
  static const size_t channel = 1;
};

template <class T>
struct DefaultImageValTraits<T, typename std::enable_if<std::is_floating_point<T>::value>::type> {
  static constexpr int maxValue = 1;
  static constexpr bool isEigen = false;
  using Scalar = T;
  static const size_t channel = 1;
};

template <class T, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct DefaultImageValTraits<Eigen::Matrix<T, Rows, Cols, Options, MaxRows, MaxCols>> {
  static constexpr int maxValue = DefaultImageValTraits<T>::maxValue;
  static constexpr bool isEigen = true;
  using Scalar = T;
  static const size_t channel = Rows;
};
} // namespace projectaria::tools::image
