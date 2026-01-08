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
#include <Eigen/Geometry>

#include "DefaultImageValTraits.h"
#include "ImageIterator.h"
#include "Portability.h"

namespace projectaria::tools::image {
/**
 * @brief Image class that stores a weak ptr to its memory. This class is CUDA aware: it does not
 * require CUDA, but supports being compiled in CUDA. If HAVE_CUDA is defined, then some operations
 * will use CUDA functions.
 * @tparam T the pixel type
 * @tparam MaxValue the maximum intensity value
 */
template <typename T, int MaxValue = DefaultImageValTraits<T>::maxValue>
struct Image {
  static constexpr int maxValue = MaxValue;
  using BaseType = T;

  //////////////////////////////////////////////////////
  // Constructor

  /** @brief Creates an empty image */
  PROJECTARIA_HOST_DEVICE inline Image() noexcept;

  /**
   * @brief Creates an image based on pre-allocated data， pitch is set to be sizeof(T) * w
   * @param ptr Pointer to the data
   * @param w image width, i.e. number of columns in image
   * @param w image height i.e. number of rows in image
   */
  PROJECTARIA_HOST_DEVICE inline Image(T* ptr, size_t w, size_t h) noexcept;
  /**
   * @brief Creates an image based on pre-allocated data
   * @param ptr Pointer to the data
   * @param w image width, i.e. number of columns in image
   * @param w image height i.e. number of rows in image
   * @param w image pitch, i.e. number of bytes per row
   */
  PROJECTARIA_HOST_DEVICE inline Image(T* ptr, size_t w, size_t h, size_t pitch) noexcept;

  /**
   * @brief Make a shallow copy of another image (copy constructor)
   * @param other the Image class that is copied from
   */
  PROJECTARIA_HOST_DEVICE inline Image(const Image<std::remove_cv_t<T>, MaxValue>& other) noexcept;

  //////////////////////////////////////////////////////
  // Query dimensions

  /** @brief Returns the number of columns */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline size_t width() const;
  /** @brief Returns the number of rows */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline size_t height() const;
  /** @brief Returns the number of channels */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline size_t channel() const;
  /** @brief Returns (the number of columns, the number of rows) in an Eigen::Vector2i vector */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline Eigen::Vector2i dim() const;

  //////////////////////////////////////////////////////
  // Storage
  /** @brief Returns if the image is empty */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline bool isValid() const;
  /** @brief Returns if the image pixel occupies a contiguous memory, i.e. pitch == sizeof(T) * w */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline bool isContiguous() const;
  /** @brief Returns the data pointer */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline T* data() const;
  /** @brief Returns the pitch, i.e. number of bytes per row */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline size_t pitch() const;
  /** @brief Returns size of bytes occupied by the entire image */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline size_t sizeBytes() const;

  //////////////////////////////////////////////////////
  // Iterators

  /** @brief Returns the iterator pointing to the begin of the image */
  PROJECTARIA_HOST_DEVICE inline ImageIterator<T> begin();
  /** @brief Returns the iterator pointing to the end of the image */
  PROJECTARIA_HOST_DEVICE inline ImageIterator<T> end();
  /** @brief Returns the const iterator pointing to the begin of the image */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline ImageIterator<const T> begin() const;
  /** @brief Returns the const iterator pointing to the end of the image */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline ImageIterator<const T> end() const;

  //////////////////////////////////////////////////////
  // Pixel Access

  /** @brief Returns the row pointers for fast pixel access */
  PROJECTARIA_HOST_DEVICE inline T* rowPtr(size_t y);
  /** Returns the const row pointers for fast pixel access */
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline const T* rowPtr(size_t y) const;

  /** @brief Returns a reference to the pixel at column x and row y */
  template <typename P, typename std::enable_if_t<std::is_integral<P>::value>* = nullptr>
  PROJECTARIA_HOST_DEVICE inline T& operator()(P x, P y);

  /**
   * @brief Returns a const reference to the pixel at column x and row y
   * @pre: x and y are integers
   */
  template <typename P, typename std::enable_if_t<std::is_integral<P>::value>* = nullptr>
  PROJECTARIA_HOST_DEVICE inline const T& operator()(P x, P y) const;

  /**
   * @brief Returns the bilinearly-interpolated pixel at point (x, y)
   * @pre: 0 <= x < W, 0 <= y < H, x and y are float point values
   */
  template <typename P, typename std::enable_if_t<std::is_floating_point<P>::value>* = nullptr>
  PROJECTARIA_HOST_DEVICE inline T operator()(P u, P v) const;

  //////////////////////////////////////////////////////
  // Bounds Checking

  /** @brief Returns if column x and row y is in bound */
  template <typename P, typename std::enable_if_t<std::is_integral<P>::value>* = nullptr>
  PROJECTARIA_HOST_DEVICE inline bool inBounds(P x, P y, P border = 0) const;

  /**
   * @brief Returns if point (x, y) is in bound, i.e. -0.5 <= x < W-0.5 and -0.5 <= y < H-0.5
   * @pre x and y are float point values
   */
  template <typename P, typename std::enable_if_t<std::is_floating_point<P>::value>* = nullptr>
  PROJECTARIA_HOST_DEVICE inline bool inBounds(P x, P y, P border = 0) const;

 protected:
  PROJECTARIA_HOST_DEVICE [[nodiscard]] inline bool yInBounds(int y) const;

 protected:
  size_t pitch_;
  T* ptr_;
  size_t w_;
  size_t h_;
};

// 8-bit grayscale image
using ImageU8 = Image<uint8_t>;
using Image2U8 = Image<Eigen::Matrix<uint8_t, 2, 1>>;
using Image3U8 = Image<Eigen::Matrix<uint8_t, 3, 1>>;
using Image4U8 = Image<Eigen::Matrix<uint8_t, 4, 1>>;

// 10-bit grayscale image
using ImageU10 = Image<uint16_t, 1023>;
using Image2U10 = Image<Eigen::Matrix<uint16_t, 2, 1>, 1023>;
using Image3U10 = Image<Eigen::Matrix<uint16_t, 3, 1>, 1023>;

// 12-bit grayscale image
using ImageU12 = Image<uint16_t, 4095>;
using Image2U12 = Image<Eigen::Matrix<uint16_t, 2, 1>, 4095>;
using Image3U12 = Image<Eigen::Matrix<uint16_t, 3, 1>, 4095>;

// 16-bit grayscale image
using ImageU16 = Image<uint16_t, 65535>;
using Image2U16 = Image<Eigen::Matrix<uint16_t, 2, 1>, 65535>;
using Image3U16 = Image<Eigen::Matrix<uint16_t, 3, 1>, 65535>;

// 16-bit float ABGR image
using Image4F16 = Image<Eigen::Matrix<Eigen::half, 4, 1>, 65504>;

// 32-bit float grayscale image
using ImageF32 = Image<float>;
using Image2F32 = Image<Eigen::Matrix<float, 2, 1>>;
using Image3F32 = Image<Eigen::Matrix<float, 3, 1>>;
using Image4F32 = Image<Eigen::Matrix<float, 4, 1>>;

// 64-bit grayscale image
using ImageU64 = Image<uint64_t>;

//////////////////////////////////////////////////////
// implementation below

namespace {
// Scalar case
template <class T>
struct Zero {
  static T val() {
    return T(0);
  }
};

// Specialization for Eigen types
template <class T, int M, int N, int Opts>
struct Zero<Eigen::Matrix<T, M, N, Opts>> {
  static Eigen::Matrix<T, M, N, Opts> val() {
    Eigen::Matrix<T, M, N, Opts> o;
    o.setZero();
    return o;
  }
};
} // namespace

template <typename Scalar, std::enable_if_t<std::is_floating_point<Scalar>::value, int> = 0>
Eigen::Vector2<Scalar> imageCenter(size_t imageWidth, size_t imageHeight) {
  return Eigen::Vector2<Scalar>{
      static_cast<Scalar>(imageWidth) / Scalar(2.0) - Scalar(0.5),
      static_cast<Scalar>(imageHeight) / Scalar(2.0) - Scalar(0.5)};
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline Image<T, MaxValue>::Image() noexcept
    : pitch_(0), ptr_(nullptr), w_(0), h_(0) {}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline Image<T, MaxValue>::Image(T* ptr, size_t w, size_t h) noexcept
    : pitch_(w * sizeof(T)), ptr_(ptr), w_(w), h_(h) {}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline Image<T, MaxValue>::Image(
    T* ptr,
    size_t w,
    size_t h,
    size_t pitch) noexcept
    : pitch_(pitch), ptr_(ptr), w_(w), h_(h) {}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline Image<T, MaxValue>::Image(
    const Image<std::remove_cv_t<T>, MaxValue>& other) noexcept
    : pitch_(other.pitch_), ptr_(other.ptr_), w_(other.w_), h_(other.h_) {}

//////////////////////////////////////////////////////
// Query dimensions
//////////////////////////////////////////////////////

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline size_t Image<T, MaxValue>::width() const {
  return w_;
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline size_t Image<T, MaxValue>::height() const {
  return h_;
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline size_t Image<T, MaxValue>::channel() const {
  return DefaultImageValTraits<T>::channel;
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline Eigen::Vector2i Image<T, MaxValue>::dim() const {
  return Eigen::Vector2i(w_, h_);
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline bool Image<T, MaxValue>::isValid() const {
  return ptr_;
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline bool Image<T, MaxValue>::isContiguous() const {
  return w_ * sizeof(T) == pitch_;
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline T* Image<T, MaxValue>::data() const {
  return ptr_;
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline size_t Image<T, MaxValue>::sizeBytes() const {
  return pitch_ * h_;
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline size_t Image<T, MaxValue>::pitch() const {
  return pitch_;
}

//////////////////////////////////////////////////////
// Iterators
//////////////////////////////////////////////////////
template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline ImageIterator<T> Image<T, MaxValue>::begin() {
  return ImageIterator<T>{ptr_, ptr_, ptr_ + w_, pitch_};
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline ImageIterator<T> Image<T, MaxValue>::end() {
  T* onePastLastRow = (T*)((uint8_t*)(ptr_) + h_ * pitch_);
  return ImageIterator<T>{onePastLastRow, onePastLastRow, onePastLastRow + w_, pitch_};
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline ImageIterator<const T> Image<T, MaxValue>::begin() const {
  return ImageIterator<const T>{ptr_, ptr_, ptr_ + w_, pitch_};
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline ImageIterator<const T> Image<T, MaxValue>::end() const {
  const T* const onePastLastRow = (T* const)((uint8_t* const)(ptr_) + h_ * pitch_);
  return ImageIterator<const T>{onePastLastRow, onePastLastRow, onePastLastRow + w_, pitch_};
}

//////////////////////////////////////////////////////
// Discrete Pixel Access
template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline T* Image<T, MaxValue>::rowPtr(size_t y) {
  PROJECTARIA_CUDASAFE_ASSERT(yInBounds(y));
  return (T*)((unsigned char*)(ptr_) + y * pitch_);
}

template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline const T* Image<T, MaxValue>::rowPtr(size_t y) const {
  PROJECTARIA_CUDASAFE_ASSERT(yInBounds(y));
  return (T*)((unsigned char*)(ptr_) + y * pitch_);
}

template <typename T, int MaxValue>
template <typename P, typename std::enable_if_t<std::is_integral<P>::value>*>
PROJECTARIA_HOST_DEVICE inline T& Image<T, MaxValue>::operator()(P x, P y) {
  PROJECTARIA_CUDASAFE_ASSERT(inBounds(x, y));
  return rowPtr(y)[x];
}

template <typename T, int MaxValue>
template <typename P, typename std::enable_if_t<std::is_integral<P>::value>*>
PROJECTARIA_HOST_DEVICE inline const T& Image<T, MaxValue>::operator()(P x, P y) const {
  PROJECTARIA_CUDASAFE_ASSERT(inBounds(x, y));
  return rowPtr(y)[x];
}

//////////////////////////////////////////////////////
// Bounds Checking
//////////////////////////////////////////////////////
template <typename T, int MaxValue>
PROJECTARIA_HOST_DEVICE inline bool Image<T, MaxValue>::yInBounds(int y) const {
  return 0 <= y && y < (int)h_;
}

template <typename T, int MaxValue>
template <typename P, typename std::enable_if_t<std::is_integral<P>::value>*>
PROJECTARIA_HOST_DEVICE inline bool Image<T, MaxValue>::inBounds(P x, P y, P border) const {
  return border <= static_cast<int64_t>(x) &&
      static_cast<int64_t>(x) < static_cast<int64_t>(w_) - border &&
      border <= static_cast<int64_t>(y) &&
      static_cast<int64_t>(y) < static_cast<int64_t>(h_) - border;
}

template <typename T, int MaxValue>
template <typename P, typename std::enable_if_t<std::is_floating_point<P>::value>*>
PROJECTARIA_HOST_DEVICE inline bool Image<T, MaxValue>::inBounds(P x, P y, P border) const {
  return x >= border - static_cast<P>(0.5) && y >= border - static_cast<P>(0.5) &&
      x <= w_ - border - static_cast<P>(0.5) && y <= h_ - border - static_cast<P>(0.5);
}

template <typename T, int MaxValue>
template <typename P, typename std::enable_if_t<std::is_floating_point<P>::value>*>
PROJECTARIA_HOST_DEVICE inline T Image<T, MaxValue>::operator()(P u, P v) const {
  const int ix = (int)(u);
  const int iy = (int)(v);
  const float dx = u - ix;
  const float dy = v - iy;
  const float dxdy = dx * dy;

  const char* bpc = (const char*)(ptr_ + ix) + iy * pitch_;
  if constexpr (DefaultImageValTraits<T>::isEigen) {
    auto val = ((const T*)(bpc + pitch_))[1].template cast<float>() * dxdy +
        ((const T*)(bpc + pitch_))[0].template cast<float>() * (dy - dxdy) +
        ((const T*)(bpc))[1].template cast<float>() * (dx - dxdy) +
        ((const T*)(bpc))[0].template cast<float>() * (1 - dx - dy + dxdy);
    using Scalar = typename DefaultImageValTraits<T>::Scalar;
    return val.template cast<Scalar>();
  } else {
    return static_cast<float>(((const T*)(bpc + pitch_))[1]) * dxdy +
        static_cast<float>(((const T*)(bpc + pitch_))[0]) * (dy - dxdy) +
        static_cast<float>(((const T*)(bpc))[1]) * (dx - dxdy) +
        static_cast<float>(((const T*)(bpc))[0]) * (1.0 - dx - dy + dxdy);
  }
}

static_assert(Image<uint8_t>::maxValue == 255, "Compile time sanity check");
static_assert(Image<float>::maxValue == 1, "Compile time sanity check");
static_assert(Image<uint16_t, 1023>::maxValue == 1023, "Compile time sanity check");
static_assert(Image<float, 0>::maxValue == 0, "Compile time sanity check");
static_assert(Image<Eigen::Matrix<uint8_t, 3, 3>>::maxValue == 255, "Compile time sanity check");
static_assert(Image<Eigen::Matrix<float, 1, 2>>::maxValue == 1, "Compile time sanity check");
static_assert(
    Image<Eigen::Matrix<uint16_t, 1, 3>, 1023>::maxValue == 1023,
    "Compile time sanity check");
static_assert(Image<Eigen::Vector3d, -1>::maxValue == -1, "Compile time sanity check");
} // namespace projectaria::tools::image
