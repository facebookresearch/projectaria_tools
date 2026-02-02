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

#include "Image.h"

namespace projectaria::tools::image {
template <class T>
using DefaultImageAllocator = std::allocator<T>;

/**
 * @brief Image that manages its own memory, storing a strong pointer to its memory. The class is
 * non-copyable but movable.
 * @tparam T the pixel type
 * @tparam Allocator_ allocator of memory
 * @tparam MaxValue the maximum intensity value
 */
template <
    typename T,
    class Allocator_ = DefaultImageAllocator<T>,
    int MaxValue = DefaultImageValTraits<T>::maxValue>
class ManagedImage : public Image<T, MaxValue> {
 public:
  using Base = Image<T, MaxValue>;
  using Allocator = Allocator_;

  /** @brief Creates an empty image */
  inline ManagedImage();

  /** @brief Creates an 2D image. */
  inline ManagedImage(size_t w, size_t h);

  inline ManagedImage(const ManagedImage& other) = delete; // Not copy constructable

  /** @brief Moves from another image */
  inline ManagedImage(ManagedImage&& img) noexcept;
  /** @brief Moves from another image */
  inline void operator=(ManagedImage&& img);

  inline ~ManagedImage();

  /**
   * @brief Resize the image to specific size, all image content become invalid
   * @param width number of columns of new image
   * @param height number of rows of new image
   */
  inline void reInitialize(size_t width, size_t height);
  /**
   * @brief Resize the image to specific size, all image content become invalid
   * @param dim new image size (width, height)
   */
  inline void reInitialize(const Eigen::Vector2i& dim);

  /** @brief Returns the underlying Image class */
  inline Image<T, MaxValue> getWeakPtr();

 protected:
  inline void deallocate();
};

// 8-bit grayscale image
using ManagedImageU8 = ManagedImage<uint8_t>;
using ManagedImage2U8 = ManagedImage<Eigen::Matrix<uint8_t, 2, 1>>;
using ManagedImage3U8 = ManagedImage<Eigen::Matrix<uint8_t, 3, 1>>;
using ManagedImage4U8 = ManagedImage<Eigen::Matrix<uint8_t, 4, 1>>;

// 10-bit grayscale image
using ManagedImage3U8 = ManagedImage<
    Eigen::Matrix<uint8_t, 3, 1>,
    DefaultImageAllocator<Eigen::Matrix<uint8_t, 3, 1>>,
    255>;
using ManagedImageU10 = ManagedImage<uint16_t, DefaultImageAllocator<uint16_t>, 1023>;
using ManagedImage2U10 = ManagedImage<
    Eigen::Matrix<uint16_t, 2, 1>,
    DefaultImageAllocator<Eigen::Matrix<uint16_t, 2, 1>>,
    1023>;
using ManagedImage3U10 = ManagedImage<
    Eigen::Matrix<uint16_t, 3, 1>,
    DefaultImageAllocator<Eigen::Matrix<uint16_t, 3, 1>>,
    1023>;

// 12-bit grayscale image
using ManagedImageU12 = ManagedImage<uint16_t, DefaultImageAllocator<uint16_t>, 4095>;
using ManagedImage2U12 = ManagedImage<
    Eigen::Matrix<uint16_t, 2, 1>,
    DefaultImageAllocator<Eigen::Matrix<uint16_t, 2, 1>>,
    4095>;
using ManagedImage3U12 = ManagedImage<
    Eigen::Matrix<uint16_t, 3, 1>,
    DefaultImageAllocator<Eigen::Matrix<uint16_t, 3, 1>>,
    4095>;

// 16-bit grayscale image
using ManagedImageU16 = ManagedImage<uint16_t, DefaultImageAllocator<uint16_t>, 65535>;
using ManagedImage2U16 = ManagedImage<
    Eigen::Matrix<uint16_t, 2, 1>,
    DefaultImageAllocator<Eigen::Matrix<uint16_t, 2, 1>>,
    65535>;
using ManagedImage3U16 = ManagedImage<
    Eigen::Matrix<uint16_t, 3, 1>,
    DefaultImageAllocator<Eigen::Matrix<uint16_t, 3, 1>>,
    65535>;

// 16-bit float ABGR image
using ManagedImage4F16 = ManagedImage<
    Eigen::Matrix<Eigen::half, 4, 1>,
    DefaultImageAllocator<Eigen::Matrix<Eigen::half, 4, 1>>,
    65504>;

// 32 bit grayscale image
using ManagedImageF32 = ManagedImage<float>;
using ManagedImage2F32 = ManagedImage<Eigen::Matrix<float, 2, 1>>;
using ManagedImage3F32 = ManagedImage<Eigen::Matrix<float, 3, 1>>;
using ManagedImage4F32 = ManagedImage<Eigen::Matrix<float, 4, 1>>;

// 64-bit grayscale image
using ManagedImageU64 = ManagedImage<uint64_t>;

template <typename T, template <typename...> class Alloc>
using ManagedImageAlloc = ManagedImage<T, Alloc<T>>;

//////////////////////////////////////////////////////////////////////////
// Implementation below

// Destructor
template <typename T, class Allocator_, int MaxValue>
inline ManagedImage<T, Allocator_, MaxValue>::~ManagedImage() {
  deallocate();
}

// Constructor
template <typename T, class Allocator_, int MaxValue>
inline ManagedImage<T, Allocator_, MaxValue>::ManagedImage() = default;

template <typename T, class Allocator_, int MaxValue>
inline ManagedImage<T, Allocator_, MaxValue>::ManagedImage(size_t w, size_t h)
    : Base(Allocator().allocate(w * h), w, h, w * sizeof(T)) {
  PROJECTARIA_CUDASAFE_ASSERT(w != 0 && h != 0, "{},{}", w, h);
}

// Move constructor and move assignment
template <typename T, class Allocator_, int MaxValue>
inline ManagedImage<T, Allocator_, MaxValue>::ManagedImage(ManagedImage&& img) noexcept
    : Base(img.ptr_, img.w_, img.h_, img.pitch_) {
  img.ptr_ = nullptr;
}

template <typename T, class Allocator_, int MaxValue>
inline void ManagedImage<T, Allocator_, MaxValue>::operator=(ManagedImage&& img) {
  deallocate();
  Base::pitch_ = img.pitch_;
  Base::ptr_ = img.ptr_;
  Base::w_ = img.w_;
  Base::h_ = img.h_;
  img.ptr_ = nullptr;
}

template <typename T, class Allocator_, int MaxValue>
inline void ManagedImage<T, Allocator_, MaxValue>::reInitialize(size_t width, size_t height) {
  if (width == 0 || height == 0) {
    *this = ManagedImage();
  } else if (!Base::ptr || Base::Width() != width || Base::Height() != height) {
    *this = ManagedImage(width, height);
  }
}

template <typename T, class Allocator_, int MaxValue>
inline Image<T, MaxValue> ManagedImage<T, Allocator_, MaxValue>::getWeakPtr() {
  return Image<T, MaxValue>(Image<T>::ptr_, Image<T>::w_, Image<T>::h_, Image<T>::pitch_);
}

template <typename T, class Allocator_, int MaxValue>
inline void ManagedImage<T, Allocator_, MaxValue>::deallocate() {
  if (Base::ptr_) {
    Allocator().deallocate(Base::ptr_, Base::sizeBytes() / sizeof(T));
    Base::ptr_ = nullptr;
  }
}
} // namespace projectaria::tools::image
