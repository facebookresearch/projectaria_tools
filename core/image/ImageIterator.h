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

#include <iterator>

#include "Portability.h"

namespace projectaria::tools::image {

template <typename T>
class ImageIterator {
 public:
  using iterator_category = std::random_access_iterator_tag;
  using difference_type = ptrdiff_t;
  using value_type = T;
  using pointer = T*;
  using reference = T&;

  PROJECTARIA_HOST_DEVICE
  ImageIterator(pointer ptr, pointer rowStart, pointer rowEnd, const size_t pitch)
      : ptr(ptr), rowStart(rowStart), rowEnd(rowEnd), pitch(pitch) {}

  PROJECTARIA_HOST_DEVICE_INLINE reference operator*() const {
    return *ptr;
  }

  PROJECTARIA_HOST_DEVICE_INLINE ImageIterator& operator++() {
    ++ptr;
    if (ptr == rowEnd) {
      pitchForward(rowStart);
      pitchForward(rowEnd);
      ptr = rowStart;
    }
    return *this;
  }

  PROJECTARIA_HOST_DEVICE_INLINE ImageIterator& operator++(int) {
    ImageIterator copy = *this;
    ++(*this);
    return copy;
  }

  PROJECTARIA_HOST_DEVICE_INLINE ImageIterator& operator--() {
    if (ptr == rowStart) {
      pitchBackward(rowStart);
      pitchBackward(rowEnd);
      ptr = rowEnd;
    }
    --ptr;
    return *this;
  }

  PROJECTARIA_HOST_DEVICE_INLINE ImageIterator& operator--(int) {
    ImageIterator copy = *this;
    --(*this);
    return copy;
  }

  PROJECTARIA_HOST_DEVICE_INLINE ImageIterator operator+(difference_type delta) const {
    ImageIterator copy = *this;
    if (delta > 0) {
      copy.stepForward(delta);
    } else {
      copy.stepBackward(-delta);
    }
    return copy;
  }

  PROJECTARIA_HOST_DEVICE_INLINE ImageIterator operator-(difference_type delta) const {
    ImageIterator copy = *this;
    if (delta > 0) {
      copy.stepBackward(delta);
    } else {
      copy.stepForward(-delta);
    }
    return copy;
  }

  PROJECTARIA_HOST_DEVICE_INLINE ImageIterator& operator+=(difference_type delta) {
    if (delta > 0) {
      stepForward(delta);
    } else {
      stepBackward(-delta);
    }
    return *this;
  }

  PROJECTARIA_HOST_DEVICE_INLINE ImageIterator& operator-=(difference_type delta) {
    if (delta > 0) {
      stepBackward(delta);
    } else {
      stepForward(-delta);
    }
    return *this;
  }

  PROJECTARIA_HOST_DEVICE_INLINE difference_type operator-(const ImageIterator& other) const {
    const difference_type rowDiff =
        ((uint8_t*)rowStart - (uint8_t*)other.rowStart) / static_cast<difference_type>(pitch);
    return rowDiff * width() + (ptr - rowStart) - (other.ptr - other.rowStart);
  }

  PROJECTARIA_HOST_DEVICE_INLINE bool operator<(const ImageIterator& other) const {
    return ptr < other.ptr;
  }

  PROJECTARIA_HOST_DEVICE_INLINE bool operator==(const ImageIterator& other) const {
    return other.ptr == ptr;
  }

  PROJECTARIA_HOST_DEVICE_INLINE bool operator!=(const ImageIterator& other) const {
    return other.ptr != ptr;
  }

 private:
  PROJECTARIA_HOST_DEVICE_INLINE void pitchForward(pointer& toMove, const size_t numRows = 1) {
    toMove = (pointer)((uint8_t*)(toMove) + numRows * pitch);
  }
  PROJECTARIA_HOST_DEVICE_INLINE void pitchBackward(pointer& toMove, const size_t numRows = 1) {
    toMove = (pointer)((uint8_t*)(toMove)-numRows * pitch);
  }
  PROJECTARIA_HOST_DEVICE_INLINE size_t width() const {
    return rowEnd - rowStart;
  }
  PROJECTARIA_HOST_DEVICE_INLINE void stepForward(const difference_type delta) {
    const size_t remainingInRow = std::distance(ptr, rowEnd);
    if (remainingInRow <= delta) {
      const size_t w = width();
      const size_t numRows = 1 + (delta - remainingInRow) / w;
      const size_t rowPosition = (delta - remainingInRow) % w;
      pitchForward(rowStart, numRows);
      pitchForward(rowEnd, numRows);
      ptr = rowStart + rowPosition;
    } else {
      ptr += delta;
    }
  }
  PROJECTARIA_HOST_DEVICE_INLINE void stepBackward(const difference_type delta) {
    const size_t remainingInRow = ptr - rowStart;
    if (remainingInRow < delta) {
      const size_t w = width();
      const size_t numRows = 1 + (delta - remainingInRow - 1) / w;
      const size_t rowPosition = 1 + (delta - remainingInRow - 1) % w;
      pitchBackward(rowStart, numRows);
      pitchBackward(rowEnd, numRows);
      ptr = rowEnd - rowPosition;
    } else {
      ptr -= delta;
    }
  }

  pointer ptr;
  pointer rowStart;
  pointer rowEnd;
  size_t pitch;
};

} // namespace projectaria::tools::image
