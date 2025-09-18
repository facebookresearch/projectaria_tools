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

#include <type_traits>
#include <variant>

#include "Image.h"
#include "ManagedImage.h"

namespace projectaria::tools::image {
using ImageVariant = std::variant<
    ImageU8,
    Image2U8,
    Image3U8,
    Image4U8,
    ImageU10,
    ImageU12,
    ImageU16,
    Image2U10,
    Image2U12,
    Image2U16,
    Image3U10,
    Image3U12,
    Image3U16,
    Image4F16,
    ImageF32,
    Image2F32,
    Image3F32,
    Image4F32,
    ImageU64>;

using ManagedImageVariant = std::variant<
    ManagedImageU8,
    ManagedImage2U8,
    ManagedImage3U8,
    ManagedImage4U8,
    ManagedImageU10,
    ManagedImageU12,
    ManagedImageU16,
    ManagedImage2U10,
    ManagedImage2U12,
    ManagedImage2U16,
    ManagedImage3U10,
    ManagedImage3U12,
    ManagedImage3U16,
    ManagedImage4F16,
    ManagedImageF32,
    ManagedImage2F32,
    ManagedImage3F32,
    ManagedImage4F32,
    ManagedImageU64>;

using PixelValueVariant = std::variant<float, uint8_t, uint16_t, uint64_t, Eigen::half>;
/**
 * @brief Returns the pixel at specified coordinate in an image
 * @param imageVariant the input image
 * @param x column index
 * @param y row index
 * @param ch channel index
 */
inline PixelValueVariant at(const ImageVariant& imageVariant, int x, int y, int ch);

/**
 * @brief Returns the raw pointer to the data of an image
 * @param imageVariant the input image
 */
inline void* getDataPtr(const ImageVariant& imageVariant);

/**
 * @brief Returns the number of columns in an image
 * @param imageVariant the input image
 */
inline int getWidth(const ImageVariant& imageVariant);

/**
 * @brief Returns the number of rows in an image
 * @param imageVariant the input image
 */
inline int getHeight(const ImageVariant& imageVariant);

/**
 * @brief Returns the number of bytes per row in an image
 * @param imageVariant the input image
 */
inline int getPitch(const ImageVariant& imageVariant);

/**
 * @brief Returns the number of channels in an image
 * @param imageVariant the input image
 */
inline int getChannel(const ImageVariant& imageVariant);

/**
 * @brief Converts from ManagedImageVariant to ImageVariant
 * @param imageVariant the ManagedImageVariant object
 */
inline ImageVariant toImageVariant(const ManagedImageVariant& managedImageVariant);

template <class T, int MaxVal>
ManagedImage<T, DefaultImageAllocator<T>, MaxVal> copyToManagedImage(
    const Image<T, MaxVal>& srcImage) {
  ManagedImage<T, DefaultImageAllocator<T>, MaxVal> dstImage(srcImage.width(), srcImage.height());
  for (int col = 0; col < srcImage.width(); ++col) {
    for (int row = 0; row < srcImage.height(); ++row) {
      dstImage(col, row) = srcImage(col, row);
    }
  }
  return dstImage;
}

inline ManagedImageVariant toManagedImageVariant(const ImageVariant& imageVariant) {
  return std::visit(
      [](const auto& image) -> ManagedImageVariant { return copyToManagedImage(image); },
      imageVariant);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation below

// get pixel value at (x, y, channel)
// std::visit(PixelVisitor(x, y, channel), variant);
struct PixelVisitor {
  PixelVisitor(int xIn, int yIn, int ch) : x(xIn), y(yIn), channel(ch) {}

  template <class T, int M>
  PixelValueVariant operator()(const projectaria::tools::image::Image<T, M>& image) const {
    if (channel != 0) {
      throw std::runtime_error("Gray scale image but input channel index != 0");
    }

    if (!image.inBounds(x, y)) {
      throw std::runtime_error("Pixel not in bound");
    }
    return PixelValueVariant(image(x, y));
  }

  template <class T, int M, int D>
  PixelValueVariant operator()(
      const projectaria::tools::image::Image<Eigen::Matrix<T, D, 1, 0, D, 1>, M>& image) const {
    if (channel < 0 || channel >= D) {
      throw std::runtime_error("Channel value out of range");
    }

    if (!image.inBounds(x, y)) {
      throw std::runtime_error("Pixel not in bound");
    }
    return PixelValueVariant(image(x, y)(channel));
  }

 public:
  int x;
  int y;
  int channel;
};

inline PixelValueVariant at(const ImageVariant& imageVariant, int x, int y, int ch) {
  return std::visit(PixelVisitor(x, y, ch), imageVariant);
}

inline void* getDataPtr(const ImageVariant& imageVariant) {
  return std::visit([](const auto& img) { return static_cast<void*>(img.data()); }, imageVariant);
}

inline int getWidth(const ImageVariant& imageVariant) {
  return std::visit([](const auto& img) { return static_cast<int>(img.width()); }, imageVariant);
}

inline int getHeight(const ImageVariant& imageVariant) {
  return std::visit([](const auto& img) { return static_cast<int>(img.height()); }, imageVariant);
}

inline int getPitch(const ImageVariant& imageVariant) {
  return std::visit([](const auto& img) { return static_cast<int>(img.pitch()); }, imageVariant);
}

inline int getChannel(const ImageVariant& imageVariant) {
  return std::visit([](const auto& img) { return static_cast<int>(img.channel()); }, imageVariant);
}

inline ImageVariant toImageVariant(const ManagedImageVariant& managedImageVariant) {
  return std::visit(
      [](auto& managedImage) { return ImageVariant{managedImage}; }, managedImageVariant);
}
} // namespace projectaria::tools::image
