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

#include "FromPixelFrame.h"
#include <cstdint>

namespace projectaria::tools::image {
std::optional<ImageVariant> fromPixelFrame(std::shared_ptr<vrs::utils::PixelFrame> pixelFrame) {
  if (!pixelFrame) {
    return {};
  }

  uint8_t* buffer = pixelFrame->wdata();
  int width = pixelFrame->getWidth();
  int height = pixelFrame->getHeight();
  int stride = pixelFrame->getStride();

  switch (pixelFrame->getPixelFormat()) {
    case vrs::PixelFormat::UNDEFINED: ///< 1 uint8_t
      return {};
    case vrs::PixelFormat::GREY8: ///< 1 uint8_t
    {
      ImageU8 image(buffer, width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::RGB8: ///< 3 uint8_t values, red + green + blue.
    case vrs::PixelFormat::BGR8: ///< 3 uint8_t values, blue + green + red.
    {
      Image3U8 image(
          reinterpret_cast<Eigen::Matrix<uint8_t, 3, 1>*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::DEPTH32F: ///< 1 32 bit float value, representing a depth.
    {
      Image<float> image(reinterpret_cast<float*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::RGBA8: ///< 4 uint8_t values, red + blue + green + alpha.
    {
      Image<Eigen::Matrix<uint8_t, 4, 1>> image(
          reinterpret_cast<Eigen::Matrix<uint8_t, 4, 1>*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::RGB10: ///< uses 16 bit little-endian values. 6 most significant bits are
                                  ///< unused and set to 0.
    {
      Image3U10 image(
          reinterpret_cast<Eigen::Matrix<uint16_t, 3, 1>*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::RGB12: ///< uses 16 bit little-endian values. 4 most significant bits are
                                  ///< unused and set to 0.
    {
      Image3U12 image(
          reinterpret_cast<Eigen::Matrix<uint16_t, 3, 1>*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::GREY10: ///< uses 16 bit little-endian values. 6 most significant bits
                                   ///< are unused and set to 0.
    {
      ImageU10 image(reinterpret_cast<uint16_t*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::GREY12: ///< uses 16 bit little-endian values. 4 most significant bits
                                   ///< are unused and set to 0.
    {
      ImageU12 image(reinterpret_cast<uint16_t*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::GREY16: ///< uses 16 bit little-endian values.
    {
      ImageU16 image(reinterpret_cast<uint16_t*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::RGB32F: ///< 1 32 bit float value.
    {
      Image<Eigen::Vector3f> image(
          reinterpret_cast<Eigen::Vector3f*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::SCALAR64F: ///< 1 64 bit float value, representing high precision image
                                      ///< data.
    {
      Image<uint64_t> image(reinterpret_cast<uint64_t*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::YUY2: ///< 4 uint8_t values, 4:2:2, single plane.
    {
      ImageU8 image(buffer, width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::RGBA32F: ///< 1 32 bit float value.
    {
      Image<Eigen::Vector4f> image(
          reinterpret_cast<Eigen::Vector4f*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::BAYER8_RGGB: ///< 8 bit per pixel, RGGB bayer pattern.
    {
      ImageU8 image(buffer, width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::
        RAW10: ///< https://developer.android.com/reference/android/graphics/ImageFormat#RAW10
    case vrs::PixelFormat::RAW10_BAYER_RGGB: ///< 10 bit per pixel, RGGB bayer pattern.
    case vrs::PixelFormat::RAW10_BAYER_BGGR: ///< 10 bit per pixel, BGGR bayer pattern.
    {
      ImageU10 image(reinterpret_cast<uint16_t*>(buffer), width, height, stride);
      return ImageVariant(image);
    }
    case vrs::PixelFormat::YUV_I420_SPLIT: ///< 3 uint8_t values, 4:2:0. The 3 planes are stored
                                           ///< separately.
    case vrs::PixelFormat::RGB_IR_RAW_4X4: ///< As seen on the OV2312, a 4x4 pattern of BGRG GIrGIr
                                           ///< RGBG GIrGIr where Ir means infrared.
    case vrs::PixelFormat::YUV_420_NV21: ///< Y plane + half width/half height chroma plane with
                                         ///< weaved V and U values.
    case vrs::PixelFormat::YUV_420_NV12: ///< Y plane + half width/half height chroma plane with
                                         ///< weaved U and V values.
    default:
      return {};
  }
}
} // namespace projectaria::tools::image
