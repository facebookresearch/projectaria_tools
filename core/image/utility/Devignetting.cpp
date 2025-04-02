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

#include "Devignetting.h"

namespace projectaria::tools::image {

template <class T, int MaxVal>
ManagedImage<T, DefaultImageAllocator<T>, MaxVal> devignettingImage(
    const Image<T, MaxVal>& srcImage,
    const ManagedImage3F32& devignettingMask) {
  ManagedImage<T, DefaultImageAllocator<T>, MaxVal> dstImage(srcImage.width(), srcImage.height());
  // Note: Image class uses col, row convention while Eigen::MatrixXf uses row, col
  // convention.
  if (srcImage.width() != devignettingMask.width() ||
      srcImage.height() != devignettingMask.height()) {
    throw std::runtime_error(
        "devignetting mask size (" + std::to_string(devignettingMask.width()) + "," +
        std::to_string(devignettingMask.height()) + ") does not match source image size (" +
        std::to_string(srcImage.width()) + "," + std::to_string(srcImage.height()) + ")");
  }
  for (int col = 0; col < srcImage.width(); ++col) {
    for (int row = 0; row < srcImage.height(); ++row) {
      if constexpr (DefaultImageValTraits<T>::isEigen) {
        if constexpr (DefaultImageValTraits<T>::channel == 3) {
          auto srcPixel = srcImage(col, row).template cast<double>();
          auto maskPixel = devignettingMask(col, row).template cast<double>();
          auto val = (srcPixel.array() * maskPixel.array()).cwiseMin(255.0);
          using Scalar = typename DefaultImageValTraits<T>::Scalar;
          dstImage(col, row) = val.template cast<Scalar>();
        } else {
          throw std::runtime_error("Unsupported image type, channel must be 3 or 1");
        }
      } else {
        double val = static_cast<double>(srcImage(col, row)) *
            static_cast<double>(devignettingMask(col, row)(0));
        val = std::min(val, 255.0);
        dstImage(col, row) = static_cast<T>(val);
      }
    }
  }
  return dstImage;
}

ManagedImageVariant devignetting(
    const ImageVariant& srcImage,
    const ManagedImage3F32& devignettingMask) {
  return std::visit(
      [&devignettingMask](const auto& srcImage) -> image::ManagedImageVariant {
        return devignettingImage(srcImage, devignettingMask);
      },
      srcImage);
}

} // namespace projectaria::tools::image
