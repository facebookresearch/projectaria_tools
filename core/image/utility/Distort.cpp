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

#include "Distort.h"
#include <dispenso/parallel_for.h>
#include <iostream>
#include <vector>

namespace projectaria::tools::image {

template <class T, int MaxVal>
ManagedImage<T, DefaultImageAllocator<T>, MaxVal> distortImage(
    const Image<T, MaxVal>& src,
    const std::function<std::optional<Eigen::Vector2f>(const Eigen::Vector2f&)>& inverseWarp,
    const Eigen::Vector2i& imageSize,
    const InterpolationMethod method) {
  ManagedImage<T, DefaultImageAllocator<T>, MaxVal> dst(imageSize(0), imageSize(1));

  std::vector<Eigen::Vector2i> dstPixels;
  dstPixels.reserve(dst.height() * dst.width());
  for (int y = 0; y < dst.height(); y++) {
    for (int x = 0; x < dst.width(); x++) {
      dstPixels.emplace_back(x, y);
    }
  }

  dispenso::parallel_for(
      0, dstPixels.size(), [&src, &dst, &inverseWarp, &dstPixels, &method](size_t index) {
        Eigen::Vector2f pixel(
            static_cast<float>(dstPixels[index](0)), static_cast<float>(dstPixels[index](1)));
        std::optional<Eigen::Vector2f> maybeSrcPixel = inverseWarp(pixel);
        dst(dstPixels[index].x(), dstPixels[index].y()) = Zero<T>::val();
        if (maybeSrcPixel && src.inBounds((*maybeSrcPixel)(0), (*maybeSrcPixel)(1), 0.5f)) {
          switch (method) {
            case InterpolationMethod::Bilinear:
              dst(dstPixels[index].x(), dstPixels[index].y()) =
                  src((*maybeSrcPixel)(0), (*maybeSrcPixel)(1));
              break;
            case InterpolationMethod::NearestNeighbor:
              Eigen::Vector2i nearestPixel =
                  (*maybeSrcPixel + Eigen::Vector2f(0.5, 0.5)).template cast<int>();
              dst(dstPixels[index].x(), dstPixels[index].y()) =
                  src(nearestPixel(0), nearestPixel(1));
              break;
          }
        }
      });

  return dst;
}

ManagedImageVariant distortImageVariant(
    const ImageVariant& srcVariant,
    const std::function<std::optional<Eigen::Vector2f>(const Eigen::Vector2f&)>& inverseWarp,
    const Eigen::Vector2i& imageSize,
    const InterpolationMethod method) {
  return std::visit(
      [&inverseWarp, &imageSize, &method](const auto& src) {
        return ManagedImageVariant{distortImage(src, inverseWarp, imageSize, method)};
      },
      srcVariant);
}

} // namespace projectaria::tools::image
