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
#include <iostream>

namespace projectaria::tools::image {

template <class T, int MaxVal>
ManagedImage<T, DefaultImageAllocator<T>, MaxVal> distortImage(
    const Image<T, MaxVal>& src,
    const std::function<std::optional<Eigen::Vector2f>(const Eigen::Vector2f&)>& inverseWarp,
    const Eigen::Vector2i& imageSize,
    const InterpolationMethod method) {
  ManagedImage<T, DefaultImageAllocator<T>, MaxVal> dst(imageSize(0), imageSize(1));

  for (int y = 0; y < dst.height(); y++) {
    for (int x = 0; x < dst.width(); x++) {
      Eigen::Vector2f dstPixel(static_cast<float>(x), static_cast<float>(y));
      std::optional<Eigen::Vector2f> maybeSrcPixel = inverseWarp(dstPixel);
      if (!maybeSrcPixel || !src.inBounds((*maybeSrcPixel)(0), (*maybeSrcPixel)(1), 0.5f)) {
        dst(x, y) = Zero<T>::val();
      } else {
        switch (method) {
          case InterpolationMethod::Bilinear:
            dst(x, y) = src((*maybeSrcPixel)(0), (*maybeSrcPixel)(1));
            break;
          case InterpolationMethod::NearestNeighbor:
            Eigen::Vector2i nearestPixel =
                (*maybeSrcPixel + Eigen::Vector2f(0.5, 0.5)).template cast<int>();
            dst(x, y) = src(nearestPixel(0), nearestPixel(1));
            break;
        }
      }
    }
  }

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
