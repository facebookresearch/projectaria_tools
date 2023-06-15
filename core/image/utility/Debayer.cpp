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

#include "Debayer.h"

#include <array>
namespace projectaria::tools::image {

image::ManagedImage3U8 debayer(const image::ImageU8& raw) {
  ManagedImage3U8 debayeredImage(raw.width(), raw.height());
  const std::array<float, 3> rgbAt5000k = {225., 238., 206.};
  // pixel average ratio at 5000 Kevin at 5000 Kelvin daylight
  const float colorCalibRGr5000k = 0.604492; // r / gr
  const float colorCalibGbGr5000k = 1.000977; // gb / gr
  const float colorCalibBGb5000k = 0.5; // b / gb

  // r, gr, gb, b scaled to gr
  const std::array<float, 4> colorCalibration = {
      colorCalibRGr5000k / rgbAt5000k[0] * rgbAt5000k[1],
      1.0,
      colorCalibGbGr5000k,
      colorCalibBGb5000k * colorCalibGbGr5000k / rgbAt5000k[2] * rgbAt5000k[1]};

  const std::array<float, 9> kGreenKernel{0.f, 0.25f, 0.f, 0.25f, 1.f, 0.25f, 0.f, 0.25f, 0.f};
  const std::array<float, 9> kRedBlueKernel{
      0.25f, 0.5f, 0.25f, 0.5f, 1.f, 0.5f, 0.25f, 0.5f, 0.25f};
  const std::array<int, 9> kDx{-1, 0, 1, -1, 0, 1, -1, 0, 1};
  const std::array<int, 9> kDy{-1, -1, -1, 0, 0, 0, 1, 1, 1};

  auto isOn = [](int x, int y, int channel) -> bool {
    if (channel == 0) {
      return x % 2 == 0 && y % 2 == 0;
    } else if (channel == 2) {
      return x % 2 == 1 && y % 2 == 1;
    } else {
      return x % 2 != y % 2;
    }
  };
  auto colorCalibIntensity = [&colorCalibration](int x, int y) -> float {
    int index = (y % 2) * 2 + (x % 2);
    return colorCalibration[index];
  };

  for (int channel = 0; channel < 3; ++channel) {
    const auto& weights = (channel == 1) ? kGreenKernel : kRedBlueKernel;
    for (int x = 0; x < raw.width(); ++x) {
      for (int y = 0; y < raw.height(); ++y) {
        float totalWeight = 0;
        float weightedPixelSum = 0;
        for (int k = 0; k < 9; ++k) {
          int xNeighbor = x + kDx[k];
          int yNeighbor = y + kDy[k];
          if (!raw.inBounds(xNeighbor, yNeighbor)) {
            continue;
          }
          float weight = isOn(xNeighbor, yNeighbor, channel) ? weights[k] : 0.f;
          // correct spectral responses of different filters
          float normalizedIntensity =
              raw(xNeighbor, yNeighbor) / colorCalibIntensity(xNeighbor, yNeighbor);
          weightedPixelSum += normalizedIntensity * weight;
          totalWeight += weight;
        }
        debayeredImage(x, y)[channel] =
            static_cast<uint8_t>(std::min(weightedPixelSum / totalWeight, 255.f));
      }
    }
  }

  return debayeredImage;
}

} // namespace projectaria::tools::image
