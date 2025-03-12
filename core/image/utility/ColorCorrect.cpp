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

#include "ColorCorrect.h"
#include "ColorCorrectData.h"

namespace projectaria::tools::image {

// https://www.nayuki.io/page/srgb-transform-library
inline double linearToSrgb(double linearPixelVal) {
  double srgbPixelVal = 0;
  if (linearPixelVal <= 0.0031308) {
    srgbPixelVal = linearPixelVal * 12.92;
  } else {
    srgbPixelVal = 1.055 * std::pow(linearPixelVal, 1.0 / 2.4) - 0.055;
  }
  srgbPixelVal = std::max(std::min(srgbPixelVal, 1.0), 0.0);
  return srgbPixelVal;
}

template <class T, int MaxVal>
ManagedImage<T, DefaultImageAllocator<T>, MaxVal> colorCorrectImage(
    const Image<T, MaxVal>& srcImage) {
  ManagedImage<T, DefaultImageAllocator<T>, MaxVal> dstImage(srcImage.width(), srcImage.height());
  for (int col = 0; col < srcImage.width(); ++col) {
    for (int row = 0; row < srcImage.height(); ++row) {
      if constexpr (DefaultImageValTraits<T>::isEigen && DefaultImageValTraits<T>::channel == 3) {
        auto srcIntValue = static_cast<T>(srcImage(col, row));
        // Convert to linear color space
        Eigen::Vector3<double> linearColorValue{
            cameraInvCRFTable[static_cast<uint8_t>(srcIntValue[0])],
            cameraInvCRFTable[static_cast<uint8_t>(srcIntValue[1])],
            cameraInvCRFTable[static_cast<uint8_t>(srcIntValue[2])]};
        // Multiply with corrected CCM 3x3 matrix
        Eigen::Vector3<double> correctedColorValue = colorCorrectionMatrix * linearColorValue;
        // Convert to srgb color space
        Eigen::Vector3<double> srgbColorValue{
            linearToSrgb(correctedColorValue[0]),
            linearToSrgb(correctedColorValue[1]),
            linearToSrgb(correctedColorValue[2])};

        srgbColorValue = srgbColorValue * 255.0;
        using Scalar = typename DefaultImageValTraits<T>::Scalar;
        dstImage(col, row) = srgbColorValue.template cast<Scalar>();
      }
    }
  }
  return dstImage;
}

ManagedImageVariant colorCorrect(const ImageVariant& srcImage) {
  return std::visit(
      [](const auto& srcImage) -> image::ManagedImageVariant {
        return colorCorrectImage(srcImage);
      },
      srcImage);
}

} // namespace projectaria::tools::image
