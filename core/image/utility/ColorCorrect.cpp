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

#include <Eigen/Dense>
#include <thread>

namespace projectaria::tools::image {

// Convert color from linear to srgb color space see:
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
  // Do color correction to fix the color distortion in Aria captured images before Aria V1.12
  // update. This includes 2 fixes:
  // 1. Correct non conventional gamma curve
  // 2. Correct non conventional color temperature and set to 5000K
  // We initialize colorCorrectionMatrixData as const std::array<double, 9> and assign it to
  // Eigen::Matrix3d to avoid compiler's warning about "-Wglobal-constructors"
  const Eigen::Matrix3d colorCorrectionMatrix =
      Eigen::Map<const Eigen::Matrix3d>(&colorCorrectionMatrixData[0]).transpose();
  unsigned int numThreads = std::thread::hardware_concurrency();
  const int chunkSize = (srcImage.height() + numThreads - 1) / numThreads;

  ManagedImage<T, DefaultImageAllocator<T>, MaxVal> dstImage(srcImage.width(), srcImage.height());
  auto processRow = [&](int startRow) {
    for (int col = 0; col < srcImage.width(); ++col) {
      int endRow = std::min(startRow + chunkSize, static_cast<int>(srcImage.height()));
      for (int row = startRow; row < endRow; ++row) {
        if constexpr (DefaultImageValTraits<T>::isEigen && DefaultImageValTraits<T>::channel == 3) {
          auto srcIntValue = static_cast<T>(srcImage(col, row));
          Eigen::Vector3<double> linearColorValue{
              cameraInvCRFTable[static_cast<uint8_t>(srcIntValue[0])],
              cameraInvCRFTable[static_cast<uint8_t>(srcIntValue[1])],
              cameraInvCRFTable[static_cast<uint8_t>(srcIntValue[2])]};
          Eigen::Vector3<double> correctedColorValue = colorCorrectionMatrix * linearColorValue;
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
  };

  // Multi-threaded processing, we use std::thread instead of std::execution::par as
  // std::execution::par doesn't show any improvements potentially due to c++ compiler settings.
  std::vector<std::thread> threads;
  for (int i = 0; i < srcImage.height(); i += chunkSize) {
    threads.emplace_back(processRow, i);
  }
  for (auto& thread : threads) {
    thread.join();
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
