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

#include "AseCalibrationProvider.h"

namespace projectaria::dataset::ase {

CameraCalibration getAseRgbCalibration(const size_t imageSize) {
  // only supports three image sizes
  if (imageSize != 704 && imageSize != 1408) {
    throw std::runtime_error(
        "Input image size " + std::to_string(imageSize) +
        " not supported. Supported image sizes are 704 or 1408.");
  }

  // calibration at 704
  std::string label = "camera-rgb";
  projectaria::tools::calibration::CameraProjection::ModelType type =
      projectaria::tools::calibration::CameraProjection::ModelType::Fisheye624;
  Eigen::VectorXd projectionParams(projectaria::tools::calibration::Fisheye624::kNumParams);
  projectionParams << 297.6375381033778, 357.6599197217746, 349.1922497127481, 0.3650890375644368,
      -0.1738082418112771, -0.7534945484033189, 2.434788882752295, -2.57786220300886,
      0.8788483538598834, 0.0008005198595407136, -0.000294237814554143, 0., 0., 0., 0.;

  Sophus::SE3d T_Camera_Device;
  T_Camera_Device.setQuaternion( // w, x, y ,z
      {0.9441858687689326, 0.326409343828490850, 0.029274992008313648, 0.033361059956531547});
  T_Camera_Device.translation() << -0.007530096566173914, -0.010908549841580260,
      -0.003598063315542823;
  const double kRgbValidRadius = 1415.0 / 4.0;
  const double kRgbMaxSolidAngleRad = 1;
  const int imageWidth = 704;
  const int imageHeight = 704;
  CameraCalibration calib(
      label,
      type,
      projectionParams,
      T_Camera_Device,
      imageWidth,
      imageHeight,
      kRgbValidRadius,
      kRgbMaxSolidAngleRad);

  if (imageSize == 704) {
    return calib;
  } else { // 1408:
    return calib.rescale(Eigen::Vector2i(1408, 1408), 2, -Eigen::Vector2d{0, 0});
  }
}
} // namespace projectaria::dataset::ase
