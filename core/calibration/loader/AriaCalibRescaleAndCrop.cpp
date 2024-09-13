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

#include <calibration/loader/AriaCalibRescaleAndCrop.h>

#include <logging/Checks.h>
#define DEFAULT_LOG_CHANNEL "AriaCalibRescaleAndCrop"
#include <logging/Log.h>

#include <stdexcept>

namespace projectaria::tools::calibration {

namespace {
// we only support 2880x2880, 1408x1408 or 704x704 RGB sizes
CameraCalibration rescaleAriaRgb(
    const CameraCalibration& camCalib,
    const Eigen::Vector2i& newImageSize) {
  Eigen::Vector2i calibImageSize = camCalib.getImageSize();
  XR_CHECK(
      calibImageSize == Eigen::Vector2i(2880, 2880),
      "Original image size in calibration are assumed to be (2880, 2880) for Aria RGB images. Detected size: ({}, {})",
      calibImageSize.x(),
      calibImageSize.y());
  XR_CHECK(
      newImageSize == Eigen::Vector2i(1408, 1408) || newImageSize == Eigen::Vector2i(704, 704),
      "Supported downscaled image size are assumed to be (1408, 1408) or (704, 704) for Aria RGB images. Detected size: ({}, {})",
      newImageSize.x(),
      newImageSize.y());

  double rescaleFactor = newImageSize == Eigen::Vector2i(1408, 1408) ? 0.5 : 0.25;
  return camCalib.rescale(newImageSize, rescaleFactor, {32.0, 32.0});
}

// we only support 640x480 or 320x240 ET sizes
CameraCalibration rescaleAriaEyetracking(
    const CameraCalibration& camCalib,
    const Eigen::Vector2i& newImageSize) {
  Eigen::Vector2i calibImageSize = camCalib.getImageSize();
  XR_CHECK(
      calibImageSize == Eigen::Vector2i(640, 480),
      "Original image size in calibration are assumed to be (640, 480) for Aria ET images. Detected size: ({}, {})",
      calibImageSize.x(),
      calibImageSize.y());
  XR_CHECK(
      newImageSize == Eigen::Vector2i(320, 240),
      "Supported downscaled image size are assumed to be (320, 240) for Aria ET images. Detected size: ({}, {})",
      newImageSize.x(),
      newImageSize.y());

  return camCalib.rescale(newImageSize, 0.5);
}
} // namespace

void tryCropAndScaleCameraCalibration(
    DeviceCalibration& deviceCalibration,
    const std::map<std::string, Eigen::Vector2i>& labelToImageResolution) {
  for (const auto& [label, resolution] : labelToImageResolution) {
    std::optional<CameraCalibration> maybeCamCalib = deviceCalibration.getCameraCalib(label);
    // obtain if specified camera exists in DeviceCalibration
    XR_CHECK(
        maybeCamCalib,
        "specified camera {} does not exist in cameraCalibs. No rescaling performed.",
        label);
    CameraCalibration camCalib = *maybeCamCalib;
    if (label == "camera-rgb") {
      camCalib = rescaleAriaRgb(camCalib, resolution);
    } else if (label == "camera-et-left" || label == "camera-et-right") {
      camCalib = rescaleAriaEyetracking(camCalib, resolution);
    } else {
      const std::string error =
          fmt::format("camera {} does not support any resolution scaling!", label);
      XR_LOGE("{}", error);
      throw std::runtime_error{error};
    }
    deviceCalibration.setCameraCalibration(label, camCalib);
  }
}
} // namespace projectaria::tools::calibration
