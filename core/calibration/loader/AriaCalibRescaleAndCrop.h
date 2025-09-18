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

#include <calibration/DeviceCalibration.h>

namespace projectaria::tools::calibration {

// A struct to represent the parameters of the underlying rescaling operation
struct RescaleParam {
  double scale;
  Eigen::Vector2d offset;
};

// A struct to represent the input configuration of a camera rescaling operation
struct RescaleInput {
  DeviceVersion deviceVersion;
  std::string cameraLabel;
  Eigen::Vector2i originalResolution;
  Eigen::Vector2i newResolution;

  // Equality operator
  bool operator==(const RescaleInput& other) const {
    return (deviceVersion == other.deviceVersion) && (cameraLabel == other.cameraLabel) &&
        (originalResolution == other.originalResolution) && (newResolution == other.newResolution);
  }

  // Customized Hash function
  struct Hash {
    std::size_t operator()(const RescaleInput& value) const {
      std::size_t h = 2166136261; // initial offset basis
      std::size_t prime = 0x811C9DC5; // FNV prime
      h ^= std::hash<int>{}(static_cast<int>(value.deviceVersion));
      h *= prime;
      h ^= std::hash<std::string>{}(value.cameraLabel);
      // original res does not need to be hashed, because it is 1-1 mapped to camera label.
      h *= prime;
      h ^= std::hash<int>{}(value.newResolution.x());
      h *= prime;
      h ^= std::hash<int>{}(value.newResolution.y());
      h *= prime;
      return h;
    }
  };
};

// Utility function to get rescale parameters for a given camera label and resolution
RescaleParam getRescaleParam(const RescaleInput& rescaleInput);

// Utility functions to rescale a single camera calibration to a new resolution
CameraCalibration rescaleSingleCamera(
    const CameraCalibration& inputCalib,
    const Eigen::Vector2i& newImageSize,
    const RescaleParam& rescaleParam);
CameraCalibration rescaleSingleCamera(
    const CameraCalibration& inputCalib,
    const Eigen::Vector2i& newImageSize,
    const DeviceVersion& deviceVersion);

/**
 * @brief A utility function to crop and rescale an Aria camera calibration. This function exactly
 * replicates the process of the camera firmware. Therefore it only supports a fixed number of
 * operations, otherwise it will throw. We also support a case where the RGB images are 704x704
 * which is not possible by the Aria firmware. This is to account for the case of Aria simulation
 * data which is often at that lower resolution
 * - Aria RGB camera from [2880, 2880] -> [1408, 1408] or [704, 704].
 * - Aria ET camera from [640, 480] -> [320, 240].
 * @param deviceCalibration The device calibration to rescale and crop. This function will modify
 * the calibration in-place.
 * @param labelToImageResolution A map from the camera label to its desired image resolution.
 */
void tryCropAndScaleCameraCalibration(
    DeviceCalibration& deviceCalibration,
    const std::map<std::string, Eigen::Vector2i>& labelToImageResolution);
} // namespace projectaria::tools::calibration
