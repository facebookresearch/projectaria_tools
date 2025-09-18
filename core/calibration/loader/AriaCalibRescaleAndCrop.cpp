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
using RescaleParamMap = std::unordered_map<RescaleInput, RescaleParam, RescaleInput::Hash>;

// Supported rescale options for Gen1 devices
void initForGen1(RescaleParamMap& supportedMapping) {
  // RGB options 1: {2880,2880} -> {1408, 1408}
  supportedMapping.emplace(
      RescaleInput{
          .deviceVersion = DeviceVersion::Gen1,
          .cameraLabel = "camera-rgb",
          .originalResolution = Eigen::Vector2i{2880, 2880},
          .newResolution = Eigen::Vector2i{1408, 1408}},
      RescaleParam{.scale = 0.5, .offset = Eigen::Vector2d{32.0, 32.0}});
  // RGB options 2: {2880,2880} -> {704, 704}
  supportedMapping.emplace(
      RescaleInput{
          .deviceVersion = DeviceVersion::Gen1,
          .cameraLabel = "camera-rgb",
          .originalResolution = Eigen::Vector2i{2880, 2880},
          .newResolution = Eigen::Vector2i{704, 704}},
      RescaleParam{.scale = 0.25, .offset = Eigen::Vector2d{32.0, 32.0}});

  // ET options 1: {640, 480} -> {320, 240}
  supportedMapping.emplace(
      RescaleInput{
          .deviceVersion = DeviceVersion::Gen1,
          .cameraLabel = "camera-et-left",
          .originalResolution = Eigen::Vector2i{640, 480},
          .newResolution = Eigen::Vector2i{320, 240}},
      RescaleParam{.scale = 0.5, .offset = Eigen::Vector2d{0.0, 0.0}});
  supportedMapping.emplace(
      RescaleInput{
          .deviceVersion = DeviceVersion::Gen1,
          .cameraLabel = "camera-et-right",
          .originalResolution = Eigen::Vector2i{640, 480},
          .newResolution = Eigen::Vector2i{320, 240}},
      RescaleParam{.scale = 0.5, .offset = Eigen::Vector2d{0.0, 0.0}});
}

// Supported rescale options for Gen2 devices
void initForGen2(RescaleParamMap& supportedMapping) {
  // RGB options 1: {4032, 3024} -> {2016, 1512}
  supportedMapping.emplace(
      RescaleInput{
          .deviceVersion = DeviceVersion::Gen2,
          .cameraLabel = "camera-rgb",
          .originalResolution = Eigen::Vector2i{4032, 3024},
          .newResolution = Eigen::Vector2i{2016, 1512}},
      RescaleParam{.scale = 0.5, .offset = Eigen::Vector2d{0.0, 0.0}});

  // RGB options 2: {4032, 3024} -> {2560, 3024}
  supportedMapping.emplace(
      RescaleInput{
          .deviceVersion = DeviceVersion::Gen2,
          .cameraLabel = "camera-rgb",
          .originalResolution = Eigen::Vector2i{4032, 3024},
          .newResolution = Eigen::Vector2i{2560, 3024}},
      RescaleParam{.scale = 1.0, .offset = Eigen::Vector2d{640.0, 0.0}});

  // RGB options 3: {4032, 3024} -> {2560, 1920}
  supportedMapping.emplace(
      RescaleInput{
          .deviceVersion = DeviceVersion::Gen2,
          .cameraLabel = "camera-rgb",
          .originalResolution = Eigen::Vector2i{4032, 3024},
          .newResolution = Eigen::Vector2i{2560, 1920}},
      RescaleParam{.scale = 0.635, .offset = Eigen::Vector2d{0.0, 0.0}});

  // ET options 1: {400, 400} -> {200, 200}
  supportedMapping.emplace(
      RescaleInput{
          .deviceVersion = DeviceVersion::Gen2,
          .cameraLabel = "camera-et-left",
          .originalResolution = Eigen::Vector2i{400, 400},
          .newResolution = Eigen::Vector2i{200, 200}},
      RescaleParam{.scale = 0.5, .offset = Eigen::Vector2d{0.0, 0.0}});
  supportedMapping.emplace(
      RescaleInput{
          .deviceVersion = DeviceVersion::Gen2,
          .cameraLabel = "camera-et-right",
          .originalResolution = Eigen::Vector2i{400, 400},
          .newResolution = Eigen::Vector2i{200, 200}},
      RescaleParam{.scale = 0.5, .offset = Eigen::Vector2d{0.0, 0.0}});
}

// A helper function to initialize the supported rescale mapping
RescaleParamMap buildSupportedRescaleMapping() {
  RescaleParamMap supportedRescaleMapping;
  initForGen1(supportedRescaleMapping);
  initForGen2(supportedRescaleMapping);

  return supportedRescaleMapping;
}

} // namespace

RescaleParam getRescaleParam(const RescaleInput& rescaleInput) {
  // Initialize a static mapping of supported rescale parameters, this will only be initialized
  // once!
  static const RescaleParamMap kSupportedRescaleMapping = buildSupportedRescaleMapping();

  // Check if input is supported
  const auto rescaleIter = kSupportedRescaleMapping.find(rescaleInput);
  XR_CHECK(
      rescaleIter != kSupportedRescaleMapping.end(),
      "Unsupported camera rescale input of {}:{}, from {},{} to {},{}",
      getName(rescaleInput.deviceVersion),
      rescaleInput.cameraLabel,
      rescaleInput.originalResolution[0],
      rescaleInput.originalResolution[1],
      rescaleInput.newResolution[0],
      rescaleInput.newResolution[1]);
  return rescaleIter->second;
}

CameraCalibration rescaleSingleCamera(
    const CameraCalibration& inputCalib,
    const Eigen::Vector2i& newImageSize,
    const RescaleParam& rescaleParam) {
  return inputCalib.rescale(newImageSize, rescaleParam.scale, rescaleParam.offset);
}

CameraCalibration rescaleSingleCamera(
    const CameraCalibration& inputCalib,
    const Eigen::Vector2i& newImageSize,
    const DeviceVersion& deviceVersion) {
  RescaleInput rescaleInput{
      .deviceVersion = deviceVersion,
      .cameraLabel = inputCalib.getLabel(),
      .originalResolution = inputCalib.getImageSize(),
      .newResolution = newImageSize};
  return rescaleSingleCamera(inputCalib, newImageSize, getRescaleParam(rescaleInput));
}

void tryCropAndScaleCameraCalibration(
    DeviceCalibration& deviceCalibration,
    const std::map<std::string, Eigen::Vector2i>& labelToImageResolution) {
  const auto deviceVersion = deviceCalibration.getDeviceVersion();
  for (const auto& [label, resolution] : labelToImageResolution) {
    std::optional<CameraCalibration> maybeCamCalib = deviceCalibration.getCameraCalib(label);
    // obtain if specified camera exists in DeviceCalibration
    XR_CHECK(
        maybeCamCalib,
        "specified camera {} does not exist in cameraCalibs. No rescaling performed.",
        label);
    CameraCalibration camCalib = *maybeCamCalib;
    camCalib = rescaleSingleCamera(camCalib, resolution, deviceVersion);
    deviceCalibration.setCameraCalibration(label, camCalib);
  }
}
} // namespace projectaria::tools::calibration
