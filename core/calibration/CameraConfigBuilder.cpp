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

#include <calibration/CameraConfigBuilder.h>

#include <fstream>
#include <iostream>
#include <sstream>

#include <fmt/format.h>
#include "nlohmann/json.hpp"

#include <calibration/ConfigFileLoader.h>

namespace projectaria::tools::calibration {

CameraConfigBuilder::CameraConfigBuilder(const DeviceVersion& deviceVersion) {
  bool loadFlag = loadConfigMappingFromJsonFile(deviceVersion);

  if (!loadFlag) {
    throw std::runtime_error("Failed to load camera config mapping from json file");
  }
}

namespace {
// A helper function to parse a json content into CameraConfigData
bool fromJson(const nlohmann::json& j, CameraConfigData& config) {
  // List of required fields. maybeValidRadius is optional.
  const std::vector<std::string> requiredFields = {
      "image_width",
      "image_height",
      "max_solid_angle_radians",
  };
  // Check if all required fields are present
  for (const auto& field : requiredFields) {
    if (!j.contains(field)) {
      fmt::print("ERROR: Missing field {} in camera config file. \n", field);
      return false;
    }
  }

  config.imageWidth = j["image_width"];
  config.imageHeight = j["image_height"];
  config.maxSolidAngle = j["max_solid_angle_radians"];

  // Read optional fields
  if (j.contains("maybe_valid_radius")) {
    config.maybeValidRadius = j["maybe_valid_radius"];
  }

  return true;
}
} // namespace

bool CameraConfigBuilder::loadConfigMappingFromJsonFile(const DeviceVersion& deviceVersion) {
  // Open camera config json file
  std::stringstream configFileStream(loadCameraConfigJsonAsString(deviceVersion));

  // Parse json file content
  nlohmann::json configJson;
  try {
    configFileStream >> configJson;
  } catch (const nlohmann::json::parse_error& e) {
    fmt::print("ERROR: JSON parse error: {}\n", e.what());
    return false;
  }
  for (const auto& [label, perCamJson] : configJson.items()) {
    CameraConfigData perCamConfig;
    if (!fromJson(perCamJson, perCamConfig)) {
      fmt::print("ERROR: Failed to parse config json for camera {}\n", label);
      return false;
    }
    cameraLabelToConfig_[label] = perCamConfig;
  }
  return true;
}

std::optional<CameraConfigData> CameraConfigBuilder::getCameraConfigData(
    const std::string& cameraLabel) const {
  if (cameraLabelToConfig_.find(cameraLabel) == cameraLabelToConfig_.end()) {
    return std::nullopt;
  }
  return cameraLabelToConfig_.at(cameraLabel);
}

} // namespace projectaria::tools::calibration
