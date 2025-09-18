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

#include <cmath>
#include <optional>
#include <unordered_map>

#include <calibration/DeviceVersion.h>

namespace projectaria::tools::calibration {

struct CameraConfigData {
  int imageWidth = 0;
  int imageHeight = 0;
  double maxSolidAngle = 0.0;
  std::optional<double> maybeValidRadius = std::nullopt;
};

class CameraConfigBuilder {
 public:
  explicit CameraConfigBuilder(const DeviceVersion& deviceVersion);

  std::optional<CameraConfigData> getCameraConfigData(const std::string& cameraLabel) const;

 private:
  bool loadConfigMappingFromJsonFile(const DeviceVersion& deviceVersion);

  std::unordered_map<std::string, CameraConfigData> cameraLabelToConfig_;
};

} // namespace projectaria::tools::calibration
