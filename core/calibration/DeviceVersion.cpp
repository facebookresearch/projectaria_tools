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

#include <calibration/DeviceVersion.h>

#include <algorithm>
#include <cctype>
#include <map>

namespace projectaria::tools::calibration {

namespace {
// Helper function to convert a string to lowercase
std::string toLowerCase(const std::string& input) {
  std::string result = input; // Create a copy of the input string
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  return result;
}
} // namespace

std::string getName(const DeviceVersion deviceVersion) {
  static const std::map<DeviceVersion, std::string> kVersionNameMap = {
      {DeviceVersion::Gen1, "AriaGen1"},
      {DeviceVersion::Gen2, "AriaGen2"},
      {DeviceVersion::NotValid, "invalid"}};
  return kVersionNameMap.at(deviceVersion);
}

DeviceVersion fromDeviceClassName(const std::string& name) {
  if (toLowerCase(name.substr(0, 4)) == "aria") {
    return DeviceVersion::Gen1;
  } else if (toLowerCase(name) == "oatmeal") {
    return DeviceVersion::Gen2;
  } else {
    return DeviceVersion::NotValid;
  }
}

} // namespace projectaria::tools::calibration
