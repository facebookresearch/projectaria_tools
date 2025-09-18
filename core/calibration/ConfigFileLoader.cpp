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

#include "ConfigFileLoader.h"
#include <calibration/ConfigData.h>
#include <fmt/format.h>
#include <unordered_map>

namespace projectaria::tools::calibration {

// Customized Hash for std::pair<DeviceVersion, DeviceSubtype>
struct PairHash {
  std::size_t operator()(const std::pair<DeviceVersion, std::string>& pair) const {
    // Hash the enum class and the string separately
    std::size_t hash1 = std::hash<int>{}(static_cast<int>(pair.first));
    std::size_t hash2 = std::hash<std::string>{}(pair.second);
    // Combine the two hash values
    return hash1 ^ (hash2 << 1);
  }
};

std::string loadCameraConfigJsonAsString(const DeviceVersion& deviceVersion) {
  // Mapping from device version / subtype to the corresponding config string
  const std::unordered_map<DeviceVersion, std::string> kCameraConfigStringMapping = {
      {DeviceVersion::Gen1, kCameraConfigJsonGen1()},
      {DeviceVersion::Gen2, kCameraConfigJsonGen2()},
  };
  // This is the implementation for BUCK build
  const auto& strIter = kCameraConfigStringMapping.find(deviceVersion);
  if (strIter == kCameraConfigStringMapping.end()) {
    throw std::runtime_error(fmt::format(
        "Unsupported device version to load camera config json file: {}", getName(deviceVersion)));
  }
  return strIter->second;
}

std::string loadDeviceCadExtrinsicsCsvAsString(
    const DeviceVersion& deviceVersion,
    const std::string& deviceSubtype) {
  const std::unordered_map<std::pair<DeviceVersion, std::string>, std::string, PairHash>
      kDeviceCadExtrinsicsStringMapping = {
          {std::make_pair(DeviceVersion::Gen1, "DVT-S"), kDeviceCadExtrinsicsGen1DVTS()},
          {std::make_pair(DeviceVersion::Gen1, "DVT-L"), kDeviceCadExtrinsicsGen1DVTL()},
          {std::make_pair(DeviceVersion::Gen1, "DVT-MARIA"), kDeviceCadExtrinsicsGen1DVTMARIA()},
          {std::make_pair(DeviceVersion::Gen1, "EVT-S"), kDeviceCadExtrinsicsGen1EVTS()},
          {std::make_pair(DeviceVersion::Gen1, "EVT-L"), kDeviceCadExtrinsicsGen1EVTL()},
          {std::make_pair(DeviceVersion::Gen2, "EVT_SS"), kDeviceCadExtrinsicsGen2EVTSS()},
          {std::make_pair(DeviceVersion::Gen2, "EVT_SL"), kDeviceCadExtrinsicsGen2EVTSL()},
          {std::make_pair(DeviceVersion::Gen2, "EVT_LL"), kDeviceCadExtrinsicsGen2EVTLL()},
          {std::make_pair(DeviceVersion::Gen2, "EVT_LS"), kDeviceCadExtrinsicsGen2EVTLS()},
          {std::make_pair(DeviceVersion::Gen2, "DVT_SS"), kDeviceCadExtrinsicsGen2DVTSS()},
          {std::make_pair(DeviceVersion::Gen2, "DVT_SL"), kDeviceCadExtrinsicsGen2DVTSL()},
          {std::make_pair(DeviceVersion::Gen2, "DVT_LL"), kDeviceCadExtrinsicsGen2DVTLL()},
          {std::make_pair(DeviceVersion::Gen2, "DVT_LS"), kDeviceCadExtrinsicsGen2DVTLS()},
      };
  const auto& strIter =
      kDeviceCadExtrinsicsStringMapping.find(std::make_pair(deviceVersion, deviceSubtype));

  if (strIter == kDeviceCadExtrinsicsStringMapping.end()) {
    throw std::runtime_error(fmt::format(
        "Unsupported device version and subtype to load device CAD extrinsics csv file: {},{}",
        getName(deviceVersion),
        deviceSubtype));
  }
  return strIter->second;
}

} // namespace projectaria::tools::calibration
