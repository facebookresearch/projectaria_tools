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

#include <data_provider/SensorDataType.h>

#include <map>

#include <vrs/StreamId.h>

namespace projectaria::tools::data_provider {

std::string getName(const SensorDataType type) {
  static const std::map<SensorDataType, std::string> kTypeNameMap = {
      {SensorDataType::Image, "image"},
      {SensorDataType::Imu, "IMU"},
      {SensorDataType::Audio, "audio"},
      {SensorDataType::Barometer, "barometer"},
      {SensorDataType::Gps, "GPS"},
      {SensorDataType::Wps, "WPS"},
      {SensorDataType::Magnetometer, "magnetometer"},
      {SensorDataType::Bluetooth, "bluetooth"},
      {SensorDataType::NotValid, "invalid"}};
  return kTypeNameMap.at(type);
}

bool supportsHostTimeDomain(SensorDataType type) {
  return (
      (type == SensorDataType::Image) || (type == SensorDataType::Imu) ||
      (type == SensorDataType::Magnetometer));
}

bool hasCalibration(SensorDataType type) {
  return (
      (type == SensorDataType::Image) || (type == SensorDataType::Imu) ||
      (type == SensorDataType::Magnetometer) || (type == SensorDataType::Barometer) ||
      (type == SensorDataType::Audio));
}

} // namespace projectaria::tools::data_provider
