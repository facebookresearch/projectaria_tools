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

#include <data_provider/StreamIdConfigurationMapper.h>

#include <queue>

#define DEFAULT_LOG_CHANNEL "StreamIdConfigurationMapper"
#include <logging/Checks.h>
#include <logging/Log.h>

namespace projectaria::tools::data_provider {
StreamIdConfigurationMapper::StreamIdConfigurationMapper(
    std::shared_ptr<vrs::MultiRecordFileReader>,
    std::map<vrs::StreamId, std::shared_ptr<ImageSensorPlayer>>& imagePlayers,
    std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>>& motionPlayers,
    std::map<vrs::StreamId, std::shared_ptr<GpsPlayer>>& gpsPlayers,
    std::map<vrs::StreamId, std::shared_ptr<WifiBeaconPlayer>>& wpsPlayers,
    std::map<vrs::StreamId, std::shared_ptr<AudioPlayer>>& audioPlayers,
    std::map<vrs::StreamId, std::shared_ptr<BarometerPlayer>>& barometerPlayers,
    std::map<vrs::StreamId, std::shared_ptr<BluetoothBeaconPlayer>>& bluetoothPlayers,
    std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>>& magnetometerPlayers) {
  for (const auto& [streamId, imagePlayer] : imagePlayers) {
    streamIdToImageConfig_.emplace(streamId, imagePlayer->getConfigRecord());
  }
  for (const auto& [streamId, motionPlayer] : motionPlayers) {
    streamIdToImuConfig_.emplace(streamId, motionPlayer->getConfigRecord());
  }
  for (const auto& [streamId, gpsPlayer] : gpsPlayers) {
    streamIdToGpsConfig_.emplace(streamId, gpsPlayer->getConfigRecord());
  }
  for (const auto& [streamId, wpsPlayer] : wpsPlayers) {
    streamIdToWpsConfig_.emplace(streamId, wpsPlayer->getConfigRecord());
  }
  for (const auto& [streamId, audioPlayer] : audioPlayers) {
    streamIdToAudioConfig_.emplace(streamId, audioPlayer->getConfigRecord());
  }
  for (const auto& [streamId, barometerPlayer] : barometerPlayers) {
    streamIdToBaroConfig_.emplace(streamId, barometerPlayer->getConfigRecord());
  }
  for (const auto& [streamId, bluetoothPlayer] : bluetoothPlayers) {
    streamIdToBluetoothConfig_.emplace(streamId, bluetoothPlayer->getConfigRecord());
  }
  for (const auto& [streamId, magnetometerPlayer] : magnetometerPlayers) {
    streamIdToMagConfig_.emplace(streamId, magnetometerPlayer->getConfigRecord());
  }
}

/*******************************
 get configurations
 *******************************/
ImageConfigRecord StreamIdConfigurationMapper::getImageConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToImageConfig_.at(streamId);
}

MotionConfigRecord StreamIdConfigurationMapper::getImuConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToImuConfig_.at(streamId);
}

GpsConfigRecord StreamIdConfigurationMapper::getGpsConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToGpsConfig_.at(streamId);
}

WifiBeaconConfigRecord StreamIdConfigurationMapper::getWpsConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToWpsConfig_.at(streamId);
}

AudioConfig StreamIdConfigurationMapper::getAudioConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToAudioConfig_.at(streamId);
}

BarometerConfigRecord StreamIdConfigurationMapper::getBarometerConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToBaroConfig_.at(streamId);
}

BluetoothBeaconConfigRecord StreamIdConfigurationMapper::getBluetoothConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToBluetoothConfig_.at(streamId);
}

MotionConfigRecord StreamIdConfigurationMapper::getMagnetometerConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToMagConfig_.at(streamId);
}
} // namespace projectaria::tools::data_provider
