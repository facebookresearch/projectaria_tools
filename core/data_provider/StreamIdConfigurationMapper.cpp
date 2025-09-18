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
    std::map<vrs::StreamId, std::shared_ptr<BatteryStatusPlayer>>& batteryStatusPlayers,
    std::map<vrs::StreamId, std::shared_ptr<BluetoothBeaconPlayer>>& bluetoothPlayers,
    std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>>& magnetometerPlayers,
    std::map<vrs::StreamId, std::shared_ptr<PpgPlayer>>& ppgPlayers,
    std::map<vrs::StreamId, std::shared_ptr<AlsPlayer>>& alsPlayers,
    std::map<vrs::StreamId, std::shared_ptr<TemperaturePlayer>>& temperaturePlayers,
    std::map<vrs::StreamId, std::shared_ptr<EyeGazePlayer>>& EyeGazePlayers,
    std::map<vrs::StreamId, std::shared_ptr<HandPosePlayer>>& handPosePlayers,
    std::map<vrs::StreamId, std::shared_ptr<VioPlayer>>& vioPlayers,
    std::map<vrs::StreamId, std::shared_ptr<VioHighFrequencyPlayer>>& vioHighFreqPlayers) {
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
  for (const auto& [streamId, batteryStatusPlayer] : batteryStatusPlayers) {
    streamIdToBatteryStatusConfig_.emplace(streamId, batteryStatusPlayer->getConfigRecord());
  }
  for (const auto& [streamId, bluetoothPlayer] : bluetoothPlayers) {
    streamIdToBluetoothConfig_.emplace(streamId, bluetoothPlayer->getConfigRecord());
  }
  for (const auto& [streamId, magnetometerPlayer] : magnetometerPlayers) {
    streamIdToMagConfig_.emplace(streamId, magnetometerPlayer->getConfigRecord());
  }
  for (const auto& [streamId, eyeGazePlayer] : EyeGazePlayers) {
    streamIdToEyeGazeConfig_.emplace(streamId, eyeGazePlayer->getConfigRecord());
  }
  for (const auto& [streamId, handPosePlayer] : handPosePlayers) {
    streamIdToHandPoseConfig_.emplace(streamId, handPosePlayer->getConfigRecord());
  }
  for (const auto& [streamId, vioPlayer] : vioPlayers) {
    streamIdToVioConfig_.emplace(streamId, vioPlayer->getConfigRecord());
  }
  for (const auto& [streamId, vioHighFreqPlayer] : vioHighFreqPlayers) {
    streamIdToVioHighFreqConfig_.emplace(streamId, vioHighFreqPlayer->getConfigRecord());
  }
  for (const auto& [streamId, ppgPlayer] : ppgPlayers) {
    streamIdToPpgConfig_.emplace(streamId, ppgPlayer->getConfigRecord());
  }
  for (const auto& [streamId, alsPlayer] : alsPlayers) {
    streamIdToAlsConfig_.emplace(streamId, alsPlayer->getConfigRecord());
  }
  for (const auto& [streamId, temperaturePlayer] : temperaturePlayers) {
    streamIdToTemperatureConfig_.emplace(streamId, temperaturePlayer->getConfigRecord());
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

BatteryStatusConfiguration StreamIdConfigurationMapper::getBatteryStatusConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToBatteryStatusConfig_.at(streamId);
}

BluetoothBeaconConfigRecord StreamIdConfigurationMapper::getBluetoothConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToBluetoothConfig_.at(streamId);
}

MotionConfigRecord StreamIdConfigurationMapper::getMagnetometerConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToMagConfig_.at(streamId);
}

PpgConfiguration StreamIdConfigurationMapper::getPpgConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToPpgConfig_.at(streamId);
}

AlsConfiguration StreamIdConfigurationMapper::getAlsConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToAlsConfig_.at(streamId);
}

TemperatureConfiguration StreamIdConfigurationMapper::getTemperatureConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToTemperatureConfig_.at(streamId);
}

EyeGazeConfiguration StreamIdConfigurationMapper::getEyeGazeConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToEyeGazeConfig_.at(streamId);
}
HandPoseConfiguration StreamIdConfigurationMapper::getHandPoseConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToHandPoseConfig_.at(streamId);
}
VioConfiguration StreamIdConfigurationMapper::getVioConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToVioConfig_.at(streamId);
}
VioHighFreqConfiguration StreamIdConfigurationMapper::getVioHighFreqConfiguration(
    const vrs::StreamId& streamId) const {
  return streamIdToVioHighFreqConfig_.at(streamId);
}
} // namespace projectaria::tools::data_provider
