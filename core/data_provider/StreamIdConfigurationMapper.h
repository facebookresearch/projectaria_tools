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

#include <map>

#include <data_provider/SensorConfiguration.h>
#include <data_provider/players/BatteryStatusPlayer.h>

namespace projectaria::tools::data_provider {
class StreamIdConfigurationMapper {
 public:
  StreamIdConfigurationMapper() = default;
  StreamIdConfigurationMapper(
      std::shared_ptr<vrs::MultiRecordFileReader> reader,
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
      std::map<vrs::StreamId, std::shared_ptr<VioHighFrequencyPlayer>>& vioHighFreqPlayers);

  // APIs to query two-way mapping of StreamId <-> sensor label, where the former is the id of a
  // stream in the VRS file.
  [[nodiscard]] ImageConfigRecord getImageConfiguration(const vrs::StreamId& streamId) const;
  [[nodiscard]] MotionConfigRecord getImuConfiguration(const vrs::StreamId& streamId) const;
  [[nodiscard]] GpsConfigRecord getGpsConfiguration(const vrs::StreamId& streamId) const;
  [[nodiscard]] WifiBeaconConfigRecord getWpsConfiguration(const vrs::StreamId& streamId) const;
  [[nodiscard]] AudioConfig getAudioConfiguration(const vrs::StreamId& streamId) const;
  [[nodiscard]] BarometerConfigRecord getBarometerConfiguration(
      const vrs::StreamId& streamId) const;
  [[nodiscard]] BatteryStatusConfiguration getBatteryStatusConfiguration(
      const vrs::StreamId& streamId) const;
  [[nodiscard]] BluetoothBeaconConfigRecord getBluetoothConfiguration(
      const vrs::StreamId& streamId) const;
  [[nodiscard]] MotionConfigRecord getMagnetometerConfiguration(
      const vrs::StreamId& streamId) const;
  [[nodiscard]] PpgConfiguration getPpgConfiguration(const vrs::StreamId& streamId) const;
  [[nodiscard]] AlsConfiguration getAlsConfiguration(const vrs::StreamId& streamId) const;
  [[nodiscard]] TemperatureConfiguration getTemperatureConfiguration(
      const vrs::StreamId& streamId) const;
  [[nodiscard]] EyeGazeConfiguration getEyeGazeConfiguration(const vrs::StreamId& streamId) const;
  [[nodiscard]] HandPoseConfiguration getHandPoseConfiguration(const vrs::StreamId& streamId) const;
  [[nodiscard]] VioConfiguration getVioConfiguration(const vrs::StreamId& streamId) const;
  [[nodiscard]] VioHighFreqConfiguration getVioHighFreqConfiguration(
      const vrs::StreamId& streamId) const;

 private:
  std::map<vrs::StreamId, ImageConfigRecord> streamIdToImageConfig_;
  std::map<vrs::StreamId, MotionConfigRecord> streamIdToImuConfig_;
  std::map<vrs::StreamId, GpsConfigRecord> streamIdToGpsConfig_;
  std::map<vrs::StreamId, WifiBeaconConfigRecord> streamIdToWpsConfig_;
  std::map<vrs::StreamId, AudioConfig> streamIdToAudioConfig_;
  std::map<vrs::StreamId, BarometerConfigRecord> streamIdToBaroConfig_;
  std::map<vrs::StreamId, BatteryStatusConfiguration> streamIdToBatteryStatusConfig_;
  std::map<vrs::StreamId, BluetoothBeaconConfigRecord> streamIdToBluetoothConfig_;
  std::map<vrs::StreamId, MotionConfigRecord> streamIdToMagConfig_;
  std::map<vrs::StreamId, PpgConfiguration> streamIdToPpgConfig_;
  std::map<vrs::StreamId, AlsConfiguration> streamIdToAlsConfig_;
  std::map<vrs::StreamId, TemperatureConfiguration> streamIdToTemperatureConfig_;
  std::map<vrs::StreamId, EyeGazeConfiguration> streamIdToEyeGazeConfig_;
  std::map<vrs::StreamId, HandPoseConfiguration> streamIdToHandPoseConfig_;
  std::map<vrs::StreamId, VioConfiguration> streamIdToVioConfig_;
  std::map<vrs::StreamId, VioHighFreqConfiguration> streamIdToVioHighFreqConfig_;
};
} // namespace projectaria::tools::data_provider
