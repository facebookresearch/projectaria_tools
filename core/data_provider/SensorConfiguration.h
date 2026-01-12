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

#include <variant>

#include <data_provider/players/AlsPlayer.h>
#include <data_provider/players/AudioPlayer.h>
#include <data_provider/players/BarometerPlayer.h>
#include <data_provider/players/BatteryStatusPlayer.h>
#include <data_provider/players/BluetoothBeaconPlayer.h>
#include <data_provider/players/EyeGazePlayer.h>
#include <data_provider/players/GpsPlayer.h>
#include <data_provider/players/HandPosePlayer.h>
#include <data_provider/players/ImageSensorPlayer.h>
#include <data_provider/players/MotionSensorPlayer.h>
#include <data_provider/players/PpgPlayer.h>
#include <data_provider/players/TemperaturePlayer.h>
#include <data_provider/players/VioHighFrequencyPlayer.h>
#include <data_provider/players/VioPlayer.h>
#include <data_provider/players/WifiBeaconPlayer.h>
#include <vrs/StreamId.h>

#include <data_provider/SensorDataType.h>

namespace projectaria::tools::data_provider {

/**
 * @brief Configuration of a sensor stream, such as stream id, nominal frame rate
 */
class SensorConfiguration {
 public:
  using SensorConfigurationVariant = std::variant<
      std::monostate,
      ImageConfigRecord,
      MotionConfigRecord,
      GpsConfigRecord,
      WifiBeaconConfigRecord,
      AudioConfig,
      BarometerConfigRecord,
      BatteryStatusConfiguration,
      BluetoothBeaconConfigRecord,
      PpgConfiguration,
      AlsConfiguration,
      TemperatureConfiguration,
      EyeGazeConfiguration,
      HandPoseConfiguration,
      VioConfiguration,
      VioHighFreqConfiguration>;

 public:
  explicit SensorConfiguration(
      const SensorConfigurationVariant& dataVariant,
      const SensorDataType& sensorDataType);

  /** @brief Returns the type of sensor data */
  [[nodiscard]] SensorDataType sensorDataType() const;

  /**
   * @brief Returns the sensor configuration as ImageConfigRecord
   * @pre type is Image
   */
  [[nodiscard]] ImageConfigRecord imageConfiguration() const;

  /**
   *@brief Returns the sensor configuration as MotionConfigRecord
   * @pre type is Imu
   */
  [[nodiscard]] MotionConfigRecord imuConfiguration() const;

  /**
   * @brief Returns the sensor configuration as GpsConfigRecord
   * @pre type is Gps
   */
  [[nodiscard]] GpsConfigRecord gpsConfiguration() const;

  /**
   * @brief Returns the sensor configuration as WifiBeaconConfigRecord
   * @pre type is Wps
   */
  [[nodiscard]] WifiBeaconConfigRecord wpsConfiguration() const;

  /**
   * @brief Returns the sensor configuration as AudioConfig
   * @pre type is Audio
   */
  [[nodiscard]] AudioConfig audioConfiguration() const;

  /**
   * @brief Returns the sensor configuration as BarometerConfigRecord
   * @pre type is Barometer
   */

  [[nodiscard]] BarometerConfigRecord barometerConfiguration() const;

  /**
   * @brief Returns the sensor configuration as BatteryStatusConfiguration
   * @pre type is BatteryStatus
   */
  [[nodiscard]] BatteryStatusConfiguration batteryStatusConfiguration() const;

  /**
   * @brief Returns the sensor configuration as Bluetooth
   * @pre type is Bluetooth
   */
  [[nodiscard]] BluetoothBeaconConfigRecord bluetoothConfiguration() const;
  /**
   * @brief Returns the sensor configuration as MotionConfigRecord
   * @pre type is Magnetometer
   */
  [[nodiscard]] MotionConfigRecord magnetometerConfiguration() const;

  /**
   * @brief Returns the sensor configuration as PpgConfiguration
   * @pre type is Ppg
   */
  [[nodiscard]] PpgConfiguration ppgConfiguration() const;

  /**
   * @brief Returns the sensor configuration as AlsConfiguration
   * @pre type is Als
   */
  [[nodiscard]] AlsConfiguration alsConfiguration() const;

  /**
   * @brief Returns the sensor configuration as TemperatureConfiguration
   * @pre type is Temperature
   */
  [[nodiscard]] TemperatureConfiguration temperatureConfiguration() const;

  /**
   * @brief Returns the sensor configuration as HandPoseConfiguration
   * @pre type is EyeGaze
   */
  [[nodiscard]] EyeGazeConfiguration eyeGazeConfiguration() const;

  /**
   * @brief Returns the sensor configuration as HandPoseConfiguration
   * @pre type is HandPose
   */
  [[nodiscard]] HandPoseConfiguration handPoseConfiguration() const;

  /**
   * @brief Returns the sensor configuration as VioConfiguration
   * @pre type is Vio
   */
  [[nodiscard]] VioConfiguration vioConfiguration() const;

  /**
   * @brief Returns the sensor configuration as VioHighFreqConfiguration
   * @pre type is VioHighFreq
   */
  [[nodiscard]] VioHighFreqConfiguration vioHighFreqConfiguration() const;

  /**
   * @brief Returns the nominal frame rate of the sensor
   */
  [[nodiscard]] double getNominalRateHz() const;

 private:
  SensorConfigurationVariant sensorConfigurationVariant_;
  SensorDataType sensorDataType_;
};
} // namespace projectaria::tools::data_provider
