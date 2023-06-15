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

#include <players/AudioPlayer.h>
#include <players/BarometerPlayer.h>
#include <players/BluetoothBeaconPlayer.h>
#include <players/GpsPlayer.h>
#include <players/ImageSensorPlayer.h>
#include <players/MotionSensorPlayer.h>
#include <players/WifiBeaconPlayer.h>
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
      BluetoothBeaconConfigRecord>;

 public:
  explicit SensorConfiguration(
      const SensorConfigurationVariant& dataVariant,
      const SensorDataType& sensorDataType);

  /** @brief Returns the type of sensor data */
  SensorDataType sensorDataType() const;

  /**
   * @brief Returns the sensor configuration as ImageConfigRecord
   * @pre type is Image
   */
  ImageConfigRecord imageConfiguration() const;

  /**
   *@brief Returns the sensor configuration as MotionConfigRecord
   * @pre type is Imu
   */
  MotionConfigRecord imuConfiguration() const;

  /**
   * @brief Returns the sensor configuration as GpsConfigRecord
   * @pre type is Gps
   */
  GpsConfigRecord gpsConfiguration() const;

  /**
   * @brief Returns the sensor configuration as WifiBeaconConfigRecord
   * @pre type is Wps
   */
  WifiBeaconConfigRecord wpsConfiguration() const;

  /**
   * @brief Returns the sensor configuration as AudioConfig
   * @pre type is Audio
   */
  AudioConfig audioConfiguration() const;

  /**
   * @brief Returns the sensor configuration as BarometerConfigRecord
   * @pre type is Barometer
   */

  BarometerConfigRecord barometerConfiguration() const;

  /**
   * @brief Returns the sensor configuration as Bluetooth
   * @pre type is Bluetooth
   */
  BluetoothBeaconConfigRecord bluetoothConfiguration() const;
  /**
   * @brief Returns the sensor configuration as MotionConfigRecord
   * @pre type is Magnetometer
   */
  MotionConfigRecord magnetometerConfiguration() const;

  /**
   * @brief Returns the nominal frame rate of the sensor
   */
  double getNominalRateHz() const;

 private:
  SensorConfigurationVariant sensorConfigurationVariant_;
  SensorDataType sensorDataType_;
};
} // namespace projectaria::tools::data_provider
