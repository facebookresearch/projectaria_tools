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
#include <players/TimeSyncPlayer.h>
#include <players/WifiBeaconPlayer.h>

#include <vrs/StreamId.h>

#include <data_provider/SensorDataType.h>

namespace projectaria::tools::data_provider {

using ImageDataAndRecord = std::pair<ImageData, ImageDataRecord>;
using AudioDataAndRecord = std::pair<AudioData, AudioDataRecord>;
/**
 *@brief Class holding multi - modal sensor data
 */
class SensorData {
 public:
  using SensorDataVariant = std::variant<
      std::monostate,
      ImageDataAndRecord,
      MotionData,
      GpsData,
      WifiBeaconData,
      AudioDataAndRecord,
      BarometerData,
      BluetoothBeaconData>;

 public:
  /**
   * @brief Constructs a sensor data.
   * @param streamId ID of the VRS Stream the data belongs to
   * @param dataVariant the sensor data itself
   * @param sensorDataType type of the sensor data
   * @param recordInfoTimeNs the timestamp of the data in the Record domain
   * @param timeSyncTimeNs the timestamp of the data in the different TimeSyncMode that includes
   * TimeCode domain and TicSyncDomain
   */
  SensorData(
      const vrs::StreamId& streamId,
      const SensorDataVariant& dataVariant,
      const SensorDataType& sensorDataType,
      const int64_t recordInfoTimeNs,
      const std::map<TimeSyncMode, int64_t>& timeSyncTimeNs);

  /** @brief Returns the ID of the VRS Stream the data belongs to */
  vrs::StreamId streamId() const;

  /**
   * @brief Returns the type of the sensor data
   */

  SensorDataType sensorDataType() const;

  /**
   * @brief Returns the sensor data as ImageDataAndRecord
   * @pre type is Image
   */
  ImageDataAndRecord imageDataAndRecord() const;

  /**
   * @brief Returns the sensor data as MotionData
   * @pre type is IMU
   */

  MotionData imuData() const;

  /**
   * @brief Returns the sensor data as GpsData
   * @pre type is GPS
   */

  GpsData gpsData() const;

  /**
   * @brief Returns the sensor data as WifiBeaconData
   * @pre type is WPS
   */
  WifiBeaconData wpsData() const;

  /**
   * @brief Returns the sensor data as AudioDataAndRecord
   * @pre type is Audio
   */
  AudioDataAndRecord audioDataAndRecord() const;

  /**
   * @brief Returns the sensor data as BarometerData
   * @pre type is Barometer
   */
  BarometerData barometerData() const;
  /**
   * @brief Returns the sensor data as BluetoothBeaconData
   * @pre type is Bluetooth
   */
  BluetoothBeaconData bluetoothData() const;

  /**
   * @brief Returns the sensor data as MotionData
   * @pre type is Magnetometer
   */
  MotionData magnetometerData() const;

  /** @brief Returns timestamp in a specified time domains */
  int64_t getTimeNs(TimeDomain timeDomain) const;

 private:
  vrs::StreamId streamId_;
  SensorDataVariant dataVariant_;
  SensorDataType sensorDataType_;
  int64_t recordInfoTimeNs_;
  std::map<TimeSyncMode, int64_t> timeSyncTimeNs_;

  // get timestamp in device or host time domain
  int64_t getDeviceTime() const;
  int64_t getHostTime() const;
};
} // namespace projectaria::tools::data_provider
