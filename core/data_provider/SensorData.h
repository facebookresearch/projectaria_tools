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

#include <data_provider/data_types/FrontendOutput.h>
#include <data_provider/players/AlsPlayer.h>
#include <data_provider/players/AudioPlayer.h>
#include <data_provider/players/BarometerPlayer.h>
#include <data_provider/players/BatteryStatusPlayer.h>
#include <data_provider/players/BluetoothBeaconPlayer.h>
#include <data_provider/players/GpsPlayer.h>
#include <data_provider/players/ImageSensorPlayer.h>
#include <data_provider/players/MotionSensorPlayer.h>
#include <data_provider/players/PpgPlayer.h>
#include <data_provider/players/TemperaturePlayer.h>
#include <data_provider/players/TimeSyncPlayer.h>
#include <data_provider/players/VioPlayer.h>
#include <data_provider/players/WifiBeaconPlayer.h>
#include <mps/EyeGaze.h>
#include <mps/HandTracking.h>
#include <mps/Trajectory.h>

#include <vrs/StreamId.h>

#include <data_provider/SensorDataType.h>

namespace projectaria::tools::data_provider {

using ImageDataAndRecord = std::pair<ImageData, ImageDataRecord>;
using AudioDataAndRecord = std::pair<AudioData, AudioDataRecord>;

// Choose which data type to use for on-device MP data.
// OnDeviceVio data is FrontendOutput
using OnDeviceVioHighFreqData = mps::OpenLoopTrajectoryPose;
using OnDeviceEyeGazeData = mps::EyeGaze;
using OnDeviceHandPoseData = mps::HandTrackingResult;

/**
 *@brief Class holding multi - modal sensor and on-device MP data
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
      BatteryStatusData,
      BluetoothBeaconData,
      PpgData,
      AlsData,
      TemperatureData,
      /* on device MP data types*/
      FrontendOutput,
      OnDeviceVioHighFreqData,
      OnDeviceEyeGazeData,
      OnDeviceHandPoseData>;

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
      int64_t recordInfoTimeNs,
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

  /**
   * @brief Returns the sensor data as PpgData
   * @pre type is PPG
   */
  [[nodiscard]] PpgData ppgData() const;

  /**
   * @brief Returns the sensor data as AlsData
   * @pre type is ALS
   */
  [[nodiscard]] AlsData alsData() const;

  /**
   * @brief Returns the sensor data as TemperatureData
   * @pre type is Temperature
   */
  [[nodiscard]] TemperatureData temperatureData() const;

  /**
   * @brief Returns the sensor data as BatteryStatusData
   * @pre type is BatteryStatus
   */
  [[nodiscard]] BatteryStatusData batteryStatusData() const;

  /**
   * @brief Returns the sensor data as Vio data
   * @pre type is VioHighFreq
   */
  [[nodiscard]] FrontendOutput vioData() const;

  /**
   * @brief Returns the sensor data as VioHighFreq data
   * @pre type is VioHighFreq
   */
  [[nodiscard]] OnDeviceVioHighFreqData vioHighFreqData() const;

  /**
   * @brief Returns the sensor data as EyeGaze data
   * @pre type is EyeGaze
   */
  [[nodiscard]] OnDeviceEyeGazeData eyeGazeData() const;

  /**
   * @brief Returns the sensor data as HandPose data
   * @pre type is HandPose
   */
  [[nodiscard]] OnDeviceHandPoseData handPoseData() const;

  /** @brief Returns timestamp in a specified time domains */
  int64_t getTimeNs(TimeDomain timeDomain) const;

 private:
  vrs::StreamId streamId_;

 protected:
  SensorDataVariant dataVariant_;

 private:
  SensorDataType sensorDataType_;
  int64_t recordInfoTimeNs_;
  std::map<TimeSyncMode, int64_t> timeSyncTimeNs_;
  friend class VrsDataProvider;

  // get timestamp in device or host time domain
  int64_t getDeviceTime() const;
  int64_t getHostTime() const;
};
} // namespace projectaria::tools::data_provider
