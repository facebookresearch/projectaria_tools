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

#include <array>
#include <string>

#include <data_provider/TimeTypes.h>
#include <vrs/StreamId.h>

namespace projectaria::tools::data_provider {

/**
 * @brief Enum class for different types of sensor data used in projectaria_tools
 */
enum class SensorDataType {
  NotValid, /**< Unsupported stream type, note TimeCode is considered unsupported as it is not
               sensor data */
  Image, /**< camera image streams */
  Imu, /**< Inertial measurement unit (IMU) data streams, including accelerometer and gyroscope,
          note that magnetometer is a different stream */
  Gps, /**< Global positioning system (GPS) data streams*/
  Wps, /**< Wifi beacon data streams*/
  Audio, /**< Audio data streams*/
  Barometer, /**< Barometer data streams*/
  Bluetooth, /**< Bluetooth data streams*/
  Magnetometer, /**< Magnetometer data streams*/
  Ppg, /** Ppg data streams */
  Als, /** ALS (Ambient Light Sensor) data streams */
  Temperature, /** Temperature data streams */
  BatteryStatus, /** Battery status data streams */
  Vio, /**< On Device VIO data streams*/
  VioHighFreq, /**< On Device VIO high frequency data streams*/
  EyeGaze, /**< On Device Eye gaze data streams*/
  HandPose, /**< On Device Hand tracking data streams*/
};

/** @brief converts the enum to readable std::string */
std::string getName(SensorDataType type);

/**
 * @brief checks if host time domain is supported by a type.
 * Note we encourage user to avoid using host time domains as arrival timestamps are inaccurate.
 */
bool supportsHostTimeDomain(SensorDataType type);

/**
 * @brief checks if calibration exists for a specific stream
 */
bool hasCalibration(SensorDataType type);
} // namespace projectaria::tools::data_provider
