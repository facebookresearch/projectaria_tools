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

#include <data_layout/MotionSensorMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

namespace projectaria::tools::data_provider {

using MotionCallback =
    std::function<bool(const vrs::CurrentRecord& r, vrs::DataLayout& dataLayout, bool verbose)>;

/**
 * @brief Motion sensor configurations
 */
struct MotionConfigRecord {
  uint32_t streamIndex; ///< @brief ID of the VRS stream
  std::string deviceType; ///< @brief type of the device
  std::string deviceVersion; ///< @brief OS version on the device
  std::string deviceSerial; ///< @brief serial of the device
  uint64_t deviceId; ///< @brief ID of the IMU, 0 to N
  std::string sensorModel; ///< @brief model of the IMU sensor
  // note IMU does not have sensor serial
  double nominalRateHz; ///< @brief number of frames per second
  bool hasAccelerometer; ///< @brief if the sensor contains a accelerometer
  bool hasGyroscope; ///< @brief if the sensor contains a gyroscope
  bool hasMagnetometer; ///< @brief if the sensor contains a magnetometer
  // Optional field, not filled when use with Aria
  std::string factoryCalibration;
  std::string onlineCalibration;
  std::string description;
};

/**
 * @brief Motion data type
 */
struct MotionData {
  bool accelValid; ///< @brief if the data contains accelerometer data
  bool gyroValid; ///< @brief if the data contains gyroscope data
  bool magValid; ///< @brief if the data contains magnetometer data
  double temperature; ///< @brief temperature in celsius degrees
  int64_t captureTimestampNs; ///< @brief capture time in device time domain
  int64_t arrivalTimestampNs; ///< @brief arrival time in host time domain
  std::vector<float> accelMSec2; ///< @brief accelerometer data in m/sec2
  std::vector<float> gyroRadSec; ///< @brief gyroscope data in rad/sec2
  std::vector<float> magTesla; ///<@brief magnetometer data in Tesla
};

class MotionSensorPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit MotionSensorPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  MotionSensorPlayer(const MotionSensorPlayer&) = delete;
  MotionSensorPlayer& operator=(const MotionSensorPlayer&) = delete;
  MotionSensorPlayer& operator=(MotionSensorPlayer&) = delete;
  MotionSensorPlayer(MotionSensorPlayer&&) = default;

  void setCallback(MotionCallback callback) {
    callback_ = callback;
  }

  const MotionConfigRecord& getConfigRecord() const {
    return configRecord_;
  }

  const MotionData& getDataRecord() const {
    return dataRecord_;
  }

  const vrs::StreamId& getStreamId() const {
    return streamId_;
  }

  double getNextTimestampSec() const {
    return nextTimestampSec_;
  }

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

 private:
  bool onDataLayoutRead(const vrs::CurrentRecord& r, size_t blockIndex, vrs::DataLayout& dl)
      override;

  const vrs::StreamId streamId_;
  MotionCallback callback_ = [](const vrs::CurrentRecord&, vrs::DataLayout&, bool) { return true; };

  MotionConfigRecord configRecord_;
  MotionData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};

} // namespace projectaria::tools::data_provider
