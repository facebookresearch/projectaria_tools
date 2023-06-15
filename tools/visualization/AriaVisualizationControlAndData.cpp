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

#include "AriaVisualizationControlAndData.h"
#include "AriaStreamIds.h"
#include "vrs/StreamId.h"

#include <chrono>

void AriaVisualizationData::initDataStreams(
    const std::vector<vrs::StreamId>& kImageStreamIds,
    const std::vector<vrs::StreamId>& kImuStreamIds,
    const std::vector<vrs::StreamId>& kDataStreamIds) {
  imageStreamIds_ = kImageStreamIds;
  imuStreamIds_ = kImuStreamIds;
  dataStreamIds_ = kDataStreamIds;
  playbackStreamIds_.insert(
      playbackStreamIds_.end(), imageStreamIds_.begin(), imageStreamIds_.end());
  playbackStreamIds_.insert(playbackStreamIds_.end(), imuStreamIds_.begin(), imuStreamIds_.end());
  playbackStreamIds_.insert(playbackStreamIds_.end(), dataStreamIds_.begin(), dataStreamIds_.end());
}

bool AriaVisualizationData::updateData(
    const projectaria::tools::data_provider::SensorData& sensorData) {
  using namespace projectaria::tools::data_provider;
  vrs::StreamId streamId = sensorData.streamId();
  setDataChanged(true, streamId);
  SensorDataType type = sensorData.sensorDataType();
  switch (type) {
    case SensorDataType::Image: {
      cameraImageBufferMap_[streamId] = sensorData.imageDataAndRecord().first;
      break;
    }
    case SensorDataType::Imu: {
      // Read data from last timestamp to current timestamp
      const auto& accel = sensorData.imuData().accelMSec2;
      const auto& gyro = sensorData.imuData().gyroRadSec;
      accMSec2Map_[streamId] = accel;
      gyroRadSecMap_[streamId] = gyro;
      break;
    }
    case SensorDataType::Magnetometer: {
      magMicroTesla_[streamId] = sensorData.magnetometerData().magTesla;
      magMicroTesla_[streamId][0] *= 1e6;
      magMicroTesla_[streamId][1] *= 1e6;
      magMicroTesla_[streamId][2] *= 1e6;
      break;
    }
    case SensorDataType::Barometer: {
      std::vector<float> temperature, pressure;
      temperature.emplace_back(sensorData.barometerData().temperature);
      pressure.emplace_back(sensorData.barometerData().pressure);

      temperature_ = temperature;
      pressure_ = pressure;
      break;
    }
    case SensorDataType::Audio: {
      const int dataLength = sensorData.audioDataAndRecord().first.data.size();
      if (dataLength == 0) {
        break;
      }
      const int numSamples = sensorData.audioDataAndRecord().second.captureTimestampsNs.size();
      const int numChannels =
          dataLength / sensorData.audioDataAndRecord().second.captureTimestampsNs.size();

      using ConstMapInt32 =
          Eigen::Map<const Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
      ConstMapInt32 audioMatrix(
          sensorData.audioDataAndRecord().first.data.data(), numSamples, numChannels);

      // See if we need to allocate our audio memory buffer
      if (audio_.empty()) {
        audio_ = std::vector<std::vector<float>>(numSamples, std::vector<float>(numChannels, 0));
      }

      // Audio samples are 32bit; convert to float for visualization
      for (size_t i = 0; i < audioMatrix.rows(); ++i) {
        for (size_t c = 0; c < audioMatrix.cols(); ++c) {
          audio_[i][c] = (float)(audioMatrix(i, c) / (float)std::numeric_limits<int32_t>::max());
        }
      }
      break;
    }
    // case SensorDataType::Gps:
    // case SensorDataType::Wps:
    // case SensorDataType::Bluetooth:
    // case SensorDataType::NotValid:
    //   break;
    default:
      break;
  }
  return true;
}
