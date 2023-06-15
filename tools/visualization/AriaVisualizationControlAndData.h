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

#include <data_provider/VrsDataProvider.h>
#include <vrs/StreamId.h>

#include <Eigen/Core>

#include <map>
#include <string>
#include <vector>

#include <atomic>
#include <mutex>

struct AriaVisualizationControl {
  std::atomic<bool> isPlaying_ = false;
  std::atomic<bool> shouldClose_ = false;
  std::atomic<int64_t> timestampNs_ = -1;
};

struct AriaVisualizationData {
  AriaVisualizationData() = default;
  virtual ~AriaVisualizationData() = default;

  void setDataChanged(bool dataChanged, const vrs::StreamId& streamId) {
    dataChangedMap_[streamId] = dataChanged;
  }

  bool isDataChanged(const vrs::StreamId& streamId) const {
    if (dataChangedMap_.count(streamId) == 0) {
      return false;
    }
    return dataChangedMap_.at(streamId);
  }

  // Initialize with data stream Ids the Viewer will be able to capture and display
  void initDataStreams(
      const std::vector<vrs::StreamId>& kImageStreamIds,
      const std::vector<vrs::StreamId>& kImuStreamIds = {},
      const std::vector<vrs::StreamId>& kDataStreamsIds = {});

  // read data until currentTimestampNs
  bool updateData(const projectaria::tools::data_provider::SensorData& sensorData);

  // Store boolean information to know if a given data chunk has been updated
  std::map<vrs::StreamId, bool> dataChangedMap_;

  //
  // Data chunk storage
  // - current images data chunks
  std::map<vrs::StreamId, projectaria::tools::data_provider::ImageData> cameraImageBufferMap_;
  // - current Array data accMSec2, gyroRadSec, magnetometer chunks
  std::map<vrs::StreamId, std::vector<float>> accMSec2Map_, gyroRadSecMap_, magMicroTesla_;

  // - current audio chunks
  std::vector<std::vector<float>> audio_;

  // - current barometer chunks
  std::vector<float> temperature_, pressure_;

  // Aria VRS data provider
  projectaria::tools::data_provider::VrsDataProvider* dataProvider_;
  // VRS stream data handled by this interface
  std::vector<vrs::StreamId> imageStreamIds_, imuStreamIds_, dataStreamIds_, playbackStreamIds_;

  std::mutex viewerMutex_;
};
