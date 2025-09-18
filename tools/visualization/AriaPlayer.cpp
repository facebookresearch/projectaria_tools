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

#include "AriaPlayer.h"

#ifdef BUILD_INTERNAL_PROJECTARIA_TOOLS
#include <data_provider/internal/VrsDataProvider.h>
#endif

using namespace projectaria::tools::data_provider;
constexpr int64_t kSleepInteruptPeriodNs = 100000000; // 100 ms

AriaPlayer::AriaPlayer(
    std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider,
    std::shared_ptr<AriaVisualizationData> visData,
    std::shared_ptr<AriaVisualizationControl> visControl)
    : dataProvider_(dataProvider), visData_(visData), visControl_(visControl) {}

void AriaPlayer::run() {
  while (!visControl_->shouldClose_) {
    int64_t timestampNs = visControl_->timestampNs_;
    if (timestampNs != lastTimestampNs_ || visControl_->isPlaying_) {
      playFromTimeNsMultiThread(timestampNs);
      updateImagesStatic(visControl_->timestampNs_);
      lastTimestampNs_ = timestampNs;
    }
  }
}

void AriaPlayer::updateImagesStatic(int64_t timestampNs) {
  for (auto streamId : visData_->playbackStreamIds_) {
    if (!dataProvider_->checkStreamIsActive(streamId)) {
      continue;
    }
    if (dataProvider_->checkStreamIsType(streamId, SensorDataType::Image) ||
        dataProvider_->checkStreamIsType(streamId, SensorDataType::EyeGaze) ||
        dataProvider_->checkStreamIsType(streamId, SensorDataType::HandPose) ||
        dataProvider_->checkStreamIsType(streamId, SensorDataType::VioHighFreq) ||
        dataProvider_->checkStreamIsType(streamId, SensorDataType::Vio)) {
      int index = dataProvider_->getIndexByTimeNs(
          streamId, timestampNs, TimeDomain::DeviceTime, TimeQueryOptions::After);

      auto sensorData = dataProvider_->getSensorDataByIndex(streamId, index);
      visData_->updateData(sensorData);
    }
  }
}

void AriaPlayer::playFromTimeNsMultiThread(int64_t timestampNs) {
  std::vector<std::thread> threadPool;
  for (auto streamId : visData_->playbackStreamIds_) {
    if (!dataProvider_->checkStreamIsActive(streamId)) {
      continue;
    }
    threadPool.emplace_back(
        [this, timestampNs, streamId]() { playStreamFromTimeNs(timestampNs, streamId); });
  }
  for (auto& thread : threadPool) {
    thread.join();
  }
  // request to stop playing
  visControl_->isPlaying_ = false;
}

void AriaPlayer::playStreamFromTimeNs(int64_t timestampNs, const vrs::StreamId& streamId) {
  const int numData = dataProvider_->getNumData(streamId);
  int startIndex = dataProvider_->getIndexByTimeNs(
      streamId, timestampNs, TimeDomain::DeviceTime, TimeQueryOptions::After);

  int64_t playbackStartTime = timestampNs;
  auto systemStartTime = std::chrono::steady_clock::now();
  for (size_t i = startIndex; i < numData; ++i) {
    if (!visControl_->isPlaying_ || visControl_->shouldClose_) {
      return;
    }

    auto sensorData = dataProvider_->getSensorDataByIndex(streamId, i);
    visData_->updateData(sensorData);

    const int64_t currentTimestampNs = sensorData.getTimeNs(TimeDomain::DeviceTime);

    if (visControl_->timestampNs_ < currentTimestampNs) {
      visControl_->timestampNs_ = currentTimestampNs;
    }
    auto sensorTimeDiffNs = currentTimestampNs - playbackStartTime;
    auto systemDiffNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - systemStartTime)
                            .count();
    auto waitTimeNs =
        sensorTimeDiffNs - static_cast<int64_t>(systemDiffNs * visControl_->playbackSpeed_);

    // Sleep for waitTimeNs and check if the viewer gets interupted during the sleep period
    // This prevents the viewer from doing nothing if waitTimeNs is too long, which happens when
    // playbackSpeed is set to too high and the renderer cannot catch up
    for (int64_t timer = 0; timer < waitTimeNs - kSleepInteruptPeriodNs;
         timer += kSleepInteruptPeriodNs) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(kSleepInteruptPeriodNs));
      if (!visControl_->isPlaying_ || visControl_->shouldClose_) {
        return;
      }
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(waitTimeNs % kSleepInteruptPeriodNs));
  }
}

// A helper function to obtain stream ids by multiple RecordableTypeId, from a VRS data provider
std::vector<vrs::StreamId> getStreamIdsByMultipleType(
    const DeliverQueuedOptions& deliverOptions,
    const std::vector<vrs::RecordableTypeId>& typeIdVec) {
  const std::set<vrs::RecordableTypeId> allTypeIds = deliverOptions.getTypeIds();
  std::vector<vrs::StreamId> streamIds;

  for (const auto& typeId : typeIdVec) {
    // Check if the typeId is in the VRS file
    auto itor = allTypeIds.find(typeId);
    if (itor == allTypeIds.end()) {
      continue;
    } else {
      const std::set<vrs::StreamId> currentStreams = deliverOptions.getStreamIds(typeId);
      streamIds.insert(streamIds.end(), currentStreams.begin(), currentStreams.end());
    }
  }

  return streamIds;
}

std::shared_ptr<AriaPlayer> createAriaPlayer(const std::string& vrsPath) {
// Open the VRS File
#ifdef BUILD_INTERNAL_PROJECTARIA_TOOLS
  auto internalDataProvider = surreal::data_provider::createVrsDataProvider(vrsPath);
  auto dataProvider = std::static_pointer_cast<projectaria::tools::data_provider::VrsDataProvider>(
      internalDataProvider);
#else
  auto dataProvider = createVrsDataProvider(vrsPath);
#endif

  if (!dataProvider) {
    return nullptr;
  }

  fmt::print(stdout, "Opened '{}'.\n", vrsPath);

  // Obtain the image streams from the VRS data provider
  const DeliverQueuedOptions deliverOptions = dataProvider->getDefaultDeliverQueuedOptions();
  const auto imageStreamIds = getStreamIdsByMultipleType(
      deliverOptions,
      {vrs::RecordableTypeId::SlamCameraData,
       vrs::RecordableTypeId::RgbCameraRecordableClass,
       vrs::RecordableTypeId::EyeCameraRecordableClass});
  const auto imuStreamIds =
      getStreamIdsByMultipleType(deliverOptions, {vrs::RecordableTypeId::SlamImuData});

  const auto dataStreamIds = getStreamIdsByMultipleType(
      deliverOptions,
      {
          vrs::RecordableTypeId::StereoAudioRecordableClass,
          vrs::RecordableTypeId::BarometerRecordableClass,
          vrs::RecordableTypeId::GpsRecordableClass,
          vrs::RecordableTypeId::WifiBeaconRecordableClass,
          vrs::RecordableTypeId::MagnetometerRecordableClass,
          vrs::RecordableTypeId::SlamMagnetometerData,
      });
  const auto onDeviceMpStreams = getStreamIdsByMultipleType(
      deliverOptions,
      {
          vrs::RecordableTypeId::GazeRecordableClass,
          vrs::RecordableTypeId::PoseRecordableClass,
      });

  // Print out the stream ids
  for (const auto& streamId : imageStreamIds) {
    fmt::print("Visualizing image stream: {}\n", streamId.getNumericName());
  }
  for (const auto& streamId : imuStreamIds) {
    fmt::print("Visualizing imu stream: {}\n", streamId.getNumericName());
  }
  for (const auto& streamId : dataStreamIds) {
    fmt::print("Visualizing 1D raw sensor data stream: {}\n", streamId.getNumericName());
  }
  for (const auto& streamId : onDeviceMpStreams) {
    fmt::print("Visualizing on device MP data stream: {}\n", streamId.getNumericName());
  }

  // Set the data buffer for visualization and Variable for playback control
  auto visData = std::make_shared<AriaVisualizationData>(dataProvider);
  visData->initDataStreams(imageStreamIds, imuStreamIds, dataStreamIds, onDeviceMpStreams);
  auto visControl = std::make_shared<AriaVisualizationControl>();

  visControl->isPlaying_ = true;
  visControl->timestampNs_ = dataProvider->getFirstTimeNsAllStreams(TimeDomain::DeviceTime);
  return std::make_shared<AriaPlayer>(dataProvider, visData, visControl);
}
