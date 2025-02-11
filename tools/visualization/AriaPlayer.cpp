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
    if (dataProvider_->checkStreamIsType(streamId, SensorDataType::Image)) {
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
  return;
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

std::shared_ptr<AriaPlayer> createAriaPlayer(
    const std::string& vrsPath,
    const std::vector<vrs::StreamId>& imageStreamIds,
    const std::vector<vrs::StreamId>& imuStreamIds,
    const std::vector<vrs::StreamId>& dataStreamIds) {
  // Open the VRS File
  auto dataProvider = createVrsDataProvider(vrsPath);

  if (!dataProvider) {
    return nullptr;
  }

  fmt::print(stdout, "Opened '{}'.\n", vrsPath);

  // Set the data buffer for visualization and Variable for playback control
  auto visData = std::make_shared<AriaVisualizationData>(dataProvider);
  visData->initDataStreams(imageStreamIds, imuStreamIds, dataStreamIds);
  auto visControl = std::make_shared<AriaVisualizationControl>();

  visControl->isPlaying_ = true;
  visControl->timestampNs_ = dataProvider->getFirstTimeNsAllStreams(TimeDomain::DeviceTime);
  return std::make_shared<AriaPlayer>(dataProvider, visData, visControl);
}
