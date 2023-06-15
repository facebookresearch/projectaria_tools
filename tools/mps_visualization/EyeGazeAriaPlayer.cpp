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

#include "EyeGazeAriaPlayer.h"
#include "EyeGazeReader.h"

using namespace projectaria::tools;
using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::mps;

EyeGazeAriaPlayer::EyeGazeAriaPlayer(
    std::shared_ptr<VrsDataProvider> dataProvider,
    std::shared_ptr<EyeGazes> eyeGazes,
    std::shared_ptr<AriaVisualizationData> visData,
    std::shared_ptr<EyeGazeVisualizationData> eyeGazesVisData,
    std::shared_ptr<AriaVisualizationControl> visControl)
    : AriaPlayer(dataProvider, visData, visControl),
      eyeGazes_(eyeGazes),
      eyeGazesVisData_(eyeGazesVisData) {}

void EyeGazeAriaPlayer::playFromTimeNsMultiThread(int64_t timestampNs) {
  std::vector<std::thread> threadPool;
  for (auto streamId : visData_->playbackStreamIds_) {
    if (!dataProvider_->checkStreamIsActive(streamId)) {
      continue;
    }
    threadPool.emplace_back(
        [this, timestampNs, streamId]() { playStreamFromTimeNs(timestampNs, streamId); });
  }
  threadPool.emplace_back([this, timestampNs]() { playEyetrackingFromTimeNs(timestampNs); });
  for (auto& thread : threadPool) {
    thread.join();
  }
  // request to stop playing
  visControl_->isPlaying_ = false;
  return;
}

namespace {
// Convert a Timestamp stored in double to microseconds with std::chrono
inline constexpr auto durationDoubleToChronoUsCast(const double time_s) {
  using namespace std::chrono;
  using fsec = duration<double>;
  return round<microseconds>(fsec{time_s});
}

inline int queryEyetrackIndex(const EyeGazes& eyeGazes, std::chrono::microseconds timestampUs) {
  if (timestampUs < eyeGazes.begin()->trackingTimestamp ||
      timestampUs > eyeGazes.rbegin()->trackingTimestamp || eyeGazes.size() == 0) {
    return -1;
  }
  // Linear interpolation if timestamp is falling before two records
  const auto laterEyePtr = std::lower_bound(
      eyeGazes.begin(),
      eyeGazes.end(),
      timestampUs,
      [](const EyeGaze& gaze, const std::chrono::microseconds& queryTimestamp) {
        return gaze.trackingTimestamp < queryTimestamp;
      });
  if (laterEyePtr == eyeGazes.end()) {
    return -1;
  } else {
    return std::distance(eyeGazes.begin(), laterEyePtr);
  }
}
} // namespace

void EyeGazeAriaPlayer::updateImagesStatic(int64_t timestampNs) {
  AriaPlayer::updateImagesStatic(timestampNs);
  auto timestampUs = durationDoubleToChronoUsCast(timestampNs * 1e-9);
  int startIndex = queryEyetrackIndex(*eyeGazes_, timestampUs);
  if (startIndex == -1) {
    return;
  }
  auto eyegaze = eyeGazes_->at(startIndex);
  eyeGazesVisData_->lastYawPitch = Sophus::Vector2d(eyegaze.yaw, eyegaze.pitch);
}

void EyeGazeAriaPlayer::playEyetrackingFromTimeNs(int64_t timestampNs) {
  int64_t playbackStartTime = timestampNs;
  auto systemStartTime = std::chrono::steady_clock::now();

  auto timestampUs = durationDoubleToChronoUsCast(timestampNs * 1e-9);
  int startIndex = std::max(queryEyetrackIndex(*eyeGazes_, timestampUs), 0);
  for (int i = startIndex; i < eyeGazes_->size(); ++i) {
    if (!visControl_->isPlaying_ || visControl_->shouldClose_) {
      return;
    }

    auto eyegaze = eyeGazes_->at(i);
    eyeGazesVisData_->lastYawPitch = Sophus::Vector2d(eyegaze.yaw, eyegaze.pitch);

    uint64_t currentTimestampNs = eyegaze.trackingTimestamp.count() * 1e3;
    auto sensorTimeDiffNs = currentTimestampNs - playbackStartTime;
    auto systemDiffNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - systemStartTime)
                            .count();
    auto waitTimeNs = sensorTimeDiffNs - systemDiffNs;
    std::this_thread::sleep_for(std::chrono::nanoseconds(waitTimeNs));
  }
}

std::shared_ptr<EyeGazeAriaPlayer> createEyeGazeAriaPlayer(
    const std::string& vrsPath,
    const std::string& eyeGazeRecordsPath,
    const std::vector<vrs::StreamId>& imageStreamIds) {
  // Open the VRS File
  auto dataProvider = data_provider::createVrsDataProvider(vrsPath);

  if (!dataProvider) {
    return {};
  }

  // load aria eye gaze data
  std::shared_ptr<EyeGazes> eyeGazes = std::make_shared<EyeGazes>();
  if (eyeGazeRecordsPath != "") {
    *eyeGazes = readEyeGaze(eyeGazeRecordsPath);
  }

  auto visData = std::make_shared<AriaVisualizationData>();
  visData->initDataStreams(imageStreamIds, {}, {});

  auto eyeGazesVisData = std::make_shared<EyeGazeVisualizationData>();
  eyeGazesVisData->lastYawPitch = {0, 0};

  // load visualization control
  auto visControl = std::make_shared<AriaVisualizationControl>();
  visControl->isPlaying_ = true;
  visControl->timestampNs_ = dataProvider->getFirstTimeNsAllStreams(TimeDomain::DeviceTime);

  // load aria visualization data
  return std::make_shared<EyeGazeAriaPlayer>(
      dataProvider, eyeGazes, visData, eyeGazesVisData, visControl);
}
