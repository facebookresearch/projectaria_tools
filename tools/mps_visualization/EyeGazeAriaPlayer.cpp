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
    std::shared_ptr<projectaria::tools::mps::EyeGazes> generalizedEyeGazes,
    std::shared_ptr<EyeGazeVisualizationData> generalizedEyeGazesVisData,
    std::shared_ptr<projectaria::tools::mps::EyeGazes> calibratedEyeGazes,
    std::shared_ptr<EyeGazeVisualizationData> calibratedEyeGazesVisData,
    std::shared_ptr<AriaVisualizationData> visData,
    std::shared_ptr<AriaVisualizationControl> visControl)
    : AriaPlayer(dataProvider, visData, visControl),
      generalizedEyeGazes_(generalizedEyeGazes),
      generalizedEyeGazesVisData_(generalizedEyeGazesVisData),
      calibratedEyeGazes_(calibratedEyeGazes),
      calibratedEyeGazesVisData_(calibratedEyeGazesVisData) {}

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
  int startIndex = queryEyetrackIndex(*generalizedEyeGazes_, timestampUs);
  if (startIndex != -1) {
    auto eyegaze = generalizedEyeGazes_->at(startIndex);
    generalizedEyeGazesVisData_->lastYawPitch = Sophus::Vector2d(eyegaze.yaw, eyegaze.pitch);
  }
  startIndex = queryEyetrackIndex(*calibratedEyeGazes_, timestampUs);
  if (startIndex != -1 && calibratedEyeGazes_) {
    auto eyegaze = calibratedEyeGazes_->at(startIndex);
    calibratedEyeGazesVisData_->lastYawPitch = Sophus::Vector2d(eyegaze.yaw, eyegaze.pitch);
  }
}

void EyeGazeAriaPlayer::playEyetrackingFromTimeNs(int64_t timestampNs) {
  int64_t playbackStartTime = timestampNs;
  auto systemStartTime = std::chrono::steady_clock::now();

  auto timestampUs = durationDoubleToChronoUsCast(timestampNs * 1e-9);
  int startIndex = std::max(queryEyetrackIndex(*generalizedEyeGazes_, timestampUs), 0);
  for (int i = startIndex; i < generalizedEyeGazes_->size(); ++i) {
    if (!visControl_->isPlaying_ || visControl_->shouldClose_) {
      return;
    }

    auto generalizedEyeGaze = generalizedEyeGazes_->at(i);
    generalizedEyeGazesVisData_->lastYawPitch =
        Sophus::Vector2d(generalizedEyeGaze.yaw, generalizedEyeGaze.pitch);

    if (calibratedEyeGazes_ && !calibratedEyeGazes_->empty()) {
      auto calibratedEyeGaze = calibratedEyeGazes_->at(i);
      calibratedEyeGazesVisData_->lastYawPitch =
          Sophus::Vector2d(calibratedEyeGaze.yaw, calibratedEyeGaze.pitch);
    }

    uint64_t currentTimestampNs = generalizedEyeGaze.trackingTimestamp.count() * 1e3;
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
    const std::string& generalizedGazePath,
    const std::string& calibratedGazePath,
    const std::vector<vrs::StreamId>& imageStreamIds) {
  // Open the VRS File
  auto dataProvider = data_provider::createVrsDataProvider(vrsPath);

  if (!dataProvider) {
    return {};
  }

  // load aria eye gaze data
  std::shared_ptr<EyeGazes> generalizedEyeGazes;
  std::shared_ptr<EyeGazeVisualizationData> generalizedEyeGazesVisData;
  if (generalizedGazePath != "") {
    generalizedEyeGazes = std::make_shared<EyeGazes>();
    *generalizedEyeGazes = readEyeGaze(generalizedGazePath);
    generalizedEyeGazesVisData = std::make_shared<EyeGazeVisualizationData>();
    generalizedEyeGazesVisData->lastYawPitch = {0, 0};
    generalizedEyeGazesVisData->calibrated = false;
  }
  std::shared_ptr<EyeGazes> calibratedEyeGazes;
  std::shared_ptr<EyeGazeVisualizationData> calibratedEyeGazesVisData;
  if (calibratedGazePath != "") {
    calibratedEyeGazes = std::make_shared<EyeGazes>();
    *calibratedEyeGazes = readEyeGaze(calibratedGazePath);
    calibratedEyeGazesVisData = std::make_shared<EyeGazeVisualizationData>();
    calibratedEyeGazesVisData->lastYawPitch = {0, 0};
    calibratedEyeGazesVisData->calibrated = true;
  }

  auto visData = std::make_shared<AriaVisualizationData>();
  visData->initDataStreams(imageStreamIds, {}, {});

  // load visualization control
  auto visControl = std::make_shared<AriaVisualizationControl>();
  visControl->isPlaying_ = true;
  visControl->timestampNs_ = dataProvider->getFirstTimeNsAllStreams(TimeDomain::DeviceTime);

  // load aria visualization data
  return std::make_shared<EyeGazeAriaPlayer>(
      dataProvider,
      generalizedEyeGazes,
      generalizedEyeGazesVisData,
      calibratedEyeGazes,
      calibratedEyeGazesVisData,
      visData,
      visControl);
}
