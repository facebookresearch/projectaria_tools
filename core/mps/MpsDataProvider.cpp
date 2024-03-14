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

#include <mps/MpsDataProvider.h>

#include <data_provider/QueryMapByTimestamp.h>
#include <mps/EyeGazeReader.h>
#include <mps/GlobalPointCloudReader.h>
#include <mps/HandTrackingReader.h>
#include <mps/OnlineCalibrationsReader.h>
#include <mps/PointObservationReader.h>
#include <mps/TrajectoryReaders.h>

#define DEFAULT_LOG_CHANNEL "MpsDataProvider"
#include <logging/Log.h>

constexpr int64_t kUsToNs{1000};

namespace projectaria::tools::mps {

MpsDataProvider::MpsDataProvider(const MpsDataPaths& mpsDataPaths) : dataPaths_(mpsDataPaths) {}

bool MpsDataProvider::hasGeneralEyeGaze() const {
  return !dataPaths_.eyegaze.generalEyegaze.empty();
}

bool MpsDataProvider::hasPersonalizedEyeGaze() const {
  return !dataPaths_.eyegaze.personalizedEyegaze.empty();
}

bool MpsDataProvider::hasOpenLoopPoses() const {
  return !dataPaths_.slam.openLoopTrajectory.empty();
}

bool MpsDataProvider::hasClosedLoopPoses() const {
  return !dataPaths_.slam.closedLoopTrajectory.empty();
}

bool MpsDataProvider::hasOnlineCalibrations() const {
  return !dataPaths_.slam.onlineCalibration.empty();
}

bool MpsDataProvider::hasSemidensePointCloud() const {
  return !dataPaths_.slam.semidensePoints.empty();
}

bool MpsDataProvider::hasSemidenseObservations() const {
  return !dataPaths_.slam.semidenseObservations.empty();
}

bool MpsDataProvider::hasWristAndPalmPoses() const {
  return !dataPaths_.handTracking.wristAndPalmPoses.empty();
}

std::optional<EyeGaze> MpsDataProvider::getGeneralEyeGaze(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) {
  if (!hasGeneralEyeGaze()) {
    std::string error = "Cannot query for general eye gaze since the data is not available";
    XR_LOGE("{}", error);
    throw std::runtime_error{error};
  }
  if (generalEyeGazes_.empty()) {
    auto eyeGazesVec = readEyeGaze(dataPaths_.eyegaze.generalEyegaze);
    for (const auto& eyeGaze : eyeGazesVec) {
      generalEyeGazes_.emplace(eyeGaze.trackingTimestamp.count() * kUsToNs, eyeGaze);
    }
  }

  auto iter = data_provider::queryMapByTimestamp<EyeGaze>(
      generalEyeGazes_, deviceTimeStampNs, timeQueryOptions);
  return iter == generalEyeGazes_.end() ? std::optional<EyeGaze>() : iter->second;
}

std::optional<WristAndPalmPose> MpsDataProvider::getWristAndPalmPose(
    int64_t captureTimestampNs,
    const TimeQueryOptions& timeQueryOptions) {
  if (!hasWristAndPalmPoses()) {
    std::string error = "Cannot query for wrist and palm pose since the data is not available";
    XR_LOGE("{}", error);
    throw std::runtime_error{error};
  }
  if (wristAndPalmPoses_.empty()) {
    auto wristAndPalmPoses = readWristAndPalmPoses(dataPaths_.handTracking.wristAndPalmPoses);
    for (const auto& wristAndPalmPose : wristAndPalmPoses) {
      wristAndPalmPoses_.emplace(
          wristAndPalmPose.trackingTimestamp.count() * kUsToNs, wristAndPalmPose);
    }
  }
  auto iter = data_provider::queryMapByTimestamp<WristAndPalmPose>(
      wristAndPalmPoses_, captureTimestampNs, timeQueryOptions);
  return iter == wristAndPalmPoses_.end() ? std::optional<WristAndPalmPose>() : iter->second;
}

std::optional<EyeGaze> MpsDataProvider::getPersonalizedEyeGaze(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) {
  if (!hasPersonalizedEyeGaze()) {
    std::string error = "Cannot query for personalized eye gaze since the data is not available";
    XR_LOGE("{}", error);
    throw std::runtime_error{error};
  }
  if (personalizedEyeGazes_.empty()) {
    auto eyeGazesVec = readEyeGaze(dataPaths_.eyegaze.personalizedEyegaze);
    for (const auto& eyeGaze : eyeGazesVec) {
      personalizedEyeGazes_.emplace(eyeGaze.trackingTimestamp.count() * kUsToNs, eyeGaze);
    }
  }

  auto iter = data_provider::queryMapByTimestamp<EyeGaze>(
      personalizedEyeGazes_, deviceTimeStampNs, timeQueryOptions);
  return iter == personalizedEyeGazes_.end() ? std::optional<EyeGaze>() : iter->second;
}

std::optional<OpenLoopTrajectoryPose> MpsDataProvider::getOpenLoopPose(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) {
  if (!hasOpenLoopPoses()) {
    std::string error = "Cannot query for open loop pose since the data is not available";
    XR_LOGE("{}", error);
    throw std::runtime_error{error};
  }
  if (openLoopPoses_.empty()) {
    OpenLoopTrajectory trajectory = readOpenLoopTrajectory(dataPaths_.slam.openLoopTrajectory);
    for (const auto& pose : trajectory) {
      openLoopPoses_.emplace(pose.trackingTimestamp.count() * kUsToNs, pose);
    }
  }

  auto iter = data_provider::queryMapByTimestamp<OpenLoopTrajectoryPose>(
      openLoopPoses_, deviceTimeStampNs, timeQueryOptions);
  return iter == openLoopPoses_.end() ? std::optional<OpenLoopTrajectoryPose>() : iter->second;
}

std::optional<ClosedLoopTrajectoryPose> MpsDataProvider::getClosedLoopPose(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) {
  if (!hasClosedLoopPoses()) {
    std::string error = "Cannot query for closed loop pose since the data is not available";
    XR_LOGE("{}", error);
    throw std::runtime_error{error};
  }
  if (closedLoopPoses_.empty()) {
    ClosedLoopTrajectory trajectory =
        readClosedLoopTrajectory(dataPaths_.slam.closedLoopTrajectory);
    for (const auto& pose : trajectory) {
      closedLoopPoses_.emplace(pose.trackingTimestamp.count() * kUsToNs, pose);
    }
  }

  auto iter = data_provider::queryMapByTimestamp<ClosedLoopTrajectoryPose>(
      closedLoopPoses_, deviceTimeStampNs, timeQueryOptions);
  return iter == closedLoopPoses_.end() ? std::optional<ClosedLoopTrajectoryPose>() : iter->second;
}

std::optional<OnlineCalibration> MpsDataProvider::getOnlineCalibration(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) {
  if (!hasOnlineCalibrations()) {
    std::string error = "Cannot query for online calibration since the data is not available";
    XR_LOGE("{}", error);
    throw std::runtime_error{error};
  }
  if (onlineCalibrations_.empty()) {
    OnlineCalibrations calibrations = readOnlineCalibration(dataPaths_.slam.onlineCalibration);
    for (const auto& calibration : calibrations) {
      onlineCalibrations_.emplace(calibration.trackingTimestamp.count() * kUsToNs, calibration);
    }
  }

  auto iter = data_provider::queryMapByTimestamp<OnlineCalibration>(
      onlineCalibrations_, deviceTimeStampNs, timeQueryOptions);
  return iter == onlineCalibrations_.end() ? std::optional<OnlineCalibration>() : iter->second;
}

const GlobalPointCloud& MpsDataProvider::getSemidensePointCloud() {
  if (!hasSemidensePointCloud()) {
    std::string error = "Cannot retrieve Semidense pointcloud since the data is not available";
    XR_LOGE("{}", error);
    throw std::runtime_error{error};
  }
  if (globalPointCloud_.empty()) {
    globalPointCloud_ = readGlobalPointCloud(dataPaths_.slam.semidensePoints);
  }

  return globalPointCloud_;
}

const PointObservations& MpsDataProvider::getSemidenseObservations() {
  if (!hasSemidenseObservations()) {
    std::string error = "Cannot retrieve Semidense observations since the data is not available";
    XR_LOGE("{}", error);
    throw std::runtime_error{error};
  }
  if (pointObservations_.empty()) {
    pointObservations_ = readPointObservations(dataPaths_.slam.semidenseObservations);
  }

  return pointObservations_;
}

} // namespace projectaria::tools::mps
