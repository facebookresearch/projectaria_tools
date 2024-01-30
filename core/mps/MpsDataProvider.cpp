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

#include <mps/EyeGazeReader.h>
#include <mps/GlobalPointCloudReader.h>
#include <mps/OnlineCalibrationsReader.h>
#include <mps/PointObservationReader.h>
#include <mps/TrajectoryReaders.h>

#define DEFAULT_LOG_CHANNEL "MpsDataProvider"
#include <logging/Log.h>

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

bool MpsDataProvider::loadGeneralEyeGazesIfAvailable() {
  // check if already loaded
  if (!generalEyeGazes_.empty()) {
    return true;
  }

  // check if file exists
  if (!hasGeneralEyeGaze()) {
    return false;
  }

  // load
  auto eyeGazesVec = readEyeGaze(dataPaths_.eyegaze.generalEyegaze);
  for (const auto& eyeGaze : eyeGazesVec) {
    generalEyeGazes_.emplace(eyeGaze.trackingTimestamp.count() * 1000, eyeGaze);
  }

  return true;
}

bool MpsDataProvider::loadPersonalizedEyeGazesIfAvailable() {
  // check if already loaded
  if (!personalizedEyeGazes_.empty()) {
    return true;
  }

  // check if file exists
  if (!hasPersonalizedEyeGaze()) {
    return false;
  }

  // load
  auto eyeGazesVec = readEyeGaze(dataPaths_.eyegaze.personalizedEyegaze);
  for (const auto& eyeGaze : eyeGazesVec) {
    personalizedEyeGazes_.emplace(eyeGaze.trackingTimestamp.count() * 1000, eyeGaze);
  }
  return true;
}

bool MpsDataProvider::loadOpenLoopPosesIfAvailable() {
  // check if already loaded
  if (!openLoopPoses_.empty()) {
    return true;
  }

  // check if file exists
  if (!hasOpenLoopPoses()) {
    return false;
  }

  // load
  OpenLoopTrajectory trajectory = readOpenLoopTrajectory(dataPaths_.slam.openLoopTrajectory);
  for (const auto& pose : trajectory) {
    openLoopPoses_.emplace(pose.trackingTimestamp.count() * 1000, pose);
  }
  return true;
}

bool MpsDataProvider::loadClosedLoopPosesIfAvailable() {
  // check if already loaded
  if (!closedLoopPoses_.empty()) {
    return true;
  }

  // check if file exists
  if (!hasClosedLoopPoses()) {
    return false;
  }

  // load
  ClosedLoopTrajectory trajectory = readClosedLoopTrajectory(dataPaths_.slam.closedLoopTrajectory);
  for (const auto& pose : trajectory) {
    closedLoopPoses_.emplace(pose.trackingTimestamp.count() * 1000, pose);
  }
  return true;
}

bool MpsDataProvider::loadOnlineCalibrationsIfAvailable() {
  // check if already loaded
  if (!onlineCalibrations_.empty()) {
    return true;
  }

  // check if file exists
  if (!hasOnlineCalibrations()) {
    return false;
  }

  // load
  OnlineCalibrations calibrations = readOnlineCalibration(dataPaths_.slam.onlineCalibration);
  for (const auto& calibration : calibrations) {
    onlineCalibrations_.emplace(calibration.trackingTimestamp.count() * 1000, calibration);
  }
  return true;
}

bool MpsDataProvider::loadGlobalPointCloudIfAvailable() {
  // check if already loaded
  if (!globalPointCloud_.empty()) {
    return true;
  }

  // check if file exists
  if (!hasSemidensePointCloud()) {
    return false;
  }

  // load
  globalPointCloud_ = readGlobalPointCloud(dataPaths_.slam.semidensePoints);
  return true;
}

bool MpsDataProvider::loadPointObservationsIfAvailable() {
  // check if already loaded
  if (!pointObservations_.empty()) {
    return true;
  }

  // check if file exists
  if (!hasSemidenseObservations()) {
    return false;
  }

  // load
  pointObservations_ = readPointObservations(dataPaths_.slam.semidenseObservations);
  return true;
}

} // namespace projectaria::tools::mps
