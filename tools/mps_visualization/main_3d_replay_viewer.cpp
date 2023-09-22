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

#include <chrono>
#include <optional>

#include <data_provider/VrsDataProvider.h>
#include "EyeGazeReader.h"
#include "GlobalPointCloudReader.h"
#include "PointObservationReader.h"
#include "StaticCameraCalibrationReader.h"
#include "TrajectoryReaders.h"

#include "Boundary.h"
#include "CachedDataProviders.h"
#include "Data3DGui.h"

#include <CLI/CLI.hpp>

#define DEFAULT_LOG_CHANNEL "MPS3DReplayViewer"
#include <logging/Checks.h>
#include <logging/Log.h>

using namespace projectaria::tools;
using namespace projectaria::tools::mps;

int main(int argc, char* argv[]) {
  // Setup and parse input args
  CLI::App app{"MPS 3D replay visualization"};
  std::string vrsPath;
  app.add_option("--vrs", vrsPath, "Path to Aria .vrs file")->required();
  std::string replayTrajPath;
  app.add_option(
         "--replay-trajectory",
         replayTrajPath,
         "Path to replay 1k Hz trajectory (match with the vrs file) .csv file")
      ->required();
  std::vector<std::string> closedLoopTrajPaths;
  app.add_option(
      "--closed-loop-traj",
      closedLoopTrajPaths,
      "Path(s) to all global frame 1k Hz trajectories .csv file(s)");
  std::vector<std::string> staticCamerasPaths;
  app.add_option(
      "--static-camera",
      staticCamerasPaths,
      "Path(s) to static cameras (e.g.: GoPro) calibration .csv file(s)");
  std::vector<std::string> globalPointCloudPaths;
  app.add_option(
      "--global-point-cloud",
      globalPointCloudPaths,
      "Path(s) to semi-dense points (in global frame) .csv.gz file(s)");
  std::string pointObservationPath;
  app.add_option("--point-obs", pointObservationPath, "Path to point observation .csv.gz file");
  std::string eyeGazePath;
  app.add_option("--eye-gaze", eyeGazePath, "Path to eye gaze .csv file");
  CLI11_PARSE(app, argc, argv);

  if (closedLoopTrajPaths.empty()) {
    XR_LOGW("Closed loop trajectory file(s) is not provided");
  }
  if (staticCamerasPaths.empty()) {
    XR_LOGW("Static camera calibration file(s) is not provided");
  }
  if (globalPointCloudPaths.empty()) {
    XR_LOGW("Global point cloud file(s) is not provided");
    XR_CHECK(
        pointObservationPath.empty(),
        "Point observation file is provided without corresponding global point cloud file");
  }
  if (pointObservationPath.empty()) {
    XR_LOGW("Point observation file is not provided");
  }
  if (eyeGazePath.empty()) {
    XR_LOGW("Eye gaze file is not provided");
  }

  // Create vrs provider
  auto vrsProvider = *data_provider::createVrsDataProvider(vrsPath);

  // Get streamIds of the image sequences from the vrs provider
  const auto rgb_streamId = vrsProvider.getStreamIdFromLabel("camera-rgb");
  const auto slam_left_streamId = vrsProvider.getStreamIdFromLabel("camera-slam-left");
  const auto slam_right_streamId = vrsProvider.getStreamIdFromLabel("camera-slam-right");
  XR_CHECK(rgb_streamId);
  XR_CHECK(slam_left_streamId);
  XR_CHECK(slam_right_streamId);

  // Get number of frames from the vrs provider
  size_t numRgbData = vrsProvider.getNumData(rgb_streamId.value());
  size_t numSlamData = vrsProvider.getNumData(slam_left_streamId.value());
  XR_CHECK(numSlamData == vrsProvider.getNumData(slam_right_streamId.value()));
  XR_CHECK(numRgbData <= numSlamData);
  XR_LOGI("Loaded RGB camera frames: {}", numRgbData);
  XR_LOGI("Loaded SLAM left/right camera frames: {}", numSlamData);

  // Used to check if everything is in the same world frame
  std::set<std::string> allWorldFrameUids;

  // Read and cache replay trajectory segment within the time window of the input vrs
  const auto replayTraj = readClosedLoopTrajectory(replayTrajPath);
  const int64_t startTimestampNs =
      vrsProvider.getImageDataByIndex(slam_left_streamId.value(), 0).second.captureTimestampNs;
  const int64_t endTimestampNs =
      vrsProvider.getImageDataByIndex(slam_left_streamId.value(), numSlamData - 1)
          .second.captureTimestampNs;
  TrajectoryProvider trajProvider(replayTraj, startTimestampNs, endTimestampNs);
  XR_LOGI("Loaded and cached replay trajectory segment with poses: {}", trajProvider.numPoses());

  // Read and cache all trajectories
  std::vector<std::vector<Eigen::Vector3f>> fullTrajs;
  for (const auto& path : closedLoopTrajPaths) {
    const auto traj = readClosedLoopTrajectory(path);
    std::vector<Eigen::Vector3f> trajPoints;
    trajPoints.reserve(traj.size());
    for (const auto& pose : traj) {
      trajPoints.push_back(pose.T_world_device.translation().cast<float>());
      allWorldFrameUids.insert(pose.graphUid);
    }
    fullTrajs.push_back(std::move(trajPoints));
  }
  XR_LOGI("Loaded and cached full trajectories: {}", fullTrajs.size());

  // Get RGB camera's factory calibration from vis
  auto sensorCalib = vrsProvider.getSensorCalibration(rgb_streamId.value());
  XR_CHECK(sensorCalib);
  auto camCalibs = {sensorCalib.value().cameraCalibration()};

  // Get gopro's calibration
  StaticCameraCalibrations staticCams;
  for (const auto& path : staticCamerasPaths) {
    const auto cams = readStaticCameraCalibrations(path);
    for (const auto& cam : cams) {
      allWorldFrameUids.insert(cam.graphUid);
    }
    staticCams.insert(staticCams.end(), cams.begin(), cams.end());
  }

  // Read semi-dense point cloud
  XR_LOGI("Loading semi-dense points can take some time...");
  GlobalPointCloud ptClouds;
  for (const auto& path : globalPointCloudPaths) {
    // Load point cloud (can be .csv or .gz)
    const auto ptCloud = readGlobalPointCloud(
        path,
        std::filesystem::path(path).extension() == ".csv" ? StreamCompressionMode::NONE
                                                          : StreamCompressionMode::GZIP);
    for (const auto& point : ptCloud) {
      allWorldFrameUids.insert(point.graphUid);
    }
    ptClouds.insert(ptClouds.end(), ptCloud.begin(), ptCloud.end());
  }
  XR_LOGI("Loaded semi-dense points with size: {}", ptClouds.size());

  // Get point observations
  XR_LOGI("Loading point observations can take some time...");
  PointObservations pointObs;
  if (!pointObservationPath.empty()) {
    pointObs = readPointObservations(pointObservationPath, StreamCompressionMode::GZIP);
  }
  const auto leftCameraSerial =
      vrsProvider.getImageConfiguration(slam_left_streamId.value()).sensorSerial;
  const auto rightCameraSerial =
      vrsProvider.getImageConfiguration(slam_right_streamId.value()).sensorSerial;
  PointAndObservationProvider pointObsProvider(
      ptClouds, pointObs, leftCameraSerial, rightCameraSerial, startTimestampNs, endTimestampNs);
  XR_LOGI(
      "Loaded and cached point observations into {} left and {} right frames. Total size: {}",
      pointObsProvider.numObsFrameLeft(),
      pointObsProvider.numObsFrameRight(),
      pointObsProvider.numPointObs());

  // Get device calibration
  const auto deviceCalib = vrsProvider.getDeviceCalibration();

  // Get eye gazes
  const auto eyeGazes = readEyeGaze(eyeGazePath);
  EyeGazeProvider eyeGazeProvider(eyeGazes);
  XR_LOGI("Loaded and cached eye gazes with size: {}", eyeGazeProvider.numEyeGazes());

  XR_CHECK_LE(
      allWorldFrameUids.size(),
      1,
      "There are more than one world coordinate frames exist in the input closed loop trajectory / global point cloud / static cameras, so they cannot be visualized together in a single coordinate frame.");

  // Set 3D boundary based on multiple inputs
  Boundary boundary(replayTraj, fullTrajs);
  XR_LOGI(
      "Set raw 3D boundary with X range: [{}, {}], Y range: [{}, {}], Z range: [{}, {}]",
      boundary.minX(),
      boundary.maxX(),
      boundary.minY(),
      boundary.maxY(),
      boundary.minZ(),
      boundary.maxZ());

  // Setup GUI with flow control
  pangolin::Var<std::string> uiDelimiterReplay_{
      "ui3d.-------------- Replay controls ---------------", "", pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> uiAuto{"ui3d.auto play", true, true};
  pangolin::Var<bool> uiNext{"ui3d.next", false, false};
  int replayFrame = 0;
  int lastReplayFrame = replayFrame - 1;
  pangolin::Var<int>::Attach("ui3d.replay frame", replayFrame, 0, numSlamData - 1);
  Data3DGui gui3d(true, fullTrajs, staticCams, ptClouds, boundary.getDilatedBoundary());

  // Declare per-frame variables to draw
  std::optional<projectaria::tools::data_provider::ImageData> slamLeftImageData, slamRightImageData,
      rgbImageData;
  std::optional<Sophus::SE3d> currPose;
  std::vector<PointObservationPair> currLeftPointObs;
  std::vector<PointObservationPair> currRightPointObs;
  std::optional<EyeGaze> eyeGaze;

  // Main viewer loop
  while (!pangolin::ShouldQuit()) {
    // Check if jumping to an arbitrary frame
    const bool jumpFrame = lastReplayFrame != replayFrame - 1;
    if (jumpFrame) {
      gui3d.clearLastTraj();
    }
    // Check NOT at the end of the sequence
    if (replayFrame < numSlamData) {
      // Check moving on to another frame
      if (uiAuto || uiNext || jumpFrame) {
        uiNext.Reset();

        // Get current SLAM camera images
        const auto slamLeftImageDataAndRecord =
            vrsProvider.getImageDataByIndex(slam_left_streamId.value(), replayFrame);
        slamLeftImageData = slamLeftImageDataAndRecord.first;
        const auto slamRightImageDataAndRecord =
            vrsProvider.getImageDataByIndex(slam_right_streamId.value(), replayFrame);
        slamRightImageData = slamRightImageDataAndRecord.first;

        // Get current timestamp
        const int64_t captureTimestampNs = slamLeftImageDataAndRecord.second.captureTimestampNs;
        XR_LOGI(
            "Aria frame: {}/{}, timestamp (ns): {}",
            replayFrame,
            numSlamData - 1,
            captureTimestampNs);

        // Get current RGB camera image based on SLAM image's timestamp
        const auto rgbImageDataAndRecord =
            vrsProvider.getImageDataByTimeNs(rgb_streamId.value(), captureTimestampNs);
        if (rgbImageDataAndRecord.first.isValid()) {
          rgbImageData = rgbImageDataAndRecord.first;
        } else {
          XR_LOGW("No matching RGB image!");
        }

        // Get current pose from cached replay trajectory
        const auto pose = trajProvider.findPose(captureTimestampNs);

        // Convert pose to std::optional<Sophus::SE3d> for GUI to consume
        if (pose) {
          currPose = std::make_optional<Sophus::SE3d>(pose.value().T_world_device);
        } else {
          currPose = std::nullopt;
          XR_LOGW(
              "Does not find replay trajectory pose close to timestamp (ns): {}",
              captureTimestampNs);
        }

        // Get current point observations
        if (!pointObservationPath.empty()) {
          if (pose) {
            const int64_t poseTimestampNs =
                std::chrono::duration_cast<std::chrono::nanoseconds>(pose.value().trackingTimestamp)
                    .count();
            currLeftPointObs = pointObsProvider.findAllPointObs(leftCameraSerial, poseTimestampNs);
            currRightPointObs =
                pointObsProvider.findAllPointObs(rightCameraSerial, poseTimestampNs);
            if (!pointObservationPath.empty()) {
              if (currLeftPointObs.empty()) {
                XR_LOGW("No 3D point is observed in the left SLAM camera at this frame");
              }
              if (currRightPointObs.empty()) {
                XR_LOGW("No 3D point is observed in the right SLAM camera at this frame");
              }
            }
          } else {
            currLeftPointObs.clear();
            currRightPointObs.clear();
          }
        }

        // Get current eye gaze
        if (!eyeGazePath.empty()) {
          if (pose) {
            const int64_t poseTimestampNs =
                std::chrono::duration_cast<std::chrono::nanoseconds>(pose.value().trackingTimestamp)
                    .count();
            eyeGaze = eyeGazeProvider.findEyeGaze(poseTimestampNs);
            if (!eyeGaze) {
              XR_LOGW("No eye gaze data is available at this frame");
            }
          }
        }

        // Update frame index to the next frame
        lastReplayFrame = replayFrame;
        ++replayFrame;
      }
    } else {
      // Stop auto play at the end of the sequence
      uiAuto = false;
    }

    // Draw current rig pose and images
    gui3d.draw(
        camCalibs,
        deviceCalib,
        currPose,
        rgbImageData,
        slamLeftImageData,
        slamRightImageData,
        currLeftPointObs,
        currRightPointObs,
        eyeGaze);

    pangolin::FinishFrame();
  }
  return 0;
}
