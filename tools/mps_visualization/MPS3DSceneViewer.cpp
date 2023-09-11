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

#include "MPS3DSceneViewer.h"

#include "GlobalPointCloudReader.h"
#include "MPSData3DGui.h"
#include "StaticCameraCalibrationReader.h"
#include "TrajectoryReaders.h"

#include <CLI/CLI.hpp>
#include <logging/Checks.h>

#include <filesystem>
#include <set>
#include <string>
#include <vector>

namespace projectaria::tools {

int mps3dSceneViewerCli(int argc, const char** argv) {
  using namespace projectaria::tools::mps;
  std::string openLoopTrajPath;
  std::vector<std::string> closedLoopTrajPaths;
  std::vector<std::string> globalPointCloudPaths;
  std::vector<std::string> staticCamerasPaths;

  CLI::App app{"MPS 3D Scene Visualizer"};

  app.add_option(
         "--open-loop-traj",
         openLoopTrajPath,
         "Input open loop trajectory file path. If you input an open loop trajectory, you should not input other closed loop trajectory / global point cloud / static cameras, since open loop trajectory is defined on odometry frame, which is different from world frame.")
      ->check(CLI::ExistingPath);
  app.add_option(
         "--closed-loop-traj", closedLoopTrajPaths, "Input closed loop trajectory file path(s).")
      ->check(CLI::ExistingPath);
  app.add_option(
         "--global-point-cloud", globalPointCloudPaths, "Input global point cloud file path(s).")
      ->check(CLI::ExistingPath);
  app.add_option(
         "--static-camera", staticCamerasPaths, "Input static camera calibration file path(s).")
      ->check(CLI::ExistingPath);

  CLI11_PARSE(app, argc, argv);

  XR_CHECK(
      openLoopTrajPath.empty() ||
          (closedLoopTrajPaths.empty() && globalPointCloudPaths.empty() &&
           staticCamerasPaths.empty()),
      "Open loop trajectory should not be visualized together with closed loop trajectory / global point cloud / static cameras, since open loop trajectory is defined on odometry frame, which is different from world frame.");

  std::vector<std::vector<Eigen::Vector3f>> fullTrajs_world;

  //
  // Open loop trajectory
  //

  if (!openLoopTrajPath.empty()) {
    const auto openTraj = readOpenLoopTrajectory(openLoopTrajPath);
    std::vector<Eigen::Vector3f> openTrajPoints;
    openTrajPoints.reserve(openTraj.size());
    for (const auto& pose : openTraj) {
      openTrajPoints.push_back(pose.T_odometry_device.translation().cast<float>());
    }
    fullTrajs_world.push_back(std::move(openTrajPoints));
  }

  // used to check everything in a single world frame
  std::set<std::string> allWorldFrameUids;

  //
  // Closed loop trajectories
  //

  for (const auto& path : closedLoopTrajPaths) {
    const auto closedTraj = readClosedLoopTrajectory(path);
    std::vector<Eigen::Vector3f> closedTrajPoints;
    closedTrajPoints.reserve(closedTraj.size());
    for (const auto& pose : closedTraj) {
      closedTrajPoints.push_back(pose.T_world_device.translation().cast<float>());
      allWorldFrameUids.insert(pose.graphUid);
    }
    fullTrajs_world.push_back(std::move(closedTrajPoints));
  }

  //
  // Find & Read point cloud data
  //

  std::vector<GlobalPointCloud> ptClouds;
  ptClouds.reserve(globalPointCloudPaths.size());
  for (const auto& path : globalPointCloudPaths) {
    // Load point cloud (can be .csv or .gz)
    auto ptCloud = readGlobalPointCloud(
        path,
        std::filesystem::path(path).extension() == ".csv" ? StreamCompressionMode::NONE
                                                          : StreamCompressionMode::GZIP);
    for (const auto& point : ptCloud) {
      allWorldFrameUids.insert(point.graphUid);
    }
    ptClouds.push_back(std::move(ptCloud));
  }

  //
  // Find & Read static camera calibration data
  //

  StaticCameraCalibrations staticCams;
  for (const auto& path : staticCamerasPaths) {
    const auto cams = readStaticCameraCalibrations(path);
    for (const auto& cam : cams) {
      allWorldFrameUids.insert(cam.graphUid);
    }
    staticCams.insert(staticCams.end(), cams.begin(), cams.end());
  }

  XR_CHECK_LE(
      allWorldFrameUids.size(),
      1,
      "There are more than one world coordinate frames exist in the input closed loop trajectory / global point cloud / static cameras, so they cannot be visualized together in a single coordinate frame.");

  // GUI
  MPSData3DGui gui3d(ptClouds, fullTrajs_world, staticCams);

  while (!pangolin::ShouldQuit()) {
    gui3d.draw();
    pangolin::FinishFrame();
  }

  gui3d.finish();

  return EXIT_SUCCESS;
}

} // namespace projectaria::tools
