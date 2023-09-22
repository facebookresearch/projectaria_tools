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

#include "GlobalPointCloud.h"
#include "StaticCameraCalibration.h"

#include <pangolin/pangolin.h>

#include <thread>

class MPSData3DGui {
 public:
  MPSData3DGui(
      const std::vector<projectaria::tools::mps::GlobalPointCloud>& clouds_world,
      const std::vector<std::vector<Eigen::Vector3f>>& fullTrajs_world,
      const projectaria::tools::mps::StaticCameraCalibrations& camCalibs);

  // update plot's 3d view
  void draw();
  // flush resources
  void finish();

 private:
  // Point cloud data & filtered version
  const std::vector<projectaria::tools::mps::GlobalPointCloud>& clouds_world_;
  std::vector<std::vector<Eigen::Vector3f>> filteredClouds_;
  std::mutex filteredCloudMutex_; // Lock for updating point cloud data

  // Trajectories (vector of trajectory)
  const std::vector<std::vector<Eigen::Vector3f>>& fullTrajsWorld_;

  // Static camera calibration data
  const projectaria::tools::mps::StaticCameraCalibrations camCalibs_;

  //
  // Pangolin UI component
  //

  pangolin::Var<std::string> uiDelimiterPtCloud_{
      "ui3d.----------- Semi-dense point cloud -----------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> uiAllPoints_{"ui3d.Show all semi-dense points", true, true};
  pangolin::Var<bool> uiPointColor_{"ui3d.Colorize points by session", false, true};
  pangolin::Var<float> uiPointAlpha_{"ui3d.Point alpha", 0.5, 0, 1};

  // Point cloud filtering UI & control
  const float kMaxInvDistanceStd_ = 0.001f;
  pangolin::Var<bool> var_FilterByInvDepthStd_{"ui3d.Filter by inverse distance std", true, true};
  pangolin::Var<float> var_MapMaxInvDistStd_{
      "ui3d.Max inverse distance std",
      kMaxInvDistanceStd_,
      1e-4,
      1.0f,
      true};

  const float kMaxDistanceStd_ = 0.15f;
  pangolin::Var<bool> var_FilterByDepthStd_{"ui3d.Filter by distance std", true, true};
  pangolin::Var<float> var_MapMaxDistStd_{
      "ui3d.Max distance std",
      kMaxDistanceStd_,
      1e-4,
      1.0f,
      true};

  pangolin::Var<std::string> uiDelimiterCam_{
      "ui3d.--------------- Static cameras ---------------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> uiShowCameras_{"ui3d.Show static cameras", true, true};
  pangolin::Var<bool> uiShowCameraLabel_{"ui3d.Show static camera label", true, true};
  pangolin::Var<double> uiCameraFrustumSize_{"ui3d.Static camera frustum size(m)", 0.3, 0.1, 1.0};

  pangolin::Var<std::string> uiDelimiterAriaTraj_{
      "ui3d.---------- Aria device trajectories ----------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<float> uiTrajAlpha_{"ui3d.Trajectory alpha", 1.0, 0, 1};
  pangolin::Var<bool> uiPlotAllTraj_{"ui3d.Show all trajectories", true, true};
  std::vector<pangolin::Var<bool>> uiPlotPerTraj_;

  pangolin::OpenGlRenderState visualization3dState_;
  std::unique_ptr<pangolin::Handler3D> vis3dState_;
  pangolin::View* mapView_;

  // Functions
  void updatePointCloud();
};
