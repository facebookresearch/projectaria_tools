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

#include <functional>
#include <thread>

#include <calibration/CameraCalibration.h>
#include <data_provider/players/ImageSensorPlayer.h>
#include "EyeGaze.h"
#include "EyeGazeReader.h"
#include "GlobalPointCloudReader.h"
#include "HandTracking.h"
#include "StaticCameraCalibration.h"

#include <pangolin/display/image_view.h>
#include <pangolin/pangolin.h>

namespace projectaria::tools::mps {

using PointObservationPair = std::pair<Eigen::Vector2f, GlobalPointPosition>;

class Data3DGui {
 public:
  // Construct 3D viewer with full trajectories, static camera calibrations, and point cloud
  Data3DGui(
      const bool plot2DView,
      const std::vector<std::vector<Eigen::Vector3f>>& fullTrajsWorld,
      const StaticCameraCalibrations& camCalibs,
      const GlobalPointCloud& cloudWorld,
      const std::vector<float>& boundary);

  // update plot's 3d view, without current rig/2D images
  void draw();

  // update plot's 3d view, without current rig/2D images
  // raw frame should only used when plot2DView is set to true
  void draw(
      const std::vector<calibration::CameraCalibration>& camCalibs,
      const std::optional<calibration::DeviceCalibration>& deviceCalib,
      const std::optional<Sophus::SE3d>& T_World_Device,
      const std::optional<projectaria::tools::data_provider::ImageData>& rgbImageData,
      const std::optional<projectaria::tools::data_provider::ImageData>& slamLeftImageData,
      const std::optional<projectaria::tools::data_provider::ImageData>& slamRightImageData,
      const std::vector<PointObservationPair>& leftPtObs,
      const std::vector<PointObservationPair>& rightPtObs,
      const std::optional<EyeGaze>& generalizedEyeGaze,
      const std::optional<EyeGaze>& calibratedEyeGaze,
      const std::optional<WristAndPalmPose>& wristAndPalmPose,
      const std::optional<HandTrackingResult>& handTrackingResult);

  // Clear history of last traj
  void clearLastTraj();

  enum class ImageViewName { RGB_VIEW = 0, LEFT_SLAM_VIEW = 1, RIGHT_SLAM_VIEW = 2 };

  using ExternDrawFunction = std::function<void(pangolin::View&)>;
  // update plot's 2d view
  void draw(ImageViewName imageViewName, ExternDrawFunction&& extern_draw_function);

  void setUiPlotGeneralizedGaze(bool value);
  void setUiPlotCalibratedGaze(bool value);
  void setUiShowHandTrackingResult(bool value);

 private:
  void drawRig(
      const std::vector<calibration::CameraCalibration>& camCalibs,
      const Sophus::SE3d& T_World_Device);

  void updatePointCloud();

  void setPointColor(const GlobalPointPosition& pt) const;

  void drawLeftPointObs() const;
  void drawRightPointObs() const;

  void drawEyeGazePoint() const;

  static void drawEyeGaze(
      const Eigen::Vector3d& eyeGazePointCpf,
      const Sophus::SE3d& T_World_Device,
      const Sophus::SE3d& T_Device_Cpf,
      bool calibrated);

  pangolin::ImageView& getImageView(ImageViewName name);

  const bool plot2DView_;

  const GlobalPointCloud& cloudWorld_;
  std::vector<Eigen::Vector3f> filteredCloud_;
  std::mutex filteredCloudMutex_; // Lock for updating point cloud data

  const std::vector<std::vector<Eigen::Vector3f>>& fullTrajsWorld_;

  // Static camera calibration data
  const StaticCameraCalibrations camCalibs_;

  // 6-vector in the format of: max [x, y, z], min [x, y, z]
  const std::vector<float> boundary_;

  std::vector<Eigen::Vector3d> lastTraj_;

  pangolin::Var<std::string> uiDelimiterAriaTraj_{
      "ui3d.---------- Aria device trajectories ----------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> uiPlotAllTraj{"ui3d.plot all traj", true, true};
  pangolin::Var<float> uiTrajAlpha{"ui3d.traj alpha", 0.6, 0, 1};

  pangolin::Var<std::string> uiDelimiterCam_{
      "ui3d.------------------ Cameras -------------------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> uiShowLabel{"ui3d.show label", true, true};
  pangolin::Var<double> uiFrustumSize{"ui3d.frustum size(m)", 0.3, 0.1, 1.0};

  // Point cloud filtering UI & control
  pangolin::Var<std::string> uiDelimiterPtCloud_{
      "ui3d.----------- Semi-dense point cloud -----------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> uiPoints{"ui3d.show SD points", true, true};
  pangolin::Var<float> uiPointAlpha{"ui3d.point alpha", 0.6, 0, 1};

  const float kMaxInvDistanceStd_ = 0.001f;
  pangolin::Var<bool> var_FilterByInvDepthStd_{
      "ui3d.Filter By InvDist StdDev",
      kMaxInvDistanceStd_ > 0,
      false,
      true};
  pangolin::Var<float> var_MapMaxInvDistStd_{
      "ui3d.Map Max InvDist StdDev",
      kMaxInvDistanceStd_,
      1e-4,
      1.0f,
      true};

  const float kMaxDistanceStd_ = 0.15f;
  pangolin::Var<bool> var_FilterByDepthStd_{
      "ui3d.Filter By Dist StdDev",
      kMaxDistanceStd_ > 0,
      false,
      true};
  pangolin::Var<float> var_MapMaxDistStd_{
      "ui3d.Map Max Dist StdDev",
      kMaxDistanceStd_,
      1e-4,
      1.0f,
      true};

  pangolin::Var<bool> uiPlotObsRay{"ui3d.plot obs ray", false, true};

  // Eye Gaze UI & control
  pangolin::Var<std::string> uiDelimiterEyeGaze{
      "ui3d.------------------ Eye Gaze ------------------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> uiPlotGeneralizedGaze{"ui3d.Generalized Eye gaze", true, true};
  pangolin::Var<bool> uiPlotCalibratedGaze{"ui3d.Calibrated Eye gaze", true, true};
  pangolin::Var<float> uiGazeRayLength{"ui3d.Eye gaze depth(m) ", 0.35, 0.1, 20};

  // Hand Tracking Result UI
  pangolin::Var<std::string> uiDelimiterHandTrackingResultRef_{
      "ui3d.--------- Hand Tracking ----------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> uiShowHandTrackingResult{"ui3d.show hand tracking result", true, true};

  pangolin::Var<std::string> uiDelimiterScaleRef_{
      "ui3d.-------------- Scale reference ---------------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<int> uiWorldGridSize{"ui3d.world grid size(m) ", 10, 0, 20};

  pangolin::OpenGlRenderState visualization3dState_;
  std::unique_ptr<pangolin::Handler3D> vis3dState;
  pangolin::View* mapView;

  pangolin::ImageView* rgbView;
  pangolin::ImageView* slamView1;
  pangolin::ImageView* slamView2;
  pangolin::View* imageViews;

  std::map<ImageViewName, std::vector<ExternDrawFunction>> externalDrawFunctions_;

  uint32_t slamLeftWidth_;
  uint32_t slamLeftHeight_;
  uint32_t slamRightWidth_;
  uint32_t slamRightHeight_;
  uint32_t rgbWidth_;
  uint32_t rgbHeight_;

  std::vector<PointObservationPair> leftPtObs_;
  std::vector<PointObservationPair> rightPtObs_;

  calibration::CameraCalibration camCalib_;
  std::optional<Eigen::Vector2d> generalizedEyeGazeProj_;
  std::optional<Eigen::Vector2d> calibratedEyeGazeProj_;

  struct HandImageViewProjector {
    using HandLandmarks = std::map<HANDEDNESS, std::vector<Eigen::Vector2f>>;
    std::map<Data3DGui::ImageViewName, HandLandmarks> landmarksInImageView;

    using LineSegment = std::pair<Eigen::Vector2d, Eigen::Vector2d>;
    using HandSkeletons = std::map<HANDEDNESS, std::vector<LineSegment>>;
    std::map<Data3DGui::ImageViewName, HandSkeletons> linksInImageView;

    void drawLandmarksInImageView(Data3DGui::ImageViewName imageViewName);
  };

  HandImageViewProjector handImageViewProjector_;
  static void setHandsGlColor(HANDEDNESS handedness);
  static constexpr double MIN_CONFIDENCE_ = 0.5;
};

} // namespace projectaria::tools::mps
