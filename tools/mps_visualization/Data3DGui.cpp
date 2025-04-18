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

#include "Data3DGui.h"
#include <stdexcept>
#include "PangolinHelper.h"

#define DEFAULT_LOG_CHANNEL "Data3DGui"
#include <logging/Checks.h>
#include <logging/Log.h>

constexpr int UI_WIDTH = 400;
constexpr int kMapPanelWidth = 1920;
constexpr int kWindowWidth = kMapPanelWidth + UI_WIDTH;
constexpr int kWindowHeight = 1080;
constexpr int kLastTrajLength = 90; // 3s
constexpr float k3dViewFocal = 420.f / 4.f;
const std::vector<Eigen::Vector3f> kTrajColors{
    {0.12, 0.47, 0.71},
    {1, 0.5, 0.05},
    {0.17, 0.63, 0.17},
    {0.84, 0.15, 0.16},
    {0.58, 0.4, 0.26},
    {0.55, 0.34, 0.29},
    {0.89, 0.47, 0.76},
    {0.5, 0.5, 0.5},
    {0.74, 0.74, 0.13},
    {0.09, 0.75, 0.81}};
constexpr float kGeneralizedGazeColor[] = {0.0f, 1.0f, 1.0f};
constexpr float kCalibratedGazeColor[] = {1.0f, 0.0f, 1.0f};

// In meters
constexpr float kNormalVisLen = 0.05;

constexpr size_t kNumLandmarks = 21;
constexpr size_t kNumJointConnections = 23;
constexpr std::array<std::pair<size_t, size_t>, kNumJointConnections> kJointConnections = {
    {{5, 17}, {17, 18}, {18, 19}, {19, 4}, {5, 14}, {14, 15}, {15, 16}, {16, 3},
     {5, 11}, {11, 12}, {12, 13}, {13, 2}, {5, 8},  {8, 9},   {9, 10},  {10, 1},
     {5, 6},  {6, 7},   {7, 0},   {6, 8},  {8, 11}, {11, 14}, {14, 17}}};

extern const unsigned char AnonymousPro_ttf[];
static pangolin::GlFont kGlFont(AnonymousPro_ttf, 20);

namespace projectaria::tools::mps {

Data3DGui::Data3DGui(
    const bool plot2DView,
    const std::vector<std::vector<Eigen::Vector3f>>& fullTrajsWorld,
    const StaticCameraCalibrations& camCalibs,
    const GlobalPointCloud& cloudWorld,
    const std::vector<float>& boundary)
    : plot2DView_(plot2DView),
      cloudWorld_(cloudWorld),
      fullTrajsWorld_(fullTrajsWorld),
      camCalibs_(camCalibs),
      boundary_(boundary) {
  // check all static cameras on the same world frame
  std::set<std::string> worldGraphUids;
  for (const auto& camCalib : camCalibs) {
    worldGraphUids.insert(camCalib.graphUid);
  }
  XR_CHECK_LE(
      worldGraphUids.size(), 1, "Input static camera poses are not in the same world frame");

  lastTraj_.reserve(kLastTrajLength + 1);

  pangolin::CreateWindowAndBind("MPS 3D Replay Viewer", kWindowWidth, kWindowHeight);
  visualization3dState_ = pangolin::OpenGlRenderState(pangolin::ProjectionMatrixRDF_TopLeft(
      kMapPanelWidth, kWindowHeight, 1000, 1000, kMapPanelWidth / 2, kWindowHeight / 2, 0.1, 1000));
  vis3dState = std::make_unique<pangolin::Handler3D>(visualization3dState_);
  mapView = &pangolin::CreateDisplay()
                 .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
                 .SetLayout(pangolin::LayoutEqual)
                 .SetAspect(-static_cast<float>(kMapPanelWidth) / static_cast<float>(kWindowHeight))
                 .SetHandler(vis3dState.get());
  pangolin::CreatePanel("ui3d").SetBounds(0, 1, 0, pangolin::Attach::Pix(UI_WIDTH));

  if (plot2DView) {
    imageViews =
        &pangolin::CreateDisplay()
             .SetBounds(
                 0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), pangolin::Attach::Pix(UI_WIDTH + 480))
             .SetLayout(pangolin::LayoutEqual);

    // SLAM
    slamView1 = new pangolin::ImageView("Aria SLAM Left");
    slamView2 = new pangolin::ImageView("Aria SLAM Right");
    slamView1->SetThetaQuarterTurn(1);
    slamView2->SetThetaQuarterTurn(1);
    imageViews->AddDisplay(*slamView1).AddDisplay(*slamView2);
    // RGB
    rgbView = new pangolin::ImageView("Aria RGB");
    rgbView->SetThetaQuarterTurn(1);
    imageViews->AddDisplay(*rgbView);

    for (ImageViewName viewName :
         {ImageViewName::LEFT_SLAM_VIEW, ImageViewName::RIGHT_SLAM_VIEW, ImageViewName::RGB_VIEW}) {
      getImageView(viewName).extern_draw_function = [=, this](pangolin::View& v) {
        for (auto& extern_draw_function : this->externalDrawFunctions_[viewName]) {
          extern_draw_function(v);
        }
      };
    }
  }

  glEnable(GL_MULTISAMPLE);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);

  updatePointCloud();

  // Initialize the Camera to look to the scene
  if (!fullTrajsWorld_.empty()) {
    pangolin_helpers::centerViewOnMap(
        visualization3dState_, fullTrajsWorld_[0], k3dViewFocal, kMapPanelWidth);
  } else {
    // We need to center on the point cloud
    pangolin_helpers::centerViewOnMap(
        visualization3dState_, filteredCloud_, k3dViewFocal, kMapPanelWidth);
  }
}

void Data3DGui::draw() {
  // vis 3d
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  mapView->Activate(visualization3dState_);

  if (var_FilterByInvDepthStd_.GuiChanged() || var_MapMaxInvDistStd_.GuiChanged() ||
      var_FilterByDepthStd_.GuiChanged() || var_MapMaxDistStd_.GuiChanged()) {
    updatePointCloud();
  }

  // Plot worldlock grid based on boundary
  if (uiWorldGridSize > 0) {
    const int gridSize = uiWorldGridSize;
    const int maxX = (int(boundary_[0]) / gridSize + 1) * gridSize;
    const int maxY = (int(boundary_[1]) / gridSize + 1) * gridSize;
    const int maxZ = (int(boundary_[2]) / gridSize + 1) * gridSize;
    const int minX = (int(boundary_[3]) / gridSize - 1) * gridSize;
    const int minY = (int(boundary_[4]) / gridSize - 1) * gridSize;
    const int minZ = (int(boundary_[5]) / gridSize - 1) * gridSize;
    glColor4f(0.8, 0.8, 0.8, 0.4);
    for (int x = minX; x <= maxX; x += gridSize) {
      for (int y = minY; y <= maxY; y += gridSize) {
        pangolin::glDrawLine(x, y, maxZ, x, y, minZ);
      }
    }
    for (int y = minY; y <= maxY; y += gridSize) {
      for (int z = minZ; z <= maxZ; z += gridSize) {
        pangolin::glDrawLine(maxX, y, z, minX, y, z);
      }
    }
    for (int z = minZ; z <= maxZ; z += gridSize) {
      for (int x = minX; x <= maxX; x += gridSize) {
        pangolin::glDrawLine(x, maxY, z, x, minY, z);
      }
    }
  }

  // Point cloud visualization
  if (uiPoints) {
    std::lock_guard<std::mutex> lock(filteredCloudMutex_);
    glColor4f(0.9f, 0.95f, 1.0f, uiPointAlpha);
    glPointSize(2);
    pangolin::glDrawPoints(filteredCloud_);
  }

  // vis all traj
  if (uiPlotAllTraj) {
    for (int i = 0; i < fullTrajsWorld_.size(); i++) {
      const auto& traj = fullTrajsWorld_[i];
      const auto& color = kTrajColors[i % kTrajColors.size()];
      glColor4f(color[0], color[1], color[2], uiTrajAlpha);
      pangolin::glDrawLineStrip(traj);
    }
  }

  // Static camera visualization as Camera Frustum
  glColor3f(1.0f, 1.0f, 1.0f);
  for (int i = 0; i < camCalibs_.size(); i++) {
    const Sophus::SE3d& T_World_Cam = camCalibs_[i].T_world_cam;
    const auto pcam_world = T_World_Cam.translation();
    const float f = camCalibs_[i].intrinsics[0];
    const float w = camCalibs_[i].width;
    const float h = camCalibs_[i].height;
    Eigen::Matrix3d K;
    K << f, 0, w / 2, 0, f, h / 2, 0, 0, 1;
    Eigen::Matrix3d Kinv = K.inverse();
    pangolin_helpers::glDrawFrustum(Kinv, w, h, T_World_Cam.matrix(), uiFrustumSize.Get());
    if (uiShowLabel) {
      kGlFont.Text(camCalibs_[i].cameraUid.c_str())
          .Draw(pcam_world[0], pcam_world[1], pcam_world[2]);
    }
  }
}

pangolin::ImageView& Data3DGui::getImageView(ImageViewName name) {
  switch (name) {
    case ImageViewName::RGB_VIEW:
      return *rgbView;
    case ImageViewName::LEFT_SLAM_VIEW:
      return *slamView1;
    case ImageViewName::RIGHT_SLAM_VIEW:
      return *slamView2;
    default:
      XR_LOGE("Unknown image view name {}", static_cast<int>(name));
      throw std::runtime_error("Unknown image view name");
  }
}

void Data3DGui::draw(ImageViewName imageViewName, ExternDrawFunction&& extern_draw_function) {
  externalDrawFunctions_[imageViewName].push_back(extern_draw_function);
}

void Data3DGui::clearLastTraj() {
  lastTraj_.clear();
}

void Data3DGui::drawRig(
    const std::vector<calibration::CameraCalibration>& camCalibs,
    const Sophus::SE3d& T_World_Device) {
  glColor3f(0.0f, 1.0f, 0.0f);
  const Eigen::Vector3d pdevice_world = T_World_Device.translation();
  for (const auto& camCalib : camCalibs) {
    const auto& T_Device_Cam = camCalib.getT_Device_Camera();
    const Sophus::SE3d T_World_Cam = T_World_Device * T_Device_Cam;
    const auto pcam_world = T_World_Cam.translation();

    const float f = camCalib.projectionParams()[0];
    const float w = camCalib.getImageSize()(0);
    const float h = camCalib.getImageSize()(1);
    Eigen::Matrix3d K;
    K << f, 0, w / 2, 0, f, h / 2, 0, 0, 1;
    Eigen::Matrix3d Kinv = K.inverse();
    pangolin_helpers::glDrawFrustum(Kinv, w, h, T_World_Cam.matrix(), uiFrustumSize.Get());
    pangolin::glDrawLine(
        pdevice_world[0],
        pdevice_world[1],
        pdevice_world[2],
        pcam_world[0],
        pcam_world[1],
        pcam_world[2]);
  }
  if (uiShowLabel) {
    kGlFont.Text("Aria RGB").Draw(pdevice_world[0], pdevice_world[1], pdevice_world[2]);
  }
}

void Data3DGui::drawEyeGaze(
    const Eigen::Vector3d& eyeGazePointCpf,
    const Sophus::SE3d& T_World_Device,
    const Sophus::SE3d& T_Device_Cpf,
    bool calibrated) const {
  const auto T_World_Cpf = T_World_Device * T_Device_Cpf;
  const Eigen::Vector3d cpfPointWorld = T_World_Cpf.translation();
  const Eigen::Vector3d eyeGazePointWorld = T_World_Cpf * eyeGazePointCpf;

  if (calibrated) {
    glColor3f(kCalibratedGazeColor[0], kCalibratedGazeColor[1], kCalibratedGazeColor[2]);
  } else {
    glColor3f(kGeneralizedGazeColor[0], kGeneralizedGazeColor[1], kGeneralizedGazeColor[2]);
  }
  glLineWidth(6);
  pangolin::glDrawLine(
      cpfPointWorld[0],
      cpfPointWorld[1],
      cpfPointWorld[2],
      eyeGazePointWorld[0],
      eyeGazePointWorld[1],
      eyeGazePointWorld[2]);
}

void Data3DGui::setUiPlotGeneralizedGaze(bool value) {
  uiPlotGeneralizedGaze = value;
}

void Data3DGui::setUiPlotCalibratedGaze(bool value) {
  uiPlotCalibratedGaze = value;
}

void Data3DGui::setUiShowWristAndPalmPose(bool value) {
  uiShowWristAndPalm = value;
}

void Data3DGui::draw(
    const std::vector<calibration::CameraCalibration>& camCalibs,
    const std::optional<calibration::DeviceCalibration>& deviceCalib,
    const std::optional<Sophus::SE3d>& T_World_Device,
    const std::optional<projectaria::tools::data_provider::ImageData>& rgbImageData,
    const std::optional<projectaria::tools::data_provider::ImageData>& slamLeftImageData,
    const std::optional<projectaria::tools::data_provider::ImageData>& slamRightImageData,
    const std::vector<std::pair<Eigen::Vector2f, GlobalPointPosition>>& leftPtObs,
    const std::vector<std::pair<Eigen::Vector2f, GlobalPointPosition>>& rightPtObs,
    const std::optional<EyeGaze>& generalizedEyeGaze,
    const std::optional<EyeGaze>& calibratedEyeGaze,
    const std::optional<WristAndPalmPose>& wristAndPalmPose) {
  // Plot fixed scene
  draw();

  // Plot current pose with short history
  if (T_World_Device) {
    // vis current pose
    drawRig(camCalibs, T_World_Device.value());
    // vis SW traj
    if (lastTraj_.back() != T_World_Device.value().translation()) {
      lastTraj_.push_back(T_World_Device.value().translation());
      if (lastTraj_.size() > kLastTrajLength) {
        lastTraj_.erase(lastTraj_.begin());
      }
    }
  }
  glColor4f(0.0f, 1.0f, 0.0f, 0.75f);
  pangolin::glDrawLineStrip(lastTraj_);

  // Plot current point observations in 3D
  for (const auto& obs : leftPtObs) {
    const auto& pt = obs.second.position_world;
    const auto& pPose = lastTraj_.back();
    setPointColor(obs.second);
    glPointSize(4);
    pangolin::glDrawPoints(std::vector<Eigen::Vector3d>{pt});
    if (uiPlotObsRay) {
      glColor4f(1.0f, 0.0f, 0.0f, 0.4f);
      pangolin::glDrawLine(pPose(0), pPose(1), pPose(2), pt(0), pt(1), pt(2));
    }
  }
  for (const auto& obs : rightPtObs) {
    const auto& pt = obs.second.position_world;
    const auto& pPose = lastTraj_.back();
    setPointColor(obs.second);
    glPointSize(4);
    pangolin::glDrawPoints(std::vector<Eigen::Vector3d>{pt});
    if (uiPlotObsRay) {
      glColor4f(1.0f, 0.0f, 0.0f, 0.4f);
      pangolin::glDrawLine(pPose(0), pPose(1), pPose(2), pt(0), pt(1), pt(2));
    }
  }

  // Update cached eye gaze point for plot
  float depth = uiGazeRayLength;
  const auto T_Cpf_Sensor = deviceCalib.value().getT_Cpf_Sensor("camera-rgb", true);
  Eigen::Vector3d generalizedEyeGazePointCpf;
  if (generalizedEyeGaze && uiPlotGeneralizedGaze && uiGazeRayLength > 0.0f) {
    // Find eye gaze point in the central pupil frame (CPF)
    if (generalizedEyeGaze.value().depth > 0.0f) {
      depth = generalizedEyeGaze.value().depth;
    }
    generalizedEyeGazePointCpf = getEyeGazePointAtDepth(
        generalizedEyeGaze.value().yaw, generalizedEyeGaze.value().pitch, depth);
    // Project onto RGB image
    if (deviceCalib) {
      generalizedEyeGazeProj_ =
          camCalibs.front().project(T_Cpf_Sensor->inverse() * generalizedEyeGazePointCpf);
    } else {
      generalizedEyeGazeProj_ = camCalibs.front().project(generalizedEyeGazePointCpf);
      calibratedEyeGazeProj_ = camCalibs.front().project(generalizedEyeGazePointCpf);
    }
    // Reject if out of the image boundary
    if (!camCalibs.front().isVisible(*generalizedEyeGazeProj_)) {
      generalizedEyeGazeProj_ = {};
    }
  }
  // Update cached eye gaze point for plot
  Eigen::Vector3d calibratedEyeGazePointCpf;
  if (calibratedEyeGaze && uiPlotCalibratedGaze && uiGazeRayLength > 0.0f) {
    // Find eye gaze point in the central pupil frame (CPF)
    if (calibratedEyeGaze.value().depth > 0.0f) {
      depth = calibratedEyeGaze.value().depth;
    }
    calibratedEyeGazePointCpf = getEyeGazePointAtDepth(
        calibratedEyeGaze.value().yaw, calibratedEyeGaze.value().pitch, depth);
    // Project onto RGB image
    if (deviceCalib) {
      calibratedEyeGazeProj_ =
          camCalibs.front().project(T_Cpf_Sensor->inverse() * calibratedEyeGazePointCpf);
    } else {
      calibratedEyeGazeProj_ = camCalibs.front().project(calibratedEyeGazePointCpf);
    }
    // Reject if out of the image boundary
    if (!camCalibs.front().isVisible(*calibratedEyeGazeProj_)) {
      calibratedEyeGazeProj_ = {};
    }
  }

  // plot frame only image view is init
  if (rgbImageData && slamLeftImageData && slamRightImageData) {
    XR_CHECK(plot2DView_);
    const auto& slamLeftImage = slamLeftImageData.value().imageVariant();
    const auto& slamRightImage = slamRightImageData.value().imageVariant();
    const auto& rgbImage = rgbImageData.value().imageVariant();
    XR_CHECK(slamLeftImage && slamRightImage && rgbImage);

    // Update cached data for plot
    leftPtObs_ = leftPtObs;
    rightPtObs_ = rightPtObs;
    camCalib_ = camCalibs.front();

    // SLAM left
    slamLeftWidth_ = getWidth(slamLeftImage.value());
    slamLeftHeight_ = getHeight(slamLeftImage.value());
    slamView1->SetImage(
        static_cast<void*>(getDataPtr(slamLeftImage.value())),
        slamLeftWidth_,
        slamLeftHeight_,
        slamLeftWidth_,
        pangolin::PixelFormatFromString("GRAY8"));
    // rotate 90
    slamView1->SetAspect(static_cast<float>(slamLeftHeight_) / static_cast<float>(slamLeftWidth_));
    // Plot current point observations in SLAM left image
    externalDrawFunctions_[ImageViewName::LEFT_SLAM_VIEW] = {
        [&](pangolin::View&) { drawLeftPointObs(); }};

    // SLAM right
    slamRightWidth_ = getWidth(slamRightImage.value());
    slamRightHeight_ = getHeight(slamRightImage.value());
    slamView2->SetImage(
        static_cast<void*>(getDataPtr(slamRightImage.value())),
        slamRightWidth_,
        slamRightHeight_,
        slamRightWidth_,
        pangolin::PixelFormatFromString("GRAY8"));
    // rotate 90
    slamView2->SetAspect(
        static_cast<float>(slamRightHeight_) / static_cast<float>(slamRightWidth_));
    // Plot current point observations in SLAM right image
    externalDrawFunctions_[ImageViewName::RIGHT_SLAM_VIEW] = {
        [&](pangolin::View&) { drawRightPointObs(); }};

    // RGB
    rgbWidth_ = getWidth(rgbImage.value());
    rgbHeight_ = getHeight(rgbImage.value());
    rgbView->SetImage(
        static_cast<void*>(getDataPtr(rgbImage.value())),
        rgbWidth_,
        rgbHeight_,
        rgbWidth_ * 3,
        pangolin::PixelFormatFromString("RGB24"));
    // rotate 90
    rgbView->SetAspect(static_cast<float>(rgbHeight_) / static_cast<float>(rgbWidth_));
    // Plot 2D eye gaze point in RGB image
    externalDrawFunctions_[ImageViewName::RGB_VIEW] = {
        [&](pangolin::View&) { drawEyeGazePoint(); }};
  }

  // Plot eye gaze rays in the 3D viewer
  if (T_World_Device && generalizedEyeGaze && uiPlotGeneralizedGaze) {
    if (deviceCalib) {
      const auto T_Device_Cpf = deviceCalib.value().getT_Device_Cpf();
      drawEyeGaze(generalizedEyeGazePointCpf, T_World_Device.value(), T_Device_Cpf, false);
    } else {
      drawEyeGaze(generalizedEyeGazePointCpf, T_World_Device.value(), Sophus::SE3d(), false);
    }
  }
  if (T_World_Device && calibratedEyeGaze && uiPlotCalibratedGaze) {
    if (deviceCalib) {
      const auto T_Device_Cpf = deviceCalib.value().getT_Device_Cpf();
      drawEyeGaze(calibratedEyeGazePointCpf, T_World_Device.value(), T_Device_Cpf, true);
    } else {
      drawEyeGaze(calibratedEyeGazePointCpf, T_World_Device.value(), Sophus::SE3d(), true);
    }
  }

  // wrist and palm vis 3d
  if (uiShowWristAndPalm && T_World_Device) {
    for (HANDEDNESS handedness : {HANDEDNESS::LEFT, HANDEDNESS::RIGHT}) {
      if (wristAndPalmPose) {
        const auto& oneHandWristAndPalmPose = wristAndPalmPose.value()[handedness];
        if (!oneHandWristAndPalmPose ||
            oneHandWristAndPalmPose.value().confidence < MIN_CONFIDENCE_) {
          continue;
        }
        const auto& wristPose = oneHandWristAndPalmPose->wristPosition_device;
        const auto& palmPose = oneHandWristAndPalmPose->palmPosition_device;
        const auto& wristPose_world = T_World_Device.value() * wristPose;
        const auto& palmPose_world = T_World_Device.value() * palmPose;
        const std::vector<Eigen::Vector3d> wristAndPalmPoses_world{wristPose_world, palmPose_world};
        setHandsGlColor(handedness);
        glPointSize(10);
        pangolin::glDrawPoints(wristAndPalmPoses_world);
        pangolin::glDrawLines(wristAndPalmPoses_world);

        // draw landmarks vis 3d
        std::array<Eigen::Vector3d, kNumLandmarks> landmarks_world;
        for (size_t iLandmark = 0; iLandmark < kNumLandmarks; ++iLandmark) {
          landmarks_world[iLandmark] =
              T_World_Device.value() * oneHandWristAndPalmPose->landmarkPositions_device[iLandmark];
        }
        std::vector<Eigen::Vector3d> fingerConnections;
        fingerConnections.reserve(23);
        for (const auto& connection : kJointConnections) {
          fingerConnections.push_back(landmarks_world[connection.first]);
          fingerConnections.push_back(landmarks_world[connection.second]);
        }
        pangolin::glDrawPoints(fingerConnections);
        pangolin::glDrawLines(fingerConnections);

        // draw wrist and palm normal
        if (!oneHandWristAndPalmPose.value().wristAndPalmNormal_device) {
          continue;
        }

        const auto& wristAndPalmNormal_device =
            oneHandWristAndPalmPose.value().wristAndPalmNormal_device.value();
        const auto& wristNormal_world =
            T_World_Device.value().so3() * wristAndPalmNormal_device.wristNormal_device;
        const auto& palmNormal_world =
            T_World_Device.value().so3() * wristAndPalmNormal_device.palmNormal_device;
        const std::vector<Eigen::Vector3d> wristAndPalmNormalLines{
            wristPose_world,
            wristPose_world + wristNormal_world * kNormalVisLen,
            palmPose_world,
            palmPose_world + palmNormal_world * kNormalVisLen};
        setHandsGlColor(handedness);
        pangolin::glDrawLines(wristAndPalmNormalLines);
      }
    }
  }

  // wrist and palm vis 2d
  for (Data3DGui::ImageViewName imageViewName :
       {Data3DGui::ImageViewName::RGB_VIEW,
        Data3DGui::ImageViewName::LEFT_SLAM_VIEW,
        Data3DGui::ImageViewName::RIGHT_SLAM_VIEW}) {
    for (HANDEDNESS handedness : {HANDEDNESS::LEFT, HANDEDNESS::RIGHT}) {
      auto& wristAndPalmLandmarksInImageView =
          wristAndPalmImageViewProjector_.landmarksInImageView[imageViewName][handedness];
      auto& wristAndPalmConnectivityLinksInImageView =
          wristAndPalmImageViewProjector_.linksInImageView[imageViewName][handedness];
      wristAndPalmLandmarksInImageView.clear();
      wristAndPalmConnectivityLinksInImageView.clear();
      if (!wristAndPalmPose) {
        continue;
      }
      const auto& oneHandWristAndPalmPose = wristAndPalmPose.value()[handedness];
      if (!uiShowWristAndPalm || !deviceCalib || !oneHandWristAndPalmPose ||
          oneHandWristAndPalmPose.value().confidence < MIN_CONFIDENCE_) {
        continue;
      }

      std::string cameraLabel;
      switch (imageViewName) {
        case Data3DGui::ImageViewName::RGB_VIEW:
          cameraLabel = "camera-rgb";
          break;
        case Data3DGui::ImageViewName::LEFT_SLAM_VIEW:
          cameraLabel = "camera-slam-left";
          break;
        case Data3DGui::ImageViewName::RIGHT_SLAM_VIEW:
          cameraLabel = "camera-slam-right";
          break;
        default:
          XR_LOGE("Unsupported image view name.");
          throw std::runtime_error("Unsupported image view name.");
      }
      const auto& cameraCalib = deviceCalib->getCameraCalib(cameraLabel);
      if (!cameraCalib) {
        continue;
      }

      // Collect all lines as pair of points
      std::vector<Eigen::Vector3d> lines = {
          oneHandWristAndPalmPose->wristPosition_device,
          oneHandWristAndPalmPose->palmPosition_device};
      if (oneHandWristAndPalmPose->wristAndPalmNormal_device.has_value()) {
        const auto& wristAndPalmNormal_device =
            oneHandWristAndPalmPose->wristAndPalmNormal_device.value();
        // wrist normal line
        lines.emplace_back(oneHandWristAndPalmPose->wristPosition_device);
        lines.emplace_back(
            oneHandWristAndPalmPose->wristPosition_device +
            wristAndPalmNormal_device.wristNormal_device * kNormalVisLen);
        // palm normal line
        lines.emplace_back(oneHandWristAndPalmPose->palmPosition_device);
        lines.emplace_back(
            oneHandWristAndPalmPose->palmPosition_device +
            wristAndPalmNormal_device.palmNormal_device * kNormalVisLen);
      }
      // Add landmarks
      for (const auto& connection : kJointConnections) {
        lines.emplace_back(oneHandWristAndPalmPose->landmarkPositions_device[connection.first]);
        lines.emplace_back(oneHandWristAndPalmPose->landmarkPositions_device[connection.second]);
      }
      // Must be multiple of 2 points
      if (lines.size() % 2 != 0) {
        const auto err = fmt::format(
            "Lines to be visualized must have multiple of 2 points, but got {}!", lines.size());
        throw std::runtime_error(err);
      }
      // Project each point to image and add to projector points list
      for (const auto& landmark : lines) {
        const auto point = cameraCalib->project(
            cameraCalib->getT_Device_Camera().inverse() * landmark.cast<double>());
        if (point) {
          wristAndPalmLandmarksInImageView.emplace_back(
              cameraCalib->getImageSize()[1] - 1 - point.value()[1], point.value()[0]);
        } else {
          wristAndPalmLandmarksInImageView.emplace_back(-1, -1);
        }
      }
      // Add each line to projector line list
      for (int i = 0; i < lines.size(); i += 2) {
        const auto& p0 = wristAndPalmLandmarksInImageView.at(i).cast<double>();
        const auto& p1 = wristAndPalmLandmarksInImageView.at(i + 1).cast<double>();
        // Check visibility of link boundary points in rotated image
        const auto& imageSize = cameraCalib->getImageSize();
        if (p0[0] >= 0 && p0[0] < imageSize[1] && p0[1] >= 0 && p0[1] < imageSize[0] &&
            p1[0] >= 0 && p1[0] < imageSize[1] && p1[1] >= 0 && p1[1] < imageSize[0]) {
          wristAndPalmConnectivityLinksInImageView.emplace_back(p0, p1);
        }
      }
    }

    draw(imageViewName, [&, imageViewName](pangolin::View&) {
      wristAndPalmImageViewProjector_.drawLandmarksInImageView(imageViewName);
    });
  }
}

void Data3DGui::updatePointCloud() {
  std::lock_guard<std::mutex> lock(filteredCloudMutex_);

  filteredCloud_.clear();
  filteredCloud_.reserve(cloudWorld_.size());
  for (const auto& point : cloudWorld_) {
    if (var_FilterByInvDepthStd_ && point.inverseDistanceStd > var_MapMaxInvDistStd_) {
      continue;
    }
    if (var_FilterByDepthStd_ && point.distanceStd > var_MapMaxDistStd_) {
      continue;
    }
    filteredCloud_.emplace_back(point.position_world.cast<float>());
  }
  filteredCloud_.shrink_to_fit();
}

void Data3DGui::setPointColor(const GlobalPointPosition& pt) const {
  const auto& dist = (lastTraj_.back() - pt.position_world).norm();
  pangolin::Colour c = pangolin::Colour::Hsv(std::fmin(0.65f, dist / 10.0f));
  glColor3f(c.red, c.green, c.blue);
}

void Data3DGui::drawLeftPointObs() const {
  for (const auto& obs : leftPtObs_) {
    const auto& uv = obs.first;
    setPointColor(obs.second);
    pangolin::glDrawCircle(slamLeftHeight_ - 1 - uv(1), uv(0), 2);
  }
}
void Data3DGui::drawRightPointObs() const {
  for (const auto& obs : rightPtObs_) {
    const auto& uv = obs.first;
    setPointColor(obs.second);
    pangolin::glDrawCircle(slamRightHeight_ - 1 - uv(1), uv(0), 2);
  }
}

void Data3DGui::drawEyeGazePoint() const {
  const float scaleX = (float)rgbWidth_ / (float)camCalib_.getImageSize().x();
  const float scaleY = (float)rgbHeight_ / (float)camCalib_.getImageSize().y();
  XR_CHECK_EQ(scaleX, scaleY);
  glLineWidth(20);
  if (generalizedEyeGazeProj_ && uiPlotGeneralizedGaze) {
    glColor3f(kGeneralizedGazeColor[0], kGeneralizedGazeColor[1], kGeneralizedGazeColor[2]);
    pangolin::glDrawCircle(
        rgbHeight_ - 1 - (generalizedEyeGazeProj_->y() * scaleY),
        generalizedEyeGazeProj_->x() * scaleX,
        6);
  }
  if (calibratedEyeGazeProj_ && uiPlotCalibratedGaze) {
    glColor3f(kCalibratedGazeColor[0], kCalibratedGazeColor[1], kCalibratedGazeColor[2]);
    pangolin::glDrawCircle(
        rgbHeight_ - 1 - (calibratedEyeGazeProj_->y() * scaleY),
        calibratedEyeGazeProj_->x() * scaleX,
        6);
  }
}

void Data3DGui::HandImageViewProjector::drawLandmarksInImageView(
    Data3DGui::ImageViewName imageViewName) {
  for (HANDEDNESS handedness : {HANDEDNESS::LEFT, HANDEDNESS::RIGHT}) {
    setHandsGlColor(handedness);
    glLineWidth(2);
    for (const auto& landmark : landmarksInImageView[imageViewName][handedness]) {
      pangolin::glDrawCircle(landmark[0], landmark[1], 5);
    }
    for (const auto& link : linksInImageView[imageViewName][handedness]) {
      pangolin::glDrawLine(link.first[0], link.first[1], link.second[0], link.second[1]);
    }
  }
}

void Data3DGui::setHandsGlColor(HANDEDNESS handedness) {
  switch (handedness) {
    case HANDEDNESS::LEFT:
      glColor4f(1.0f, 0.5f, 0.0f, 1.0f);
      break;
    case HANDEDNESS::RIGHT:
      glColor4f(0.0f, 0.5f, 1.0f, 1.0f);
      break;
  }
}

} // namespace projectaria::tools::mps
