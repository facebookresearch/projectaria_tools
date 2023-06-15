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

#include "MPSData3DGui.h"

#include <string>

constexpr int UI_WIDTH = 400;
constexpr int kMapPanelWidth = 1920;
constexpr int kWindowWidth = kMapPanelWidth + UI_WIDTH;
constexpr int kWindowHeight = 1080;
constexpr float k3dViewFocal = 420.f / 4.f;

const std::vector<Eigen::Vector3f> kColors{
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

extern const unsigned char AnonymousPro_ttf[];
static pangolin::GlFont kGlFont(AnonymousPro_ttf, 20);

namespace pangolin_helpers {
template <typename T>
inline void glDrawFrustum(const Eigen::Matrix<T, 3, 3>& Kinv, int w, int h, GLfloat scale) {
  pangolin::glDrawFrustum(
      (GLfloat)Kinv(0, 2),
      (GLfloat)Kinv(1, 2),
      (GLfloat)Kinv(0, 0),
      (GLfloat)Kinv(1, 1),
      w,
      h,
      scale);
}

template <typename T>
inline void glDrawFrustum(
    const Eigen::Matrix<T, 3, 3>& Kinv,
    int w,
    int h,
    const Eigen::Matrix<T, 4, 4>& T_wf,
    T scale) {
  pangolin::glSetFrameOfReference(T_wf);
  pangolin_helpers::glDrawFrustum(Kinv, w, h, scale);
  pangolin::glUnsetFrameOfReference();
}

Eigen::MatrixXf getPositionBounds(const Eigen::MatrixXf& points) {
  Eigen::MatrixXf bounds(3, 2);
  bounds.col(0) = points.rowwise().minCoeff().cast<float>();
  bounds.col(1) = points.rowwise().maxCoeff().cast<float>();
  return bounds;
}

void centerViewOnMap(
    pangolin::OpenGlRenderState& glcam,
    const std::vector<Eigen::Vector3f>& points,
    const float focalLength,
    const int windowWidth) {
  // Map input point to an Eigen3X matrix
  using Matrix3X = Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::RowMajor>;
  using MatrixCRef = Eigen::Map<const Matrix3X>;
  MatrixCRef pointsEigen(points[0].data(), 3, points.size());

  const Eigen::Vector3f center = getPositionBounds(pointsEigen).rowwise().mean();
  const Eigen::Vector3f gdir{0.0, 0.0, -9.81};
  Eigen::Vector3f nonParallelVec(1, 0, 0);
  if (std::abs(nonParallelVec.dot(gdir)) > 0.99) {
    nonParallelVec << 0, 0, 1;
  }
  const Eigen::Vector3f perpendicularVec = nonParallelVec.cross(gdir);
  const Eigen::Vector3f viewDir = gdir + 2 * perpendicularVec;
  const Eigen::MatrixXf krFromCenter = pointsEigen.colwise() - center;
  const Eigen::VectorXf krDirectionsFromCenter = krFromCenter.colwise().norm();
  const Eigen::VectorXf distances = krDirectionsFromCenter * focalLength / (windowWidth / 2);
  const float distance = distances.maxCoeff();
  const Eigen::Vector3f eye = center - distance * viewDir;
  glcam.SetModelViewMatrix(pangolin::ModelViewLookAtRDF(
      eye.x(),
      eye.y(),
      eye.z(),
      center.x(),
      center.y(),
      center.z(),
      -gdir.x(),
      -gdir.y(),
      -gdir.z()));
}

void updateIndividualFlagsByCentralFlag(
    pangolin::Var<bool>& centralFlag,
    std::vector<pangolin::Var<bool>>& individualFlags) {
  // if central flag is the same as last, nothing change
  if (!centralFlag.GuiChanged()) {
    return;
  }
  // all individual flags match updated central flag
  for (auto& flag : individualFlags) {
    flag = centralFlag;
  }
}

} // namespace pangolin_helpers

using namespace projectaria::tools::mps;

MPSData3DGui::MPSData3DGui(
    const std::vector<GlobalPointCloud>& clouds_world,
    const std::vector<std::vector<Eigen::Vector3f>>& fullTrajs_world,
    const StaticCameraCalibrations& camCalibs)
    : clouds_world_(clouds_world), fullTrajsWorld_(fullTrajs_world), camCalibs_(camCalibs) {
  pangolin::CreateWindowAndBind("MPS 3D Visualizer", kWindowWidth, kWindowHeight);
  visualization3dState_ = pangolin::OpenGlRenderState(pangolin::ProjectionMatrixRDF_TopLeft(
      kMapPanelWidth,
      kWindowHeight,
      1000,
      1000,
      kMapPanelWidth / 2,
      kWindowHeight / 2,
      0.02,
      1000));
  vis3dState_ = std::make_unique<pangolin::Handler3D>(visualization3dState_);
  mapView_ =
      &pangolin::CreateDisplay()
           .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
           .SetLayout(pangolin::LayoutEqual)
           .SetAspect(-static_cast<float>(kMapPanelWidth) / static_cast<float>(kWindowHeight))
           .SetHandler(vis3dState_.get());

  for (int i = 0; i < fullTrajsWorld_.size(); ++i) {
    uiPlotPerTraj_.emplace_back("ui3d.Show trajectory_" + std::to_string(i), true, true);
  }

  pangolin::CreatePanel("ui3d").SetBounds(0, 1, 0, pangolin::Attach::Pix(UI_WIDTH));

  // Show Point cloud can be true only if we have some point cloud data
  uiAllPoints_ = !clouds_world_.empty();
  var_FilterByInvDepthStd_ = !clouds_world_.empty();
  var_FilterByDepthStd_ = !clouds_world_.empty();
  // Show Static Camera can be true only if we have some static cameras
  uiShowCameras_ = !camCalibs.empty();
  uiShowCameraLabel_ = !camCalibs.empty();
  // Show Plot All Trajectory can be true only if we have some trajectory data
  uiPlotAllTraj_ = !fullTrajs_world.empty();

  glEnable(GL_MULTISAMPLE);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);

  // Init point cloud:
  updatePointCloud();

  // Initialize the Camera to look to the scene
  if (!fullTrajsWorld_.empty()) {
    pangolin_helpers::centerViewOnMap(
        visualization3dState_, fullTrajsWorld_[0], k3dViewFocal, kMapPanelWidth);
  } else {
    // We need to center on the point cloud
    pangolin_helpers::centerViewOnMap(
        visualization3dState_, filteredClouds_[0], k3dViewFocal, kMapPanelWidth);
  }
}

void MPSData3DGui::draw() {
  // vis 3d
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  mapView_->Activate(visualization3dState_);

  if (var_FilterByInvDepthStd_.GuiChanged() || var_MapMaxInvDistStd_.GuiChanged() ||
      var_FilterByDepthStd_.GuiChanged() || var_MapMaxDistStd_.GuiChanged()) {
    updatePointCloud();
  }

  // Point cloud visualization
  if (uiAllPoints_) {
    std::lock_guard<std::mutex> lock(filteredCloudMutex_);
    for (int i = 0; i < filteredClouds_.size(); i++) {
      if (uiPointColor_) {
        const auto& color = kColors[i % kColors.size()];
        glColor4f(color[0], color[1], color[2], uiPointAlpha_);
      } else {
        glColor4f(0.9, 0.95, 1, uiPointAlpha_);
      }
      const auto& cloud = filteredClouds_[i];
      pangolin::glDrawPoints(cloud);
    }
  }

  // Visualization of Aria Trajectories
  pangolin_helpers::updateIndividualFlagsByCentralFlag(uiPlotAllTraj_, uiPlotPerTraj_);
  for (int i = 0; i < fullTrajsWorld_.size(); i++) {
    if (uiPlotPerTraj_[i]) {
      const auto& traj = fullTrajsWorld_[i];
      const auto& color = kColors[i % kColors.size()];
      glColor4f(color[0], color[1], color[2], uiTrajAlpha_);
      pangolin::glDrawLineStrip(traj);
    }
  }

  // Static camera visualization as Camera Frustum
  if (uiShowCameras_) {
    glColor3f(1, 1, 1);
    for (int i = 0; i < camCalibs_.size(); i++) {
      const Sophus::SE3d& T_world_cam = camCalibs_[i].T_world_cam;
      const auto pcam_world = T_world_cam.translation();
      const float f = camCalibs_[i].intrinsics[0];
      const float w = camCalibs_[i].width;
      const float h = camCalibs_[i].height;
      const std::string id = camCalibs_[i].cameraUid;
      Eigen::Matrix3d K;
      K << f, 0, w / 2, 0, f, h / 2, 0, 0, 1;
      Eigen::Matrix3d Kinv = K.inverse();
      pangolin_helpers::glDrawFrustum(Kinv, w, h, T_world_cam.matrix(), uiCameraFrustumSize_.Get());
      if (uiShowCameraLabel_) {
        kGlFont.Text(id.c_str()).Draw(pcam_world[0], pcam_world[1], pcam_world[2]);
      }
    }
  }
}

void MPSData3DGui::updatePointCloud() {
  std::lock_guard<std::mutex> lock(filteredCloudMutex_);

  filteredClouds_.clear();
  filteredClouds_.reserve(clouds_world_.size());
  for (const auto& cloud : clouds_world_) {
    auto& filteredCloud = filteredClouds_.emplace_back();
    filteredCloud.reserve(cloud.size());
    for (const auto& point : cloud) {
      if (var_FilterByInvDepthStd_ && point.inverseDistanceStd > var_MapMaxInvDistStd_) {
        continue;
      }
      if (var_FilterByDepthStd_ && point.distanceStd > var_MapMaxDistStd_) {
        continue;
      }
      filteredCloud.emplace_back(point.position_world.cast<float>());
    }
    filteredCloud.shrink_to_fit();
  }
}
