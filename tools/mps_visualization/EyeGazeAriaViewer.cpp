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

#include "EyeGazeAriaViewer.h"
#include "AriaStreamIds.h"
#include "mps/EyeGaze.h"
#include "mps/EyeGazeReader.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iterator>

#include <pangolin/display/display.h>
#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/pangolin.h>

using namespace projectaria::tools;
using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::mps;

EyeGazeAriaViewer::EyeGazeAriaViewer(
    const std::shared_ptr<EyeGazeAriaPlayer> eyegazeAriaPlayer,
    int width,
    int height,
    const std::string& name)
    : AriaViewer(width, height, name, eyegazeAriaPlayer),
      eyeGazesVisData_(eyegazeAriaPlayer->getEyeGazeVisDataPtr()) {}

void EyeGazeAriaViewer::init() {
  createWindowWithDisplay(width_, height_);

  addImageDisplays(dataProvider_);
  addEtPlot();

  // prefix to give each viewer its own set of controls
  addControlPanel(dataProvider_);

  // Widget layout (On/Off)
  addImageToggle(dataProvider_);
  std::string prefix = "ui";
  // Widget layout (On/Off)
  showETTemporal_ = std::make_unique<pangolin::Var<bool>>(prefix + ".EyeGaze Temporal", true, true);
  showETRadar_ = std::make_unique<pangolin::Var<bool>>(prefix + ".EyeGaze Radar", true, true);
}

void EyeGazeAriaViewer::run() {
  // Main loop
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    updateGuiAndControl();

    updateImages();
    updateEtPlot();

    updateImageVisibility();

    // propagate show parameters
    eyeGazePlotter_->Show(*showETTemporal_);
    eyeGazeRadar_->Show(*showETRadar_);

    pangolin::FinishFrame();
  }
  std::cout << "\nQuit Viewer." << std::endl;
  exit(1);
}

namespace {

std::mutex drawEyeGazeMutex;

// Helper function to project EyeGaze point in camera space
std::optional<Eigen::Vector2d> ProjectEyeGazePointInCamera(
    const std::string& cameraString,
    const calibration::DeviceCalibration& deviceModel,
    const Eigen::Vector3d& eyeGazePointCpf) {
  // Since EyeGaze results is defined in CPF you can project in Aria cameras
  // we are projecting directly using the camera calibration relative to CPF
  if (deviceModel.getCameraCalib(cameraString)) {
    // Get back camera of interest
    const auto vrsCamera = deviceModel.getCameraCalib(cameraString);
    const auto T_Cpf_Sensor = deviceModel.getT_Cpf_Sensor(cameraString, true);

    auto projection = vrsCamera->project(T_Cpf_Sensor->inverse() * eyeGazePointCpf);
    auto cameraWidth = vrsCamera->getImageSize()[0];
    auto cameraHeight = vrsCamera->getImageSize()[1];

    // Is projection inside the image or not
    if (projection && cameraWidth > 0 && cameraHeight > 0 && vrsCamera->isVisible(*projection)) {
      return projection;
    }
  }
  return {};
}

void drawEyeGazePoint(
    pangolin::View& v,
    pangolin::ImageView& imageView,
    const bool imageRotated,
    const std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider,
    std::shared_ptr<AriaVisualizationData> ariaVisData,
    std::shared_ptr<EyeGazeVisualizationData> eyeGazesVisData,
    const vrs::StreamId& vrsStreamId,
    const std::string& cameraString) {
  std::unique_lock lock(drawEyeGazeMutex);
  v.ActivatePixelOrthographic();
  v.ActivateAndScissor();

  if (imageView.IsShown() && ariaVisData->cameraImageBufferMap_.count(vrsStreamId)) {
    // Display EyeGaze vector reprojection in the VRS image
    {
      // scale to the current display size
      const auto& imageData = ariaVisData->cameraImageBufferMap_.at(vrsStreamId);
      // scale to the current display size (use Height since we d the image)
      const float scale = imageRotated ? (float)v.v.w / (float)imageData.getHeight()
                                       : (float)v.v.w / (float)imageData.getWidth();

      glLineWidth(10);
      glColor3f(1.0, 1.0, 1.0);
      Eigen::Vector3d gazePointCpf = getEyeGazePointAtDepth(
          eyeGazesVisData->lastYawPitch.x(), eyeGazesVisData->lastYawPitch.y(), 1.0f);
      auto gazePointProjected = ProjectEyeGazePointInCamera(
          cameraString, dataProvider->getDeviceCalibration().value(), gazePointCpf);
      if (gazePointProjected) {
        // Compensate for OpenGL coordinate system difference and 90 degree image rotation
        gazePointProjected->y() = imageData.getHeight() - gazePointProjected->y();
        if (imageRotated) {
          gazePointProjected->x() = imageData.getWidth() - gazePointProjected->x();
          std::swap(gazePointProjected->y(), gazePointProjected->x());
        }
        *gazePointProjected *= scale;
        glLineWidth(2);
        glColor3f(1.0, 0.0, 0.0);
        pangolin::glDrawCircle(*gazePointProjected);
      }
    }
  }
  v.GetBounds().DisableScissor();
}

} // namespace

void EyeGazeAriaViewer::addEtPlot() {
  // Time Series setup for {yaw, pitch}
  logEyeGaze_ = std::make_shared<pangolin::DataLog>();
  logEyeGaze_->SetLabels({"eyeGaze_yaw [rad]", "eyeGaze_pitch [rad]"});
  eyeGazePlotter_ = std::make_shared<pangolin::Plotter>(
      logEyeGaze_.get(), 0.0f, 200.f, -M_PI / 3, M_PI / 3, 50, 1.);
  eyeGazePlotter_->Track("$i");
  container_->AddDisplay(*eyeGazePlotter_);

  // 2D Radar View setup (setup an orthogonal camera viewpoint)
  eyeGazeRadar_ = std::make_shared<pangolin::View>();
  eyeGazeRadar_->SetAspect(1.0);
  radar_view_camera_ = std::make_shared<pangolin::OpenGlRenderState>(
      pangolin::ProjectionMatrixOrthographic(-2.14, 2.14, -2.14, 2.14, -1.0, 1.0));
  auto* handler = new pangolin::Handler3D(*radar_view_camera_);
  eyeGazeRadar_->SetHandler(handler);
  container_->AddDisplay(*eyeGazeRadar_);

  for (auto& streamId : ariaVisData_->imageStreamIds_) {
    if (!dataProvider_->checkStreamIsActive(streamId) || streamId == kEyeCameraStreamId) {
      continue;
    }
    streamIdToPixelFrame_[streamId]->extern_draw_function = [&](pangolin::View& v) {
      drawEyeGazePoint(
          v,
          *streamIdToPixelFrame_[streamId],
          *showNaturalImageOrientation_,
          dataProvider_,
          ariaVisData_,
          eyeGazesVisData_,
          streamId,
          *dataProvider_->getLabelFromStreamId(streamId));
    };
  }
}

void EyeGazeAriaViewer::updateEtPlot() {
  if (!ariaVisControl_->isPlaying_) {
    logEyeGaze_->Clear();
  }
  std::unique_lock lock(drawEyeGazeMutex);
  // Draw ET image and time series data as event are coming
  if (ariaVisData_->isDataChanged(kEyeCameraStreamId)) {
    // Draw ET Gaze vector (time series)
    ariaVisData_->setDataChanged(false, kEyeCameraStreamId);
    const auto gaze = eyeGazesVisData_->lastYawPitch.cast<float>();
    if (*isPlaying_) {
      logEyeGaze_->Log({gaze.x(), gaze.y()});
    }

    // Add this measurement to the EyeGaze history buffer
    eyeGazesVisData_->yawPitchHistory.push_back(eyeGazesVisData_->lastYawPitch);
    // If buffer too large remove first element
    // - create a rolling buffer
    if (eyeGazesVisData_->yawPitchHistory.size() > 10) {
      eyeGazesVisData_->yawPitchHistory.pop_front();
    }
  }

  // Draw radar (using history saved data)
  if (!eyeGazesVisData_->yawPitchHistory.empty() && *showETRadar_) {
    eyeGazeRadar_->Activate(*radar_view_camera_);

    // Draw radar view background
    glColor3f(1, 1., 1.);
    for (double r = .1; r < 2; r += 0.5)
      pangolin::glDrawCirclePerimeter(Eigen::Vector2d(0, 0), r);

    // Draw eye gaze yaw, pitch history
    glColor3f(1, 0.8, 0);
    std::vector<Eigen::Vector2d> eyeGazeHistory{
        eyeGazesVisData_->yawPitchHistory.begin(), eyeGazesVisData_->yawPitchHistory.end()};
    pangolin::glDrawLineStrip(eyeGazeHistory);
    // Draw current gaze yaw, pitch
    const Eigen::Vector2f center(
        eyeGazesVisData_->yawPitchHistory.back().x(), eyeGazesVisData_->yawPitchHistory.back().y());
    pangolin::glDrawVertices(std::vector<Eigen::Vector2f>{center}, GL_POINTS);
    for (double r = .1; r < .3; r += 0.1)
      pangolin::glDrawCirclePerimeter(center.cast<double>(), r);
  }
}
