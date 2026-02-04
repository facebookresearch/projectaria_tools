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
#include <array>
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

constexpr std::array<float, 3> kGeneralizedGazeColor = {0.0f, 1.0f, 1.0f};
constexpr std::array<float, 3> kCalibratedGazeColor = {1.0f, 0.0f, 1.0f};

namespace {
void plotEt(std::shared_ptr<EyeGazeVisualizationData> eyeGazeVisData) {
  if (eyeGazeVisData->calibrated) {
    glColor3f(kCalibratedGazeColor[0], kCalibratedGazeColor[1], kCalibratedGazeColor[2]);
  } else {
    glColor3f(kGeneralizedGazeColor[0], kGeneralizedGazeColor[1], kGeneralizedGazeColor[2]);
  }
  // Draw current gaze yaw, pitch
  const Eigen::Vector2f center(
      eyeGazeVisData->yawPitchHistory.back().x(), eyeGazeVisData->yawPitchHistory.back().y());
  pangolin::glDrawVertices(std::vector<Eigen::Vector2f>{center}, GL_POINTS);
  for (double r = .1; r < .3; r += 0.1) {
    pangolin::glDrawCirclePerimeter(center.cast<double>(), r);
  }
}
} // namespace

EyeGazeAriaViewer::EyeGazeAriaViewer(
    const std::shared_ptr<EyeGazeAriaPlayer> eyegazeAriaPlayer,
    int width,
    int height,
    const std::string& name)
    : AriaViewer(width, height, name, eyegazeAriaPlayer),
      generalizedEyeGazesVisData_(eyegazeAriaPlayer->getGeneralizedEyeGazeVisDataPtr()),
      calibratedEyeGazesVisData_(eyegazeAriaPlayer->getCalibratedEyeGazeVisDataPtr()) {}

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
  showGeneralizedGaze_ =
      std::make_unique<pangolin::Var<bool>>(prefix + ".Generalized Eye Gaze", true, true);
  showCalibratedGaze_ =
      std::make_unique<pangolin::Var<bool>>(prefix + ".Calibrated Eye Gaze", true, true);
  depth_ = std::make_unique<pangolin::Var<float>>(prefix + ".Eye Gaze depth(m) ", 0.35, 0.1, 30);
}

void EyeGazeAriaViewer::run() {
  // Main loop
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    updateGuiAndControl();
    // Keep the check box off if we don't have calibrated eye gaze data
    *showCalibratedGaze_ = *showCalibratedGaze_ && calibratedEyeGazesVisData_;

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
    float depth,
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
          eyeGazesVisData->lastYawPitch.x(), eyeGazesVisData->lastYawPitch.y(), depth);
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
        glLineWidth(20);
        if (eyeGazesVisData->calibrated) {
          glColor3f(kCalibratedGazeColor[0], kCalibratedGazeColor[1], kCalibratedGazeColor[2]);
        } else {
          glColor3f(kGeneralizedGazeColor[0], kGeneralizedGazeColor[1], kGeneralizedGazeColor[2]);
        }
        pangolin::glDrawCircle(*gazePointProjected, 2.0);
      }
    }
  }
  v.GetBounds().DisableScissor();
}

} // namespace

void EyeGazeAriaViewer::addEtPlot() {
  // Time Series setup for {yaw, pitch}
  logEyeGaze_ = std::make_shared<pangolin::DataLog>();
  std::vector<std::string> labels = {"Generalized gaze yaw [rad]", "Generalized gaze pitch [rad]"};
  if (calibratedEyeGazesVisData_) {
    labels.emplace_back("Calibrated gaze yaw [rad]");
    labels.emplace_back("Calibrated gaze pitch [rad]");
  }
  logEyeGaze_->SetLabels(labels);
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
      if (*showGeneralizedGaze_) {
        drawEyeGazePoint(
            v,
            *streamIdToPixelFrame_[streamId],
            *showNaturalImageOrientation_,
            dataProvider_,
            ariaVisData_,
            generalizedEyeGazesVisData_,
            *depth_,
            streamId,
            *dataProvider_->getLabelFromStreamId(streamId));
      }
      if (*showCalibratedGaze_ && calibratedEyeGazesVisData_) {
        drawEyeGazePoint(
            v,
            *streamIdToPixelFrame_[streamId],
            *showNaturalImageOrientation_,
            dataProvider_,
            ariaVisData_,
            calibratedEyeGazesVisData_,
            *depth_,
            streamId,
            *dataProvider_->getLabelFromStreamId(streamId));
      }
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
    if (*isPlaying_) {
      const auto gaze = generalizedEyeGazesVisData_->lastYawPitch.cast<float>();
      std::vector<float> values = {gaze.x(), gaze.y()};

      if (calibratedEyeGazesVisData_) {
        const auto calibrated_gaze = calibratedEyeGazesVisData_->lastYawPitch.cast<float>();
        values.push_back(calibrated_gaze.x());
        values.push_back(calibrated_gaze.y());
      }
      logEyeGaze_->Log(values);
    }

    // Add this measurement to the EyeGaze history buffer
    generalizedEyeGazesVisData_->yawPitchHistory.push_back(
        generalizedEyeGazesVisData_->lastYawPitch);
    if (calibratedEyeGazesVisData_) {
      calibratedEyeGazesVisData_->yawPitchHistory.push_back(
          calibratedEyeGazesVisData_->lastYawPitch);
    }
    // If buffer too large remove first element
    // - create a rolling buffer
    if (generalizedEyeGazesVisData_->yawPitchHistory.size() > 10) {
      generalizedEyeGazesVisData_->yawPitchHistory.pop_front();
    }
    if (calibratedEyeGazesVisData_ && calibratedEyeGazesVisData_->yawPitchHistory.size() > 10) {
      calibratedEyeGazesVisData_->yawPitchHistory.pop_front();
    }
  }

  // Draw radar (using history saved data)
  if (!generalizedEyeGazesVisData_->yawPitchHistory.empty() && *showETRadar_) {
    eyeGazeRadar_->Activate(*radar_view_camera_);

    // Draw radar view background
    glColor3f(1, 1., 1.);
    for (double r = .1; r < 2; r += 0.5) {
      pangolin::glDrawCirclePerimeter(Eigen::Vector2d(0, 0), r);
    }

    if (*showGeneralizedGaze_) {
      plotEt(generalizedEyeGazesVisData_);
    }
    if (*showCalibratedGaze_ && calibratedEyeGazesVisData_) {
      plotEt(calibratedEyeGazesVisData_);
    }
  }
}
