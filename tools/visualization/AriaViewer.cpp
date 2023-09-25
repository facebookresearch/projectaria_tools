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

#include "AriaViewer.h"

#include <pangolin/display/image_view.h>
#include <pangolin/gl/glpixformat.h>
#include <pangolin/pangolin.h>

#include "AriaStreamIds.h"
#include "SensorDataSequence.h"
#include "image/ImagePangolinFormat.h"
#include "vrs/RecordFormat.h"

#include <iostream>

// font defined in pangolin
extern const unsigned char AnonymousPro_ttf[];

using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::image;

AriaViewer::AriaViewer(
    int width,
    int height,
    const std::string& windowName,
    const std::shared_ptr<AriaPlayer> ariaPlayer)
    : width_(width),
      height_(height),
      windowName_(windowName),
      dataProvider_(ariaPlayer->getDataProviderPtr()),
      ariaVisData_(ariaPlayer->getVisDataPtr()),
      ariaVisControl_(ariaPlayer->getVisControlPtr()) {}

void AriaViewer::init() {
  createWindowWithDisplay(width_, height_);
  addControlPanel(dataProvider_);
  addImageToggle(dataProvider_);
  addSensorToggle(dataProvider_);

  addImageDisplays(dataProvider_);
  addImuDisplays(dataProvider_);
  addMagDisplays(dataProvider_);
  addAudioDisplays(dataProvider_);
  addBaroDisplays(dataProvider_);

  pangolin::GetBoundWindow()->RemoveCurrent();
}

void AriaViewer::run() {
  pangolin::BindToContext(windowName_);
  // Main loop
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    updateGuiAndControl();
    updateImages();
    updateSensors();

    // Handling conditional plotting
    updateImageVisibility();
    updateSensorVisibility();
    pangolin::FinishFrame();
  }
  pangolin::GetBoundWindow()->RemoveCurrent();
  pangolin::DestroyWindow(windowName_);

  ariaVisControl_->shouldClose_ = true;
  std::cout << "\nQuit Viewer." << std::endl;
}

void AriaViewer::createWindowWithDisplay(int width, int height) {
  pangolin::CreateWindowAndBind(windowName_, width, height);

  container_ = &pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0)
                    .SetLayout(pangolin::LayoutEqual);
}

void AriaViewer::addControlPanel(std::shared_ptr<VrsDataProvider> dataProvider) {
  // prefix to give each viewer its own set of controls (otherwise they are
  // shared if multiple viewers are opened)
  const std::string prefix = "ui";
  pangolin::CreatePanel(prefix).SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(228));
  // Settings
  isPlaying_ = std::make_unique<pangolin::Var<bool>>(
      prefix + ".Play", ariaVisControl_.get()->isPlaying_, true);

  // add timestamp slider
  const float startTime =
      static_cast<float>(dataProvider->getFirstTimeNsAllStreams(TimeDomain::DeviceTime)) * 1e-9;
  const float endTime =
      static_cast<float>(dataProvider->getLastTimeNsAllStreams(TimeDomain::DeviceTime)) * 1e-9;
  timestampSec_ = std::make_unique<pangolin::Var<float>>(
      prefix + ".timestamp(s)", startTime, startTime, endTime);

  pangolin::Var<std::function<void(void)>> save_window(prefix + ".Snapshot UI", [&]() {
    std::ostringstream filename;
    filename << "snapshot_" << ariaVisControl_->timestampNs_ << "_ns";
#if (PANGOLIN_VERSION_MAJOR == 0) && (PANGOLIN_VERSION_MINOR >= 7)
    pangolin::SaveWindowOnRender(filename.str(), container_->v);
#else
    pangolin::SaveWindowOnRender(filename.str());
#endif
    std::cout << "Saving snapshot to file:" << filename.str() << std::endl;
  });
}

void AriaViewer::addImageToggle(std::shared_ptr<VrsDataProvider> dataProvider) {
  // On/Off for display elements
  const std::string prefix = "ui";

  showNaturalImageOrientation_ =
      std::make_unique<pangolin::Var<bool>>(prefix + ".RotateRawImg", false, true);
  showLeftCamImg_ = std::make_unique<pangolin::Var<bool>>(
      prefix + ".LeftImg", dataProvider->checkStreamIsActive(kSlamLeftCameraStreamId), true);
  showRightCamImg_ = std::make_unique<pangolin::Var<bool>>(
      prefix + ".RightImg", dataProvider->checkStreamIsActive(kSlamRightCameraStreamId), true);
  showRgbCamImg_ = std::make_unique<pangolin::Var<bool>>(
      prefix + ".RgbImg", dataProvider->checkStreamIsActive(kRgbCameraStreamId), true);
  showEyeImg_ = std::make_unique<pangolin::Var<bool>>(
      prefix + ".EyeImg", dataProvider->checkStreamIsActive(kEyeCameraStreamId), true);
}

void AriaViewer::addSensorToggle(std::shared_ptr<VrsDataProvider> dataProvider) {
  // On/Off for display elements
  const std::string prefix = "ui";

  showLeftImu_ = std::make_unique<pangolin::Var<bool>>(
      prefix + ".LeftImu", dataProvider->checkStreamIsActive(kImuLeftStreamId), true);
  showRightImu_ = std::make_unique<pangolin::Var<bool>>(
      prefix + ".RightImu", dataProvider->checkStreamIsActive(kImuRightStreamId), true);
  showMagnetometer_ = std::make_unique<pangolin::Var<bool>>(
      prefix + ".Magnetometer", dataProvider->checkStreamIsActive(kMagnetometerStreamId), true);
  showAudio_ = std::make_unique<pangolin::Var<bool>>(
      prefix + ".Audio", dataProvider->checkStreamIsActive(kAudioStreamId), true);
}

void AriaViewer::addImageDisplays(std::shared_ptr<VrsDataProvider> dataProvider) {
  for (const auto& streamId : ariaVisData_->imageStreamIds_) {
    if (!dataProvider->checkStreamIsActive(streamId)) {
      continue;
    }
    streamIdToPixelFrame_.emplace(streamId, std::make_unique<pangolin::ImageView>());
    container_->AddDisplay(*streamIdToPixelFrame_.at(streamId));
  }
}

void AriaViewer::addMagDisplays(std::shared_ptr<VrsDataProvider> dataProvider) {
  streamIdToDataLog_[kMagnetometerStreamId] = std::make_shared<pangolin::DataLog>();
  streamIdToDataLog_.at(kMagnetometerStreamId)
      ->SetLabels({"mag_x [uT]", "mag_y [uT]", "mag_z [uT]"});
  streamIdToPlotters_[kMagnetometerStreamId] = std::make_shared<pangolin::Plotter>(
      streamIdToDataLog_.at(kMagnetometerStreamId).get(), 0.0f, 75.f, -100., 100., 100, 1.);
  streamIdToPlotters_.at(kMagnetometerStreamId)->Track("$i");
  container_->AddDisplay(*streamIdToPlotters_.at(kMagnetometerStreamId));
}

void AriaViewer::addImuDisplays(std::shared_ptr<VrsDataProvider> dataProvider) {
  // Configure IMU streams display
  static const std::array<std::string, 2> imuLabels = {"imu-left", "imu-right"};
  for (const auto& label : imuLabels) {
    auto streamId = *dataProvider->getStreamIdFromLabel(label);

    streamIdToMultiDataLog_[streamId] = std::vector<std::shared_ptr<pangolin::DataLog>>(2, nullptr);
    streamIdToMultiDataLog_[streamId] = std::vector<std::shared_ptr<pangolin::DataLog>>(2, nullptr);

    streamIdToMultiDataLog_.at(streamId).at(0) = std::make_shared<pangolin::DataLog>();
    streamIdToMultiDataLog_.at(streamId).at(0)->SetLabels(
        {label + "_acc_x [m/s2]", label + "_acc_y [m/s2]", label + "_acc_z [m/s2]"});
    streamIdToMultiDataLog_.at(streamId).at(1) = std::make_shared<pangolin::DataLog>();
    streamIdToMultiDataLog_.at(streamId).at(1)->SetLabels(
        {label + "_gyro_x [rad/s]", label + "_gyro_y [rad/s]", label + "_gyro_z [rad/s]"});

    streamIdToMultiplotters_.emplace(streamId, std::vector<std::unique_ptr<pangolin::Plotter>>{});

    auto& logs = streamIdToMultiDataLog_.at(streamId);
    streamIdToMultiplotters_.at(streamId).emplace_back(
        std::make_unique<pangolin::Plotter>(logs.at(0).get(), 0.0f, 1500.f, -20., 20., 100, 5.));
    streamIdToMultiplotters_.at(streamId).emplace_back(
        std::make_unique<pangolin::Plotter>(logs.at(1).get(), 0.0f, 1500.f, -3.14, 3.14, 100, 1.));

    for (const auto& plotter : streamIdToMultiplotters_.at(streamId)) {
      plotter->Track("$i");
      container_->AddDisplay(*plotter);
    }
  }
}

void AriaViewer::addAudioDisplays(std::shared_ptr<VrsDataProvider> dataProvider) {
  // Configure Audio stream display
  int numChannels = 0;
  if (dataProvider->checkStreamIsActive(kAudioStreamId)) {
    numChannels = dataProvider->getAudioConfiguration(kAudioStreamId).numChannels;
  }
  streamIdToDataLog_.emplace(kAudioStreamId, std::make_shared<pangolin::DataLog>());
  if (numChannels == 2) {
    streamIdToDataLog_.at(kAudioStreamId)->SetLabels({"m0", "m1"});
  } else if (numChannels == 7) {
    streamIdToDataLog_.at(kAudioStreamId)->SetLabels({"m0", "m1", "m2", "m3", "m4", "m5", "m6"});
  }

  streamIdToPlotters_[kAudioStreamId] = std::make_shared<pangolin::Plotter>(
      streamIdToDataLog_.at(kAudioStreamId).get(), 0.0f, 3 * 48000.f, -5e-2, 5e-2, 10000, 1e-3f);
  streamIdToPlotters_[kAudioStreamId]->Track("$i");
  container_->AddDisplay(*streamIdToPlotters_.at(kAudioStreamId));
}

void AriaViewer::addBaroDisplays(std::shared_ptr<VrsDataProvider> dataProvider) {
  // Configure Temperature and Pressure display
  const std::string prefix = "ui";
  temperatureDisplay_ =
      std::make_unique<pangolin::Var<float>>(prefix + ".temp [C]", 0., 0.0, 0., false);
  pressureDisplay_ =
      std::make_unique<pangolin::Var<float>>(prefix + ".pres [kPa]", 0., 0.0, 0., false);
}

void AriaViewer::updateGuiAndControl() {
  // update control values
  if (isPlaying_->GuiChanged()) { // triggers playing
    ariaVisControl_->isPlaying_ = *isPlaying_;
  } else { // may toggle off when playing to the end
    *isPlaying_ = ariaVisControl_->isPlaying_;
  }

  if (timestampSec_->GuiChanged()) { // triggers random access mode
    ariaVisControl_->isPlaying_ = false;
    *isPlaying_ = false;
    for (const auto& it : streamIdToDataLog_) {
      it.second->Clear();
    }
    for (const auto& itVec : streamIdToMultiDataLog_) {
      for (const auto& it : itVec.second) {
        it->Clear();
      }
    }
    for (const auto& it : streamIdToPlotters_) {
      it.second->ResetView();
    }
  }

  if (*isPlaying_) { // playing mode
    // update timestamp slider according to playing progress
    *timestampSec_ = static_cast<double>(ariaVisControl_->timestampNs_) * 1e-9;
  } else { // random access mode
    // update control timestamp by GUI
    ariaVisControl_->timestampNs_ = static_cast<int64_t>(*timestampSec_ * 1e9);
  }

  if (showNaturalImageOrientation_->GuiChanged()) {
    for (const auto& [streamId, imageView] : streamIdToPixelFrame_) {
      if (!dataProvider_->checkStreamIsActive(streamId)) {
        continue;
      }
      if (*showNaturalImageOrientation_ && streamId != kEyeCameraStreamId) {
        imageView->SetThetaQuarterTurn(1);
      } else {
        imageView->SetThetaQuarterTurn(0);
      }
    }
  }
}

void AriaViewer::updateImages() {
  for (const auto& [streamId, imageView] : streamIdToPixelFrame_) {
    if (ariaVisData_->cameraImageBufferMap_.count(streamId) == 0) {
      continue;
    }
    auto& imageData = ariaVisData_->cameraImageBufferMap_.at(streamId);
    if (!imageData.isValid()) {
      continue;
    }

    auto imageVariant = imageData.imageVariant().value();
    int width = getWidth(imageVariant);
    int height = getHeight(imageVariant);
    imageView->SetImage(
        getDataPtr(imageVariant),
        width,
        height,
        getPitch(imageVariant),
        pangolin::PixelFormatFromString(getPangolinFormatString(imageVariant)));

    if (*showNaturalImageOrientation_ && streamId != kEyeCameraStreamId) {
      imageView->SetAspect(static_cast<float>(height) / static_cast<float>(width));
    } else {
      imageView->SetAspect(static_cast<float>(width) / static_cast<float>(height));
    }
  }
}

void AriaViewer::updateSensors() {
  if (ariaVisData_->isDataChanged(kMagnetometerStreamId)) {
    streamIdToDataLog_.at(kMagnetometerStreamId)
        ->Log(ariaVisData_->magMicroTesla_.at(kMagnetometerStreamId));
    ariaVisData_->setDataChanged(false, kMagnetometerStreamId);
  }

  if (ariaVisData_->isDataChanged(kImuRightStreamId)) {
    streamIdToMultiDataLog_.at(kImuRightStreamId)
        .at(0)
        ->Log(ariaVisData_->accMSec2Map_.at(kImuRightStreamId));
    streamIdToMultiDataLog_.at(kImuRightStreamId)
        .at(1)
        ->Log(ariaVisData_->gyroRadSecMap_.at(kImuRightStreamId));
    ariaVisData_->setDataChanged(false, kImuRightStreamId);
  }

  if (ariaVisData_->isDataChanged(kImuLeftStreamId)) {
    streamIdToMultiDataLog_.at(kImuLeftStreamId)
        .at(0)
        ->Log(ariaVisData_->accMSec2Map_.at(kImuLeftStreamId));
    streamIdToMultiDataLog_.at(kImuLeftStreamId)
        .at(1)
        ->Log(ariaVisData_->gyroRadSecMap_.at(kImuLeftStreamId));
    ariaVisData_->setDataChanged(false, kImuLeftStreamId);
  }

  if (ariaVisData_->isDataChanged(kBarometerStreamId)) {
    ariaVisData_->setDataChanged(false, kBarometerStreamId);
    *pressureDisplay_ = ariaVisData_->pressure_.back() * 1e-3; // kPa
    *temperatureDisplay_ = ariaVisData_->temperature_.back(); // C
  }

  if (ariaVisData_->isDataChanged(kAudioStreamId)) {
    if (!ariaVisData_->audio_.empty()) {
      ariaVisData_->setDataChanged(false, kAudioStreamId);
      for (const auto& audio : ariaVisData_->audio_) {
        streamIdToDataLog_.at(kAudioStreamId)->Log(audio);
      }
    }
  }
}

void AriaViewer::updateImageVisibility() {
  for (const auto& [streamId, imageView] : streamIdToPixelFrame_) {
    if (streamId == kRgbCameraStreamId) {
      imageView->Show(*showRgbCamImg_);
    } else if (streamId == kSlamLeftCameraStreamId) {
      imageView->Show(*showLeftCamImg_);
    } else if (streamId == kSlamRightCameraStreamId) {
      imageView->Show(*showRightCamImg_);
    } else if (streamId == kEyeCameraStreamId) {
      imageView->Show(*showEyeImg_);
    }
  }
}

void AriaViewer::updateSensorVisibility() {
  streamIdToPlotters_.at(kAudioStreamId)->Show(*showAudio_);
  streamIdToPlotters_.at(kMagnetometerStreamId)->Show(*showMagnetometer_);
  for (const auto& plotter : streamIdToMultiplotters_.at(kImuLeftStreamId)) {
    plotter->Show(*showLeftImu_);
  }
  for (const auto& plotter : streamIdToMultiplotters_.at(kImuRightStreamId)) {
    plotter->Show(*showRightImu_);
  }
}
