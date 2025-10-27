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

#include <pangolin/gl/glpixformat.h>
#include "PangolinColor.h"
#include "PlottingHelper.h"
#include "SensorDataSequence.h"
#include "image/ImagePangolinFormat.h"
#include "vrs/RecordFormat.h"

#include <algorithm>
#include <iostream>

// font defined in pangolin
extern const unsigned char AnonymousPro_ttf[];

using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::calibration;
using namespace projectaria::tools::image;
using namespace projectaria::tools::mps;
using namespace projectaria::tools::viz;

namespace {
// A helper function to check if a streamID is SLAM or RGB image.
inline bool checkStreamIdIsSlamOrRgb(const vrs::StreamId& streamId) {
  return (streamId.getTypeId() == vrs::RecordableTypeId::SlamCameraData) ||
      (streamId.getTypeId() == vrs::RecordableTypeId::RgbCameraRecordableClass);
}

} // namespace

AriaViewer::AriaViewer(
    int width,
    int height,
    const std::string& windowName,
    const std::shared_ptr<AriaPlayer> ariaPlayer)
    : width_(width),
      height_(height),
      windowName_(windowName),
      dataProvider_(ariaPlayer->getDataProviderPtr()),
      maybeDeviceCalib_(dataProvider_->getDeviceCalibration()),
      ariaVisData_(ariaPlayer->getVisDataPtr()),
      ariaVisControl_(ariaPlayer->getVisControlPtr()) {
  if (maybeDeviceCalib_.has_value()) {
    T_Device_Rgb_ = maybeDeviceCalib_->getT_Device_Sensor("camera-rgb").value_or(Sophus::SE3d());
  }
  deviceVersion_ = dataProvider_->getDeviceVersion();
}

void AriaViewer::init() {
  obtainStreamIdsFromVrs();

  createWindowWithDisplay(width_, height_);
  addControlPanel(dataProvider_);

  addImageToggle(dataProvider_);
  addSensorToggle(dataProvider_);
  addOnDeviceMpToggle(dataProvider_);

  addImageDisplays(dataProvider_);
  addImuDisplays(dataProvider_);
  addMagDisplays(dataProvider_);
  addAudioDisplays(dataProvider_);
  addBaroDisplays(dataProvider_);

  add3dDisplay(dataProvider_);
  cacheVioTrajectory();

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
    update3dView();

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

void AriaViewer::obtainStreamIdsFromVrs() {
  maybeGen1EyeCameraStreamId_ = dataProvider_->getStreamIdFromLabel("camera-et");
  maybeImuLeftStreamId_ = dataProvider_->getStreamIdFromLabel("imu-left");
  maybeImuRightStreamId_ = dataProvider_->getStreamIdFromLabel("imu-right");
  maybeAudioStreamId_ = dataProvider_->getStreamIdFromLabel("mic");
  maybeMagnetometerStreamId_ = dataProvider_->getStreamIdFromLabel("mag0");
  maybeBarometerStreamId_ = dataProvider_->getStreamIdFromLabel("baro0");

  maybeEyeGazeStreamId_ = dataProvider_->getStreamIdFromLabel("eyegaze");
  maybeHandPoseStreamId_ = dataProvider_->getStreamIdFromLabel("handtracking");
  maybeVioHighFreqStreamId_ = dataProvider_->getStreamIdFromLabel("vio_high_frequency");
  maybeVioStreamId_ = dataProvider_->getStreamIdFromLabel("vio");
}

void AriaViewer::createWindowWithDisplay(int width, int height) {
  pangolin::CreateWindowAndBind(windowName_, width, height);

  container_ = &pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(228), 1.0)
                    .SetLayout(pangolin::LayoutEqual);
}

void AriaViewer::addControlPanel(std::shared_ptr<VrsDataProvider> dataProvider) {
  // prefix to give each viewer its own set of controls (otherwise they are
  // shared if multiple viewers are opened)
  const std::string prefix = "ui";
  pangolin::CreatePanel(prefix).SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(228));
  // Settings
  isPlaying_ =
      std::make_unique<pangolin::Var<bool>>(prefix + ".Play", ariaVisControl_->isPlaying_, true);

  // add timestamp slider
  const float startTime =
      static_cast<float>(dataProvider->getFirstTimeNsAllStreams(TimeDomain::DeviceTime)) * 1e-9;
  const float endTime =
      static_cast<float>(dataProvider->getLastTimeNsAllStreams(TimeDomain::DeviceTime)) * 1e-9;
  timestampSec_ = std::make_unique<pangolin::Var<float>>(
      prefix + ".timestamp(s)", startTime, startTime, endTime);
  playbackSpeed_ =
      std::make_unique<pangolin::Var<float>>(prefix + ".playback speed", 1.0, 0.01, 10.0);

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

  // Rotate 90 button only for Gen1
  if (deviceVersion_ == DeviceVersion::Gen1) {
    showNaturalImageOrientation_ =
        std::make_unique<pangolin::Var<bool>>(prefix + ".RotateRawImg", false, true);
  }

  const auto maybeDeviceCalib = dataProvider->getDeviceCalibration();
  if (!maybeDeviceCalib.has_value()) {
    fmt::print("WARNING: Cannot obtain valid device calibration, no image toggles are added \n");
    return;
  }
  const auto deviceVersion = maybeDeviceCalib->getDeviceVersion();

  std::vector<std::string> allCameraLabels;

  if (deviceVersion == DeviceVersion::Gen1) {
    allCameraLabels = {"camera-rgb", "camera-slam-left", "camera-slam-right", "camera-et"};
  } else if (deviceVersion == DeviceVersion::Gen2) {
    allCameraLabels = {
        "camera-rgb",
        "slam-front-left",
        "slam-front-right",
        "slam-side-left",
        "slam-side-right",
        "camera-et-left",
        "camera-et-right"};
  }

  for (const auto& camLabel : allCameraLabels) {
    auto maybeStreamId = dataProvider->getStreamIdFromLabel(camLabel);
    if (!maybeStreamId.has_value()) {
      continue;
    }
    showAllCamImgMap_.emplace(
        maybeStreamId.value(),
        std::make_unique<pangolin::Var<bool>>(
            prefix + fmt::format(".{}", camLabel),
            dataProvider->checkStreamIsActive(maybeStreamId.value()),
            true));
  }
}

void AriaViewer::addSensorToggle(std::shared_ptr<VrsDataProvider> dataProvider) {
  // On/Off for display elements
  const std::string prefix = "ui";

  if (maybeImuLeftStreamId_.has_value()) {
    showLeftImu_ = std::make_unique<pangolin::Var<bool>>(
        prefix + ".LeftImu",
        dataProvider->checkStreamIsActive(maybeImuLeftStreamId_.value()),
        true);
  }
  if (maybeImuRightStreamId_.has_value()) {
    showRightImu_ = std::make_unique<pangolin::Var<bool>>(
        prefix + ".RightImu",
        dataProvider->checkStreamIsActive(maybeImuRightStreamId_.value()),
        true);
  }
  if (maybeMagnetometerStreamId_.has_value()) {
    showMagnetometer_ = std::make_unique<pangolin::Var<bool>>(
        prefix + ".Magnetometer",
        dataProvider->checkStreamIsActive(maybeMagnetometerStreamId_.value()),
        true);
  }

  if (maybeBarometerStreamId_.has_value()) {
    showBaroTemp_ = std::make_unique<pangolin::Var<bool>>(
        prefix + ".Baro_Temp",
        dataProvider->checkStreamIsActive(maybeBarometerStreamId_.value()),
        true);
  }

  if (maybeAudioStreamId_.has_value()) {
    showAudio_ = std::make_unique<pangolin::Var<bool>>(
        prefix + ".Audio", dataProvider->checkStreamIsActive(maybeAudioStreamId_.value()), true);
  }
}

void AriaViewer::addOnDeviceMpToggle(std::shared_ptr<VrsDataProvider> dataProvider) {
  const std::string prefix = "ui";

  // Add toggle for eye gaze
  if (maybeEyeGazeStreamId_.has_value()) {
    showEyeGaze_ = std::make_unique<pangolin::Var<bool>>(
        prefix + ".eyegaze",
        dataProvider->checkStreamIsActive(maybeEyeGazeStreamId_.value()),
        true);
  }
  // Add toggle for hand pose
  if (maybeHandPoseStreamId_.has_value()) {
    showHandPose_ = std::make_unique<pangolin::Var<bool>>(
        prefix + ".hand_pose",
        dataProvider->checkStreamIsActive(maybeHandPoseStreamId_.value()),
        true);
  }

  // Add toggle for vio high freq pose
  if (maybeVioHighFreqStreamId_.has_value()) {
    showVioHighFreq_ = std::make_unique<pangolin::Var<bool>>(
        prefix + ".vio_high_frequency",
        dataProvider->checkStreamIsActive(maybeVioHighFreqStreamId_.value()),
        true);
  }

  // Add toggle for vio pose
  if (maybeVioStreamId_.has_value()) {
    showVio_ = std::make_unique<pangolin::Var<bool>>(
        prefix + ".vio", dataProvider->checkStreamIsActive(maybeVioStreamId_.value()), true);
  }
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
  // Skpping if magnetomter stream is not available
  if (!maybeMagnetometerStreamId_.has_value()) {
    return;
  }
  const auto streamId = maybeMagnetometerStreamId_.value();

  streamIdToDataLog_[streamId] = std::make_shared<pangolin::DataLog>();
  streamIdToDataLog_.at(streamId)->SetLabels({"mag_x [uT]", "mag_y [uT]", "mag_z [uT]"});
  streamIdToPlotters_[streamId] = std::make_shared<pangolin::Plotter>(
      streamIdToDataLog_.at(streamId).get(), 0.0f, 75.f, -100., 100., 100, 1.);
  streamIdToPlotters_.at(streamId)->Track("$i");

  container_->AddDisplay(*streamIdToPlotters_.at(streamId));
}

void AriaViewer::addImuDisplays(std::shared_ptr<VrsDataProvider> dataProvider) {
  // Configure IMU streams display
  static const std::array<std::string, 2> imuLabels = {"imu-left", "imu-right"};
  for (const auto& label : imuLabels) {
    auto maybeStreamId = dataProvider->getStreamIdFromLabel(label);

    // Skpping if stream is not available
    if (!maybeStreamId.has_value()) {
      continue;
    }
    const auto streamId = maybeStreamId.value();

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
  // Skip if audio stream is not available
  if (!maybeAudioStreamId_.has_value()) {
    return;
  }
  const auto streamId = maybeAudioStreamId_.value();

  // Configure Audio stream display
  int numChannels = 0;
  if (dataProvider->checkStreamIsActive(streamId)) {
    numChannels = dataProvider->getAudioConfiguration(streamId).numChannels;
  }
  streamIdToDataLog_.emplace(streamId, std::make_shared<pangolin::DataLog>());
  if (numChannels == 2) {
    streamIdToDataLog_.at(streamId)->SetLabels({"m0", "m1"});
  } else if (numChannels == 7) {
    streamIdToDataLog_.at(streamId)->SetLabels({"m0", "m1", "m2", "m3", "m4", "m5", "m6"});
  }

  streamIdToPlotters_[streamId] = std::make_shared<pangolin::Plotter>(
      streamIdToDataLog_.at(streamId).get(), 0.0f, 3 * 48000.f, -5e-2, 5e-2, 10000, 1e-3f);
  streamIdToPlotters_[streamId]->Track("$i");
  container_->AddDisplay(*streamIdToPlotters_.at(streamId));
}

void AriaViewer::addBaroDisplays(std::shared_ptr<VrsDataProvider> dataProvider) {
  // Skip if barometer stream is not available
  if (!maybeBarometerStreamId_.has_value()) {
    return;
  }
  const auto streamId = maybeBarometerStreamId_.value();

  // Configure Temperature and Pressure display
  const std::string prefix = "ui";
  temperatureDisplay_ =
      std::make_unique<pangolin::Var<float>>(prefix + ".temp [C]", 0., 0.0, 0., false);
  pressureDisplay_ =
      std::make_unique<pangolin::Var<float>>(prefix + ".pres [kPa]", 0., 0.0, 0., false);

  // setup line plots
  streamIdToMultiDataLog_[streamId] = std::vector<std::shared_ptr<pangolin::DataLog>>(2, nullptr);
  streamIdToMultiDataLog_.at(streamId).at(0) = std::make_shared<pangolin::DataLog>();
  streamIdToMultiDataLog_.at(streamId).at(0)->SetLabels({"Pressure [kPa]"});
  streamIdToMultiDataLog_.at(streamId).at(1) = std::make_shared<pangolin::DataLog>();
  streamIdToMultiDataLog_.at(streamId).at(1)->SetLabels({"Temperature [C]"});

  streamIdToMultiplotters_.emplace(streamId, std::vector<std::unique_ptr<pangolin::Plotter>>{});

  auto& logs = streamIdToMultiDataLog_.at(streamId);
  streamIdToMultiplotters_.at(streamId).emplace_back(
      std::make_unique<pangolin::Plotter>(logs.at(0).get(), 0.0f, 1500.f, 99., 102., 100, 5.));
  streamIdToMultiplotters_.at(streamId).emplace_back(
      std::make_unique<pangolin::Plotter>(logs.at(1).get(), 0.0f, 1500.f, 20., 30., 100, 1.));

  for (const auto& plotter : streamIdToMultiplotters_.at(streamId)) {
    plotter->Track("$i");
    container_->AddDisplay(*plotter);
  }
}

void AriaViewer::add3dDisplay(std::shared_ptr<VrsDataProvider> dataProvider) {
  // Only add 3D view if VIO MP stream is valid
  if (maybeVioHighFreqStreamId_.has_value()) {
    pangoCamera_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrixRDF_TopLeft(640, 480, 420, 420, 320, 240, 0.05, 200),
        pangolin::ModelViewLookAtRDF(-0.03, 0, 0.0, -0.00046, 0, -0.00521, 0, 0, 1));
    view3d_ = std::make_unique<pangolin::View>();
    view3d_->SetAspect(640.0 / 480.0).SetHandler(new pangolin::Handler3D(pangoCamera_));

    container_->AddDisplay(*view3d_);
  }
}

void AriaViewer::cacheVioTrajectory() {
  // Cache the entire VIO trajectory for plotting purpose
  if (maybeVioStreamId_.has_value()) {
    // Clear up cache,
    entireVioTrajectory_.clear();
    entireVioTrajectoryWithTime_.clear();

    size_t numData = dataProvider_->getNumData(maybeVioStreamId_.value());
    entireVioTrajectoryWithTime_.reserve(numData);
    entireVioTrajectory_.reserve(numData);

    for (int i = 0; i < numData; i++) {
      const auto vioData = dataProvider_->getVioDataByIndex(maybeVioStreamId_.value(), i);
      const auto T_Odometry_Device = vioData.T_Odometry_BodyImu * vioData.T_BodyImu_Device;
      entireVioTrajectoryWithTime_.emplace_back(
          vioData.captureTimestampNs, T_Odometry_Device.translation());
      entireVioTrajectory_.push_back(T_Odometry_Device.translation());
    }
  }

  // Also cache VIO high_freq trajectory for plotting purpose
  if (maybeVioHighFreqStreamId_.has_value()) {
    // Clear up cache,
    entireHighFreqTrajectory_.clear();
    entireHighFreqTrajectoryWithTime_.clear();

    size_t numData = dataProvider_->getNumData(maybeVioHighFreqStreamId_.value());
    entireHighFreqTrajectory_.reserve(numData / 10);
    entireHighFreqTrajectoryWithTime_.reserve(numData / 10);

    // subsample from 800 Hz -> 80Hz
    for (int i = 0; i < dataProvider_->getNumData(maybeVioHighFreqStreamId_.value()); i += 10) {
      const auto vioHighFreqData =
          dataProvider_->getVioHighFreqDataByIndex(maybeVioHighFreqStreamId_.value(), i);
      entireHighFreqTrajectoryWithTime_.emplace_back(
          vioHighFreqData.trackingTimestamp.count(),
          vioHighFreqData.T_odometry_device.translation());
      entireHighFreqTrajectory_.push_back(vioHighFreqData.T_odometry_device.translation());
    }
  }
}

void AriaViewer::updateGuiAndControl() {
  // update control values
  if (isPlaying_->GuiChanged()) { // triggers playing
    ariaVisControl_->isPlaying_ = *isPlaying_;
  } else { // may toggle off when playing to the end
    *isPlaying_ = ariaVisControl_->isPlaying_;
  }

  if (timestampSec_->GuiChanged() || playbackSpeed_->GuiChanged()) { // triggers random access mode
    ariaVisControl_->playbackSpeed_ = *playbackSpeed_;

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
    for (const auto& itVec : streamIdToMultiplotters_) {
      for (const auto& it : itVec.second) {
        it->ResetView();
      }
    }
  }

  if (*isPlaying_) { // playing mode
    // update timestamp slider according to playing progress
    *timestampSec_ = static_cast<double>(ariaVisControl_->timestampNs_) * 1e-9;
  } else { // random access mode
    // update control timestamp by GUI
    ariaVisControl_->timestampNs_ = static_cast<int64_t>(*timestampSec_ * 1e9);
  }

  if (showNaturalImageOrientation_ && showNaturalImageOrientation_->GuiChanged()) {
    for (const auto& [streamId, imageView] : streamIdToPixelFrame_) {
      if (!dataProvider_->checkStreamIsActive(streamId)) {
        continue;
      }
      // Only apply 90 deg rotation to Gen1, non-Eyetracking cameras
      if (deviceVersion_ == DeviceVersion::Gen1 && *showNaturalImageOrientation_ &&
          maybeGen1EyeCameraStreamId_.has_value() &&
          streamId != maybeGen1EyeCameraStreamId_.value()) {
        imageView->SetThetaQuarterTurn(1);
      } else {
        imageView->SetThetaQuarterTurn(0);
      }
    }
  }
}

void AriaViewer::updateImages() {
  for (const auto& [streamId, imageView] : streamIdToPixelFrame_) {
    //
    // Step 1, plot camera image
    //
    if (!ariaVisData_->isDataChanged(streamId) ||
        ariaVisData_->cameraImageBufferMap_.count(streamId) == 0) {
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

    // Only apply 90 deg rotation to Gen1, non-Eyetracking cameras
    if (deviceVersion_ == DeviceVersion::Gen1 && *showNaturalImageOrientation_ &&
        maybeGen1EyeCameraStreamId_.has_value() &&
        streamId != maybeGen1EyeCameraStreamId_.value()) {
      imageView->SetAspect(static_cast<float>(height) / static_cast<float>(width));
    } else {
      imageView->SetAspect(static_cast<float>(width) / static_cast<float>(height));
    }

    // unset data updated flag to avoid redundant pangolin::SetImage() call
    ariaVisData_->setDataChanged(false, streamId);

    //
    // Step 2, plot On-device MP data as overlay in SLAM / RGB cameras
    //
    if (!checkStreamIdIsSlamOrRgb(streamId)) {
      continue;
    }

    // Ensure Device Calib exists
    if (!maybeDeviceCalib_.has_value()) {
      fmt::print(
          "WARNING: Device calibration does not exist for this VRS. On Device MP data cannot be properly plotted in 2D images. Skipping");
      return;
    }

    const std::string camLabel = dataProvider_->getLabelFromStreamId(streamId).value();
    const auto maybeT_Cpf_Camera = maybeDeviceCalib_->getT_Cpf_Sensor(camLabel);
    const auto camCalib = maybeDeviceCalib_->getCameraCalib(camLabel).value();

    imageView->extern_draw_function = [=, this](pangolin::View& v) {
      v.Activate();

      // 1. Draw eye gaze, if stream is valid, pangolin's show button is turned on, and eyegaze data
      // is valid
      if (maybeEyeGazeStreamId_.has_value() && (*showEyeGaze_) &&
          ariaVisData_->isDataChanged(maybeEyeGazeStreamId_.value()) &&
          ariaVisData_->eyeGazeData_.has_value()) {
        plotProjectedEyeGaze(
            ariaVisData_->eyeGazeData_.value(), camCalib, maybeT_Cpf_Camera.value(), camLabel);
      }

      // 2. Draw hand pose, if stream is valid, pangolin's show button is turned on, and hand data
      // is valid
      if (maybeHandPoseStreamId_.has_value() && (*showHandPose_) &&
          ariaVisData_->isDataChanged(maybeHandPoseStreamId_.value()) &&
          ariaVisData_->handPoseData_.has_value()) {
        plotProjectedHandPose(ariaVisData_->handPoseData_.value(), camCalib, camLabel);
      }
    }; // end of imageView->extern_draw_function
  } // end of for streamId
}

void AriaViewer::updateSensors() {
  // Update mag data
  if (maybeMagnetometerStreamId_.has_value() &&
      ariaVisData_->isDataChanged(maybeMagnetometerStreamId_.value())) {
    const auto& streamId = maybeMagnetometerStreamId_.value();
    for (const auto& data : ariaVisData_->magMicroTesla_.at(streamId)) {
      streamIdToDataLog_.at(streamId)->Log({data.begin(), data.end()});
    }
    ariaVisData_->clearData(streamId);
    ariaVisData_->setDataChanged(false, streamId);
  }

  // Update IMU right data
  if (maybeImuRightStreamId_.has_value() &&
      ariaVisData_->isDataChanged(maybeImuRightStreamId_.value())) {
    const auto& streamId = maybeImuRightStreamId_.value();
    for (const auto& data : ariaVisData_->accMSec2Map_.at(streamId)) {
      streamIdToMultiDataLog_.at(streamId).at(0)->Log({data.begin(), data.end()});
    }

    for (const auto& data : ariaVisData_->gyroRadSecMap_.at(streamId)) {
      streamIdToMultiDataLog_.at(streamId).at(1)->Log({data.begin(), data.end()});
    }

    ariaVisData_->clearData(streamId);
    ariaVisData_->setDataChanged(false, streamId);
  }

  if (maybeImuLeftStreamId_.has_value() &&
      ariaVisData_->isDataChanged(maybeImuLeftStreamId_.value())) {
    const auto& streamId = maybeImuLeftStreamId_.value();
    for (const auto& data : ariaVisData_->accMSec2Map_.at(streamId)) {
      streamIdToMultiDataLog_.at(streamId).at(0)->Log({data.begin(), data.end()});
    }

    for (const auto& data : ariaVisData_->gyroRadSecMap_.at(streamId)) {
      streamIdToMultiDataLog_.at(streamId).at(1)->Log({data.begin(), data.end()});
    }
    ariaVisData_->clearData(streamId);
    ariaVisData_->setDataChanged(false, streamId);
  }

  if (maybeBarometerStreamId_.has_value() &&
      ariaVisData_->isDataChanged(maybeBarometerStreamId_.value())) {
    const auto& streamId = maybeBarometerStreamId_.value();
    streamIdToMultiDataLog_.at(streamId).at(0)->Log(ariaVisData_->pressure_);
    streamIdToMultiDataLog_.at(streamId).at(1)->Log(ariaVisData_->temperature_);
    *pressureDisplay_ = ariaVisData_->pressure_.back(); // kPa
    *temperatureDisplay_ = ariaVisData_->temperature_.back(); // C
    ariaVisData_->clearData(streamId);
    ariaVisData_->setDataChanged(false, streamId);
  }

  if (maybeAudioStreamId_.has_value() && ariaVisData_->isDataChanged(maybeAudioStreamId_.value())) {
    const auto& streamId = maybeAudioStreamId_.value();
    if (!ariaVisData_->audio_.empty()) {
      for (const auto& audio : ariaVisData_->audio_) {
        streamIdToDataLog_.at(streamId)->Log(audio);
      }
      ariaVisData_->setDataChanged(false, streamId);
    }
  }
}

void AriaViewer::update3dView() {
  // Skip if 3D view is not available
  if (!view3d_) {
    return;
  }

  // Sophus::SE3d T_World_Device = Sophus::SE3d();

  /************* Step 1.1: Draw VIO Trajectory and Pose **************/
  if (maybeVioStreamId_.has_value() && ariaVisData_->isDataChanged(maybeVioStreamId_.value()) &&
      ariaVisData_->vioData_.has_value() && ariaVisData_->vioData_->status == VioStatus::VALID &&
      ariaVisData_->vioData_->poseQuality == TrackingQuality::GOOD) {
    const auto vioData = ariaVisData_->vioData_.value();

    // Update T_World_Device_ from VIO data, and update the pangolin camera to follow the pose
    T_World_Device_ = (vioData.T_Odometry_BodyImu * vioData.T_BodyImu_Device).cast<double>();
    Sophus::SE3f T_World_Rgb = (T_World_Device_ * T_Device_Rgb_).cast<float>();
    pangoCamera_.Follow(T_World_Rgb.matrix(), true);

    // Note that view needs to be activated after Follow() is called
    view3d_->Activate(pangoCamera_);
    glEnable(GL_DEPTH_TEST);

    // Always plot the entire trajectory in dark color
    setPlotColor("dark_gray", 0.1);
    glLineWidth(1.0);
    pangolin::glDrawLineStrip(entireVioTrajectory_);

    // Then plot the partial trajectory in solid color
    size_t previousTrajIndex = 0;
    auto itor = std::upper_bound(
        entireVioTrajectoryWithTime_.begin(),
        entireVioTrajectoryWithTime_.end(),
        vioData.captureTimestampNs,
        [](int64_t targetTime, const std::pair<int64_t, Eigen::Vector3f>& timeAndLocation) {
          return targetTime < timeAndLocation.first;
        });
    if (itor != entireVioTrajectoryWithTime_.begin()) {
      itor--;
      previousTrajIndex = std::distance(entireVioTrajectoryWithTime_.begin(), itor);

      setPlotColor("light_sky_blue");
      glLineWidth(3.0);
      pangolin::glDrawVertices(previousTrajIndex + 1, entireVioTrajectory_.data(), GL_LINE_STRIP);
      std::vector<Eigen::Vector3d> lastSegment = {
          entireVioTrajectory_.at(previousTrajIndex).cast<double>(), T_World_Device_.translation()};
      pangolin::glDrawLineStrip(lastSegment);
    }
  }

  /************* Step 1.2: Draw VIO HighFreq Trajectory **************/
  if (maybeVioHighFreqStreamId_.has_value() &&
      ariaVisData_->isDataChanged(maybeVioHighFreqStreamId_.value()) &&
      ariaVisData_->vioHighFreqData_.has_value()) {
    const auto vioHighFreqData = ariaVisData_->vioHighFreqData_.value();
    Sophus::SE3d T_World_DeviceHighFreq = vioHighFreqData.T_odometry_device;

    // Just plot the partial trajectory in solid color for high freq traj
    size_t previousTrajIndex = 0;
    auto itor = std::upper_bound(
        entireHighFreqTrajectoryWithTime_.begin(),
        entireHighFreqTrajectoryWithTime_.end(),
        vioHighFreqData.trackingTimestamp.count(),
        [](int64_t targetTime, const std::pair<int64_t, Eigen::Vector3d>& timeAndLocation) {
          return targetTime < timeAndLocation.first;
        });
    if (itor != entireHighFreqTrajectoryWithTime_.begin()) {
      itor--;
      previousTrajIndex = std::distance(entireHighFreqTrajectoryWithTime_.begin(), itor);

      setPlotColor("lemon_yellow");
      glLineWidth(3.0);
      pangolin::glDrawVertices(
          previousTrajIndex + 1, entireHighFreqTrajectory_.data(), GL_LINE_STRIP);
      pangolin::glDrawLineStrip(
          std::vector<Eigen::Vector3d>(
              {entireHighFreqTrajectory_.at(previousTrajIndex),
               T_World_DeviceHighFreq.translation()}));
    }
  }

  /************* Step 2, plot Aria glass outline *************/
  glLineWidth(1.0);
  plotAriaGlassOutline(maybeDeviceCalib_.value(), T_World_Device_);

  /************** Step 3, plot eyegaze vector **********/
  if (maybeEyeGazeStreamId_.has_value() && (*showEyeGaze_) &&
      ariaVisData_->isDataChanged(maybeEyeGazeStreamId_.value()) &&
      ariaVisData_->eyeGazeData_.has_value()) {
    plotEyeGazeIn3dView(
        ariaVisData_->eyeGazeData_.value(), T_World_Device_ * maybeDeviceCalib_->getT_Device_Cpf());
  }

  /************** Step 4, plot hand poses  ***************/
  if (maybeHandPoseStreamId_.has_value() && (*showHandPose_) &&
      ariaVisData_->isDataChanged(maybeHandPoseStreamId_.value()) &&
      ariaVisData_->handPoseData_.has_value()) {
    plotHandPoseIn3dView(ariaVisData_->handPoseData_.value(), T_World_Device_);
  }
  glDisable(GL_DEPTH_TEST);
}

void AriaViewer::updateImageVisibility() {
  for (const auto& [streamId, imageView] : streamIdToPixelFrame_) {
    imageView->Show(*(showAllCamImgMap_.at(streamId)));
  }
}

void AriaViewer::updateSensorVisibility() {
  if (maybeAudioStreamId_.has_value()) {
    streamIdToPlotters_.at(maybeAudioStreamId_.value())->Show(*showAudio_);
  }

  if (maybeMagnetometerStreamId_.has_value()) {
    streamIdToPlotters_.at(maybeMagnetometerStreamId_.value())->Show(*showMagnetometer_);
  }

  if (maybeImuLeftStreamId_.has_value()) {
    for (const auto& plotter : streamIdToMultiplotters_.at(maybeImuLeftStreamId_.value())) {
      plotter->Show(*showLeftImu_);
    }
  }
  if (maybeImuRightStreamId_.has_value()) {
    for (const auto& plotter : streamIdToMultiplotters_.at(maybeImuRightStreamId_.value())) {
      plotter->Show(*showRightImu_);
    }
  }
  if (maybeBarometerStreamId_.has_value()) {
    for (const auto& plotter : streamIdToMultiplotters_.at(maybeBarometerStreamId_.value())) {
      plotter->Show(*showBaroTemp_);
    }
  }
}
