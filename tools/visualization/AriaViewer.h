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

#include <pangolin/display/image_view.h>
#include <pangolin/pangolin.h>

#include "AriaPlayer.h"
#include "AriaVisualizationControlAndData.h"

class AriaViewer {
 public:
  AriaViewer(
      int width,
      int height,
      const std::string& windowName,
      std::shared_ptr<AriaPlayer> ariaPlayer);

  virtual ~AriaViewer() = default;

  virtual void init();
  virtual void run();

 protected:
  void obtainStreamIdsFromVrs();
  void updateGuiAndControl();
  void updateImages();
  void updateSensors();
  void update3dView();
  void updateImageVisibility();
  void updateSensorVisibility();

  void cacheVioTrajectory();

  void createWindowWithDisplay(int width, int height);
  void addControlPanel(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);
  // Add toggle buttons for each stream
  void addImageToggle(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);
  void addSensorToggle(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);
  void addOnDeviceMpToggle(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);

  // Add display for each stream
  void addImageDisplays(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);
  void addImuDisplays(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);
  void addMagDisplays(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);
  void addAudioDisplays(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);
  void addBaroDisplays(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);

  // Add 3D display
  void add3dDisplay(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);

 protected:
  int width_;
  int height_;
  std::string windowName_;
  pangolin::View* container_; // Pangolin window
  /// Pangolin variables
  std::unique_ptr<pangolin::Var<bool>> isPlaying_;
  std::unique_ptr<pangolin::Var<float>> timestampSec_, temperatureDisplay_, pressureDisplay_,
      playbackSpeed_;
  std::unique_ptr<pangolin::Var<bool>> showLeftImu_, showRightImu_, showMagnetometer_, showAudio_,
      showNaturalImageOrientation_, showBaroTemp_, showEyeGaze_, showHandPose_, showVioHighFreq_,
      showVio_;
  std::map<vrs::StreamId, std::unique_ptr<pangolin::Var<bool>>> showAllCamImgMap_;

  // Pangolin graphic elements
  std::map<vrs::StreamId, std::unique_ptr<pangolin::ImageView>> streamIdToPixelFrame_;
  std::map<vrs::StreamId, std::shared_ptr<pangolin::DataLog>> streamIdToDataLog_;
  std::map<vrs::StreamId, std::shared_ptr<pangolin::Plotter>> streamIdToPlotters_;

  std::map<vrs::StreamId, std::vector<std::shared_ptr<pangolin::DataLog>>> streamIdToMultiDataLog_;
  std::map<vrs::StreamId, std::vector<std::unique_ptr<pangolin::Plotter>>> streamIdToMultiplotters_;

  // Pangolin 3D view
  pangolin::OpenGlRenderState pangoCamera_;
  std::shared_ptr<pangolin::View> view3d_ = nullptr;
  Sophus::SE3d T_Device_Rgb_; // Used for let pangolin camera follow RGB pose

  std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider_;
  std::optional<projectaria::tools::calibration::DeviceCalibration> maybeDeviceCalib_;
  projectaria::tools::calibration::DeviceVersion deviceVersion_;
  std::shared_ptr<AriaVisualizationData> ariaVisData_;
  std::shared_ptr<AriaVisualizationControl> ariaVisControl_;

  // stream ids for all streams
  // Cameras (only Gen1 ET camera stream id is needed in the class, because we need to apply 90 deg
  // rotation to non-ET cameras in Gen1.)
  std::optional<vrs::StreamId> maybeGen1EyeCameraStreamId_;
  // 1D Sensor data
  std::optional<vrs::StreamId> maybeImuLeftStreamId_;
  std::optional<vrs::StreamId> maybeImuRightStreamId_;
  std::optional<vrs::StreamId> maybeAudioStreamId_;
  std::optional<vrs::StreamId> maybeMagnetometerStreamId_;
  std::optional<vrs::StreamId> maybeBarometerStreamId_;
  // On Device Machine Perception data
  std::optional<vrs::StreamId> maybeEyeGazeStreamId_;
  std::optional<vrs::StreamId> maybeHandPoseStreamId_;
  std::optional<vrs::StreamId> maybeVioHighFreqStreamId_;
  std::optional<vrs::StreamId> maybeVioStreamId_;

  // Cached VIO trajectory for plotting purpose. Stores (timestamp, translation)
  std::vector<std::pair<int64_t, Eigen::Vector3f>> entireVioTrajectoryWithTime_;
  std::vector<Eigen::Vector3f> entireVioTrajectory_;
  std::vector<std::pair<int64_t, Eigen::Vector3d>> entireHighFreqTrajectoryWithTime_;
  std::vector<Eigen::Vector3d> entireHighFreqTrajectory_;
  Sophus::SE3d T_World_Device_; // Used for caching VIO trajectory
};
