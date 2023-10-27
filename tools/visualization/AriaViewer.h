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
      const std::shared_ptr<AriaPlayer> ariaPlayer);

  virtual ~AriaViewer() = default;

  virtual void init();
  virtual void run();

 protected:
  void updateGuiAndControl();
  void updateImages();
  void updateSensors();
  void updateImageVisibility();
  void updateSensorVisibility();

  void createWindowWithDisplay(int width, int height);
  void addControlPanel(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);
  void addImageToggle(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);
  void addSensorToggle(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider);
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

 protected:
  int width_;
  int height_;
  std::string windowName_;
  pangolin::View* container_; // Pangolin window
  /// Pangolin variables
  std::unique_ptr<pangolin::Var<bool>> isPlaying_;
  std::unique_ptr<pangolin::Var<float>> timestampSec_, temperatureDisplay_, pressureDisplay_,
      playbackSpeed_;
  std::unique_ptr<pangolin::Var<bool>> showLeftCamImg_, showRightCamImg_, showRgbCamImg_,
      showEyeImg_, showLeftImu_, showRightImu_, showMagnetometer_, showAudio_,
      showNaturalImageOrientation_, showBaroTemp_;

  // Pangolin graphic elements
  std::map<vrs::StreamId, std::unique_ptr<pangolin::ImageView>> streamIdToPixelFrame_;
  std::map<vrs::StreamId, std::shared_ptr<pangolin::DataLog>> streamIdToDataLog_;
  std::map<vrs::StreamId, std::shared_ptr<pangolin::Plotter>> streamIdToPlotters_;

  std::map<vrs::StreamId, std::vector<std::shared_ptr<pangolin::DataLog>>> streamIdToMultiDataLog_;
  std::map<vrs::StreamId, std::vector<std::unique_ptr<pangolin::Plotter>>> streamIdToMultiplotters_;

  std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider_;
  std::shared_ptr<AriaVisualizationData> ariaVisData_;
  std::shared_ptr<AriaVisualizationControl> ariaVisControl_;
};
