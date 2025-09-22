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

#include "AriaViewer.h"
#include "EyeGazeAriaPlayer.h"

#include <pangolin/display/image_view.h>
#include <pangolin/pangolin.h>

class EyeGazeAriaViewer : public AriaViewer {
 public:
  EyeGazeAriaViewer(
      std::shared_ptr<EyeGazeAriaPlayer> ariaPlayer,
      int width,
      int height,
      const std::string& name = "AriaEyeGazeViewer");

  void init() override;
  void run() override;

 private:
  void addEtPlot();
  void updateEtPlot();

 private:
  std::shared_ptr<EyeGazeVisualizationData> generalizedEyeGazesVisData_;
  std::shared_ptr<EyeGazeVisualizationData> calibratedEyeGazesVisData_;

  std::shared_ptr<pangolin::DataLog> logEyeGaze_;

  std::shared_ptr<pangolin::Plotter> eyeGazePlotter_;
  std::shared_ptr<pangolin::View> eyeGazeRadar_;
  std::shared_ptr<pangolin::OpenGlRenderState> radar_view_camera_;

  std::unique_ptr<pangolin::Var<bool>> showETTemporal_, showETRadar_, showGeneralizedGaze_,
      showCalibratedGaze_;
  std::unique_ptr<pangolin::Var<float>> depth_;
};
