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

#include <data_provider/VrsDataProvider.h>
#include "AriaPlayer.h"
#include "EyeGaze.h"
#include "EyeGazeAriaVisualizationData.h"

class EyeGazeAriaPlayer : public AriaPlayer {
 public:
  EyeGazeAriaPlayer(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider,
      std::shared_ptr<projectaria::tools::mps::EyeGazes> eyeGazes,
      std::shared_ptr<AriaVisualizationData> visData,
      std::shared_ptr<EyeGazeVisualizationData> eyeGazesVisData,
      std::shared_ptr<AriaVisualizationControl> visControl);

  std::shared_ptr<EyeGazeVisualizationData> getEyeGazeVisDataPtr() {
    return eyeGazesVisData_;
  }

 protected:
  void playFromTimeNsMultiThread(int64_t timestampNs) override;
  void updateImagesStatic(int64_t timestampNs) override;
  void playEyetrackingFromTimeNs(int64_t timestampNs);

 private:
  std::shared_ptr<projectaria::tools::mps::EyeGazes> eyeGazes_;
  std::shared_ptr<EyeGazeVisualizationData> eyeGazesVisData_;
};

std::shared_ptr<EyeGazeAriaPlayer> createEyeGazeAriaPlayer(
    const std::string& vrsPath,
    const std::string& eyeGazeRecordsPath,
    const std::vector<vrs::StreamId>& imageStreamIds);
