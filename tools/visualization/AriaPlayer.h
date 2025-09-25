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

#include "AriaVisualizationControlAndData.h"

class AriaPlayer {
 public:
  AriaPlayer(
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider,
      std::shared_ptr<AriaVisualizationData> visData,
      std::shared_ptr<AriaVisualizationControl> visControl);
  virtual ~AriaPlayer() = default;

  void run();

  std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> getDataProviderPtr() {
    return dataProvider_;
  }
  std::shared_ptr<AriaVisualizationData> getVisDataPtr() {
    return visData_;
  }
  std::shared_ptr<AriaVisualizationControl> getVisControlPtr() {
    return visControl_;
  }

 protected:
  virtual void playFromTimeNsMultiThread(int64_t timestampNs);
  virtual void updateImagesStatic(int64_t timestampNs);

  void playStreamFromTimeNs(int64_t timestampNs, const vrs::StreamId& streamId);

 protected:
  std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider_;
  std::shared_ptr<AriaVisualizationData> visData_;
  std::shared_ptr<AriaVisualizationControl> visControl_;
  int64_t lastTimestampNs_ = -1;
};

std::shared_ptr<AriaPlayer> createAriaPlayer(
    const std::string& vrsPath,
    const std::vector<vrs::StreamId>& imageStreamIds,
    const std::vector<vrs::StreamId>& imuStreamIds,
    const std::vector<vrs::StreamId>& dataStreamIds);
