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

#include "AriaDigitalTwinDataProvider.h"
#include "AriaDigitalTwinUtils.h"

namespace projectaria::dataset::adt {

class AriaDigitalTwinViewer {
 public:
  explicit AriaDigitalTwinViewer(
      const AriaDigitalTwinDataPaths& dataPaths,
      const std::string& renderPath = "");
  ~AriaDigitalTwinViewer() = default;
  void run();

 private:
  void loadData(const AriaDigitalTwinDataPaths& dataPaths);

  void setupPangolin();

  void drawRawImage(
      vrs::utils::PixelFrame& image,
      pangolin::ImageView& view,
      int64_t tNs,
      const vrs::StreamId& streamId);

  void drawSegmentationImages(int64_t tNs);

  void drawDepthImages(int64_t tNs);

  void drawSyntheticImages(int64_t tNs);

  void updateDisplay();

  bool checkUserInput();

  std::vector<std::vector<Eigen::Vector2d>> get3dBboxProjections(
      int64_t tNs,
      const projectaria::tools::calibration::CameraCalibration& camModel) const;

  void getSkeletonsProjections(
      int64_t tNs,
      const projectaria::tools::calibration::CameraCalibration& camModel,
      std::vector<std::vector<Eigen::Vector2d>>& connections,
      std::vector<Eigen::Vector2d>& joints) const;

  std::unique_ptr<AriaDigitalTwinDataProvider> adtDataProvider_;
  const std::string renderPath_;

  const vrs::StreamId rgbStreamId_ = vrs::StreamId::fromNumericName("214-1");
  const vrs::StreamId slamLeftStreamId_ = vrs::StreamId::fromNumericName("1201-1");
  const vrs::StreamId slamRightStreamId_ = vrs::StreamId::fromNumericName("1201-2");
  bool containsSegmentationImages_{true};
  bool containsDepthImages_{true};
  bool containsSyntheticImages_{true};

  // tracking data to display
  std::vector<int64_t> tsNsRgb_;
  std::vector<int64_t>::iterator tsNsRgbIter_;
  std::vector<int64_t>::iterator tsNsOverlapStartIter_;
  std::vector<int64_t>::iterator tsNsOverlapEndIter_;
  int numberOfFrames_;

  // pangolin variables
  pangolin::View* container_;
  pangolin::ImageView cameraRgbView_;
  pangolin::ImageView cameraSlamLeftView_;
  pangolin::ImageView cameraSlamRightView_;
  pangolin::ImageView cameraRgbSegView_;
  pangolin::ImageView cameraSlamLeftSegView_;
  pangolin::ImageView cameraSlamRightSegView_;
  pangolin::ImageView cameraRgbDepthView_;
  pangolin::ImageView cameraSlamLeftDepthView_;
  pangolin::ImageView cameraSlamRightDepthView_;
  pangolin::ImageView cameraRgbSyntheticView_;
  pangolin::ImageView cameraSlamLeftSyntheticView_;
  pangolin::ImageView cameraSlamRightSyntheticView_;
  const std::string kViewerPrefix_{"ui0"};
  pangolin::Var<bool> isPlaying_{pangolin::Var<bool>(kViewerPrefix_ + ".Play", false, true)};
  std::unique_ptr<pangolin::Var<int>>
      frameBar_; // pointer to be initialized when we have number of frames
  pangolin::Var<bool> nextFrame_{pangolin::Var<bool>(kViewerPrefix_ + ".Next", false, false)};
  pangolin::Var<bool> prevFrame_{pangolin::Var<bool>(kViewerPrefix_ + ".Previous", false, false)};
  pangolin::Var<bool> showRgbCamImg_{pangolin::Var<bool>(kViewerPrefix_ + ".RgbImg", true, true)};
  pangolin::Var<bool> showLeftSlamImg_{
      pangolin::Var<bool>(kViewerPrefix_ + ".LeftImg", true, true)};
  pangolin::Var<bool> showRightSlamImg_{
      pangolin::Var<bool>(kViewerPrefix_ + ".RightImg", true, true)};
  pangolin::Var<bool> showSegmentations_{
      pangolin::Var<bool>(kViewerPrefix_ + ".Segmentations", true, true)};
  pangolin::Var<bool> showDepthImages_{
      pangolin::Var<bool>(kViewerPrefix_ + ".DepthImgs", true, true)};
  pangolin::Var<bool> showSyntheticImages_{
      pangolin::Var<bool>(kViewerPrefix_ + ".SyntheticImgs", true, true)};
  pangolin::Var<bool> show2dBbox_{
      pangolin::Var<bool>(kViewerPrefix_ + ".Object 2dBboxes", true, true)};
  pangolin::Var<bool> show3dBbox_{pangolin::Var<bool>(kViewerPrefix_ + ".3dBboxes", true, true)};
  pangolin::Var<bool> showEyeGaze_{pangolin::Var<bool>(kViewerPrefix_ + ".EyeGaze", true, true)};
  pangolin::Var<bool> showSkeletons_{
      pangolin::Var<bool>(kViewerPrefix_ + ".Skeletons", true, true)};
  pangolin::Var<bool> showSkeleton2dBbox_{
      pangolin::Var<bool>(kViewerPrefix_ + ".Skeleton 2dBboxes", true, true)};
  std::vector<int> rgbContainerIds_;
  std::vector<int> segContainerIds_;
  std::vector<int> depContainerIds_;
  std::vector<int> synContainerIds_;
  std::vector<int> slamLContainerIds_;
  std::vector<int> slamRContainerIds_;

  // debugging
  HighResolutionTimer timer_;
};

} // namespace projectaria::dataset::adt
