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

#include <fmt/core.h>
#include <pangolin/gl/glpixformat.h>
#include <filesystem>

#include <mps/EyeGazeReader.h>
#include "AriaDigitalTwinViewer.h"

namespace projectaria::dataset::adt {

constexpr int kWindowSizeX{1100};
constexpr int kWindowSizeY{1280};
constexpr double kJointsPointSize{3.0};
constexpr int kRenderEveryNFrames{3}; // setting this to 3 means we are rendering at 10fps

constexpr double kEyeGazeMinDepth{0.1}; // minimum eye gaze depth to render is 10 cm
constexpr double kEyeGazeCrossScale{0.05}; // defined as a fraction of the image width
constexpr double kEyeGazeCircleScale{0.0075}; // defined as a fraction of the image width
constexpr bool kOutputRenderTime{false};

AriaDigitalTwinViewer::AriaDigitalTwinViewer(
    const AriaDigitalTwinDataPaths& dataPaths,
    const std::string& renderPath)
    : renderPath_(renderPath) {
  loadData(dataPaths);
  setupPangolin();
}

void AriaDigitalTwinViewer::loadData(const AriaDigitalTwinDataPaths& dataPaths) {
  fmt::print("Loading vrs and ground truth data...\n");
  adtDataProvider_ = std::make_unique<AriaDigitalTwinDataProvider>(dataPaths);

  // get overlapping time
  tsNsRgb_ = adtDataProvider_->getAriaDeviceCaptureTimestampsNs(rgbStreamId_);

  // get start time:
  tsNsOverlapStartIter_ =
      std::lower_bound(tsNsRgb_.begin(), tsNsRgb_.end(), adtDataProvider_->getStartTimeNs());

  // get end time and count
  numberOfFrames_ = 0;
  tsNsOverlapEndIter_ = std::prev(tsNsRgb_.end());
  for (auto iter = tsNsOverlapStartIter_; iter != tsNsRgb_.end(); iter++) {
    numberOfFrames_++;
    if (*iter >= adtDataProvider_->getEndTimeNs()) {
      tsNsOverlapEndIter_ = std::prev(iter);
      break;
    }
  }

  tsNsRgbIter_ = tsNsOverlapStartIter_;

  // see if segmentation, depth images and synthetic video are available
  containsSegmentationImages_ = adtDataProvider_->hasSegmentationImages();
  containsDepthImages_ = adtDataProvider_->hasDepthImages();
  containsSyntheticImages_ = adtDataProvider_->hasSyntheticImages();
}

void AriaDigitalTwinViewer::setupPangolin() {
  if (renderPath_.empty()) {
    pangolin::CreateWindowAndBind("AriaDigitalTwinViewer", kWindowSizeX, kWindowSizeY);
  } else {
    fmt::print("Render path set, rending images and setting pangolin to headless\n");
    pangolin::CreateWindowAndBind(
        "AriaDigitalTwinViewer",
        kWindowSizeX,
        kWindowSizeY,
        pangolin::Params({{"scheme", "headless"}}));
  }

  container_ = &(pangolin::CreateDisplay());
  container_->SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0);
  container_->SetLayout(pangolin::LayoutEqual);

  // add raw image views
  int containerCounter{0};
  container_->AddDisplay(cameraRgbView_);
  cameraRgbView_.SetThetaQuarterTurn(1);
  rgbContainerIds_.push_back(containerCounter++);
  container_->AddDisplay(cameraSlamLeftView_);
  cameraSlamLeftView_.SetThetaQuarterTurn(1);
  slamLContainerIds_.push_back(containerCounter++);
  container_->AddDisplay(cameraSlamRightView_);
  cameraSlamRightView_.SetThetaQuarterTurn(1);
  slamRContainerIds_.push_back(containerCounter++);

  // add segmentation views if available
  if (containsSegmentationImages_) {
    cameraRgbSegView_.SetThetaQuarterTurn(1);
    cameraSlamLeftSegView_.SetThetaQuarterTurn(1);
    cameraSlamRightSegView_.SetThetaQuarterTurn(1);
    container_->AddDisplay(cameraRgbSegView_);
    segContainerIds_.push_back(containerCounter);
    rgbContainerIds_.push_back(containerCounter++);
    container_->AddDisplay(cameraSlamLeftSegView_);
    segContainerIds_.push_back(containerCounter);
    slamLContainerIds_.push_back(containerCounter++);
    container_->AddDisplay(cameraSlamRightSegView_);
    segContainerIds_.push_back(containerCounter);
    slamRContainerIds_.push_back(containerCounter++);
  }

  // add depth views if available
  if (containsDepthImages_) {
    cameraRgbDepthView_.SetThetaQuarterTurn(1);
    cameraSlamLeftDepthView_.SetThetaQuarterTurn(1);
    cameraSlamRightDepthView_.SetThetaQuarterTurn(1);
    container_->AddDisplay(cameraRgbDepthView_);
    depContainerIds_.push_back(containerCounter);
    rgbContainerIds_.push_back(containerCounter++);
    container_->AddDisplay(cameraSlamLeftDepthView_);
    depContainerIds_.push_back(containerCounter);
    slamLContainerIds_.push_back(containerCounter++);
    container_->AddDisplay(cameraSlamRightDepthView_);
    depContainerIds_.push_back(containerCounter);
    slamRContainerIds_.push_back(containerCounter++);
  }

  // add synthetic views if available
  if (containsSyntheticImages_) {
    cameraRgbSyntheticView_.SetThetaQuarterTurn(1);
    cameraSlamLeftSyntheticView_.SetThetaQuarterTurn(1);
    cameraSlamRightSyntheticView_.SetThetaQuarterTurn(1);
    container_->AddDisplay(cameraRgbSyntheticView_);
    synContainerIds_.push_back(containerCounter);
    rgbContainerIds_.push_back(containerCounter++);
    container_->AddDisplay(cameraSlamLeftSyntheticView_);
    synContainerIds_.push_back(containerCounter);
    slamLContainerIds_.push_back(containerCounter++);
    container_->AddDisplay(cameraSlamRightSyntheticView_);
    synContainerIds_.push_back(containerCounter);
    slamRContainerIds_.push_back(containerCounter++);
  }
  pangolin::CreatePanel(kViewerPrefix_).SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

  // init variables
  frameBar_ =
      std::make_unique<pangolin::Var<int>>(kViewerPrefix_ + ".Frame", 0, 0, numberOfFrames_ - 1);
}

void AriaDigitalTwinViewer::run() {
  if (!renderPath_.empty() && !std::filesystem::exists(renderPath_)) {
    fmt::print("ERROR: Render path does not exist: {}\n", renderPath_);
    fmt::print("Exiting...\n");
    exit(1);
  }

  bool isFirstView = true; // always run the viz the first time to populate display
  while (!pangolin::ShouldQuit()) {
    timer_.reset();
    // check if we should update the view
    bool updateViz = isPlaying_ && tsNsRgbIter_ != tsNsOverlapEndIter_;
    if (checkUserInput()) {
      updateViz = true;
    }
    if (isFirstView) {
      updateViz = true;
      isFirstView = false;
    }
    if (*tsNsRgbIter_ > *tsNsOverlapEndIter_ || *tsNsRgbIter_ < *tsNsOverlapStartIter_) {
      tsNsRgbIter_ = tsNsOverlapStartIter_;
      *frameBar_ = 0;
    }
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    {
      if (updateViz) {
        *frameBar_ = frameBar_->Get() + 1;
        int64_t tsQuery = *tsNsRgbIter_;
        auto imgRgbWithDt = adtDataProvider_->getAriaImageByTimestampNs(tsQuery, rgbStreamId_);
        auto imageDataWithDtSlamL =
            adtDataProvider_->getAriaImageByTimestampNs(tsQuery, slamLeftStreamId_);
        auto imageDataWithDtSlamR =
            adtDataProvider_->getAriaImageByTimestampNs(tsQuery, slamRightStreamId_);

        if (!imgRgbWithDt.isValid() || !imageDataWithDtSlamL.isValid() ||
            !imageDataWithDtSlamR.isValid()) {
          fmt::print("Cannot retrieve image with query timestamp: {}\n", tsQuery);
          tsNsRgbIter_++;
          continue;
        }

        drawRawImage(*imgRgbWithDt.data().pixelFrame, cameraRgbView_, tsQuery, rgbStreamId_);
        drawRawImage(
            *imageDataWithDtSlamL.data().pixelFrame,
            cameraSlamLeftView_,
            tsQuery,
            slamLeftStreamId_);
        drawRawImage(
            *imageDataWithDtSlamR.data().pixelFrame,
            cameraSlamRightView_,
            tsQuery,
            slamRightStreamId_);
        drawSegmentationImages(tsQuery);
        drawDepthImages(tsQuery);
        drawSyntheticImages(tsQuery);
        ++tsNsRgbIter_;
      }

      if (!renderPath_.empty()) {
        isPlaying_ = true;
        int count = frameBar_->Get();
        if (count % kRenderEveryNFrames == 0) {
          fmt::print("Rendering frame {}/{}\n", count, numberOfFrames_);
          std::filesystem::path savePath =
              std::filesystem::path(renderPath_) / std::filesystem::path(std::to_string(count));
          container_->SaveOnRender(savePath.string());
        }

        if (count >= numberOfFrames_ || tsNsRgbIter_ == tsNsOverlapEndIter_) {
          fmt::print("Done rendering frames, quitting viewer\n");
          exit(1);
        }
      }

      updateDisplay();
    }
    if (kOutputRenderTime) {
      fmt::print("Total render time: {} Ms\n", timer_.elapsedInMs());
      fmt::print("Render rate: {} Hz\n", timer_.elapsedInHz());
    }
  }

  fmt::print("Quit viewer\n");
  exit(1);
}

void AriaDigitalTwinViewer::drawRawImage(
    vrs::utils::PixelFrame& image,
    pangolin::ImageView& view,
    int64_t tNs,
    const vrs::StreamId& streamId) {
  uint32_t w = image.getWidth();
  uint32_t h = image.getHeight();
  bool isRgb = &view == &cameraRgbView_;
  view.SetImage(
      static_cast<void*>(image.getBuffer().data()),
      w,
      h,
      isRgb ? image.getWidth() * 3 : image.getWidth(),
      isRgb ? pangolin::PixelFormatFromString("RGB24") : pangolin::PixelFormatFromString("GRAY8"));
  view.SetAspect(static_cast<float>(image.getHeight()) / static_cast<float>(image.getWidth()));

  float transX1 = static_cast<float>(view.GetDefaultView().x.Size()) / 2 - 0.5;
  float transY1 = static_cast<float>(view.GetDefaultView().y.Size()) / 2 - 0.5;
  float rot = 90 * view.GetThetaQuarterTurn();
  float transX2 = -static_cast<float>(image.getWidth()) / 2 + 0.5;
  float transY2 = -static_cast<float>(image.getHeight()) / 2 + 0.5;

  // markup images with GT data
  view.extern_draw_function = [=](pangolin::View& v) {
    v.Activate();

    // rotate 90 before drawing
    glTranslatef(transX1, transY1, 0);
    glRotatef(rot, 0, 0, 1);
    glTranslatef(transX2, transY2, 0);

    // draw 2d bounding boxes
    if (show2dBbox_ && adtDataProvider_->hasInstance2dBoundingBoxes()) {
      BoundingBox2dDataWithDt objectBboxes2d =
          adtDataProvider_->getObject2dBoundingBoxesByTimestampNs(tNs, streamId);
      if (objectBboxes2d.isValid()) {
        glColor4f(0, 1, 0, 1);
        for (const auto& [objId, boxData] : objectBboxes2d.data()) {
          std::vector<Eigen::Vector2d> boxLines = bbox2dToImageLineCoordinates(boxData.boxRange);
          pangolin::glDrawLineStrip(boxLines);
        }
        glColor4f(1, 0, 0, 1);
      }
    }

    if (showSkeleton2dBbox_ && adtDataProvider_->hasInstance2dBoundingBoxes() &&
        adtDataProvider_->hasSkeleton()) {
      BoundingBox2dDataWithDt Skeletonbboxes2d =
          adtDataProvider_->getSkeleton2dBoundingBoxesByTimestampNs(tNs, streamId);
      if (Skeletonbboxes2d.isValid()) {
        glColor4f(1, 0, 0, 1);
        for (const auto& [objId, boxData] : Skeletonbboxes2d.data()) {
          std::vector<Eigen::Vector2d> boxLines = bbox2dToImageLineCoordinates(boxData.boxRange);
          pangolin::glDrawLineStrip(boxLines);
        }
        glColor4f(1, 0, 0, 1);
      }
    }

    const auto maybeCamModel = adtDataProvider_->getAriaCameraCalibration(streamId);
    if (!maybeCamModel.has_value()) {
      throw std::runtime_error{"camera not found"};
    }
    const projectaria::tools::calibration::CameraCalibration& camModel = maybeCamModel.value();

    // draw 3d bounding boxes
    if (show3dBbox_ && adtDataProvider_->hasObject3dBoundingboxes()) {
      glColor4f(1, 0, 1, 1);
      std::vector<std::vector<Eigen::Vector2d>> projectedLines =
          get3dBboxProjections(tNs, camModel);
      for (const auto& line : projectedLines) {
        pangolin::glDrawLineStrip(line);
      }
      glColor4f(1, 0, 0, 1);
    }

    // draw skeletons
    if (showSkeletons_ && adtDataProvider_->hasSkeleton()) {
      std::vector<std::vector<Eigen::Vector2d>> connections;
      std::vector<Eigen::Vector2d> joints;
      getSkeletonsProjections(tNs, camModel, connections, joints);
      glColor4f(1, 0, 0, 1);
      for (const auto& line : connections) {
        pangolin::glDrawLineStrip(line);
      }
      glColor4f(1, 1, 1, 1);
      glPointSize(kJointsPointSize);
      pangolin::glDrawPoints(joints);
      glPointSize(1.0);
      glColor4f(1, 0, 0, 1);
    }

    // draw eye gaze (closest sample)
    if (showEyeGaze_ && adtDataProvider_->hasEyeGaze()) {
      EyeGazeWithDt eyeGazeWithDt =
          adtDataProvider_->getEyeGazeByTimestampNs(tNs, TimeQueryOptions::Closest);
      if (eyeGazeWithDt.isValid() && eyeGazeWithDt.data().depth >= kEyeGazeMinDepth) {
        glColor4f(1, 0, 0, 1);

        // Calculate the "center of gaze" in camera frame
        const auto& eyeGaze = eyeGazeWithDt.data();
        const Eigen::Vector3d gazeCenterInCpf = Eigen::Vector3d(
            tan(eyeGaze.yaw) * eyeGaze.depth, tan(eyeGaze.pitch) * eyeGaze.depth, eyeGaze.depth);
        const auto maybeT_Cpf_Camera =
            adtDataProvider_->rawDataProviderPtr()->getDeviceCalibration()->getT_Cpf_Sensor(
                camModel.getLabel(), true);

        if (!maybeT_Cpf_Camera.has_value()) {
          fmt::print("WARNING: T_Cpf_camera not found for {}\n", camModel.getLabel());
          return;
        }

        const auto gazeCenterInCameraFrame = maybeT_Cpf_Camera->inverse() * gazeCenterInCpf;
        const auto maybeGazeCenterInPixels = camModel.project(gazeCenterInCameraFrame);

        if (!maybeGazeCenterInPixels.has_value()) {
          return;
        }
        pangolin::glDrawCross(maybeGazeCenterInPixels.value(), kEyeGazeCrossScale * w);
        pangolin::glDrawCircle(maybeGazeCenterInPixels.value(), kEyeGazeCircleScale * w);
      }
    }
  };
}

void AriaDigitalTwinViewer::drawSegmentationImages(int64_t tNs) {
  if (!containsSegmentationImages_ || !showSegmentations_) {
    return;
  }
  auto rgbSegmentationsWithDt =
      adtDataProvider_->getSegmentationImageByTimestampNs(tNs, rgbStreamId_);
  auto slamLeftSegmentationsWithDt =
      adtDataProvider_->getSegmentationImageByTimestampNs(tNs, slamLeftStreamId_);
  auto slamRightSegmentationsWithDt =
      adtDataProvider_->getSegmentationImageByTimestampNs(tNs, slamRightStreamId_);
  if (!rgbSegmentationsWithDt.isValid() || !slamLeftSegmentationsWithDt.isValid() ||
      !slamRightSegmentationsWithDt.isValid()) {
    return;
  }
  std::vector<std::pair<
      std::reference_wrapper<SegmentationDataWithDt>,
      std::reference_wrapper<pangolin::ImageView>>>
      segmentationFrames = {
          std::make_pair(std::ref(rgbSegmentationsWithDt), std::ref(cameraRgbSegView_)),
          std::make_pair(std::ref(slamLeftSegmentationsWithDt), std::ref(cameraSlamLeftSegView_)),
          std::make_pair(std::ref(slamRightSegmentationsWithDt), std::ref(cameraSlamRightSegView_)),
      };

  for (const auto& [imageRef, viewRef] : segmentationFrames) {
    if (imageRef.get().data().isValid()) {
      viewRef.get().SetImage(
          imageRef.get().data().getVisualizable().data(),
          imageRef.get().data().getWidth(),
          imageRef.get().data().getHeight(),
          imageRef.get().data().getWidth() * 3,
          pangolin::PixelFormatFromString("RGB24"));
      viewRef.get().SetAspect(
          static_cast<float>(imageRef.get().data().getHeight()) /
          static_cast<float>(imageRef.get().data().getWidth()));
    }
  }
}

void AriaDigitalTwinViewer::drawDepthImages(int64_t tNs) {
  if (!containsDepthImages_ || !showDepthImages_) {
    return;
  }

  auto rgbDepthWithDt = adtDataProvider_->getDepthImageByTimestampNs(tNs, rgbStreamId_);
  auto slamLeftDepthWithDt = adtDataProvider_->getDepthImageByTimestampNs(tNs, slamLeftStreamId_);
  auto slamRightDepthWithDt = adtDataProvider_->getDepthImageByTimestampNs(tNs, slamRightStreamId_);
  if (!rgbDepthWithDt.isValid() || !slamLeftDepthWithDt.isValid() ||
      !slamRightDepthWithDt.isValid()) {
    return;
  }
  std::vector<std::pair<
      std::reference_wrapper<DepthDataWithDt>,
      std::reference_wrapper<pangolin::ImageView>>>
      depthFrames{
          std::make_pair(std::ref(rgbDepthWithDt), std::ref(cameraRgbDepthView_)),
          std::make_pair(std::ref(slamLeftDepthWithDt), std::ref(cameraSlamLeftDepthView_)),
          std::make_pair(std::ref(slamRightDepthWithDt), std::ref(cameraSlamRightDepthView_)),
      };

  for (const auto& [imageRef, viewRef] : depthFrames) {
    if (imageRef.get().data().isValid()) {
      viewRef.get().SetImage(
          imageRef.get().data().getVisualizable().data(),
          imageRef.get().data().getWidth(),
          imageRef.get().data().getHeight(),
          imageRef.get().data().getWidth(),
          pangolin::PixelFormatFromString("GRAY8"));
      viewRef.get().SetAspect(
          static_cast<float>(imageRef.get().data().getHeight()) /
          static_cast<float>(imageRef.get().data().getWidth()));
    }
  }
}

void AriaDigitalTwinViewer::drawSyntheticImages(int64_t tNs) {
  if (!containsSyntheticImages_ || !showSyntheticImages_) {
    return;
  }

  auto rgbSyntheticWithDt = adtDataProvider_->getSyntheticImageByTimestampNs(tNs, rgbStreamId_);
  auto slamLeftSyntheticWithDt =
      adtDataProvider_->getSyntheticImageByTimestampNs(tNs, slamLeftStreamId_);
  auto slamRightSyntheticWithDt =
      adtDataProvider_->getSyntheticImageByTimestampNs(tNs, slamRightStreamId_);
  if (!rgbSyntheticWithDt.isValid() || !slamLeftSyntheticWithDt.isValid() ||
      !slamRightSyntheticWithDt.isValid()) {
    return;
  }
  std::vector<std::pair<
      std::reference_wrapper<SyntheticDataWithDt>,
      std::reference_wrapper<pangolin::ImageView>>>
      syntheticFrames{
          std::make_pair(std::ref(rgbSyntheticWithDt), std::ref(cameraRgbSyntheticView_)),
          std::make_pair(std::ref(slamLeftSyntheticWithDt), std::ref(cameraSlamLeftSyntheticView_)),
          std::make_pair(
              std::ref(slamRightSyntheticWithDt), std::ref(cameraSlamRightSyntheticView_)),
      };

  for (const auto& [imageRef, viewRef] : syntheticFrames) {
    const bool rgb = &viewRef.get() == &cameraRgbSyntheticView_;
    if (imageRef.get().isValid()) {
      void* data = nullptr;
      if (rgb) {
        data =
            std::get<projectaria::tools::image::Image3U8>(imageRef.get().data().getVisualizable())
                .data();
      } else {
        data = std::get<projectaria::tools::image::ImageU8>(imageRef.get().data().getVisualizable())
                   .data();
      }
      viewRef.get().SetImage(
          data,
          imageRef.get().data().getWidth(),
          imageRef.get().data().getHeight(),
          rgb ? imageRef.get().data().getWidth() * 3 : imageRef.get().data().getWidth(),
          rgb ? pangolin::PixelFormatFromString("RGB24")
              : pangolin::PixelFormatFromString("GRAY8"));
      viewRef.get().SetAspect(
          static_cast<float>(imageRef.get().data().getHeight()) /
          static_cast<float>(imageRef.get().data().getWidth()));
    }
  }
}

void AriaDigitalTwinViewer::updateDisplay() {
  for (int i : rgbContainerIds_) {
    (*container_)[i].Show(showRgbCamImg_);
  }
  for (int i : slamLContainerIds_) {
    (*container_)[i].Show(showLeftSlamImg_);
  }
  for (int i : slamRContainerIds_) {
    (*container_)[i].Show(showRightSlamImg_);
  }
  if (!showSegmentations_) {
    for (int i : segContainerIds_) {
      (*container_)[i].Show(showSegmentations_);
    }
  }
  if (!showDepthImages_) {
    for (int i : depContainerIds_) {
      (*container_)[i].Show(showDepthImages_);
    }
  }
  if (!showSyntheticImages_) {
    for (int i : synContainerIds_) {
      (*container_)[i].Show(showSyntheticImages_);
    }
  }
  pangolin::FinishFrame();
}

bool AriaDigitalTwinViewer::checkUserInput() {
  if (nextFrame_.GuiChanged()) {
    // no need to increment because it'll already happen
    nextFrame_ = false;
    return true;
  }

  if (prevFrame_.GuiChanged()) {
    tsNsRgbIter_--;
    tsNsRgbIter_--;
    *frameBar_ = frameBar_->Get() - 2;
    prevFrame_ = false;
    return true;
  }

  if (frameBar_->GuiChanged()) {
    fmt::print("Setting current frame to {}\n", frameBar_->Get());
    tsNsRgbIter_ = tsNsOverlapStartIter_;
    std::advance(tsNsRgbIter_, frameBar_->Get());
    return true;
  }
  return false;
}

std::vector<std::vector<Eigen::Vector2d>> AriaDigitalTwinViewer::get3dBboxProjections(
    int64_t tNs,
    const projectaria::tools::calibration::CameraCalibration& camModel) const {
  std::vector<std::vector<Eigen::Vector2d>> projectedLines;

  const auto aria3dPoseWithDt = getInterpolatedAria3dPoseAtTimestampNs(*adtDataProvider_, tNs);
  if (!aria3dPoseWithDt.isValid()) {
    return projectedLines;
  }

  const auto& T_Scene_Device = aria3dPoseWithDt.data().T_Scene_Device;

  const auto& object3dBoundingBoxesGt =
      getInterpolatedObject3dBoundingBoxesAtTimestampNs(*adtDataProvider_, tNs);

  if (!object3dBoundingBoxesGt.isValid()) {
    return projectedLines;
  }

  const auto& object3dBoundingBoxes = object3dBoundingBoxesGt.data();

  Sophus::SE3d T_Cam_Scene = camModel.getT_Device_Camera().inverse() * T_Scene_Device.inverse();
  for (const auto& [objId, object3dBoundingBox] : object3dBoundingBoxes) {
    std::optional<Vector6d> aabb = object3dBoundingBox.aabb;
    if (!aabb.has_value()) {
      continue;
    }
    const auto& T_Scene_Object = object3dBoundingBox.T_Scene_Object;
    Sophus::SE3d T_Cam_Obj = T_Cam_Scene * T_Scene_Object;
    std::vector<Eigen::Vector3d> pointsInObj = bbox3dToCoordinates(aabb.value());
    std::vector<Eigen::Vector2d> bboxCoordsProjected;
    for (const Eigen::Vector3d& pInObj : pointsInObj) {
      // transform point into camera frame
      Eigen::Vector3d pInCam = T_Cam_Obj * pInObj;

      std::optional<Eigen::Vector2d> maybeProjected = camModel.project(pInCam);
      if (!maybeProjected) {
        break;
      }

      bboxCoordsProjected.push_back(maybeProjected.value());
    }

    // only plot bboxes that are fully visible (i.e., 8 successful projections)
    if (bboxCoordsProjected.size() != 8) {
      continue;
    }

    // convert to drawable lines (8 -> 16) by adding lines between top and bottom
    std::vector<Eigen::Vector2d> bboxLines{
        bboxCoordsProjected[0], /* b1 */
        bboxCoordsProjected[1], /* b2 */
        bboxCoordsProjected[2], /* b3 */
        bboxCoordsProjected[3], /* b4 */
        bboxCoordsProjected[0], /* b1 */
        bboxCoordsProjected[4], /* t1 */
        bboxCoordsProjected[5], /* t2 */
        bboxCoordsProjected[6], /* t3 */
        bboxCoordsProjected[7], /* t4 */
        bboxCoordsProjected[4], /* t1 */
        bboxCoordsProjected[5], /* t2 */
        bboxCoordsProjected[1], /* b2 */
        bboxCoordsProjected[2], /* b3 */
        bboxCoordsProjected[6], /* t3 */
        bboxCoordsProjected[7], /* t4 */
        bboxCoordsProjected[3], /* b4 */
    };
    projectedLines.push_back(bboxLines);
  }

  return projectedLines;
}

void AriaDigitalTwinViewer::getSkeletonsProjections(
    int64_t tNs,
    const projectaria::tools::calibration::CameraCalibration& camModel,
    std::vector<std::vector<Eigen::Vector2d>>& connections,
    std::vector<Eigen::Vector2d>& joints) const {
  connections.clear();
  joints.clear();

  const auto aria3dPoseWithDt = getInterpolatedAria3dPoseAtTimestampNs(*adtDataProvider_, tNs);
  if (!aria3dPoseWithDt.isValid()) {
    return;
  }

  const auto& T_Scene_Device = aria3dPoseWithDt.data().T_Scene_Device;

  Sophus::SE3d T_Cam_Scene = camModel.getT_Device_Camera().inverse() * T_Scene_Device.inverse();

  for (const auto& id : adtDataProvider_->getSkeletonIds()) {
    const auto& skeletonData = adtDataProvider_->getSkeletonProvider(id);
    SkeletonFrameWithDt skeletonFrame = skeletonData.getSkeletonByTimestampNs(tNs);
    if (!skeletonFrame.isValid()) {
      continue;
    }

    // project joints to camera & store joint ID -> pixels for drawing connections
    std::unordered_map<int, Eigen::Vector2d> jointIdToPixel;
    for (int ptId = 0; ptId < skeletonFrame.data().joints.size(); ptId++) {
      const Eigen::Vector3d& jointInScene = skeletonFrame.data().joints.at(ptId);
      Eigen::Vector3d pInCam = T_Cam_Scene * jointInScene;

      std::optional<Eigen::Vector2d> maybeProjected = camModel.project(pInCam);

      // check if in image plane
      if (!maybeProjected) {
        continue;
      }

      // skip all joints that are part of hand
      const std::string& jointLabel = AriaDigitalTwinSkeletonProvider::getJointLabels().at(ptId);
      if (jointLabel.find("Thumb") != std::string::npos ||
          jointLabel.find("Index") != std::string::npos ||
          jointLabel.find("Middle") != std::string::npos ||
          jointLabel.find("Ring") != std::string::npos ||
          jointLabel.find("Pinky") != std::string::npos) {
        continue;
      }

      joints.push_back(maybeProjected.value());
      jointIdToPixel.emplace(ptId, maybeProjected.value());
    }

    // get all connections
    for (const auto& [jointId1, jointId2] :
         AriaDigitalTwinSkeletonProvider::getJointConnections()) {
      auto joint1Iter = jointIdToPixel.find(jointId1);
      auto joint2Iter = jointIdToPixel.find(jointId2);
      if (joint1Iter == jointIdToPixel.end() || joint2Iter == jointIdToPixel.end()) {
        continue;
      }
      connections.push_back({joint1Iter->second, joint2Iter->second});
    }
  }
}

} // namespace projectaria::dataset::adt
