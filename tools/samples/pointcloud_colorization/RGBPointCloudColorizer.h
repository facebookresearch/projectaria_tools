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

#include "AriaStreamIds.h"
#include "CachedDataProviders.h"
#include "PointCloudColorizer.h"

#include <boost/timer/progress_display.hpp>
#include <unordered_map>

namespace {

// Colorize a GlobalPointCloud given its RGB and SLAM image observations.
//
// This implementation does not target speed, but being used as a tutorial
//
class RGBPointCloudColorizer : public PointCloudColorizer {
 public:
  bool rgbOnly_ = false; // Tell if we are using or rejecting the SLAM images

  // Color accumulator (for each point uid, accumulate normalized RGB color [0.f, 1.f])
  std::unordered_map<uint32_t, Eigen::Vector3f> colorAccumulator_;
  // Accumulator count (Keep track of visited point to know how many pixel observations has been
  // accumulated)
  std::unordered_map<uint32_t, uint32_t> accumulatorCount_;

  // NOLINTNEXTLINE(clang-diagnostic-unused-member-function)
  explicit RGBPointCloudColorizer(
      const projectaria::tools::mps::ClosedLoopTrajectory& trajectory,
      const projectaria::tools::mps::GlobalPointCloud& globalPointCloud,
      const projectaria::tools::mps::PointObservations& pointObservations,
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider,
      bool rgbOnly = false)
      : PointCloudColorizer(trajectory, globalPointCloud, pointObservations, dataProvider),
        rgbOnly_(rgbOnly) {}

  ~RGBPointCloudColorizer() override = default;

  void Colorize() override {
    using namespace projectaria::tools::data_provider;
    using namespace projectaria::tools::image;
    using namespace projectaria::tools::mps;

    colorAccumulator_.clear();
    accumulatorCount_.clear();

    const size_t numRgbData = dataProvider_->getNumData(kRgbCameraStreamId);
    const int64_t startTimestampNs = dataProvider_->getFirstTimeNs(kRgbCameraStreamId);
    const int64_t endTimestampNs = dataProvider_->getLastTimeNs(kRgbCameraStreamId);

    const auto leftCameraSerial =
        dataProvider_->getImageConfiguration(kSlamLeftCameraStreamId).sensorSerial;
    const auto rightCameraSerial =
        dataProvider_->getImageConfiguration(kSlamRightCameraStreamId).sensorSerial;

    // Cache point observations (group by timestamp)
    PointAndObservationProvider pointObsProvider(
        globalPointCloud_,
        pointObservations_,
        leftCameraSerial,
        rightCameraSerial,
        startTimestampNs,
        endTimestampNs);
    std::cout << "Point observations cache:\n"
              << " - #left frames: " << pointObsProvider.numObsFrameLeft() << "\n"
              << " - #right frames: " << pointObsProvider.numObsFrameRight() << "\n"
              << " - #point cache size: " << pointObsProvider.numPointObs() << std::endl;

    TrajectoryProvider trajectoryProvider(trajectory_, startTimestampNs, endTimestampNs);

    // Loop through the frames and color the points cloud
    {
      boost::timer::progress_display progressBar(
          numRgbData, std::cout, "Looping over VRS image frames and PointCloud observations.\n");
      for (int64_t frameId = 0; frameId < numRgbData; ++frameId) {
        ++progressBar;

        //
        // Organize data as [RGB, SLAM LEFT, SLAM RIGHT] arrays and loop over the point observations
        //

        // Get RGB camera image
        std::vector<ImageDataAndRecord> imageDataAndRecord{
            dataProvider_->getImageDataByIndex(kRgbCameraStreamId, frameId)};

        // Get current timestamp
        const int64_t captureTimestampNs = imageDataAndRecord[0].second.captureTimestampNs;

        // Append closest time SLAM images
        imageDataAndRecord.push_back(
            dataProvider_->getImageDataByTimeNs(kSlamLeftCameraStreamId, captureTimestampNs));
        imageDataAndRecord.push_back(
            dataProvider_->getImageDataByTimeNs(kSlamRightCameraStreamId, captureTimestampNs));

        // Get current pose from cached trajectory
        const auto pose = trajectoryProvider.findPose(captureTimestampNs);
        if (!pose) {
          continue;
        }

        const std::array<projectaria::tools::calibration::CameraCalibration, 3> cameras{
            *dataProvider_->getDeviceCalibration()->getCameraCalib("camera-rgb"),
            *dataProvider_->getDeviceCalibration()->getCameraCalib("camera-slam-left"),
            *dataProvider_->getDeviceCalibration()->getCameraCalib("camera-slam-right")};

        std::vector<std::vector<PointObservationPair>> currentFramePointObservations;
        currentFramePointObservations.push_back(
            pointObsProvider.findAllPointObs(leftCameraSerial, captureTimestampNs));
        currentFramePointObservations.push_back(
            pointObsProvider.findAllPointObs(rightCameraSerial, captureTimestampNs));

        // For all observations:
        // - try to re-project 3D point to RGB camera
        //   - if valid reprojection -> sample from RGB image, else sample for SLAM images
        // Note: It is not guaranteed that all 3D point can be projected in RGB images
        for (const auto& observations : currentFramePointObservations) {
          for (const auto& ptIt : observations) {
            const auto ptUid = ptIt.second.uid;

            for (const int index : {0, 1, 2}) // RGB, SLAM LEFT, SLAM RIGHT
            {
              // Use Point Projection
              const auto T_Device_camera = cameras[index].getT_Device_Camera();

              // Retrieve corresponding World Point
              const auto position_world = ptIt.second.position_world;
              const auto T_world_cam = pose->T_world_device * T_Device_camera;
              const auto position_local = T_world_cam.inverse() * position_world;

              const auto projection = cameras[index].project(position_local);
              if (!projection) {
                // 3D point is not visible, continue to next image
                continue;
              }
              Eigen::Vector2f uv = projection->cast<float>();

              // Color sampling
              if (index == 0) { // RGB image
                const Image3U8 image =
                    std::get<Image3U8>(imageDataAndRecord[index].first.imageVariant().value());
                const auto pixValue = image(uv.x(), uv.y());
                if (colorAccumulator_.count(ptUid) == 0) {
                  colorAccumulator_[ptUid] =
                      Eigen::Vector3f(pixValue.x(), pixValue.y(), pixValue.z()) / 255.f;
                  accumulatorCount_[ptUid] = 1;
                } else {
                  colorAccumulator_.at(ptUid) +=
                      Eigen::Vector3f(pixValue.x(), pixValue.y(), pixValue.z()) / 255.f;
                  ++accumulatorCount_.at(ptUid);
                }
              } else { // SLAM image
                const ImageU8 image =
                    std::get<ImageU8>(imageDataAndRecord[index].first.imageVariant().value());
                const float pixValue = static_cast<float>(image(uv.x(), uv.y())) / 255.f;
                const Eigen::Vector3f color{pixValue, pixValue, pixValue};
                if (colorAccumulator_.count(ptUid) == 0) {
                  colorAccumulator_[ptUid] = color;
                  accumulatorCount_[ptUid] = 1;
                } else {
                  colorAccumulator_.at(ptUid) += color;
                  ++accumulatorCount_.at(ptUid);
                }
              }

              break; // We already sampled the point color, no need to check another image
            }
          }
        }
      }
    }
  }

  /// Return RGB colors in range [0, 255]
  std::optional<std::vector<Eigen::Vector3f>> getRGB() override {
    assert(globalPointCloud_.size() == accumulatorCount_.size());
    assert(accumulatorCount_.size() == colorAccumulator_.size());
    // Normalize colors
    std::vector<Eigen::Vector3f> colors;
    colors.reserve(globalPointCloud_.size());
    for (const auto& pt : globalPointCloud_) {
      if (accumulatorCount_.count(pt.uid) == 0) {
        // If point has been unseen set a default color
        colors.emplace_back(Eigen::Vector3f(255, 0, 0) / 255.f);
        continue;
      }
      colors.push_back(colorAccumulator_.at(pt.uid));
      // Normalize the color
      colors.back() /= accumulatorCount_.at(pt.uid);
      colors.back() *= 255;
    }
    return colors;
  }
};

} // namespace
