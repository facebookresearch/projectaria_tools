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
// Colorize a GlobalPointCloud given its SLAM image observations.
// Colorization could be done:
// - using the existing stored image observations (u,v) (FAST)
// - or by projecting the 3D points to the SLAM images (SLOWER)
//
// This implementation does not target speed, but being used as a tutorial
//
class SLAMPointCloudColorizer : public PointCloudColorizer {
 public:
  bool useProjectedPoints_ = false; // Tell if we are using Projected 3D points or using the
                                    // existing (u,v) point coordinates

  // Color accumulator (for each point uid, accumulate normalized color [0.f, 1.f])
  std::unordered_map<uint32_t, float> colorAccumulator_;
  // Accumulator count (Keep track of visited point to know how many pixel observations has been
  // accumulated)
  std::unordered_map<uint32_t, uint32_t> accumulatorCount_;

  explicit SLAMPointCloudColorizer(
      const projectaria::tools::mps::ClosedLoopTrajectory& trajectory,
      const projectaria::tools::mps::GlobalPointCloud& globalPointCloud,
      const projectaria::tools::mps::PointObservations& pointObservations,
      std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider,
      bool useProjectedPoints = false)
      : PointCloudColorizer(trajectory, globalPointCloud, pointObservations, dataProvider),
        useProjectedPoints_(useProjectedPoints) {}

  ~SLAMPointCloudColorizer() override = default;

  void Colorize() override {
    using namespace projectaria::tools::data_provider;
    using namespace projectaria::tools::image;
    using namespace projectaria::tools::mps;

    colorAccumulator_.clear();
    accumulatorCount_.clear();

    const size_t numSlamData = dataProvider_->getNumData(kSlamLeftCameraStreamId);
    const int64_t startTimestampNs = dataProvider_->getFirstTimeNs(kSlamLeftCameraStreamId);
    const int64_t endTimestampNs = dataProvider_->getLastTimeNs(kSlamLeftCameraStreamId);

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
          numSlamData, std::cout, "Looping over VRS image frames and PointCloud observations.\n");
      for (int64_t frameId = 0; frameId < numSlamData; ++frameId) {
        ++progressBar;

        //
        // Organize data as [LEFT, RIGHT] array and loop over the point observations
        //

        // Get SLAM camera images
        const std::array<ImageDataAndRecord, 2> imageDataAndRecord{
            dataProvider_->getImageDataByIndex(kSlamLeftCameraStreamId, frameId),
            dataProvider_->getImageDataByIndex(kSlamRightCameraStreamId, frameId)};

        // Get current timestamp
        const int64_t captureTimestampNs = imageDataAndRecord[0].second.captureTimestampNs;

        // Get current pose from cached trajectory
        const auto pose = trajectoryProvider.findPose(captureTimestampNs);
        if (!pose) {
          continue;
        }

        const std::array<ImageU8, 2> images{
            std::get<ImageU8>(imageDataAndRecord[0].first.imageVariant().value()),
            std::get<ImageU8>(imageDataAndRecord[1].first.imageVariant().value())};

        const std::array<projectaria::tools::calibration::CameraCalibration, 2> cameras{
            *dataProvider_->getDeviceCalibration()->getCameraCalib("camera-slam-left"),
            *dataProvider_->getDeviceCalibration()->getCameraCalib("camera-slam-right")};

        const std::array<std::vector<PointObservationPair>, 2> currentFramePointObservations{
            pointObsProvider.findAllPointObs(leftCameraSerial, captureTimestampNs),
            pointObsProvider.findAllPointObs(rightCameraSerial, captureTimestampNs)};

        // Sample GRAY VRS image colors from the LEFT and RIGHT point observations
        for (const int index : {0, 1}) {
          const auto& currentFrameObservations = currentFramePointObservations[index];

          Eigen::Vector2f uv;
          for (const auto& ptIt : currentFrameObservations) {
            const auto ptUid = ptIt.second.uid;

            if (useProjectedPoints_) {
              // Use Point Projection
              const auto T_Device_camera = cameras[index].getT_Device_Camera();

              // Retrieve corresponding World Point
              const auto position_world = ptIt.second.position_world;
              const auto T_world_cam = pose->T_world_device * T_Device_camera;
              const auto position_local = T_world_cam.inverse() * position_world;

              const auto projection = cameras[index].project(position_local);
              if (!projection) {
                // 3D point is not visible
                continue;
              }
              uv = projection->cast<float>();
            } else {
              // Directly use the feature coordinate
              uv = ptIt.first;
            }

            // Color sampling
            const auto& image = images[index];
            if (colorAccumulator_.count(ptUid) == 0) {
              colorAccumulator_[ptUid] = (image(uv.x(), uv.y()) / 255.f);
              accumulatorCount_[ptUid] = 1;
            } else {
              colorAccumulator_.at(ptUid) += (image(uv.x(), uv.y()) / 255.f);
              ++accumulatorCount_.at(ptUid);
            }
          }
        }
      }
    }
  }

  /// Return gray colors in range [0, 255]
  std::optional<std::vector<float>> getGray() override {
    assert(globalPointCloud_.size() == accumulatorCount_.size());
    assert(accumulatorCount_.size() == colorAccumulator_.size());
    // Normalize colors
    std::vector<float> colors;
    colors.reserve(globalPointCloud_.size());
    for (const auto& pt : globalPointCloud_) {
      if (accumulatorCount_.count(pt.uid) == 0) {
        // If point has been unseen set a default color
        colors.push_back(0.f);
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
