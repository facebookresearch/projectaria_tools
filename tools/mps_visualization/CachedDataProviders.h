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

#include <logging/Checks.h>

namespace projectaria::tools::mps {

using PointObservationPair = std::pair<Eigen::Vector2f, GlobalPointPosition>;

// Cache trajectory poses based on their quantized timestamps for fast lookup
class TrajectoryProvider {
 public:
  // Construct with given relaxed start/end timestamps and rough trajectory frame rate
  TrajectoryProvider(
      const ClosedLoopTrajectory& singleSessionTraj,
      const int64_t startTimestampNs,
      const int64_t endTimestampNs,
      const int64_t quantizedFrameRateNs = 1000000)
      : startTimestampNs_(startTimestampNs - quantizedFrameRateNs),
        endTimestampNs_(endTimestampNs + quantizedFrameRateNs),
        quantizedFrameRateNs_(quantizedFrameRateNs) {
    for (const auto& pose : singleSessionTraj) {
      addPose(pose);
    }
  }

  // Find the pose with closest timestamp to the query timestamp
  std::optional<ClosedLoopTrajectoryPose> findPose(
      const int64_t queryTimestampNs,
      const int64_t maxTimestampOffNs = 1000000) const {
    if (queryTimestampNs < startTimestampNs_ || queryTimestampNs > endTimestampNs_) {
      return {};
    }
    const int64_t quantizedQueryTimestampNs = queryTimestampNs / quantizedFrameRateNs_;

    // Collect candidate poses within a local window that takes the quantization error into account
    std::vector<ClosedLoopTrajectoryPose> candidatePoses;
    for (int64_t t = quantizedQueryTimestampNs - 1; t <= quantizedQueryTimestampNs + 1; ++t) {
      const auto iter = quantizedTimestampPoses_.find(t);
      if (iter != quantizedTimestampPoses_.end()) {
        const auto& poses = iter->second;
        for (const auto& pose : poses) {
          candidatePoses.push_back(pose);
        }
      }
    }
    if (candidatePoses.empty()) {
      return {};
    } else {
      // Find the pose with closest timestamp to the query timestamp within the candidate poses
      int64_t minTimestampDiffNs = maxTimestampOffNs;
      ClosedLoopTrajectoryPose closestPose;
      for (const auto& pose : candidatePoses) {
        const int64_t timestampNs =
            std::chrono::duration_cast<std::chrono::nanoseconds>(pose.trackingTimestamp).count();
        const int64_t timestampDiffNs = std::abs(timestampNs - queryTimestampNs);
        if (timestampDiffNs < minTimestampDiffNs) {
          minTimestampDiffNs = timestampDiffNs;
          closestPose = pose;
        }
      }
      return closestPose;
    }
  }

  // Get the number of cached poses
  size_t numPoses() const {
    return numPoses_;
  }

 private:
  // Add the pose to the cache if it is within the time window
  void addPose(const ClosedLoopTrajectoryPose& pose) {
    const int64_t timestampNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(pose.trackingTimestamp).count();
    if (timestampNs > startTimestampNs_ && timestampNs < endTimestampNs_) {
      const int64_t quantizedTimestampNs = timestampNs / quantizedFrameRateNs_;
      quantizedTimestampPoses_[quantizedTimestampNs].push_back(pose);
      ++numPoses_;
    }
  }

  // Cache of the poses for quick lookup
  std::unordered_map<int64_t, std::vector<ClosedLoopTrajectoryPose>> quantizedTimestampPoses_;
  size_t numPoses_ = 0;

  // Start and end timestamps of the replay trajectory
  const int64_t startTimestampNs_;
  const int64_t endTimestampNs_;

  // Quantized frame rate MUST be selected based on the frequency of the input trajectory
  const int64_t quantizedFrameRateNs_;
};

// Cache point observations based on their observed timestamps for fast lookup
class PointAndObservationProvider {
 public:
  // Construct with point cloud, point observations, SLAM camera serials, and start/end timestamps
  // of the corresponding trajectory
  PointAndObservationProvider(
      const GlobalPointCloud& ptCloud,
      const PointObservations& pointObs,
      const std::string& leftCameraSerial,
      const std::string& rightCameraSerial,
      const int64_t startTimestampNs,
      const int64_t endTimestampNs,
      const int64_t numericalErrorNs = 1000000)
      : leftCameraSerial_(leftCameraSerial),
        rightCameraSerial_(rightCameraSerial),
        startTimestampNs_(startTimestampNs - numericalErrorNs),
        endTimestampNs_(endTimestampNs + numericalErrorNs),
        numericalErrorNs_(numericalErrorNs) {
    // Cache point cloud for faster look up
    for (const auto& pt : ptCloud) {
      uidPoints_[pt.uid] = &pt;
    }
    // Cache and group point observations for faster look up
    for (const auto& obs : pointObs) {
      addPointObs(obs);
    }
  }

  // Find the point with the query uid
  std::optional<GlobalPointPosition> findPoint(const uint32_t pointUid) const {
    const auto iter = uidPoints_.find(pointUid);
    if (iter != uidPoints_.end()) {
      return *(iter->second);
    } else {
      return {};
    }
  }

  // Find the set of points observed by the query camera serial at the query timestamp
  std::vector<PointObservationPair> findAllPointObs(
      const std::string& queryCameraSerial,
      const int64_t queryTimestampNs) const {
    if (queryTimestampNs < startTimestampNs_ || queryTimestampNs > endTimestampNs_) {
      return {};
    }
    const int64_t quantizedQueryTimestampNs = queryTimestampNs / numericalErrorNs_;

    for (int64_t t = quantizedQueryTimestampNs - 1; t <= quantizedQueryTimestampNs + 1; ++t) {
      if (queryCameraSerial == leftCameraSerial_) {
        const auto iter = timestampLeftPoints_.find(t);
        if (iter != timestampLeftPoints_.end()) {
          return iter->second;
        }
      } else if (queryCameraSerial == rightCameraSerial_) {
        const auto iter = timestampRightPoints_.find(t);
        if (iter != timestampRightPoints_.end()) {
          return iter->second;
        }
      }
    }
    return {};
  }

  // Get the number of cached point observations
  size_t numPointObs() const {
    return numPointObs_;
  }

  size_t numObsFrameLeft() const {
    return timestampLeftPoints_.size();
  }

  size_t numObsFrameRight() const {
    return timestampRightPoints_.size();
  }

 private:
  // Add the point observation to the cache if it has valid camera serial and point uid, and is
  // within the time window
  void addPointObs(const PointObservation& obs) {
    const int64_t timestampNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(obs.frameCaptureTimestamp).count();
    if (timestampNs > startTimestampNs_ && timestampNs < endTimestampNs_) {
      XR_CHECK(obs.cameraSerial == leftCameraSerial_ || obs.cameraSerial == rightCameraSerial_);

      const auto iter = uidPoints_.find(obs.pointUid);
      XR_CHECK(
          iter != uidPoints_.end(),
          "No 3D point can be found to match with point observation. Maybe Global point cloud file(s) or Point observation file is not correctly provided");

      const int64_t quantizedTimestampNs = timestampNs / numericalErrorNs_;

      // Cache and group based on left or right camera
      if (obs.cameraSerial == leftCameraSerial_) {
        timestampLeftPoints_[quantizedTimestampNs].emplace_back(obs.uv, *(iter->second));
      } else { // obs.cameraSerial == rightCameraSerial_
        timestampRightPoints_[quantizedTimestampNs].emplace_back(obs.uv, *(iter->second));
      }
      ++numPointObs_;
    }
  }

  // Cache of the 2D/3D points for quick lookup
  std::unordered_map<uint32_t, const GlobalPointPosition*> uidPoints_;
  std::unordered_map<int64_t, std::vector<PointObservationPair>> timestampLeftPoints_;
  std::unordered_map<int64_t, std::vector<PointObservationPair>> timestampRightPoints_;
  size_t numPointObs_ = 0;

  const std::string leftCameraSerial_;
  const std::string rightCameraSerial_;
  const int64_t startTimestampNs_;
  const int64_t endTimestampNs_;

  // Quantized frame rate MUST be selected based on the frequency of the input trajectory
  const int64_t numericalErrorNs_;
};

class EyeGazeProvider {
 public:
  explicit EyeGazeProvider(const EyeGazes& eyeGazes, const int64_t slamEyeTrackingOffsetNs = 100000)
      : slamEyeTrackingOffsetNs_(slamEyeTrackingOffsetNs) {
    for (const auto& eyeGaze : eyeGazes) {
      const int64_t timestampNs =
          std::chrono::duration_cast<std::chrono::nanoseconds>(eyeGaze.trackingTimestamp).count();
      const int64_t quantizedTimestampNs = timestampNs / slamEyeTrackingOffsetNs_;
      timestampEyeGazes_[quantizedTimestampNs] = eyeGaze;
    }
  }

  std::optional<EyeGaze> findEyeGaze(const int64_t queryTimestampNs) const {
    const int64_t quantizedQueryTimestampNs = queryTimestampNs / slamEyeTrackingOffsetNs_;
    for (int64_t t = quantizedQueryTimestampNs - 1; t <= quantizedQueryTimestampNs + 1; ++t) {
      const auto iter = timestampEyeGazes_.find(t);
      if (iter != timestampEyeGazes_.end()) {
        return iter->second;
      }
    }
    return {};
  }

  size_t numEyeGazes() const {
    return timestampEyeGazes_.size();
  }

 private:
  std::unordered_map<int64_t, EyeGaze> timestampEyeGazes_;

  // Quantized frame rate MUST be selected based on the time offset between SLAM and eye tracking
  const int64_t slamEyeTrackingOffsetNs_;
};

} // namespace projectaria::tools::mps
