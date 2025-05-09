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

#include <optional>
#include <string>

#include <data_provider/TimeTypes.h>
#include <mps/EyeGaze.h>
#include <mps/GlobalPointCloud.h>
#include <mps/HandTracking.h>
#include <mps/MpsDataPathsProvider.h>
#include <mps/OnlineCalibration.h>
#include <mps/PointObservation.h>
#include <mps/Trajectory.h>

namespace projectaria::tools::mps {

using TimeQueryOptions = data_provider::TimeQueryOptions;

/**
 * @brief This class is to load all MPS data given an MpsDataPaths object, and also provide all API
 * needed to query that data.
 * NOTE: to minimize disk usage, this data provider only loads data from disk after that data
 * type is first queried.
 */
class MpsDataProvider {
 public:
  /**
   * @brief Construct a new Mps Data Provider object given an MPS data paths object
   */
  explicit MpsDataProvider(const MpsDataPaths& mpsDataPaths);

  /**
   * @brief Check if general eye gaze data is available in the MPS data paths
   * @return true if data is available, false otherwise
   */
  [[nodiscard]] bool hasGeneralEyeGaze() const;

  /**
   * @brief Check if personalized eye gaze data is available in the MPS data paths
   * @return true if data is available, false otherwise
   */
  [[nodiscard]] bool hasPersonalizedEyeGaze() const;

  /**
   * @brief Check if open loop poses are available in the MPS data paths
   * @return true if data is available, false otherwise
   */
  [[nodiscard]] bool hasOpenLoopPoses() const;

  /**
   * @brief Check if closed loop poses are available in the MPS data paths
   * @return true if data is available, false otherwise
   */
  [[nodiscard]] bool hasClosedLoopPoses() const;

  /**
   * @brief Check if online calbrations are available in the MPS data paths
   * @return true if data is available, false otherwise
   */
  [[nodiscard]] bool hasOnlineCalibrations() const;

  /**
   * @brief Check if semidense point cloud data is available in the MPS data paths
   * @return true if data is available, false otherwise
   */
  [[nodiscard]] bool hasSemidensePointCloud() const;

  /**
   * @brief Check if semidense observations are available in the MPS data paths
   * @return true if data is available, false otherwise
   */
  [[nodiscard]] bool hasSemidenseObservations() const;

  /**
   * @brief Query MPS for general EyeGaze at a specific timestamp. This will throw an exception if
   * general eye gaze data is not available. Check for data availability first using
   * `hasGeneralEyeGaze()`
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Defaults to
   * CLOSEST.
   * @return optional EyeGaze, will return invalid if the query time is invalid
   */
  std::optional<EyeGaze> getGeneralEyeGaze(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest);

  /**
   * @brief Query MPS for personalized EyeGaze at a specific timestamp. This will throw an exception
   * if personalized eye gaze data is not available. Check for data availability first using
   * `hasPersonalizedEyeGaze()`
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Defaults to
   * CLOSEST.
   * @return optional EyeGaze, will return invalid if the query time is invalid
   */
  std::optional<EyeGaze> getPersonalizedEyeGaze(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest);

  /**
   * @brief Query MPS for OpenLoopTrajectoryPose at a specific timestamp. This will throw an
   * exception if open loop trajectory data is not available. Check for data availability first
   * using `hasOpenLoopPoses()`
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Defaults to
   * CLOSEST.
   * @return optional OpenLoopTrajectoryPose, will return invalid if the query time is invalid
   */
  std::optional<OpenLoopTrajectoryPose> getOpenLoopPose(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest);

  /**
   * @brief Query MPS for ClosedLoopTrajectoryPose at a specific timestamp. This will throw an
   * exception if closed loop trajectory data is not available. Check for data availability first
   * using `hasClosedLoopPoses()`
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Defaults to
   * CLOSEST.
   * @return optional ClosedLoopTrajectoryPose, will return invalid if the query time is invalid
   */
  std::optional<ClosedLoopTrajectoryPose> getClosedLoopPose(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest);

  /**
   * @brief Query MPS for ClosedLoopTrajectoryPose at a specific timestamp, and return interpolated
   * pose.
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @return optional ClosedLoopTrajectoryPose, will return invalid if the query is
   * out of bound
   */
  std::optional<ClosedLoopTrajectoryPose> getInterpolatedClosedLoopPose(int64_t deviceTimeStampNs);

  /**
   * @brief Query MPS for OnlineCalibration at a specific timestamp. This will throw an exception if
   * online calibration data is not available. Check for data availability first using
   * `hasOnlineCalibrations()`
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Defaults to
   * CLOSEST.
   * @return optional OnlineCalibration, will return invalid if the query time is invalid
   */
  std::optional<OnlineCalibration> getOnlineCalibration(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest);

  /**
   * @brief Get the MPS semidense point cloud. This will throw an exception if the point cloud is
   * not available. Check for data availability first using 'hasSemidensePointCloud()'
   * @return constant reference to the semidense point cloud
   */
  const GlobalPointCloud& getSemidensePointCloud();

  /**
   * @brief Get the MPS point observations. This will throw an exception if the observations are
   * not available. Check for data availability first using 'hasSemidenseObservations()'
   * @return constant reference to the semidense observations
   */
  const PointObservations& getSemidenseObservations();

  /**
   * @brief Query MPS for WristAndPalmPose at a specific timestamp. This will throw an exception if
   * WristAndPalmPoses data is not available. Check for data availability first using
   * `hasWristAndPalmPoses()`
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Defaults to
   * CLOSEST.
   * @return optional WristAndPalmPose, will return invalid if the query time is invalid
   */
  [[deprecated(
      "WristAndPalmPose and getWristAndPalmPose are to be deprecated. Use HandTrackingResult and getHandTrackingResult instead.")]]
  std::optional<WristAndPalmPose> getWristAndPalmPose(
      int64_t captureTimestampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest);

  /**
   * @brief Check if WristAndPalmPoses are available in the MPS data paths
   * @return true if data is available, false otherwise
   */
  [[deprecated(
      "WristAndPalmPose and hasWristAndPalmPoses are to be deprecated. Use HandTrackingResult and hasHandTrackingResults instead.")]] [[nodiscard]] bool
  hasWristAndPalmPoses() const;

  /**
   * @brief Query MPS for HandTrackingResult (landmark positions, wrist transform, wrist and palm
   * normals, etc.) at a specific timestamp. This will throw an exception if HandTrackingResult
   * data is not available. Check for data availability first using `hasHandTrackingResults()`
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Defaults to
   * CLOSEST.
   * @return optional HandTrackingResult, will return invalid if the query time is invalid
   */
  std::optional<HandTrackingResult> getHandTrackingResult(
      int64_t captureTimestampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest);

  /**
   * @brief Check if HandTrackingResults are available in the MPS data paths
   * @return true if data is available, false otherwise
   */
  [[nodiscard]] bool hasHandTrackingResults() const;

  /**
   * @brief Get the RGB frame closed loop pose relative to world at a specific timestamp. This will
   * query the rgb frame, and apply the online calibration correction if available.
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Defaults to
   * CLOSEST.
   * @return optional T_world_rgb, will return invalid if the query time is invalid
   */
  std::optional<Sophus::SE3d> getRgbCorrectedClosedLoopPose(
      int64_t captureTimestampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest);

  /**
   * @brief Get the corrected rgb frame timestamp based on the online calibration.
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Defaults to
   * CLOSEST.
   * @return optional corrected time stamp for rgb frame in Nanoseconds
   */
  std::optional<int64_t> getRgbCorrectedTimestampNs(
      int64_t captureTimestampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest);

  /**
   * @brief Get the SLAM version.
   * @return optional string, None if the file is missing, invalid, or does not contain a version
   */
  std::optional<std::string> getSlamVersion();

  /**
   * @brief Get the Eye Gaze version.
   * @return optional string, None if the file is missing, invalid, or does not contain a version
   */
  std::optional<std::string> getEyeGazeVersion();

  /**
   * @brief Get the Hand Tracking version.
   * @return optional string, None if the file is missing, invalid, or does not contain a version
   */
  std::optional<std::string> getHandTrackingVersion();

 private:
  MpsDataPaths dataPaths_;
  std::map<int64_t, EyeGaze> generalEyeGazes_;
  std::map<int64_t, EyeGaze> personalizedEyeGazes_;
  std::map<int64_t, OpenLoopTrajectoryPose> openLoopPoses_;
  std::map<int64_t, ClosedLoopTrajectoryPose> closedLoopPoses_;
  std::map<int64_t, OnlineCalibration> onlineCalibrations_;
  std::map<int64_t, WristAndPalmPose> wristAndPalmPoses_;
  std::map<int64_t, HandTrackingResult> handTrackingResults_;
  GlobalPointCloud globalPointCloud_;
  PointObservations pointObservations_;
  std::optional<std::string> slamVersion_;
  std::optional<std::string> eyeGazeVersion_;
  std::optional<std::string> handTrackingVersion_;
};

} // namespace projectaria::tools::mps
