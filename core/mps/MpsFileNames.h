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

#include <string>

namespace projectaria::tools::mps {

// SLAM
inline const std::string kMpsSlamClosedLoopTrajectoryFile = "closed_loop_trajectory.csv";
inline const std::string kMpsSlamOpenLoopTrajectoryFile = "open_loop_trajectory.csv";
inline const std::string kMpsSlamSemidensePointsFile = "semidense_points.csv.gz";
inline const std::string kMpsSlamSemidensePointsFileDeprecated = "global_points.csv.gz";
inline const std::string kMpsSlamSemidenseObservationsFile = "semidense_observations.csv.gz";
inline const std::string MpsSlamOnlineCalibrationFile = "online_calibration.jsonl";
inline const std::string kMpsSlamSummaryFile = "summary.json";

// EyeGaze
inline const std::string kMpsGeneralEyegazeFile = "general_eye_gaze.csv";
inline const std::string kMpsGeneralEyegazeFileDeprecated = "generalized_eye_gaze.csv";
inline const std::string kMpsPersonalEyegazeFile = "personalized_eye_gaze.csv";
inline const std::string kMpsPersonalEyegazeFileDeprecated = "calibrated_eye_gaze.csv";
inline const std::string kMpsEyegazeSummaryFile = "summary.json";

// HandTracking
inline const std::string kMpsWristAndPalmPosesFile = "wrist_and_palm_poses.csv";
inline const std::string kMpsHandTrackingSummaryFile = "summary.json";

// Folders
inline const std::string kMpsSlamFolder = "slam";
inline const std::string kMpsSlamFolderDeprecated = "trajectory";
inline const std::string kMpsEyegazeFolder = "eye_gaze";
inline const std::string kMpsHandTrackingFolder = "hand_tracking";

} // namespace projectaria::tools::mps
