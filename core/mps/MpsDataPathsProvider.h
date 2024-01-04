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

/**
 * @brief A struct that includes the file paths of all MPS eyegaze files associated with a sequence
 */
struct MpsEyegazeDataPaths {
  std::string generalEyegaze; /**< general eye gaze results */
  std::string personalizedEyegaze; /**< personalized eye gaze results */
  std::string summary; /**< eye gaze processing summary */
};

/**
 * @brief A struct that includes the file paths of all MPS slam files associated with a sequence
 */
struct MpsSlamDataPaths {
  std::string closedLoopTrajectory; /**< closed loop trajectory */
  std::string openLoopTrajectory; /**< open loop trajectory */
  std::string semidensePoints; /**< semidense pointcloud */
  std::string semidenseObservations; /**< point observations from the semidense cloud */
  std::string onlineCalibration; /**< online calibration results from slam */
  std::string summary; /**< path to slam summary json */
};

/**
 * @brief A struct that includes the file paths of all MPS files associated with a sequence
 */
struct MpsDataPaths {
  MpsSlamDataPaths slam; /**< SLAM data paths */
  MpsEyegazeDataPaths eyegaze; /**< EyeGaze data paths */
};

/**
 * @brief This class is to load all MPS data file paths from an Aria dataset sequence using the
 * sequence root path.
 */
class MpsDataPathsProvider {
 public:
  explicit MpsDataPathsProvider(const std::string& mpsRootPath);

  /**
   * @brief Get the resulting data paths
   * @return MPS data paths object
   */
  MpsDataPaths getDataPaths() const;

 private:
  void loadDataPaths();

  std::string mpsRootPath_;
  MpsDataPaths dataPaths_;
};

} // namespace projectaria::tools::mps
