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

#include <filesystem>

#include "AriaDigitalTwinDataTypes.h"

namespace projectaria::dataset::adt {

/**
 * @brief Class for loading and accessing skeleton marker and joint information from an ADT
 * sequence. Motive (the software running Optitrack) generates a frame of marker positions for each
 * capture corresponding to each bodysuit in the scene. It then estimates the person's joint
 * positions for each of these marker frames. ADT converts these measurements to the Scene frame to
 * be consistent with all ground truth data and this class allows the user to load and query that
 * data. We provide a separate class from the main AriaDigitalTwinDataProvider to separate out the
 * skeleton loading and allow users to call this API without loading all other ADT data.
 */
class AriaDigitalTwinSkeletonProvider {
 public:
  /**
   * @brief construct a AriaDigitalTwinSkeletonProvider and load all data
   * @param skeletonJsonPath path to skeleton json file (e.g., ~/data/Skeleton_T.json)
   */
  explicit AriaDigitalTwinSkeletonProvider(const std::string& skeletonJsonPath);

  /**
   * @brief Gets a skeleton frame by timestamp
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions Method used for querying time. The options are one of {BEFORE, AFTER,
   * CLOSEST}. Default to CLOSEST.
   * @return SkeletonFrameWithDt skeleton frame data, wrapped using DataWithDt
   */
  [[nodiscard]] SkeletonFrameWithDt getSkeletonByTimestampNs(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief get the connections between joint IDs. (e.g, head id -> neck id)
   * @return const reference to std::vector<std::pair<int, int>> which is a vector of maps between
   * one joint Id and its connecting join Id
   */
  static const std::vector<std::pair<int, int>>& getJointConnections();

  /**
   * @brief get the labels associated with each joint
   * @return const reference to std::vector<std::string> which is a vector of strings where the i^th
   * element in the vector corresponds to joint Id i.
   */
  static const std::vector<std::string>& getJointLabels();

  /**
   * @brief get the labels associated with each marker
   * @return const reference to std::vector<std::string> which is a vector of strings where the i^th
   * element in the vector corresponds to maker Id i.
   */
  static const std::vector<std::string>& getMarkerLabels();

 private:
  void readSkeletonJson(const std::filesystem::path& path);

  std::map<int64_t, SkeletonFrame> frames_;
};

} // namespace projectaria::dataset::adt
