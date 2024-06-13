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
#include <set>
#include <vector>

#include <rapidjson/document.h>

#include <mps/MpsDataPathsProvider.h>
#include <vrs/StreamId.h>

#include "AriaDigitalTwinDataPaths.h"

namespace projectaria::dataset::adt {

/**
 * @brief This class is to load all data file paths from ADT data structure given a sequence path.
 * This class supports both v1.X dataset versions as well as v2.X dataset versions (and beyond)
 * which have different formats:
 *
 *
 * v1.X: Each ADT collection sequence may contain more than one Aria device wearers. The data
 * associated with each Aria device is called a subsequence: <br>
 *
 * ├── sequencePath        <br>
 * │   ├── subsequence1_Name <br>
 * │   │   ├── 2d_bounding_box.csv <br>
 * │   │   ├── 2d_bounding_box_with_skeleton.csv <br>
 * │   │   ├── 3d_bounding_box.csv <br>
 * │   │   ├── Skeleton_C.json <br>
 * │   │   ├── skeleton_aria_association.json <br>
 * │   │   ├── aria_trajectory.csv <br>
 * │   │   ├── depth_images.vrs <br>
 * │   │   ├── depth_images_with_skeleton.vrs <br>
 * │   │   ├── eyegaze.csv <br>
 * │   │   ├── instances.csv <br>
 * │   │   ├── scene_objects.csv <br>
 * │   │   ├── segmentations.vrs <br>
 * │   │   ├── segmentations_with_skeleton.vrs <br>
 * │   │   └── video.vrs <br>
 * │   ├── subsequence2_Name <br>
 * │   │   ├── 2d_bounding_box.csv <br>
 * │   │   ├── ... <br>
 * │   └── metadata.json <br>
 *
 * v2.X and beyond: We have removed the concept of subsequence. Each sequence can only contain one
 * Aria recording, and concurrent recordings can be fetched by looking the field in the metadata
 * file. This means we have the following file structure:
 *
 * ├── sequencePath        <br>
 * │   ├── 2d_bounding_box.csv <br>
 * │   ├── 2d_bounding_box_with_skeleton.csv <br>
 * │   ├── 3d_bounding_box.csv <br>
 * │   ├── Skeleton_C.json <br>
 * │   ├── skeleton_aria_association.json <br>
 * │   ├── aria_trajectory.csv <br>
 * │   ├── depth_images.vrs <br>
 * │   ├── depth_images_with_skeleton.vrs <br>
 * │   ├── eyegaze.csv <br>
 * │   ├── instances.csv <br>
 * │   ├── scene_objects.csv <br>
 * │   ├── segmentations.vrs <br>
 * │   ├── segmentations_with_skeleton.vrs <br>
 * │   └── video.vrs <br>
 * │   └── metadata.json <br>
 */
class AriaDigitalTwinDataPathsProvider {
 public:
  AriaDigitalTwinDataPathsProvider(const std::string& sequencePath);

  /**
   * @brief retrieve the DataPaths for this sequence. If loading a sequence that has version < 2.0
   * and has multiple subsequences, this will return the data paths associated with the first device
   * serial.
   * @param skeletonFlag a flag to indicate if load skeleton data or not. <b>By default this is set
   * to false</b>
   * @return A `AriaDigitalTwinDataPaths` object, `nullopt` if the function fails.
   */
  std::optional<AriaDigitalTwinDataPaths> getDataPaths(bool skeletonFlag = false) const;

  /**
   * @brief retrieve the DataPaths from a device based on its index
   * DEPRECATION NOTE: With dataset versions 2.0 and beyond, this function has been deprecated since
   * there is only one device per sequence. If you are using this on older data, it will still work.
   * If using on new data, it will only work if deviceNum is 0.
   * @param deviceNum the index of the device
   * @param skeletonFlag a flag to indicate if load skeleton data or not. <b>By default this is set
   * to false</b>
   * @return A `AriaDigitalTwinDataPaths` object, `nullopt` if the function fails.
   */
  [[deprecated]] std::optional<AriaDigitalTwinDataPaths> getDataPathsByDeviceNum(
      int deviceNum,
      bool skeletonFlag = false) const;

  /**
   * @brief retrieve the DataPaths from a device based on its serial number
   * DEPRECATION NOTE: With dataset versions 2.0 and beyond, this function has been deprecated since
   * there is only one device per sequence. This function will still work with old or newer data as
   * long as you are querying with the correct serial associated with this sequence.
   * @param deviceSerial the serial number of the device
   * @param skeletonFlag a flag to indicate if load skeleton data or not. <b>By default this is set
   * to false</b>
   * @return A `AriaDigitalTwinDataPaths` object, `nullopt` if the function fails.
   */
  [[deprecated]] std::optional<AriaDigitalTwinDataPaths> getDataPathsByDeviceSerial(
      const std::string& deviceSerial,
      bool skeletonFlag = false) const;

  /**
   * @brief get all device serial numbers in the recording sequence
   * DEPRECATION NOTE: With dataset versions 2.0 and beyond, this function has been deprecated since
   * there is only one device per sequence. This function will still work with old or newer data,
   * however, we recommend using getDeviceSerialNumber instead for newer data & @return a const
   * reference to a vector of string
   */
  [[deprecated]] const std::vector<std::string>& getDeviceSerialNumbers() const {
    return deviceSerialNumbers_;
  }

  /**
   * @brief get the device serial number. If loading a sequence that has version < 2.0 and has
   * multiple subsequences, this will return the first device serial.
   */
  const std::string& getDeviceSerialNumber() const;

  /**
   * @brief get the scene name of the recording sequence
   & @return a const reference to the scene name string
   */
  const std::string& getSceneName() const {
    return sceneName_;
  }

  /**
   * @brief check if the sequence is a multi-person sequence
   */
  bool isMultiPerson() const {
    return isMultiPerson_;
  }

  /**
   * @brief get the number of skeletons in the current sequence
   */
  int getNumSkeletons() const {
    return numSkeletons_;
  }

  /**
   * @brief get the name of the sequence that was recorded at the same time as this sequence
   * @return concurrent sequence name if it exists, otherwise 'nullopt'
   */
  std::optional<std::string> getConcurrentSequenceName() const;

 protected:
  static std::optional<AriaDigitalTwinDataPaths> getDataPathsUsingSubtourName(
      const std::string& sequencePath,
      const std::string& subtourName,
      const std::string& fileMetadata,
      bool skeletonFlag);

  static std::optional<AriaDigitalTwinDataPaths> getDataPathsUsingMainDataPath(
      const std::string& mainDataPath,
      const std::string& fileMetadata,
      bool skeletonFlag);

 private:
  // returns map from serial -> subsequence name
  void loadDeviceSerialToSubtourName();
  void loadSequenceMetaData();
  void loadV1Metadata(const rapidjson::Document& jdocConst);
  std::string sequencePath_;
  std::string fileMetadata_;
  std::unordered_map<std::string, std::string> serialToSubtourName_;
  std::vector<std::string> deviceSerialNumbers_;
  std::string sceneName_;
  std::string concurrentSequenceName_;
  std::string datasetVersion_;
  bool isMultiPerson_;
  int numSkeletons_;
};

} // namespace projectaria::dataset::adt
