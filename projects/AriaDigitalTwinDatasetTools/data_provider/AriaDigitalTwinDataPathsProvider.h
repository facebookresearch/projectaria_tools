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
#include <string>
#include <unordered_map>
#include <vector>

#include <vrs/StreamId.h>

namespace projectaria::dataset::adt {

/**
 * @brief Customized Hash function for vrs::StreamId, needed for any `unordered_map<StreamId, T>`
 */
struct StreamIdHash {
  size_t operator()(const vrs::StreamId& streamId) const {
    return std::hash<std::string>{}(streamId.getNumericName());
  }
};

/**
 * @brief A struct that includes the file paths of all ADT data files for one sequence of one
 * device.
 */
struct AriaDigitalTwinDataPaths {
  std::string ariaVrsFilePath; /**< Aria vrs */
  std::string ariaTrajectoryFilePath; /**< Aria 6DoF pose trajectory */
  std::string objectTrajectoriesFilePath; /**< object 6Dof pose trajectory */
  std::string objectBoundingBox3dFilePath; /**< axis-aligned bounding box (AABB) of objects in its
                                              local coordinate frame */
  std::string boundingBoxes2dFilePath =
      ""; /**< 2D object bounding boxes for all cameras, stored as <cameraId, filePath> */
  std::string segmentationsFilePath; /**< 2D segmentation maps */
  std::string depthImagesFilePath; /**< depth maps */
  std::string syntheticVrsFilePath; /**< synthetic file */
  std::string eyeGazesFilePath; /**< eye gaze file */
  std::unordered_map<uint64_t, std::string>
      skeletonsFilePaths; /**< skeleton files: skeletonId -> filepath */
  std::string skeletonMetaDataFilePath; /**< skeleton metadata file */
  std::string metaDataFilePath; /**< data collection metadata file */
  std::string instancesFilePath; /**< instances file, a.k.a. object instance information file */

  std::string toString() const;
};

/**
 * @brief This class is to load all data file paths from ADT data structure given a sequence path.
 * Each ADT collection sequence may contain more than one Aria device wearers. The data associated
 * with each Aria device is called a subsequence: <br>
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
 * This class allows you use dataset root path and sequence name to
 * query all Aria devices in a sequence and to query all data associated with an Aria device.
 */
class AriaDigitalTwinDataPathsProvider {
 public:
  AriaDigitalTwinDataPathsProvider(const std::string& sequencePath);

  /**
   * @brief retrieve the DataPaths from a device based on its index
   * @param deviceNum the index of the device
   * @param skeletonFlag a flag to indicate if load skeleton data or not. <b>By default this is set
   * to false</b>
   * @return A `AriaDigitalTwinDataPaths` object, `nullopt` if the function fails.
   */
  std::optional<AriaDigitalTwinDataPaths> getDataPathsByDeviceNum(
      int deviceNum,
      bool skeletonFlag = false) const;

  /**
   * @brief retrieve the DataPaths from a device based on its serial number
   * @param deviceSerial the serial number of the device
   * @param skeletonFlag a flag to indicate if load skeleton data or not. <b>By default this is set
   * to false</b>
   * @return A `AriaDigitalTwinDataPaths` object, `nullopt` if the function fails.
   */
  std::optional<AriaDigitalTwinDataPaths> getDataPathsByDeviceSerial(
      const std::string& deviceSerial,
      bool skeletonFlag = false) const;

  /**
   * @brief get all device serial numbers in the recording sequence
   & @return a const reference to a vector of string
   */
  const std::vector<std::string>& getDeviceSerialNumbers() const {
    return deviceSerialNumbers_;
  }

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

 protected:
  static std::optional<AriaDigitalTwinDataPaths> getDataPathsUsingSubtourName(
      const std::string& sequencePath,
      const std::string& subtourName,
      bool skeletonFlag);

 private:
  // returns map from serial -> subsequence name
  void loadDeviceSerialToSubtourName();
  void loadSequenceMetaData();
  std::string sequencePath_;
  std::string fileMetadata_;
  std::unordered_map<std::string, std::string> serialToSubtourName_;
  std::vector<std::string> deviceSerialNumbers_;
  std::string sceneName_;
  bool isMultiPerson_;
  int numSkeletons_;
};

} // namespace projectaria::dataset::adt
