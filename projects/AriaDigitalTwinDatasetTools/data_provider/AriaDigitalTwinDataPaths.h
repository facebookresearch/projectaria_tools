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

#include <cstdint>
#include <string>
#include <unordered_map>

#include <mps/MpsDataPathsProvider.h>

namespace projectaria::dataset::adt {

/**
 * @brief A struct that includes the file paths of all ADT data files for one sequence of one
 * device.
 */
struct AriaDigitalTwinDataPaths {
  std::string sequenceName; /**< name of the sequence loaded */
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

  tools::mps::MpsDataPaths mps;

  std::string toString() const;
};

} // namespace projectaria::dataset::adt
