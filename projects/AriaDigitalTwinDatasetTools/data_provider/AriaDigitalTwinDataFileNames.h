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

namespace projectaria::dataset::adt {

// sequence metadata file name
inline const std::string kMetadataFile = "metadata.json";
inline const std::string kMetadataFileDeprecated = "gt-metadata.json";

// Gt file names
inline const std::string kInstanceFile = "instances.json";
inline const std::string k3dBoxFile = "3d_bounding_box.csv";
inline const std::string kObjTrajFile = "scene_objects.csv";
inline const std::string kAriaTrajFile = "aria_trajectory.csv";
inline const std::string kEyeGazeFile = "eyegaze.csv";
inline const std::string k2dBoxFileNoSkel = "2d_bounding_box.csv";
inline const std::string k2dBoxFileWithSkel = "2d_bounding_box_with_skeleton.csv";
inline const std::string kDepthImageFileNoSkel = "depth_images.vrs";
inline const std::string kDepthImageFileWithSkel = "depth_images_with_skeleton.vrs";
inline const std::string kSegmentationImageFileNoSkel = "segmentations.vrs";
inline const std::string kSegmentationImageFileWithSkel = "segmentations_with_skeleton.vrs";
inline const std::string kSyntheticVrsFile = "synthetic_video.vrs";
inline const std::string kSkeletonMetadataFile = "skeleton_aria_association.json";
inline const std::string kVrsFileName = "video.vrs";

} // namespace projectaria::dataset::adt
