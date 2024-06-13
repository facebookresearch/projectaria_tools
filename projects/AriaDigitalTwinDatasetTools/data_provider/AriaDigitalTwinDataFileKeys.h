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
#include <unordered_map>

namespace projectaria::dataset::adt {

// sequence metadata keys
inline const std::string kDeviceSerialKey = "serial";
inline const std::string kSceneKey = "scene";
inline const std::string kIsMultiPersonKey = "is_multi_person";
inline const std::string kNumSkeletonKey = "num_skeletons";
inline const std::string kConcurrentSequenceNameKey = "concurrent_sequence";

// instance keys
inline const std::string kInstanceIdKey = "instance_id";
inline const std::string kInstanceNameKey = "instance_name";
inline const std::string kPrototypeNameKey = "prototype_name";
inline const std::string kCategoryKey = "category";
inline const std::string kCategoryUidKey = "category_uid";
inline const std::string kMotionTypeKey = "motion_type";
inline const std::string kInstanceTypeKey = "instance_type";
inline const std::string kRigidityKey = "rigidity";
inline const std::string kRotationalSymmetryKey = "rotational_symmetry";
inline const std::string kRotationalSymmetryIsAnnotatedKey = "is_annotated";
inline const std::string kRotationalSymmetryAxesKey = "axes";
inline const std::string kRotationalSymmetryAxisKey = "axis";
inline const std::string kRotationalSymmetryAngleDegreeKey = "angle_degree";
inline const std::string kCanonicalPoseKey = "canonical_pose";
inline const std::string kCanonicalPoseUpKey = "up_vector";
inline const std::string kCanonicalPoseFrontKey = "front_vector";
inline const std::string kInstanceTypeObjectValue = "object";
inline const std::string kInstanceTypeHumanValue = "human";
inline const std::string kMotionTypeStaticValue = "static";
inline const std::string kMotionTypeDynamicValue = "dynamic";
inline const std::string kRigidityTypeRigidValue = "rigid";
inline const std::string kRigidityTypeDeformableValue = "deformable";

// skeleton data
inline const std::string kSkeletonDeviceSerialKey = "AssociatedDeviceSerial";
inline const std::string kInvalidSkeletonName = "NONE";
inline const std::string kSkeletonMetadataKey = "SkeletonMetadata";
inline const std::string kSkeletonNameKey = "SkeletonName";
inline const std::string kSkeletonIdKey = "SkeletonId";
inline const std::string kSkeletonFileKey = "Skeleton_";
inline const std::string kSkeletonFramesKey{"frames"};
inline const std::string kSkeletonMarkersKey{"markers"};
inline const std::string kSkeletonJointsKey{"joints"};
inline const std::string kSkeletonTimestampKey{"timestamp_ns"};

// dataset version
inline const std::string kDatasetVersionKey = "dataset_version";
inline const std::string kDatasetNameDefault = "ADT_2023";
inline const std::string kDatasetVersionDefault = "1.0";
inline const std::string kDatasetVersionUnknown = "Unknown";
inline const std::string kDatasetNameKey = "dataset_name";
inline const std::unordered_map<std::string, std::string> kLatestDatasetVersions{
    {"ADT_2023", "2.0"}};
inline const std::unordered_map<std::string, std::string> kCorruptDatasets{
    {"Apartment_release_multiuser_party_seq145", "IMU data corrupted"},
    {"Apartment_release_multiuser_clean_seq115", "IMU data corrupted"},
    {"Apartment_release_clean_seq139", "IMU data corrupted"},
    {"Apartment_release_multiskeleton_party_seq112", "IMU data corrupted"},
    {"Apartment_release_multiskeleton_party_seq109", "IMU data corrupted"}};

// data values
constexpr int64_t kInvalidDeviceTimestampNs = -1;

// DEPRECATED
inline const std::string kSubtoursKey = "subtours";
inline const std::string kSubtourNameKey = "subtour_name";
inline const std::string kDeviceSerialKeyDeprecated = "device_serial";

} // namespace projectaria::dataset::adt
