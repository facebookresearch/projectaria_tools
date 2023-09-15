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

#include "AriaDigitalTwinDataPathsProvider.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#define RAPIDJSON_NAMESPACE rapidjson
#include <rapidjson/document.h>
#include <rapidjson/rapidjson.h>

#define DEFAULT_LOG_CHANNEL "AriaDigitalTwinDataPathsProvider"
#include <logging/Log.h>

#include "AriaDigitalTwinDataFileKeys.h"
#include "AriaDigitalTwinDataFileNames.h"

namespace fs = std::filesystem;

namespace projectaria::dataset::adt {

std::optional<AriaDigitalTwinDataPaths>
AriaDigitalTwinDataPathsProvider::getDataPathsUsingSubtourName(
    const std::string& sequencePath,
    const std::string& subtourName,
    bool skeletonFlag) {
  AriaDigitalTwinDataPaths paths;

  // get main data root
  const auto mainPath = fs::path(sequencePath) / fs::path(subtourName);

  // get Aria vrs file path
  const std::string ariaVrsFilePath = (fs::path(mainPath) / kVrsFileName).string();
  if (!fs::exists(ariaVrsFilePath)) {
    XR_LOGW("aria vrs file not found at {}", ariaVrsFilePath);
  } else {
    paths.ariaVrsFilePath = ariaVrsFilePath;
  }

  // get 6DoF Aria trajectory
  const std::string ariaTrajectoryFilePath = (fs::path(mainPath) / kAriaTrajFile).string();
  if (!fs::exists(ariaTrajectoryFilePath)) {
    XR_LOGW("aria trajectory file not found at {}", ariaTrajectoryFilePath);
  } else {
    paths.ariaTrajectoryFilePath = ariaTrajectoryFilePath;
  }

  // get 6DoF object trajectory
  const std::string objectTrajectoriesFilePath = (fs::path(mainPath) / kObjTrajFile).string();
  if (!fs::exists(objectTrajectoriesFilePath)) {
    XR_LOGW("object trajectories file not found at {}", objectTrajectoriesFilePath);
  } else {
    paths.objectTrajectoriesFilePath = objectTrajectoriesFilePath;
  }

  // get object 3D bbox
  const std::string objectBoundingBox3dFilePath = (fs::path(mainPath) / k3dBoxFile).string();
  if (!fs::exists(objectBoundingBox3dFilePath)) {
    XR_LOGW("object 3d bounding box file not found at {}", objectBoundingBox3dFilePath);
  } else {
    paths.objectBoundingBox3dFilePath = objectBoundingBox3dFilePath;
  }

  // get 2d bbox file path
  const fs::path boundingBoxes2dFileName =
      skeletonFlag ? fs::path(k2dBoxFileWithSkel) : fs::path(k2dBoxFileNoSkel);
  const std::string boundingBoxes2dFilePath =
      (fs::path(mainPath) / boundingBoxes2dFileName).string();
  if (!fs::exists(boundingBoxes2dFilePath)) {
    XR_LOGW("2d bounding box data found at {}", boundingBoxes2dFilePath);
  } else {
    paths.boundingBoxes2dFilePath = boundingBoxes2dFilePath;
  }

  // segmentation file
  const fs::path segmentationsFileName = skeletonFlag ? fs::path(kSegmentationImageFileWithSkel)
                                                      : fs::path(kSegmentationImageFileNoSkel);
  const std::string segmentationsFilePath = (fs::path(mainPath) / segmentationsFileName).string();
  if (!fs::exists(segmentationsFilePath)) {
    XR_LOGW("no segmentation image data found at {}", segmentationsFilePath);
  } else {
    paths.segmentationsFilePath = segmentationsFilePath;
  }

  // get depth images file
  const fs::path depthImagesFileName =
      skeletonFlag ? fs::path(kDepthImageFileWithSkel) : fs::path(kDepthImageFileNoSkel);
  const std::string depthImagesFilePath = (fs::path(mainPath) / depthImagesFileName).string();
  if (!fs::exists(depthImagesFilePath)) {
    XR_LOGW("no depth image data found at {}", depthImagesFilePath);
  } else {
    paths.depthImagesFilePath = depthImagesFilePath;
  }

  // get synthetic video file
  const std::string syntheticVrsFilePath = (fs::path(mainPath) / kSyntheticVrsFile).string();
  if (!fs::exists(syntheticVrsFilePath)) {
    XR_LOGW("no synthetic video data found at {}", syntheticVrsFilePath);
  } else {
    paths.syntheticVrsFilePath = syntheticVrsFilePath;
  }

  // get eye gaze files
  const std::string eyeGazesFilePath = (fs::path(mainPath) / kEyeGazeFile).string();
  if (!fs::exists(eyeGazesFilePath)) {
    XR_LOGW("no eye gaze data found at {}", eyeGazesFilePath);
  } else {
    paths.eyeGazesFilePath = eyeGazesFilePath;
  }

  // get skeleton files
  const fs::path skeletonMetaDataFilePath = fs::path(mainPath) / fs::path(kSkeletonMetadataFile);
  if (fs::exists(skeletonMetaDataFilePath)) {
    paths.skeletonMetaDataFilePath = skeletonMetaDataFilePath.string();
    // load skeleton files from metadata file
    std::ifstream fileStream(skeletonMetaDataFilePath);
    if (!fileStream.is_open()) {
      XR_LOGE(
          "Could not open skeleton metadata file from: {} \n", skeletonMetaDataFilePath.string());
      throw std::runtime_error{"Could not open skeleton metadata file "};
    }

    std::stringstream buffer;
    buffer << fileStream.rdbuf();
    rapidjson::Document jdoc;
    jdoc.Parse(buffer.str().c_str());
    const auto& jdocConst = jdoc;

    // load device serial to subsequence name mapping
    if (!jdocConst.HasMember(kSkeletonMetadataKey.c_str())) {
      const std::string errMsg = fmt::format(
          "invalid skeleton metadata file. key: '{}' not available in json file",
          kSkeletonMetadataKey);
      XR_LOGE("{}", errMsg);
      throw std::runtime_error{errMsg};
    }
    for (const auto& metadataJ : jdocConst[kSkeletonMetadataKey.c_str()].GetArray()) {
      // get name to find file
      if (!metadataJ.HasMember(kSkeletonNameKey.c_str())) {
        const std::string errMsg = fmt::format(
            "invalid skeleton metadata file. key: '{}' not available in json file",
            kSkeletonNameKey);
        XR_LOGE("{}", errMsg);
        throw std::runtime_error{errMsg};
      }
      std::string skeletonName = metadataJ[kSkeletonNameKey.c_str()].GetString();

      if (skeletonName == kInvalidSkeletonName) {
        continue;
      }

      // get id
      if (!metadataJ.HasMember(kSkeletonIdKey.c_str())) {
        const std::string errMsg = fmt::format(
            "invalid skeleton metadata file. key: '{}' not available in json file", kSkeletonIdKey);
        XR_LOGE("{}", errMsg);
        throw std::runtime_error{errMsg};
      }
      uint64_t skeletonId = metadataJ[kSkeletonIdKey.c_str()].GetInt64();

      fs::path skeletonFilePath = fs::path(mainPath) / fs::path(skeletonName + ".json");
      paths.skeletonsFilePaths.emplace(skeletonId, skeletonFilePath.string());
    }
  }

  // get meta data file
  const std::string metaDataFilePath = (fs::path(sequencePath) / kMetadataFile).string();
  const std::string metaDataFilePathDeprecated =
      (fs::path(sequencePath) / kMetadataFileDeprecated).string();
  if (fs::exists(metaDataFilePath)) {
    paths.metaDataFilePath = metaDataFilePath;
  } else if (fs::exists(metaDataFilePathDeprecated)) {
    paths.metaDataFilePath = metaDataFilePathDeprecated;
  } else {
    XR_LOGW("no meta data found at {} or {}", metaDataFilePath, metaDataFilePathDeprecated);
  }

  const std::string instancesFilePath = (fs::path(mainPath) / kInstanceFile).string();
  if (!fs::exists(instancesFilePath)) {
    XR_LOGW("no instance files found at {}", instancesFilePath);
  } else {
    paths.instancesFilePath = instancesFilePath;
  }
  return paths;
}

void AriaDigitalTwinDataPathsProvider::loadSequenceMetaData() {
  if (!fs::exists(sequencePath_)) {
    XR_LOGE("sequence path does not exist: {}", sequencePath_);
    throw std::runtime_error{"invalid sequence path is provided"};
    return;
  }

  fs::path fileMetadata = fs::path(sequencePath_) / fs::path(kMetadataFile);
  fs::path fileMetadataDeprecated = fs::path(sequencePath_) / fs::path(kMetadataFileDeprecated);

  std::ifstream fileStream;
  if (fs::exists(fileMetadata)) {
    fileStream.open(fileMetadata);
  } else if (fs::exists(fileMetadataDeprecated)) {
    fileStream.open(fileMetadataDeprecated);
  } else {
    XR_LOGW(
        "no meta data found at {} or {}", fileMetadata.string(), fileMetadataDeprecated.string());
  }

  if (!fileStream.is_open()) {
    XR_LOGE(
        "Could not find {} or {}, is this a valid sequence path?",
        kMetadataFile,
        kMetadataFileDeprecated);
    throw std::runtime_error{"Could not open gt metadata"};
  }

  std::stringstream buffer;
  buffer << fileStream.rdbuf();
  rapidjson::Document jdoc;
  jdoc.Parse(buffer.str().c_str());
  const auto& jdocConst = jdoc;

  // load device serial to subsequence name mapping
  if (!jdocConst.HasMember(kSubtoursKey.c_str())) {
    const std::string errMsg = fmt::format(
        "invalid metadata file. key: '{}' not available in metadata json file", kSubtoursKey);
    XR_LOGE("{}", errMsg);
    throw std::runtime_error{errMsg};
  }

  std::unordered_map<std::string, std::string> serialToSubtourName;
  auto subtoursObject = jdocConst[kSubtoursKey.c_str()].GetObject();
  for (auto subtourIter = subtoursObject.MemberBegin(); subtourIter != subtoursObject.MemberEnd();
       subtourIter++) {
    if (!subtourIter->value.HasMember(kDeviceSerialKey.c_str())) {
      XR_LOGE(
          "key: '{}' not available in metadata json file: {}",
          kDeviceSerialKey,
          fileMetadata.string());
      throw std::runtime_error{"invalid metadata file"};
    }
    std::string deviceSerial = subtourIter->value[kDeviceSerialKey.c_str()].GetString();

    if (!subtourIter->value.HasMember(kSubtourNameKey.c_str())) {
      XR_LOGE(
          "key: '{}' not available in metadata json file: {}",
          kSubtourNameKey,
          fileMetadata.string());
      throw std::runtime_error{"invalid metadata file"};
    }
    std::string subtourName = subtourIter->value[kSubtourNameKey.c_str()].GetString();
    serialToSubtourName_.emplace(deviceSerial, subtourName);
  }

  // load scene name
  if (!jdocConst.HasMember(kSceneKey.c_str())) {
    const std::string errMsg = fmt::format(
        "invalid metadata file. key: '{}' not available in metadata json file", kSubtoursKey);
    XR_LOGE("{}", errMsg);
    throw std::runtime_error{errMsg};
  }
  sceneName_ = jdocConst[kSceneKey.c_str()].GetString();

  // load is multi person
  if (!jdoc.HasMember(kIsMultiPersonKey.c_str())) {
    const std::string errMsg =
        fmt::format("key: '{}' not available in metadata json file", kIsMultiPersonKey);
    XR_LOGE("{}", errMsg);
    throw std::runtime_error{errMsg};
  }
  isMultiPerson_ = jdocConst[kIsMultiPersonKey.c_str()].GetBool();

  // load number of skeletons used in a sequence
  if (!jdoc.HasMember(kNumSkeletonKey.c_str())) {
    const std::string errMsg =
        fmt::format("key: '{}' not available in metadata json file", kNumSkeletonKey);
    XR_LOGE("{}", errMsg);
    throw std::runtime_error{errMsg};
  }
  numSkeletons_ = jdocConst[kNumSkeletonKey.c_str()].GetInt();

  fileStream.close();
}

AriaDigitalTwinDataPathsProvider::AriaDigitalTwinDataPathsProvider(const std::string& sequencePath)
    : sequencePath_(sequencePath) {
  loadSequenceMetaData();
  for (const auto& [deviceSerial, _] : serialToSubtourName_) {
    deviceSerialNumbers_.push_back(deviceSerial);
  }
  std::sort(deviceSerialNumbers_.begin(), deviceSerialNumbers_.end());
}

std::optional<AriaDigitalTwinDataPaths> AriaDigitalTwinDataPathsProvider::getDataPathsByDeviceNum(
    int deviceNum,
    bool skeletonFlag) const {
  if (deviceNum >= deviceSerialNumbers_.size()) {
    XR_LOGE(
        "device number {} not available in dataset, total number of devices: {}",
        deviceNum,
        deviceSerialNumbers_.size());
    return {};
  }

  const auto& deviceSerial = deviceSerialNumbers_[deviceNum];

  return getDataPathsByDeviceSerial(deviceSerial, skeletonFlag);
}

std::optional<AriaDigitalTwinDataPaths>
AriaDigitalTwinDataPathsProvider::getDataPathsByDeviceSerial(
    const std::string& deviceSerial,
    bool skeletonFlag) const {
  if (serialToSubtourName_.find(deviceSerial) == serialToSubtourName_.cend()) {
    XR_LOGE("device serial {} not available in dataset", deviceSerial);
    return {};
  }

  if (!numSkeletons_ && skeletonFlag) {
    XR_LOGE(
        "this sequence does not have skeleton ground-truth recorded, please turn off the skeletonFlag and fetch again");
    return {};
  }

  const auto& subtourName = serialToSubtourName_.at(deviceSerial);

  return getDataPathsUsingSubtourName(sequencePath_, subtourName, skeletonFlag);
}

std::string AriaDigitalTwinDataPaths::toString() const {
  std::string result;
  result += "--ariaVrsFilePath: " + ariaVrsFilePath + "\n";
  result += "--ariaTrajectoryFilePath: " + ariaTrajectoryFilePath + "\n";
  result += "--objectTrajectoriesFilePath: " + objectTrajectoriesFilePath + "\n";
  result += "--objectBoundingBox3dFilePath: " + objectBoundingBox3dFilePath + "\n";
  result += "--segmentationsFilePath: " + segmentationsFilePath + "\n";
  result += "--depthImagesFilePath: " + depthImagesFilePath + "\n";
  result += "--syntheticVideoFilePath: " + syntheticVrsFilePath + "\n";
  result += "--metaDataFilePath: " + metaDataFilePath + "\n";
  result += "--instancesFilePath: " + instancesFilePath + "\n";
  result += "--boundingBoxes2dFilePath: " + boundingBoxes2dFilePath + "\n";
  result += "--eyeGazesFilePath: " + eyeGazesFilePath + "\n";
  result += "--skeletonsFilePaths: \n";
  for (const auto& [skeletonId, skeletonFilePath] : skeletonsFilePaths) {
    result += "SkeletonId " + std::to_string(skeletonId) + ": " + skeletonFilePath + "\n";
  }
  return result;
}

} // namespace projectaria::dataset::adt
