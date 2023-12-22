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

#include "AriaDigitalTwinDataProvider.h"

#include <assert.h>
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <valarray>

#define RAPIDJSON_NAMESPACE rapidjson
#include <rapidjson/document.h>
#include <rapidjson/rapidjson.h>

#include <sophus/interpolate.hpp>

#define DEFAULT_LOG_CHANNEL "AriaDigitalTwinDataProvider"
#include <logging/Log.h>

#include <mps/EyeGazeReader.h>
#include <mps/TrajectoryReaders.h>

#include "AriaDigitalTwinDataFileKeys.h"
#include "AriaDigitalTwinDataFileNames.h"
#include "AriaDigitalTwinUtils.h"

using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::calibration;
namespace fs = std::filesystem;

namespace projectaria::dataset::adt {

constexpr auto kInstanceFileErrorTemplate =
    "invalid instance file. key: '{}' not available in instances json file for instance id {}";

const std::unordered_map<std::string, std::string> kCorruptDatasets{
    {"Apartment_release_multiuser_party_seq145", "IMU data corrupted"},
    {"Apartment_release_multiuser_clean_seq115", "IMU data corrupted"},
    {"Apartment_release_clean_seq139", "IMU data corrupted"},
    {"Apartment_release_multiskeleton_party_seq112", "IMU data corrupted"},
    {"Apartment_release_multiskeleton_party_seq109", "IMU data corrupted"}};

namespace {
std::ifstream openFile(const fs::path& filePath, bool skipHeader = true) {
  std::ifstream fileStream(filePath);
  if (!fileStream.is_open()) {
    throw std::runtime_error(fmt::format("Could not open file {} \n", filePath.string()));
  }
  if (skipHeader) {
    std::string line;
    std::getline(fileStream, line);
  }
  return fileStream;
}

void getTokens(
    std::stringstream& ss,
    std::vector<std::string>& tokensOut,
    char delimiter,
    int tokenCnt) {
  std::string token;
  while (std::getline(ss, token, delimiter)) {
    tokensOut.emplace_back(token);
  }

  if (tokensOut.size() < tokenCnt) {
    XR_LOGE("Token size {} is smaller than {}", tokensOut.size(), tokenCnt);
    throw std::runtime_error{"invalid token size"};
  }
}

MotionType getMotionTypeFromInstancesJsonValue(const std::string& motionTypeValue) {
  MotionType motionType = MotionType::Unknown;
  if (motionTypeValue == kMotionTypeStaticValue) {
    motionType = MotionType::Static;
  } else if (motionTypeValue == kMotionTypeDynamicValue) {
    motionType = MotionType::Dynamic;
  }
  return motionType;
}

RigidityType getRigidityFromInstancesJsonValue(const std::string& rigidityTypeValue) {
  RigidityType rigidityType = RigidityType::Unknown;
  if (rigidityTypeValue == kRigidityTypeRigidValue) {
    rigidityType = RigidityType::Rigid;
  } else if (rigidityTypeValue == kRigidityTypeDeformableValue) {
    rigidityType = RigidityType::Deformable;
  }
  return rigidityType;
}

InstanceType getInstanceTypeFromInstancesJsonValue(const std::string& instanceTypeValue) {
  InstanceType instanceType = InstanceType::Unknown;
  if (instanceTypeValue == kInstanceTypeHumanValue) {
    instanceType = InstanceType::Human;
  } else if (instanceTypeValue == kInstanceTypeObjectValue) {
    instanceType = InstanceType::Object;
  }
  return instanceType;
}

RotationalSymmetry getRotationalSymmetryFromInstancesJsonObject(
    rapidjson::GenericObject<true, rapidjson::GenericValue<rapidjson::UTF8<>>>&
        rotationalSymmetryObject,
    const std::string& instanceInfoKey) {
  RotationalSymmetry rotationalSymmetry;
  if (!rotationalSymmetryObject.HasMember(kRotationalSymmetryIsAnnotatedKey.c_str())) {
    const std::string errMsg =
        fmt::format(kInstanceFileErrorTemplate, kRotationalSymmetryIsAnnotatedKey, instanceInfoKey);
    XR_LOGE("{}", errMsg);
    throw std::runtime_error{errMsg};
  }
  rotationalSymmetry.isAnnotated =
      rotationalSymmetryObject[kRotationalSymmetryIsAnnotatedKey.c_str()].GetBool();
  if (rotationalSymmetry.isAnnotated) {
    if (!rotationalSymmetryObject.HasMember(kRotationalSymmetryAxesKey.c_str())) {
      const std::string errMsg =
          fmt::format(kInstanceFileErrorTemplate, kRotationalSymmetryAxesKey, instanceInfoKey);
      XR_LOGE("{}", errMsg);
      throw std::runtime_error{errMsg};
    }
    auto rotationalSymmetryArray =
        rotationalSymmetryObject[kRotationalSymmetryAxesKey.c_str()].GetArray();
    for (auto rsymIter = rotationalSymmetryArray.Begin(); rsymIter != rotationalSymmetryArray.End();
         rsymIter++) {
      RotationalSymmetryAxis rotationalSymmetryAxis;
      auto symmetryAxis = rsymIter->GetObject();
      if (symmetryAxis.HasMember(kRotationalSymmetryAxisKey.c_str()) &&
          symmetryAxis.HasMember(kRotationalSymmetryAngleDegreeKey.c_str())) {
        auto symmetryAxisArray = symmetryAxis[kRotationalSymmetryAxisKey.c_str()].GetArray();
        if (symmetryAxisArray.Size() != 3) {
          const std::string errMsg =
              fmt::format("Invalid rotation axis array size {}", symmetryAxisArray.Size());
          XR_LOGE("{}", errMsg);
          throw std::runtime_error{errMsg};
        }
        rotationalSymmetryAxis.axis = Eigen::Vector3d(
            symmetryAxisArray[0].GetDouble(),
            symmetryAxisArray[1].GetDouble(),
            symmetryAxisArray[2].GetDouble());
        rotationalSymmetryAxis.angleDegree =
            symmetryAxis[kRotationalSymmetryAngleDegreeKey.c_str()].GetDouble();
        rotationalSymmetry.axes.emplace_back(rotationalSymmetryAxis);
      } else {
        const std::string errMsg = fmt::format(
            kInstanceFileErrorTemplate,
            kRotationalSymmetryAxisKey + " or " + kRotationalSymmetryAngleDegreeKey,
            instanceInfoKey);
        XR_LOGE("{}", errMsg);
        throw std::runtime_error{errMsg};
      }
    }
  }
  return rotationalSymmetry;
}

CanonicalPose getCanonicalPoseFromInstancesJsonObject(
    rapidjson::GenericObject<true, rapidjson::GenericValue<rapidjson::UTF8<>>>& canonicalPoseObject,
    const std::string& instanceInfoKey) {
  CanonicalPose canonicalPose;
  if (canonicalPoseObject.HasMember(kCanonicalPoseUpKey.c_str()) &&
      canonicalPoseObject.HasMember(kCanonicalPoseFrontKey.c_str())) {
    auto upVecArray = canonicalPoseObject[kCanonicalPoseUpKey.c_str()].GetArray();
    if (upVecArray.Size() != 3) {
      logErrorAndThrow(fmt::format("Invalid canonical pose up vector size {}", upVecArray.Size()));
    }
    canonicalPose.upVector = Eigen::Vector3d(
        upVecArray[0].GetDouble(), upVecArray[1].GetDouble(), upVecArray[2].GetDouble());
    auto frontVecArray = canonicalPoseObject[kCanonicalPoseFrontKey.c_str()].GetArray();
    if (frontVecArray.Size() != 3) {
      logErrorAndThrow(
          fmt::format("Invalid canonical pose front vector size {}", frontVecArray.Size()));
    }
    canonicalPose.frontVector = Eigen::Vector3d(
        frontVecArray[0].GetDouble(), frontVecArray[1].GetDouble(), frontVecArray[2].GetDouble());
  }
  return canonicalPose;
}

} // namespace

AriaDigitalTwinDataProvider::AriaDigitalTwinDataProvider(const AriaDigitalTwinDataPaths& dataPaths)
    : dataPaths_(dataPaths) {
  // Load raw VRS data
  if (!dataPaths_.ariaVrsFilePath.empty()) {
    dataProvider_ = createVrsDataProvider(dataPaths_.ariaVrsFilePath);
    if (!dataProvider_->supportsTimeDomain(
            vrs::StreamId::fromNumericName("1201-1") /*left_slam*/, TimeDomain::DeviceTime)) {
      XR_LOGW("At least left slam camera should contain device (capture) time domain");
      throw std::runtime_error{
          "At least left slam camera should contain device (capture) time domain"};
    }
  } else {
    XR_LOGI("skip loading VRS data because the data path is empty");
  }

  // Load GT data
  loadDatasetVersion();
  validateDatasetVersion();
  loadInstancesInfo();
  loadObjectAABBbboxes();
  loadAria3dPoses();
  loadObject3dBoundingBoxes();
  loadInstance2dBoundingBoxes();
  loadSegmentations();
  loadDepthImages();
  loadSyntheticVrs();
  loadSkeletonInfo();
  loadSkeletons();
  loadEyeGaze();
}

std::set<vrs::StreamId> AriaDigitalTwinDataProvider::getAriaAllStreams() const {
  if (!hasAriaData()) {
    XR_LOGW("Aria data is not loaded, cannot retrieve streams\n");
    return {};
  }
  return dataProvider_->getAllStreams();
}

std::vector<int64_t> AriaDigitalTwinDataProvider::getAriaDeviceCaptureTimestampsNs(
    const vrs::StreamId& streamId) const {
  if (!hasAriaData()) {
    XR_LOGW("Aria data is not loaded, cannot get device capture timestamps\n");
    return {};
  }
  return dataProvider_->getTimestampsNs(streamId, TimeDomain::DeviceTime);
}

AriaImageDataWithDt AriaDigitalTwinDataProvider::getAriaImageByTimestampNs(
    int64_t deviceTimeStampNs,
    const vrs::StreamId& streamId,
    const TimeQueryOptions& timeQueryOptions) const {
  if (!hasAriaData()) {
    XR_LOGW("Aria Images are empty, no vrs is provided\n");
    return AriaImageDataWithDt();
  }

  bool isActive = dataProvider_->checkStreamIsActive(streamId);

  if (!isActive) {
    XR_LOGW("Stream {} is not active, no image is available\n", streamId.getNumericName());
    return AriaImageDataWithDt();
  }

  // Get image at timestamp
  ImageDataAndRecord imageData = dataProvider_->getImageDataByTimeNs(
      streamId, deviceTimeStampNs, TimeDomain::DeviceTime, timeQueryOptions);

  // Check if image is valid
  if (!imageData.first.isValid()) {
    XR_LOGW("Invalid Aria image at {}\n", deviceTimeStampNs);
    return AriaImageDataWithDt();
  }

  return AriaImageDataWithDt(
      imageData.first, imageData.second.captureTimestampNs - deviceTimeStampNs);
}

int64_t AriaDigitalTwinDataProvider::getDeviceTimeFromTimecodeNs(int64_t timecodeNs) const {
  if (!hasAriaData()) {
    XR_LOGW("Aria Images are empty, no vrs is provided\n");
    return kInvalidDeviceTimestampNs;
  }
  return dataProvider_->convertFromTimeCodeToDeviceTimeNs(timecodeNs);
}

int64_t AriaDigitalTwinDataProvider::getTimecodeFromDeviceTimeNs(int64_t deviceTimeNs) const {
  if (!hasAriaData()) {
    XR_LOGW("Aria Images are empty, no vrs is provided\n");
    return kInvalidDeviceTimestampNs;
  }
  return dataProvider_->convertFromDeviceTimeToTimeCodeNs(deviceTimeNs);
}

std::optional<CameraCalibration> AriaDigitalTwinDataProvider::getAriaCameraCalibration(
    const vrs::StreamId& streamId) const {
  if (!hasAriaData()) {
    XR_LOGW("Aria Images are empty, no vrs is provided\n");
    return {};
  }
  const auto maybeLabel = dataProvider_->getLabelFromStreamId(streamId);
  if (!maybeLabel.has_value()) {
    XR_LOGE("StreamId not found in data: {}, returning empty result", streamId.getNumericName());
    return {};
  }
  return dataProvider_->getDeviceCalibration()->getCameraCalib(maybeLabel.value());
}

Sophus::SE3d AriaDigitalTwinDataProvider::getAria_T_Device_Camera(
    const vrs::StreamId& streamId) const {
  if (!hasAriaData()) {
    XR_LOGW("Aria Images are empty, no vrs is provided\n");
    return {};
  }
  const auto maybeLabel = dataProvider_->getLabelFromStreamId(streamId);
  if (!maybeLabel.has_value()) {
    XR_LOGE("StreamId not found in data: {}, returning empty result", streamId.getNumericName());
    throw std::runtime_error{"invalid stream ID"};
  }

  const auto maybeT_Device_Camera =
      dataProvider_->getDeviceCalibration()->getT_Device_Sensor(maybeLabel.value());
  if (!maybeT_Device_Camera) {
    XR_LOGE("could not get T_Device_Camera for stream {}", streamId.getNumericName());
    throw std::runtime_error{"invalid stream ID"};
  }
  return maybeT_Device_Camera.value();
}

Aria3dPoseDataWithDt AriaDigitalTwinDataProvider::getAria3dPoseByTimestampNs(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) const {
  if (!hasAria3dPoses()) {
    XR_LOGW("Aria 3D trajectory is empty\n");
    return Aria3dPoseDataWithDt();
  }

  auto queryPoseIter =
      queryTimestampsMap<Aria3dPose>(aria3dPoses_, deviceTimeStampNs, timeQueryOptions);

  if (queryPoseIter != aria3dPoses_.end()) {
    return Aria3dPoseDataWithDt(queryPoseIter->second, queryPoseIter->first - deviceTimeStampNs);
  } else {
    return Aria3dPoseDataWithDt();
  }
}

BoundingBox3dDataWithDt AriaDigitalTwinDataProvider::getObject3dBoundingBoxesByTimestampNs(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) const {
  if (!hasObject3dBoundingboxes()) {
    XR_LOGW("Object 3D poses is empty\n");
    return BoundingBox3dDataWithDt();
  }

  TypeBoundingBox3dMap object3dBoundingBoxMap;
  // first get all static objects regardless
  object3dBoundingBoxMap.insert(
      staticObject3dBoundingBoxes_.cbegin(), staticObject3dBoundingBoxes_.cend());

  // Check if dynamic objects are empty
  if (dynamicObject3dBoundingBoxSeries_.empty()) {
    return BoundingBox3dDataWithDt(object3dBoundingBoxMap, 0);
  }

  // Query dynamic objects according to timestamp
  auto queryDynamicObjectIter = queryTimestampsMap<TypeBoundingBox3dMap>(
      dynamicObject3dBoundingBoxSeries_, deviceTimeStampNs, timeQueryOptions);
  if (queryDynamicObjectIter != dynamicObject3dBoundingBoxSeries_.end()) {
    // valid result, insert all dynamic objects into the map and return
    object3dBoundingBoxMap.insert(
        queryDynamicObjectIter->second.cbegin(), queryDynamicObjectIter->second.cend());
    return BoundingBox3dDataWithDt(
        object3dBoundingBoxMap, queryDynamicObjectIter->first - deviceTimeStampNs);
  } else {
    // invalid result
    return BoundingBox3dDataWithDt();
  }
}

BoundingBox2dDataWithDt AriaDigitalTwinDataProvider::getObject2dBoundingBoxesByTimestampNs(
    int64_t deviceTimeStampNs,
    const vrs::StreamId& streamId,
    const TimeQueryOptions& timeQueryOptions) const {
  if (instance2dBoundingBoxes_.find(streamId) == instance2dBoundingBoxes_.end()) {
    XR_LOGW("Camera {} has no object 2d box data \n", streamId.getNumericName());
    return BoundingBox2dDataWithDt();
  }

  const auto& cameraBoxes = instance2dBoundingBoxes_.at(streamId);
  if (instance2dBoundingBoxes_.at(streamId).empty()) {
    XR_LOGW("No object 2d boxes for camera {}\n", streamId.getNumericName());
    return BoundingBox2dDataWithDt();
  }

  auto iter =
      queryTimestampsMap<TypeBoundingBox2dMap>(cameraBoxes, deviceTimeStampNs, timeQueryOptions);
  if (iter == cameraBoxes.end()) {
    XR_LOGW(
        "invalid query time for object 2d bounding box data of camera {}. Query {}Ns, data range: [{}, {}]Ns\n",
        streamId.getNumericName(),
        deviceTimeStampNs,
        cameraBoxes.begin()->first,
        cameraBoxes.rbegin()->first);
    return BoundingBox2dDataWithDt();
  }

  TypeBoundingBox2dMap result;
  for (const auto& [instanceId, bbox2d] : iter->second) {
    if (hasInstanceId(instanceId) &&
        getInstanceInfoById(instanceId).instanceType == InstanceType::Object) {
      result[instanceId] = bbox2d;
    }
  }
  return BoundingBox2dDataWithDt(result, iter->first - deviceTimeStampNs);
}

BoundingBox2dDataWithDt AriaDigitalTwinDataProvider::getSkeleton2dBoundingBoxesByTimestampNs(
    int64_t deviceTimeStampNs,
    const vrs::StreamId& streamId,
    const TimeQueryOptions& timeQueryOptions) const {
  if (instance2dBoundingBoxes_.find(streamId) == instance2dBoundingBoxes_.end()) {
    XR_LOGW("Camera {} has no skeleton 2d box data \n", streamId.getNumericName());
    return BoundingBox2dDataWithDt();
  }

  const auto& cameraBoxes = instance2dBoundingBoxes_.at(streamId);
  if (instance2dBoundingBoxes_.at(streamId).empty()) {
    XR_LOGW("No skeleton 2d boxes for camera {}\n", streamId.getNumericName());
    return BoundingBox2dDataWithDt();
  }

  auto iter =
      queryTimestampsMap<TypeBoundingBox2dMap>(cameraBoxes, deviceTimeStampNs, timeQueryOptions);
  if (iter == cameraBoxes.end()) {
    XR_LOGW(
        "invalid query time for skeleton 2d bounding box data of camera {}. Query {}Ns, data range: [{}, {}]Ns\n",
        streamId.getNumericName(),
        deviceTimeStampNs,
        cameraBoxes.begin()->first,
        cameraBoxes.rbegin()->first);
    return BoundingBox2dDataWithDt();
  }
  TypeBoundingBox2dMap result;
  for (const auto& [instanceId, bbox2d] : iter->second) {
    if (hasInstanceId(instanceId) &&
        getInstanceInfoById(instanceId).instanceType == InstanceType::Human) {
      result[instanceId] = bbox2d;
    }
  }
  return BoundingBox2dDataWithDt(result, iter->first - deviceTimeStampNs);
}

EyeGazeWithDt AriaDigitalTwinDataProvider::getEyeGazeByTimestampNs(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) const {
  if (eyeGazes_.empty()) {
    XR_LOGW("No eye gaze data\n");
    return EyeGazeWithDt();
  }

  auto iter = queryTimestampsMap<EyeGaze>(eyeGazes_, deviceTimeStampNs, timeQueryOptions);
  if (iter == eyeGazes_.end()) {
    XR_LOGW(
        "invalid query time for eye gaze data. Query {}Ns, data range: [{}, {}]Ns\n",
        deviceTimeStampNs,
        eyeGazes_.begin()->first,
        eyeGazes_.rbegin()->first);
    return EyeGazeWithDt();
  }
  return EyeGazeWithDt(iter->second, iter->first - deviceTimeStampNs);
}

SegmentationDataWithDt AriaDigitalTwinDataProvider::getSegmentationImageByTimestampNs(
    int64_t deviceTimeStampNs,
    const vrs::StreamId& streamId,
    const TimeQueryOptions& timeQueryOptions) const {
  if (!hasSegmentationImages()) {
    XR_LOGW("Segmentations is not available \n");
    return SegmentationDataWithDt();
  }

  SegmentationData segmentationData;
  int64_t gtTNs = 0;

  // Segmentation VRS stream ids are mapped to video VRS using its ImageConfig's Description.
  const auto allStreamIds = segmentationProvider_->getAllStreams();
  for (const auto& segStreamId : allStreamIds) {
    const auto& imgConfig = segmentationProvider_->getImageConfiguration(segStreamId);
    vrs::StreamId mappedStreamId = vrs::StreamId::fromNumericName(imgConfig.description);
    if (mappedStreamId == streamId) {
      const auto& sensorData = segmentationProvider_->getSensorDataByTimeNs(
          segStreamId, deviceTimeStampNs, TimeDomain::DeviceTime, timeQueryOptions);
      segmentationData = SegmentationData{sensorData.imageDataAndRecord().first};
      gtTNs = sensorData.imageDataAndRecord().second.captureTimestampNs;
      break;
    }
  }

  if (!segmentationData.isValid()) {
    return SegmentationDataWithDt();
  }

  return SegmentationDataWithDt(segmentationData, gtTNs - deviceTimeStampNs);
}

DepthDataWithDt AriaDigitalTwinDataProvider::getDepthImageByTimestampNs(
    int64_t deviceTimeStampNs,
    const vrs::StreamId& streamId,
    const TimeQueryOptions& timeQueryOptions) const {
  if (!hasDepthImages()) {
    XR_LOGW("Depth Images is empty\n");
    return DepthDataWithDt();
  }

  // Depth VRS stream ids are mapped to video VRS using its ImageConfig's Description.
  DepthData depthData;
  int64_t gtTNs = 0;
  const auto allStreamIds = depthImageProvider_->getAllStreams();
  for (const auto& depthStreamId : allStreamIds) {
    const auto& imgConfig = depthImageProvider_->getImageConfiguration(depthStreamId);
    vrs::StreamId mappedStreamId = vrs::StreamId::fromNumericName(imgConfig.description);
    if (mappedStreamId == streamId) {
      const auto& sensorData = depthImageProvider_->getSensorDataByTimeNs(
          depthStreamId, deviceTimeStampNs, TimeDomain::DeviceTime, timeQueryOptions);
      depthData = DepthData{sensorData.imageDataAndRecord().first};
      gtTNs = sensorData.imageDataAndRecord().second.captureTimestampNs;
      break;
    }
  }

  if (!depthData.isValid()) {
    return DepthDataWithDt();
  }

  return DepthDataWithDt(depthData, gtTNs - deviceTimeStampNs);
}

SyntheticDataWithDt AriaDigitalTwinDataProvider::getSyntheticImageByTimestampNs(
    int64_t deviceTimeStampNs,
    const vrs::StreamId& streamId,
    const TimeQueryOptions& timeQueryOptions) const {
  if (!hasSyntheticImages()) {
    XR_LOGW("Synthetic data is empty\n");
    return SyntheticDataWithDt();
  }

  // Synthetic VRS stream ids should be the same as the video VRS
  SyntheticData syntheticData;
  int64_t gtTNs = 0;
  const auto& sensorData = syntheticVrsProvider_->getSensorDataByTimeNs(
      streamId, deviceTimeStampNs, TimeDomain::DeviceTime, timeQueryOptions);
  syntheticData = SyntheticData{sensorData.imageDataAndRecord().first};
  gtTNs = sensorData.imageDataAndRecord().second.captureTimestampNs;

  if (!syntheticData.isValid()) {
    return SyntheticDataWithDt();
  }

  return SyntheticDataWithDt(syntheticData, gtTNs - deviceTimeStampNs);
}

void AriaDigitalTwinDataProvider::loadInstancesInfo() {
  XR_LOGI("loading instance info from json file {}", dataPaths_.instancesFilePath);
  fs::path fileInstances(dataPaths_.instancesFilePath);
  if (fileInstances.empty()) {
    XR_LOGI("skip loading instances because the data path is empty");
    return;
  }

  std::ifstream fileStream(fileInstances);

  if (!fileStream.is_open()) {
    XR_LOGE("Could not open instances: {} \n", fileInstances.string());
    throw std::runtime_error{"Could not open instances file: " + fileInstances.string()};
  }

  std::stringstream buffer;
  buffer << fileStream.rdbuf();
  rapidjson::Document jdoc;
  jdoc.Parse(buffer.str().c_str());
  const auto& jdocConst = jdoc;

  auto instancesInfoObject = jdocConst.GetObject();
  for (auto instanceInfoIter = instancesInfoObject.MemberBegin();
       instanceInfoIter != instancesInfoObject.MemberEnd();
       instanceInfoIter++) {
    auto& instanceInfoValue = instanceInfoIter->value;
    auto instanceInfoKey = instanceInfoIter->name.GetString();

    InstanceInfo instanceInfo;

    if (!instanceInfoValue.HasMember(kInstanceIdKey.c_str())) {
      logErrorAndThrow(fmt::format(kInstanceFileErrorTemplate, kInstanceIdKey, instanceInfoKey));
    }
    instanceInfo.id = instanceInfoValue[kInstanceIdKey.c_str()].GetUint64();

    if (!instanceInfoValue.HasMember(kInstanceNameKey.c_str())) {
      logErrorAndThrow(fmt::format(kInstanceFileErrorTemplate, kInstanceNameKey, instanceInfoKey));
    }
    instanceInfo.name = instanceInfoValue[kInstanceNameKey.c_str()].GetString();

    if (!instanceInfoValue.HasMember(kPrototypeNameKey.c_str())) {
      logErrorAndThrow(fmt::format(kInstanceFileErrorTemplate, kPrototypeNameKey, instanceInfoKey));
    }
    instanceInfo.prototypeName = instanceInfoValue[kPrototypeNameKey.c_str()].GetString();

    if (!instanceInfoValue.HasMember(kCategoryKey.c_str())) {
      logErrorAndThrow(fmt::format(kInstanceFileErrorTemplate, kCategoryKey, instanceInfoKey));
    }
    instanceInfo.category = instanceInfoValue[kCategoryKey.c_str()].GetString();

    if (!instanceInfoValue.HasMember(kCategoryUidKey.c_str())) {
      logErrorAndThrow(fmt::format(kInstanceFileErrorTemplate, kCategoryUidKey, instanceInfoKey));
    }
    instanceInfo.categoryUid = instanceInfoValue[kCategoryUidKey.c_str()].GetInt();

    if (!instanceInfoValue.HasMember(kMotionTypeKey.c_str())) {
      logErrorAndThrow(fmt::format(kInstanceFileErrorTemplate, kMotionTypeKey, instanceInfoKey));
    }
    const std::string motionTypeValue = instanceInfoValue[kMotionTypeKey.c_str()].GetString();
    instanceInfo.motionType = getMotionTypeFromInstancesJsonValue(motionTypeValue);
    if (instanceInfo.motionType == MotionType::Unknown) {
      XR_LOGW("Unknown motion type {} for instance id {}", motionTypeValue, instanceInfoKey);
    }

    if (!instanceInfoValue.HasMember(kRigidityKey.c_str())) {
      logErrorAndThrow(fmt::format(kInstanceFileErrorTemplate, kRigidityKey, instanceInfoKey));
    }
    const std::string rigidityTypeValue = instanceInfoValue[kRigidityKey.c_str()].GetString();
    instanceInfo.rigidityType = getRigidityFromInstancesJsonValue(rigidityTypeValue);
    if (instanceInfo.rigidityType == RigidityType::Unknown) {
      XR_LOGW("Unknown rigidity type {} for instance id {}", rigidityTypeValue, instanceInfoKey);
    }

    if (!instanceInfoValue.HasMember(kInstanceTypeKey.c_str())) {
      logErrorAndThrow(fmt::format(kInstanceFileErrorTemplate, kInstanceTypeKey, instanceInfoKey));
    }
    const std::string instanceTypeValue = instanceInfoValue[kInstanceTypeKey.c_str()].GetString();
    instanceInfo.instanceType = getInstanceTypeFromInstancesJsonValue(instanceTypeValue);
    if (instanceInfo.instanceType == InstanceType::Unknown) {
      XR_LOGW(
          "Unknown instance type {} for instance id {}, skipping.",
          instanceTypeValue,
          instanceInfoKey);
      continue;
    }

    // load symmetry
    if (!instanceInfoValue.HasMember(kRotationalSymmetryKey.c_str())) {
      logErrorAndThrow(
          fmt::format(kInstanceFileErrorTemplate, kRotationalSymmetryKey, instanceInfoKey));
    }
    auto rotationalSymmetryObject = instanceInfoValue[kRotationalSymmetryKey.c_str()].GetObject();
    instanceInfo.rotationalSymmetry =
        getRotationalSymmetryFromInstancesJsonObject(rotationalSymmetryObject, instanceInfoKey);

    // load canonical poses
    if (!instanceInfoValue.HasMember(kCanonicalPoseKey.c_str())) {
      logErrorAndThrow(fmt::format(kInstanceFileErrorTemplate, kCanonicalPoseKey, instanceInfoKey));
    }
    auto canonicalPoseObject = instanceInfoValue[kCanonicalPoseKey.c_str()].GetObject();
    instanceInfo.canonicalPose =
        getCanonicalPoseFromInstancesJsonObject(canonicalPoseObject, instanceInfoKey);

    instancesInfo_[instanceInfo.id] = instanceInfo;
  }

  fileStream.close();
}

void AriaDigitalTwinDataProvider::loadObjectAABBbboxes() {
  fs::path file3dBox(dataPaths_.objectBoundingBox3dFilePath);
  if (file3dBox.empty()) {
    XR_LOGI("skip loading file3dBox because the data path is empty");
    return;
  }
  std::ifstream fileStream = openFile(file3dBox);
  std::string line;
  std::vector<std::string> tokens;
  while (std::getline(fileStream, line)) {
    std::stringstream ss(line);
    getTokens(ss, tokens, ',', 8);

    InstanceId objId = std::stoull(tokens.at(0));
    if (objectIdToAabb_.find(objId) != objectIdToAabb_.end()) {
      XR_LOGE("Multiple 3d boxes for object id {}", objId);
      throw std::runtime_error{"invalid 3d bounding boxes"};
    }

    objectIdToAabb_[objId] = {
        std::stod(tokens.at(2)),
        std::stod(tokens.at(3)),
        std::stod(tokens.at(4)),
        std::stod(tokens.at(5)),
        std::stod(tokens.at(6)),
        std::stod(tokens.at(7))};

    tokens.clear();
  }
  // Close file
  fileStream.close();
}

void AriaDigitalTwinDataProvider::loadObject3dBoundingBoxes() {
  fs::path fileObjectTraj = fs::path(dataPaths_.objectTrajectoriesFilePath);
  if (fileObjectTraj.empty()) {
    XR_LOGI("skip loading fileObjectTraj because the data path is empty");
    return;
  }

  if (objectIdToAabb_.empty()) {
    loadObjectAABBbboxes();
  }

  std::ifstream fileStream = openFile(fileObjectTraj);
  std::string line;
  std::vector<std::string> tokens;
  std::set<InstanceId> dynamicObjectIds;
  std::set<InstanceId> staticObjectIds;
  while (std::getline(fileStream, line)) {
    tokens.clear();
    std::stringstream ss(line);
    getTokens(ss, tokens, ',', 9);
    int64_t deviceTimeStampNs = std::stoll(tokens.at(1));

    if (deviceTimeStampNs < 0 && deviceTimeStampNs != -1) {
      XR_LOGE(
          "Invalid time {}, object poses time only contain -1 and non-negative integer",
          deviceTimeStampNs);
      throw std::runtime_error{"invalid time"};
    }

    InstanceId objId = std::stoull(tokens.at(0));

    BoundingBox3dData object3dBoundingBoxdata;

    object3dBoundingBoxdata.T_Scene_Object.translation() = {
        std::stod(tokens.at(2)), std::stod(tokens.at(3)), std::stod(tokens.at(4))};

    // w, x, y, z
    object3dBoundingBoxdata.T_Scene_Object.setQuaternion(Eigen::Quaternion<double>(
        std::stod(tokens.at(5)),
        std::stod(tokens.at(6)),
        std::stod(tokens.at(7)),
        std::stod(tokens.at(8))));

    // Env objects are in object traj but not in aabb
    if (objectIdToAabb_.find(objId) == objectIdToAabb_.end()) {
      XR_LOGW("object id {} does not have AABB", objId);
      continue;
    }

    object3dBoundingBoxdata.aabb = objectIdToAabb_.at(objId);

    if (deviceTimeStampNs == -1) {
      if (dynamicObjectIds.find(objId) != dynamicObjectIds.find(objId)) {
        XR_LOGE("invalid object trajectory format, object {} is both static and dynamic", objId);
        throw std::runtime_error{"invalid object trajectory format"};
      }
      staticObjectIds.insert(objId);
      staticObject3dBoundingBoxes_[objId] = object3dBoundingBoxdata;
    } else {
      if (staticObjectIds.find(objId) != staticObjectIds.find(objId)) {
        XR_LOGE("invalid object trajectory format, object {} is both static and dynamic", objId);
        throw std::runtime_error{"invalid object trajectory format"};
      }
      dynamicObjectIds.insert(objId);
      dynamicObject3dBoundingBoxSeries_[deviceTimeStampNs][objId] = object3dBoundingBoxdata;
    }
  }
  fileStream.close();
}

void AriaDigitalTwinDataProvider::loadAria3dPoses() {
  if (dataPaths_.ariaTrajectoryFilePath.empty()) {
    XR_LOGI("skip loading fileAriaTraj because the data path is empty");
    return;
  }

  tools::mps::ClosedLoopTrajectory trajectory =
      tools::mps::readClosedLoopTrajectory(dataPaths_.ariaTrajectoryFilePath);
  for (const tools::mps::ClosedLoopTrajectoryPose& closedLoopPose : trajectory) {
    int64_t deviceTimeStampNs = closedLoopPose.trackingTimestamp.count() * 1000;
    Aria3dPose aria3dPose;
    aria3dPose.T_Scene_Device = closedLoopPose.T_world_device;
    aria3dPose.deviceLinearVelocity = closedLoopPose.deviceLinearVelocity_device;
    aria3dPose.deviceRotationalVelocity = closedLoopPose.angularVelocity_device;
    aria3dPose.gravityWorld = closedLoopPose.gravity_world;
    aria3dPose.graphUid = closedLoopPose.graphUid;
    aria3dPose.qualityScore = closedLoopPose.qualityScore;
    aria3dPoses_.emplace(deviceTimeStampNs, aria3dPose);
  }
}

void AriaDigitalTwinDataProvider::loadInstance2dBoundingBoxes() {
  fs::path fileBbox2d(dataPaths_.boundingBoxes2dFilePath);
  if (fileBbox2d.empty()) {
    XR_LOGI("skip loading 2dbboxes because the data path is empty");
    return;
  }

  vrs::StreamId streamId;
  std::ifstream fileStream = openFile(fileBbox2d);
  std::string line;
  std::vector<std::string> tokens;
  while (std::getline(fileStream, line)) {
    tokens.clear();
    std::stringstream ss(line);
    getTokens(ss, tokens, ',', 8);
    streamId = vrs::StreamId::fromNumericName(tokens.at(0));
    InstanceId objId = std::stoull(tokens.at(1));
    int64_t deviceTimeStampNs = std::stoll(tokens.at(2));

    instance2dBoundingBoxes_[streamId][deviceTimeStampNs][objId] = BoundingBox2dData{
        .boxRange = {
            std::stof(tokens.at(3)),
            std::stof(tokens.at(4)),
            std::stof(tokens.at(5)),
            std::stof(tokens.at(6))}};
    instance2dBoundingBoxes_[streamId][deviceTimeStampNs][objId].visibilityRatio =
        std::stof(tokens.at(7));
  }

  fileStream.close();
}

void AriaDigitalTwinDataProvider::loadEyeGaze() {
  if (dataPaths_.eyeGazesFilePath.empty()) {
    XR_LOGI("skip loading eyeGazesFilePath because the data path is empty");
    return;
  }

  // First use mps lib to load eye gaze as a vector
  std::vector<EyeGaze> eyeGazeVec =
      projectaria::tools::mps::readEyeGaze(dataPaths_.eyeGazesFilePath);

  // re-order eye gaze vector into a ordered map
  eyeGazes_.clear();
  for (const auto& e : eyeGazeVec) {
    eyeGazes_.emplace_hint(
        eyeGazes_.cend(), e.trackingTimestamp.count() * 1000, e); /* convert from ms to ns */
  }
}

void AriaDigitalTwinDataProvider::loadDatasetVersion() {
  if (dataPaths_.metaDataFilePath.empty()) {
    XR_LOGW(
        "No metadata file provided to data provider, setting the dataset version to {}.",
        kDatasetVersionUnknown);
    datasetVersion_ = kDatasetVersionUnknown;
    return;
  }

  std::ifstream fileStream(dataPaths_.metaDataFilePath);
  if (!fileStream.is_open()) {
    XR_LOGE("Could not open ground truth metadata file: {} \n", dataPaths_.metaDataFilePath);
    throw std::runtime_error{
        "Could not open ground truth metadata file: " + dataPaths_.metaDataFilePath};
  }

  std::stringstream buffer;
  buffer << fileStream.rdbuf();
  rapidjson::Document jdoc;
  jdoc.Parse(buffer.str().c_str());
  const auto& jdocConst = jdoc;

  if (jdocConst.HasMember(kDatasetNameKey.c_str()) &&
      jdocConst.HasMember(kDatasetVersionKey.c_str())) {
    datasetName_ = jdocConst[kDatasetNameKey.c_str()].GetString();
    datasetVersion_ = jdocConst[kDatasetVersionKey.c_str()].GetString();
  } else if (
      !jdocConst.HasMember(kDatasetNameKey.c_str()) &&
      !jdocConst.HasMember(kDatasetVersionKey.c_str())) {
    // set to default
    datasetName_ = kDatasetNameDefault;
    datasetVersion_ = kDatasetVersionDefault;
  } else {
    XR_LOGE(
        "invalid metadata file, both {} and {} fields are required",
        kDatasetNameKey,
        kDatasetVersionKey);
    throw std::invalid_argument{"invalid metadata file"};
  }
}

void AriaDigitalTwinDataProvider::validateDatasetVersion() const {
  if (kCorruptDatasets.find(dataPaths_.sequenceName) != kCorruptDatasets.end()) {
    std::cout << "\n\n\n[WARNING] BAD DATASET DETECTED\n";
    XR_LOGE(
        "Dataset {} has been flagged as corrupted, please use data carefully. Reason: {}. ",
        dataPaths_.sequenceName,
        kCorruptDatasets.at(dataPaths_.sequenceName));
  }

  if (datasetVersion_ == kDatasetVersionUnknown) {
    XR_LOGW(
        "Unknown dataset version, we recommend loading with the metadata file to validate the dataset version is compatible with this version of the data provider.");
    return;
  }

  if (kLatestDatasetVersions.find(datasetName_) == kLatestDatasetVersions.end()) {
    XR_LOGE("Invalid dataset name: {}", datasetName_);
    throw std::runtime_error{"invalid dataset name"};
  }

  std::string latestVersionStr = kLatestDatasetVersions.at(datasetName_);
  if (datasetVersion_ == latestVersionStr) {
    return;
  }

  // check format
  auto pos = datasetVersion_.find('.');
  if (pos == std::string::npos) {
    const std::string errMsg = fmt::format(
        "invalid metadata file. version: '{}' is of invalid type, required: XX.XX",
        datasetVersion_);
    XR_LOGE("{}", errMsg);
    throw std::runtime_error{errMsg};
  }

  // get dataset version release
  double datasetVersion = std::stod(datasetVersion_);

  // get latest data version
  double latestVersion = std::stod(latestVersionStr);

  // check versions
  if (datasetVersion < latestVersion) {
    XR_LOGW(
        "dataset version read ({}) is not up to date with latest ({}), we recommend you redownload your ADT dataset."
        " For a full version update history, please see the ADT wiki",
        datasetVersion_,
        latestVersionStr);
    return;
  }
  if (datasetVersion > latestVersion) {
    XR_LOGE(
        "data loader version ({}) is behind dataset version read ({}), please update projectaria_tools from github.",
        datasetVersion_,
        latestVersionStr);
    throw std::runtime_error{
        "data loader version is behind dataset version, projectaria_tools needs to be updated"};
  }
}

void AriaDigitalTwinDataProvider::loadSegmentations() {
  fs::path fileSeg(dataPaths_.segmentationsFilePath);
  if (fileSeg.empty()) {
    XR_LOGI("skip loading fileSegmentation because the data path is empty");
    return;
  }
  auto maybeSegmentationProvider = createVrsDataProvider(fileSeg.string());
  if (!maybeSegmentationProvider) {
    XR_LOGE("Segmentations cannot be loaded from {}", fileSeg.string());
    return;
  }
  segmentationProvider_ = std::make_shared<projectaria::tools::data_provider::VrsDataProvider>(
      *maybeSegmentationProvider);
}

void AriaDigitalTwinDataProvider::loadDepthImages() {
  fs::path fileDep(dataPaths_.depthImagesFilePath);
  if (fileDep.empty()) {
    XR_LOGI("skip loading fileDepth because the data path is empty");
    return;
  }
  auto maybeDepthImageProvider = createVrsDataProvider(fileDep.string());
  if (!maybeDepthImageProvider) {
    XR_LOGE("depth images cannot be loaded from {}", fileDep.string());
    return;
  }
  depthImageProvider_ = std::make_shared<projectaria::tools::data_provider::VrsDataProvider>(
      *maybeDepthImageProvider);
}

void AriaDigitalTwinDataProvider::loadSyntheticVrs() {
  fs::path fileSynthetic(dataPaths_.syntheticVrsFilePath);
  if (fileSynthetic.empty()) {
    XR_LOGI("skip loading fileSynthetic because the data path is empty");
    return;
  }
  auto syntheticVrsProvider = createVrsDataProvider(fileSynthetic.string());
  if (!syntheticVrsProvider) {
    XR_LOGW("Cannot load synthetic vrs at {}", fileSynthetic.string());
    return;
  }
  syntheticVrsProvider_ =
      std::make_shared<projectaria::tools::data_provider::VrsDataProvider>(*syntheticVrsProvider);
}

void AriaDigitalTwinDataProvider::loadSkeletonInfo() {
  if (dataPaths_.skeletonMetaDataFilePath.empty()) {
    XR_LOGI("skip loading skeletonMetaDataFilePath because the data path is empty");
    return;
  }
  XR_LOGI("loading skeleton info from json file {}", dataPaths_.skeletonMetaDataFilePath);

  std::ifstream fileStream(dataPaths_.skeletonMetaDataFilePath);
  if (!fileStream.is_open()) {
    XR_LOGE("Could not open skeleton info file: {} \n", dataPaths_.skeletonMetaDataFilePath);
    throw std::runtime_error{
        "Could not open instances file: " + dataPaths_.skeletonMetaDataFilePath};
  }

  std::stringstream buffer;
  buffer << fileStream.rdbuf();
  rapidjson::Document jdoc;
  jdoc.Parse(buffer.str().c_str());
  const auto& jdocConst = jdoc;

  if (!jdocConst.HasMember(kSkeletonMetadataKey.c_str())) {
    const std::string errMsg = fmt::format(
        "invalid skeleton metadata file. key: '{}' not available in json file",
        kSkeletonMetadataKey);
    XR_LOGE("{}", errMsg);
    throw std::runtime_error{errMsg};
  }

  for (const auto& metadataJ : jdocConst[kSkeletonMetadataKey.c_str()].GetArray()) {
    if (!metadataJ.HasMember(kSkeletonNameKey.c_str())) {
      const std::string errMsg = fmt::format(
          "invalid skeleton metadata file. key: '{}' not available in json file", kSkeletonNameKey);
      XR_LOGE("{}", errMsg);
      throw std::runtime_error{errMsg};
    }
    std::string name = metadataJ[kSkeletonNameKey.c_str()].GetString();

    if (name == kInvalidSkeletonName) {
      continue;
    }

    if (!metadataJ.HasMember(kSkeletonIdKey.c_str())) {
      const std::string errMsg = fmt::format(
          "invalid skeleton metadata file. key: '{}' not available in json file", kSkeletonIdKey);
      XR_LOGE("{}", errMsg);
      throw std::runtime_error{errMsg};
    }
    InstanceId id = metadataJ[kSkeletonIdKey.c_str()].GetInt64();

    // find InstanceInfo for this skeleton
    if (instancesInfo_.find(id) == instancesInfo_.end()) {
      std::string errMsg = "skeleton ID from metadata file not found in instances file";
      XR_LOGE("{}", errMsg);
      throw std::runtime_error{errMsg};
    }
    InstanceInfo& info = instancesInfo_.at(id);

    // check names match
    if (name != info.name) {
      std::string errMsg = fmt::format(
          "skeleton name in skeleton metadata json ({}) does not match that in instances json ({})",
          name,
          info.name);
      XR_LOGE("{}", errMsg);
      throw std::runtime_error{errMsg};
    }

    // get device association
    if (!metadataJ.HasMember(kSkeletonDeviceSerialKey.c_str())) {
      const std::string errMsg = fmt::format(
          "invalid skeleton metadata file. key: '{}' not available in json file",
          kSkeletonDeviceSerialKey);
      XR_LOGE("{}", errMsg);
      throw std::runtime_error{errMsg};
    }
    std::string associatedDeviceSerial = metadataJ[kSkeletonDeviceSerialKey.c_str()].GetString();

    info.associatedDeviceSerial = associatedDeviceSerial;
  }
}

void AriaDigitalTwinDataProvider::loadSkeletons() {
  if (dataPaths_.skeletonsFilePaths.empty()) {
    XR_LOGI("skip loading skeletonsFilePaths because the data path is empty");
    return;
  }
  for (const auto& [skeletonId, skeletonFilePath] : dataPaths_.skeletonsFilePaths) {
    skeletons_.emplace(skeletonId, AriaDigitalTwinSkeletonProvider(skeletonFilePath));
  }
}

SkeletonFrameWithDt AriaDigitalTwinDataProvider::getSkeletonByTimestampNs(
    int64_t deviceTimeStampNs,
    InstanceId instanceId,
    const TimeQueryOptions& timeQueryOptions) const {
  auto iter = skeletons_.find(instanceId);
  if (iter == skeletons_.end()) {
    XR_LOGW("no skeleton with instance id: {}", instanceId);
    return {};
  }
  return iter->second.getSkeletonByTimestampNs(deviceTimeStampNs, timeQueryOptions);
}

std::vector<InstanceId> AriaDigitalTwinDataProvider::getInstanceIds() const {
  std::vector<InstanceId> instanceIds;
  for (const auto& [instanceId, _] : instancesInfo_) {
    instanceIds.push_back(instanceId);
  }
  return instanceIds;
}

bool AriaDigitalTwinDataProvider::hasInstanceId(InstanceId instanceId) const {
  return instancesInfo_.find(instanceId) != instancesInfo_.end();
}

const InstanceInfo& AriaDigitalTwinDataProvider::getInstanceInfoById(InstanceId instanceId) const {
  if (instancesInfo_.find(instanceId) == instancesInfo_.end()) {
    throw std::runtime_error(fmt::format("No such instance loaded {}", instanceId));
  }
  return instancesInfo_.at(instanceId);
}

std::vector<InstanceId> AriaDigitalTwinDataProvider::getObjectIds() const {
  std::vector<InstanceId> objectIds;
  for (const auto& [id, info] : instancesInfo_) {
    if (info.instanceType == InstanceType::Object) {
      objectIds.push_back(id);
    }
  }
  return objectIds;
}

std::vector<InstanceId> AriaDigitalTwinDataProvider::getSkeletonIds() const {
  std::vector<InstanceId> skeletonIds;
  for (const auto& [id, info] : instancesInfo_) {
    if (info.instanceType == InstanceType::Human) {
      skeletonIds.push_back(id);
    }
  }
  return skeletonIds;
}

const AriaDigitalTwinSkeletonProvider& AriaDigitalTwinDataProvider::getSkeletonProvider(
    InstanceId instanceId) const {
  if (skeletons_.find(instanceId) == skeletons_.end()) {
    throw std::runtime_error(fmt::format("No skeleton with instance id {}", instanceId));
  }
  return skeletons_.at(instanceId);
}

Aria3dPoseDataWithDt getInterpolatedAria3dPoseAtTimestampNs(
    const AriaDigitalTwinDataProvider& provider,
    int64_t deviceTimeStampNs) {
  // Check validity
  if (!provider.hasAria3dPoses()) {
    XR_LOGW("Aria 3D trajectory is empty, query will return empty result\n");
    return Aria3dPoseDataWithDt();
  }

  const Aria3dPoseDataWithDt poseBefore =
      provider.getAria3dPoseByTimestampNs(deviceTimeStampNs, TimeQueryOptions::Before);
  const Aria3dPoseDataWithDt poseAfter =
      provider.getAria3dPoseByTimestampNs(deviceTimeStampNs, TimeQueryOptions::After);

  // Handle out of range cases
  if (!poseBefore.isValid() && !poseAfter.isValid()) {
    XR_LOGE("Both before and after poses are invalid, this shouldn't happen.");
    throw std::runtime_error{"invalid poses for both before and after queried time."};
  } else if (!poseBefore.isValid() && poseAfter.isValid()) { // query before min, return min
    return poseAfter;
  } else if (poseBefore.isValid() && !poseAfter.isValid()) { // query after max, return max
    return poseBefore;
  } else if (poseBefore.dtNs() == poseAfter.dtNs()) { // exact match, return either
    return poseBefore;
  }

  // otherwise, interpolate between Before and After.
  // where alpha = -(BeforeTime - deviceTimeStampNs) / (AfterTime - deviceTimeStampNs - BeforeTime
  // + deviceTimeStampNs)
  double alpha = static_cast<double>(-poseBefore.dtNs()) /
      static_cast<double>(poseAfter.dtNs() - poseBefore.dtNs());
  Aria3dPose resultAria3dPose;
  resultAria3dPose.T_Scene_Device =
      Sophus::interpolate(poseBefore.data().T_Scene_Device, poseAfter.data().T_Scene_Device, alpha);

  resultAria3dPose.deviceLinearVelocity = poseBefore.data().deviceLinearVelocity +
      alpha * (poseAfter.data().deviceLinearVelocity - poseBefore.data().deviceLinearVelocity);

  resultAria3dPose.deviceRotationalVelocity = poseBefore.data().deviceRotationalVelocity +
      alpha *
          (poseAfter.data().deviceRotationalVelocity - poseBefore.data().deviceRotationalVelocity);

  return Aria3dPoseDataWithDt(resultAria3dPose, 0);
}

BoundingBox3dDataWithDt getInterpolatedObject3dBoundingBoxesAtTimestampNs(
    const AriaDigitalTwinDataProvider& provider,
    int64_t deviceTimeStampNs) {
  // Check validity
  if (!provider.hasObject3dBoundingboxes()) {
    XR_LOGW("Object 3D poses is empty, query will return empty result\n");
    return BoundingBox3dDataWithDt();
  }

  const BoundingBox3dDataWithDt objectsBefore =
      provider.getObject3dBoundingBoxesByTimestampNs(deviceTimeStampNs, TimeQueryOptions::Before);
  const BoundingBox3dDataWithDt objectsAfter =
      provider.getObject3dBoundingBoxesByTimestampNs(deviceTimeStampNs, TimeQueryOptions::After);

  // Handle out of range cases
  if (!objectsBefore.isValid() && !objectsAfter.isValid()) {
    XR_LOGE("Both before and after object 3D bounding boxes are invalid, this shouldn't happen.");
    throw std::runtime_error{
        "invalid object 3D bounding boxes for both before and after queried time."};
  } else if (!objectsBefore.isValid() && objectsAfter.isValid()) { // query before min, return min
    return objectsAfter;
  } else if (objectsBefore.isValid() && !objectsAfter.isValid()) { // query after max, return max
    return objectsBefore;
  } else if (objectsBefore.dtNs() == objectsAfter.dtNs()) { // exact match, return either
    return objectsBefore;
  }

  // otherwise, interpolate between Before and After.
  // only insert dynamic objects existing in both frames.
  // where alpha = -(BeforeTime - deviceTimeStampNs) / (AfterTime - deviceTimeStampNs - BeforeTime
  // + deviceTimeStampNs)
  TypeBoundingBox3dMap object3dBoundingBoxMap;
  double alpha = static_cast<double>(-objectsBefore.dtNs()) /
      static_cast<double>(objectsAfter.dtNs() - objectsBefore.dtNs());

  for (const auto& [objId, objectPoseAfter] : objectsAfter.data()) {
    if (objectsBefore.data().count(objId) > 0) {
      const auto& objectPoseBefore = objectsBefore.data().at(objId);
      // interpolate 6dof poses
      object3dBoundingBoxMap[objId].T_Scene_Object = Sophus::interpolate(
          objectPoseBefore.T_Scene_Object, objectPoseAfter.T_Scene_Object, alpha);
      // interpolate aabb
      object3dBoundingBoxMap[objId].aabb = (objectPoseBefore.aabb + objectPoseAfter.aabb) / 2.0f;
    }
  }
  return BoundingBox3dDataWithDt(object3dBoundingBoxMap, 0);
}

} // namespace projectaria::dataset::adt
