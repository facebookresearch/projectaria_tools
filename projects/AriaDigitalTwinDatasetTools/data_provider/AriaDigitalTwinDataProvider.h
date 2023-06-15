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

#include <cstddef>
#include <cstdint>
#include <optional>

#include "AriaDigitalTwinDataPathsProvider.h"
#include "AriaDigitalTwinDataTypes.h"
#include "AriaDigitalTwinSkeletonProvider.h"
#include "AriaDigitalTwinUtils.h"
#include "data_provider/VrsDataProvider.h"
#include "data_provider/players/ImageSensorPlayer.h"

namespace projectaria::dataset::adt {

/**
 * @brief This is the core data loader that should provide all the data access you will need for
 * an ADT sequence. Note that each sequence may contain multiple devices, you should create one
 * `AriaDigitalTwinDataProvider` instance for each device.
 */
class AriaDigitalTwinDataProvider {
 public:
  explicit AriaDigitalTwinDataProvider(const AriaDigitalTwinDataPaths& dataPaths);

  // ---- Query ADT Aria data ----
  /**
   * @brief Get a list of all VRS stream ids from the Aria recording. StreamId is the unique
   * identifier to query different sensor data in ADT DataProvider.
   * @return a set of StreamIds.
   */
  std::set<vrs::StreamId> getAriaAllStreams() const;

  /**
   * @brief Get all timestamps (in ns) of all observations of an Aria sensor, in
   * `TimeDomain::DeviceTime`.
   * @param streamId The stream id of the Aria sensor.
   * @return a vector of timestamps, which are already sorted in ascending order.
   */
  std::vector<int64_t> getAriaDeviceCaptureTimestampsNs(const vrs::StreamId& streamId) const;

  /**
   * @brief Query an Aria camera image by timestamp.
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param streamId The stream id of the Aria camera.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Default to
   * CLOSEST.
   * @return A `AriaImageDataWithDt` object. Will return invalid if query fails.
   */
  AriaImageDataWithDt getAriaImageByTimestampNs(
      int64_t deviceTimeStampNs,
      const vrs::StreamId& streamId,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief Get the pose of an Aria camera in the device coordinate frame.
   * @param streamId The stream id of the Aria camera.
   * @return T_Device_Camera, which can be used as: <br>
   *  `point_in_device = T_Device_Camera * point_in_camera.`
   */
  Sophus::SE3d getAria_T_Device_Camera(const vrs::StreamId& streamId) const;

  // get the camera calibration of a Aria camera which includes intrinsics, distortion parameters
  // and projection functions

  /**
   * @brief Get the camera calibration of an Aria camera, including intrinsics, distortion params,
   * and projection functions.
   * @param streamId The stream id of the Aria camera.
   * @return
   * @parblock
   * - (if successful): a struct including the calibration parameters of the queried camera.
   * - (if failed): nullopt.
   * @endparblock
   */
  std::optional<tools::calibration::CameraCalibration> getAriaCameraCalibration(
      const vrs::StreamId& streamId) const;

  /**
   * @brief ADT uses Timecode to synchronize multiple Aria devices. Use this function to convert a
   * timestamp from `TimeDomain::TimeCode` to `TimeDomain::DeviceTime`. See
   * `adt_multiperson_tutorial.ipynb` for usage example.
   */
  int64_t getDeviceTimeFromTimecodeNs(int64_t timecodeNs) const;

  /**
   * @brief ADT uses Timecode to synchronize multiple Aria devices. Use this function to convert a
   * timestamp from `TimeDomain::DeviceTime` to `TimeDomain::TimeCode`. See
   * `adt_multiperson_tutorial.ipynb` for usage example.
   */
  int64_t getTimecodeFromDeviceTimeNs(int64_t deviceTimeNs) const;

  // ---- Query ADT ground truth data by time ----
  // ---- The query timestamp is in device time domain ----
  /**
   * @brief Query the device pose of the Aria unit in the trajectory, by timestamp.
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Default to
   * CLOSEST.
   * @return A `Aria3dPoseWithDt` object containing `T_Scene_Aria`, along with linear and rotational
   * velocity at queried timestamp. Will return invalid if query fails.
   */
  Aria3dPoseDataWithDt getAria3dPoseByTimestampNs(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief Query object 3D bounding boxes by timestamp.
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Default to
   * CLOSEST.
   * @return a full map of objectId <-> 3d bounding box at query timestamp. Will return invalid if
   * query fails.
   */
  BoundingBox3dDataWithDt getObject3dBoundingBoxesByTimestampNs(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief Query 2D object bounding boxes by timestamp, in the view of a given camera.
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param streamId The stream id of the camera, need to specify this because different camera may
   * see different objects at a given timestamp.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Default to
   * CLOSEST.
   * @return a map of instanceId <-> object 2d bounding box. Note that only instances with its
   * `InstanceType == Object` will be included in the map. Will return invalid if query fails.
   */
  BoundingBox2dDataWithDt getObject2dBoundingBoxesByTimestampNs(
      int64_t deviceTimeStampNs,
      const vrs::StreamId& streamId,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;
  /**
   * @brief Query 2D skeleton bounding boxes by timestamp, in the view of a given camera.
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param streamId The stream id of the camera, need to specify this because different camera may
   * see different skeletons at a given timestamp.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Default to
   * CLOSEST.
   * @return a map of instanceId <-> skeleton 2d bounding box.  Note that only instances with its
   * `InstanceType == Human` will be included in the map. Will return invalid if query fails.
   */
  BoundingBox2dDataWithDt getSkeleton2dBoundingBoxesByTimestampNs(
      int64_t deviceTimeStampNs,
      const vrs::StreamId& streamId,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief Query a camera's segmentation image by timestamp
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param streamId The stream id of the Aria camera.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Default to
   * CLOSEST.
   * @return A `SegmentationImageDataWithDt` object. Will return invalid if query fails.
   */
  SegmentationDataWithDt getSegmentationImageByTimestampNs(
      int64_t deviceTimeStampNs,
      const vrs::StreamId& streamId,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief Query a camera's depth image by timestamp
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param streamId The stream id of the Aria camera.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Default to
   * CLOSEST.
   * @return A `DepthImageDataWithDt` object. Will return invalid if query fails.
   */
  DepthDataWithDt getDepthImageByTimestampNs(
      int64_t deviceTimeStampNs,
      const vrs::StreamId& streamId,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief Query a camera's synthetic image by timestamp
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param streamId The stream id of the Aria camera.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Default to
   * CLOSEST.
   * @return A `SyntheticImageDataWithDt` object. Will return invalid if query fails.
   */
  SyntheticDataWithDt getSyntheticImageByTimestampNs(
      int64_t deviceTimeStampNs,
      const vrs::StreamId& streamId,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief Query eye gaze by timestamp. The eye gaze is in Central-Pupil-Frame (CPF).
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Default to
   * CLOSEST.
   * @return A `EyeGazeWithDt` object. Will return invalid if query fails.
   */
  EyeGazeWithDt getEyeGazeByTimestampNs(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief Query the skeleton frame by timestamp for a specific skeleton.
   * @param deviceTimeStampNs The query timestamp in `TimeDomain::DeviceTime`.
   * @param instanceId The InstanceId of the skeleton to query.
   * @param timeQueryOptions The options for TimeQuery, one of {BEFORE, AFTER, CLOSEST}. Default to
   * CLOSEST.
   * @return A `SkeletonFrameWithDt` object. Will return invalid if query fails.
   */
  SkeletonFrameWithDt getSkeletonByTimestampNs(
      int64_t deviceTimeStampNs,
      InstanceId instanceId,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  // ---- Functions to check availability of ground-truth data ----
  bool hasAriaData() const {
    return dataProvider_ != nullptr;
  }

  bool hasAria3dPoses() const {
    return !aria3dPoses_.empty();
  }

  bool hasObject3dBoundingboxes() const {
    return !dynamicObject3dBoundingBoxSeries_.empty() || !staticObject3dBoundingBoxes_.empty();
  }

  bool hasInstance2dBoundingBoxes() const {
    return !instance2dBoundingBoxes_.empty();
  }

  bool hasSegmentationImages() const {
    return segmentationProvider_ != nullptr;
  }

  bool hasDepthImages() const {
    return depthImageProvider_ != nullptr;
  }

  bool hasSyntheticImages() const {
    return syntheticVrsProvider_ != nullptr;
  }

  bool hasEyeGaze() const {
    return !eyeGazes_.empty();
  }

  bool hasSkeleton() const {
    return !skeletons_.empty();
  }

  bool hasInstancesInfo() const {
    return !instancesInfo_.empty();
  }

  // ---- Query information in this subsequence ----
  /**
   * @brief get all instance ids as a vector, including all `InstanceType`s.
   */
  std::vector<InstanceId> getInstanceIds() const;

  /**
   * @brief query if an instance exists in current data.
   */
  bool hasInstanceId(InstanceId instanceId) const;

  /**
   * @brief get instance information by instance id.
   */
  const InstanceInfo& getInstanceInfoById(InstanceId instanceId) const;

  /**
   * @brief get all instance ids as a vector whose `InstanceType == OBJECT`.
   */
  std::vector<InstanceId> getObjectIds() const;

  /**
   * @brief get all instance ids as a vector whose `InstanceType == HUMAN`.
   */
  std::vector<InstanceId> getSkeletonIds() const;

  // ---- Query time range of ground-truth data for a sequence ----
  int64_t getStartTimeNs() const {
    return aria3dPoses_.empty() ? 0 : aria3dPoses_.begin()->first;
  }

  int64_t getEndTimeNs() const {
    return aria3dPoses_.empty() ? 0 : aria3dPoses_.rbegin()->first;
  }

  // ---- Getter of all stored data provider pointers within ADTDataProvider----
  // Also provide API to the core data provider itself.
  const std::shared_ptr<const tools::data_provider::VrsDataProvider> rawDataProviderPtr() const {
    return dataProvider_;
  }
  const std::shared_ptr<const tools::data_provider::VrsDataProvider> segmentationDataProviderPtr()
      const {
    return segmentationProvider_;
  }
  const std::shared_ptr<const tools::data_provider::VrsDataProvider> depthDataProviderPtr() const {
    return depthImageProvider_;
  }
  const std::shared_ptr<const tools::data_provider::VrsDataProvider> syntheticDataProviderPtr()
      const {
    return syntheticVrsProvider_;
  }

  const AriaDigitalTwinSkeletonProvider& getSkeletonProvider(InstanceId instanceId) const;

 private:
  void loadAria3dPoses();
  void loadObject3dBoundingBoxes();
  void loadInstance2dBoundingBoxes();
  void loadSegmentations();
  void loadDepthImages();
  void loadSyntheticVrs();
  void loadSkeletonInfo();
  void loadSkeletons();
  void loadEyeGaze();

  void loadObjectAABBbboxes();
  void loadInstancesInfo();

  // data paths that are used to load all ground truth data
  AriaDigitalTwinDataPaths dataPaths_;

  // vrs provider for raw vrs data
  std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> dataProvider_;

  // <ts, aria pose in global coordinate>
  std::map<int64_t, Aria3dPose> aria3dPoses_;
  // <ts, Object3dBoundingBoxMap> for dynamic objects
  std::map<int64_t, TypeBoundingBox3dMap> dynamicObject3dBoundingBoxSeries_;
  // <Object3dBoundingBoxMap> for static objects, ts is not needed
  TypeBoundingBox3dMap staticObject3dBoundingBoxes_;
  // 2D bboxes for instances <streamId, <ts, BoundingBox2dMap> > >
  // including both objects and skeletons
  std::unordered_map<
      vrs::StreamId,
      std::map<int64_t, TypeBoundingBox2dMap>,
      ::projectaria::dataset::adt::StreamIdHash>
      instance2dBoundingBoxes_;
  // vrs provider for segmentation
  std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> segmentationProvider_;
  // vrs provider for depth images
  std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> depthImageProvider_;
  // vrs provider for synthetic images
  std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> syntheticVrsProvider_ =
      nullptr;
  // <ts, EyeGaze>, sorted in device timestamps
  std::map<int64_t, EyeGaze> eyeGazes_;
  // all skeleton providers
  std::unordered_map<InstanceId, AriaDigitalTwinSkeletonProvider> skeletons_;
  // <objId, 3d bbox in object's local coordinate>
  std::unordered_map<InstanceId, Vector6d> objectIdToAabb_;
  std::unordered_map<InstanceId, InstanceInfo> instancesInfo_;
};

/**
 * @brief helper function to return an interpolated Aria3dPose given a query timestamp
 * @param provider: the Data Provider to query pose from.
 * @param deviceTimeStampNs: query time.
 * @return Aria3dPoseDataWithDt, where Dt = PoseTime - deviceTimeStampNs. Dt should be 0 unless the
 * query time is out of the range of provider data, where the first or the last sample is returned.
 */
Aria3dPoseDataWithDt getInterpolatedAria3dPoseAtTimestampNs(
    const AriaDigitalTwinDataProvider& provider,
    int64_t deviceTimeStampNs);

/**
 * @brief helper function to return an interpolated object 3D bounding box given a query timestamp
 * @param provider: the Data Provider to query pose from.
 * @param deviceTimeStampNs: query time.
 * @return ObjectBoundingBox3dDataWithDt, where Dt = PoseTime - deviceTimeStampNs. Dt should be 0
 * unless unless the query time is out of the range of provider data, where the first or the last
 * sample is returned.
 */
BoundingBox3dDataWithDt getInterpolatedObject3dBoundingBoxesAtTimestampNs(
    const AriaDigitalTwinDataProvider& provider,
    int64_t deviceTimeStampNs);

} // namespace projectaria::dataset::adt
