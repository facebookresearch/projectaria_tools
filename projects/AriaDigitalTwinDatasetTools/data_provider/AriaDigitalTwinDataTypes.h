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

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <data_provider/TimeTypes.h>
#include <data_provider/VrsDataProvider.h>
#include <mps/EyeGaze.h>
#include <sophus/se3.hpp>

namespace projectaria::dataset::adt {
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector3uc = Eigen::Matrix<unsigned char, 3, 1>;
using InstanceId = uint64_t;

/**
 * @brief a templated base class for storing ADT DataProvider's query result, including raw data,
 * delta_time, and a flag to indicate if the data is valid or not.
 */
template <typename T>
class DataWithDt {
 public:
  /**
   * @brief Default constructor.
   *
   * Constructs an empty instance of DataWithDt, by default it is <b>invalid</b>
   */
  DataWithDt() = default;
  DataWithDt(const T& inputData, int64_t inputDtNs)
      : data_(inputData), dtNs_(inputDtNs), isValid_(true) {}
  DataWithDt(T&& inputData, int64_t&& inputDtNs)
      : data_(std::move(inputData)), dtNs_(inputDtNs), isValid_(true) {}
  /**
   * @brief Get a reference to the raw ground-truth data.
   * @return The <b>reference</b> of the private data member.
   */
  const T& data() const {
    if (!isValid_) {
      fmt::print(
          "WARNING: attempting to access invalid data!\n"
          "make sure you check that DataWithDt is valid before accessing "
          "the data\n");
    }
    return data_;
  }

  /**
   * @brief Get the value of delta_time in nanoseconds = data_time - query_time
   */
  [[nodiscard]] int64_t dtNs() const {
    return dtNs_;
  }

  /**
   * @brief Get the value of the status flag for data validity
   */
  [[nodiscard]] bool isValid() const {
    return isValid_;
  }

 private:
  T data_;
  int64_t dtNs_;
  bool isValid_{false};
};

// ---- Ground truth basic type ----
/**
 * @brief a simple struct to represent the pose of an Aria device at a certain time
 */
struct Aria3dPose {
  // T_target_source
  Sophus::SE3d T_Scene_Device; /**< Pose of the Device at a certain time: @code{.cpp} point_in_scene
                                  = T_Scene_Device * point_in_device @endcode */
  Sophus::Vector3d deviceLinearVelocity; /**< linear velocity of the device in meter/s */
  Sophus::Vector3d deviceRotationalVelocity; /**< rotation velocity of the device in rad/s */
  Eigen::Vector3d
      gravityWorld; /**< Earth gravity vector in world frame. This vector is pointing toward the
     ground, and includes gravitation and centrifugal forces from earth rotation. */
  std::string
      graphUid; /**< Unique identifier of the world coordinate frame. When the graphUid is the same,
                   poses, velocities and point clouds are defined in the same coordinate frame. */
  float qualityScore; /**< Value between 0 and 1 which describes the tracking quality. The value is
                         determined based on Motive's mean tracking error where 0 quality
                         corresponds to 5 mm error or more */
};

/**
 * @brief a simple struct to represent a 3D bounding box for an instance
 */
struct BoundingBox3dData {
  Sophus::SE3d T_Scene_Object; /**< object 6DoF pose in the scene (world) @code{.cpp} point_in_scene
                                  = T_Scene_Object * point_in_object @endcode */
  Vector6d aabb; /**<object AABB (axes-aligned-bounding-box) in the object's local coordinate frame,
                    AABB is represented in [xmin, xmax, ymin, ymax, zmin, zmax] */
};

/**
 * @brief a simple struct to represent a 2D bounding box for an instance
 */
struct BoundingBox2dData {
  Eigen::Vector4f boxRange; /**< [xmin, xmax, ymin, ymax] */
  float visibilityRatio; /**< visibilityRatio calculated by occlusion between objects: <br>
                          * <pre>
                          *     visibilityRatio = 1. An object is not occluded by any other object
                          *     visibilityRatio = 0. An object is fully occluded by other objects
                          * </pre>
                          */
};

/**
 * @brief A class to represent segmentation image
 */
class SegmentationData : public projectaria::tools::data_provider::ImageData {
 public:
  /**
   * @brief Get stored image as 3-channel uint8 format.
   * @return A copy of the image.
   */
  [[nodiscard]] projectaria::tools::image::ManagedImage3U8 getVisualizable() const;
};

/**
 * @brief A class to represent depth image
 */
class DepthData : public projectaria::tools::data_provider::ImageData {
 public:
  /**
   * @brief Get stored image as 3-channel uint8 format.
   * @return A copy of the image.
   */
  [[nodiscard]] projectaria::tools::image::ManagedImageU8 getVisualizable() const;
};

/**
 * @brief A class to represent synthetic image
 */
class SyntheticData : public projectaria::tools::data_provider::ImageData {
 public:
  /**
   * @brief Get stored image as an image variant.
   * @return an image variant.
   */
  [[nodiscard]] projectaria::tools::image::ImageVariant getVisualizable() const {
    return imageVariant().value();
  }
};

/**
 * @brief EyeGaze data type is re-used from MPS lib
 */
using EyeGaze = projectaria::tools::mps::EyeGaze;
/**
 * @brief ordered map from timestamp -> EyeGaze
 */
using EyeGazesSeries = std::map<int64_t, EyeGaze>;

/**
 * @brief A simple struct to represent a frame of skeleton data
 */
struct SkeletonFrame {
  std::vector<Eigen::Vector3d>
      markers; /**< a vector representing optitrack markers of a skeleton */
  std::vector<Eigen::Vector3d>
      joints; /**< a vector representing the joint locations of a skeleton */
};

/**
 * @brief unordered map from InstanceId -> 2d bounding box data
 */
using TypeBoundingBox2dMap = std::unordered_map<InstanceId, BoundingBox2dData>;
/**
 * @brief unordered map from InstanceId -> 3d bounding box data
 */
using TypeBoundingBox3dMap = std::unordered_map<InstanceId, BoundingBox3dData>;

// ---- Ground truth query result type ----
// variables of type DataWithDt :
/**
 * @name a variety of ground truth query result types based on DataWithDt.
 * @{
 */
/**
 * @brief query result containing Aria image.
 */
using AriaImageDataWithDt = DataWithDt<projectaria::tools::data_provider::ImageData>;
/**
 * @brief query result containing Aria device pose.
 */
using Aria3dPoseDataWithDt = DataWithDt<Aria3dPose>;
/**
 * @brief query result containing an instance 3D bounding box.
 */
using BoundingBox3dDataWithDt = DataWithDt<TypeBoundingBox3dMap>;
/**
 * @brief query result containing an instance sD bounding box.
 */
using BoundingBox2dDataWithDt = DataWithDt<TypeBoundingBox2dMap>;
/**
 * @brief query result containing segmentation image.
 */
using SegmentationDataWithDt = DataWithDt<SegmentationData>;
/**
 * @brief query result containing depth image.
 */
using DepthDataWithDt = DataWithDt<DepthData>;
/**
 * @brief query result containing synthetic image.
 */
using SyntheticDataWithDt = DataWithDt<SyntheticData>;
/**
 * @brief query result containing eye gaze data.
 */
using EyeGazeWithDt = DataWithDt<EyeGaze>;
/**
 * @brief query result containing skeleton frame data.
 */
using SkeletonFrameWithDt = DataWithDt<SkeletonFrame>;

/** @} */

// ---- Instance information ----
/**
 * @brief Instance Type, An instance can be either an object or a human.
 */
enum class InstanceType {
  Unknown,
  Object,
  Human,
};

/**
 * @brief Instance's rigidity Type, An instance can be either rigid or deformable
 */
enum class RigidityType {
  Unknown,
  Rigid,
  Deformable,
};

/**
 * @brief Instance's motion Type, An instance can be either static or dynamic
 */
enum class MotionType {
  Unknown,
  Static,
  Dynamic,
};

/**
 * @brief A struct representing the rotational symmetry axis of an instance
 */
struct RotationalSymmetryAxis {
  Eigen::Vector3d axis; /**< rotational symmetric axis */
  float angleDegree; /**< */
  [[nodiscard]] std::string toString() const;
};

/**
 * @brief A struct representing the rotational symmetry properties of an instance
 */
struct RotationalSymmetry {
  bool isAnnotated = false; /**< if false, it means we have not annotated the instance, the
                               isSymmetry is unknown */
  std::vector<RotationalSymmetryAxis> axes; /**< all rotational symmetrical axes of an instance */
  [[nodiscard]] std::string toString() const;
};

/**
 * @brief  Canonical pose defines the transformation so that prototypes in the category face a
 * consistent direction.
 */
struct CanonicalPose {
  Eigen::Vector3d upVector;
  Eigen::Vector3d frontVector;
  /**
   * @brief A simple function to string-fy a CanonicalPose object
   */
  [[nodiscard]] std::string toString() const;
};

/**
 * @brief A struct that represents the information of an instance, where an instance can either be a
 * human or an object.
 */
struct InstanceInfo {
  InstanceId id; /**< Unique ID assigned to every instance. This is what should be used to query for
                    this instance in your dataset, e.g. 8831369073541652 */
  std::string name; /**< Unique name given to this instance. This is usually a name that well
                       describes the instance, e.g. CloudNightLight_1 */
  std::string prototypeName; /**< Prototype that this instance belongs to. A prototype is a set of
                                objects sharing the same shape and textures, e.g. CloudNightLight */
  std::string category; /**< Category that the prototype belongs to. This is some name that well
                           describes a group of related prototypes, e.g. light */
  int categoryUid; /**< Unique ID for the category */
  InstanceType instanceType =
      InstanceType::Unknown; /**< Type of instance, one of {Object, Human, Unknown} */
  MotionType motionType =
      MotionType::Unknown; /**< Type of instance motion, one of {Static, Dynamic, Unknown} */
  RigidityType rigidityType =
      RigidityType::Unknown; /**< Type of instance rigidity, one of {Rigid, Deformable, Unknown} */
  RotationalSymmetry rotationalSymmetry; /**< Rotational symmetry of the
                                            object. For each rotational symmetry, we define the
                                            rotation axis and its minimal rotation angle. */
  CanonicalPose canonicalPose; /**< Canonical pose defines the transformation so that prototypes in
                                  the category face a consistent direction */
  std::string associatedDeviceSerial; /**< Device serial associated with this object. This is only
                                         applicable to Humans, and will be NONE if the human is not
                                         wearing an Aria device */

  /**
   * @brief A simple function to string-fy an InstanceInfo object
   */
  [[nodiscard]] std::string toString() const;
};

/**
 * @brief TimeQueryOptions are re-used from data_provider lib.
 */
using TimeQueryOptions = projectaria::tools::data_provider::TimeQueryOptions;

} // namespace projectaria::dataset::adt
