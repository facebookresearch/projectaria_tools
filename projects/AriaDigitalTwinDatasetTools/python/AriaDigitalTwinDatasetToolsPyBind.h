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

#include "AriaDigitalTwinDataFileKeys.h"
#include "AriaDigitalTwinDataPathsProvider.h"
#include "AriaDigitalTwinDataProvider.h"

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <python/ImageDataHelper.h>
#include "dictobject.h"

namespace py = pybind11;

namespace projectaria::dataset::adt {

void exportAriaDigitalTwin(py::module& m) {
  // For module documentation, see: projectaria_tools/projectaria_tools/projects/adt/__init__.py

  // Ground truth data type
  py::class_<Aria3dPose>(
      m,
      "Aria3dPose",
      "a simple struct to represent the pose of an Aria device "
      "at a certain time")
      .def(py::init<>())
      .def_readwrite(
          "transform_scene_device",
          &Aria3dPose::T_Scene_Device,
          "Pose of the Device at a certain time, where "
          "point_in_scene = T_Scene_Device * point_in_device")
      .def_readwrite(
          "device_linear_velocity",
          &Aria3dPose::deviceLinearVelocity,
          "linear velocity of the device in meter/s")
      .def_readwrite(
          "device_rotational_velocity",
          &Aria3dPose::deviceRotationalVelocity,
          "rotation velocity of the device in rad/s")
      .def_readwrite(
          "gravity_world",
          &Aria3dPose::gravityWorld,
          "Earth gravity vector in world frame. This vector is pointing toward the "
          "ground, and includes gravitation and centrifugal forces from earth rotation")
      .def_readwrite(
          "graph_uid",
          &Aria3dPose::graphUid,
          "Unique identifier of the world coordinate frame. When the graphUid is the same, "
          "poses, velocities and point clouds are defined in the same coordinate frame")
      .def_readwrite(
          "quality_score",
          &Aria3dPose::qualityScore,
          "Value between 0 and 1 which describes the tracking quality. The value is "
          "determined based on Motive's mean tracking error where 0 quality corresponds to 5 mm error or more");

  py::class_<BoundingBox3dData>(
      m,
      "BoundingBox3dData",
      "a simple struct to represent a 3D bounding box "
      "for an instance")
      .def(py::init<>())
      .def_readwrite(
          "transform_scene_object",
          &BoundingBox3dData::T_Scene_Object,
          "object 6DoF pose in the scene (world), where: "
          "point_in_scene = T_Scene_Object * point_in_object")
      .def_readwrite(
          "aabb",
          &BoundingBox3dData::aabb,
          "object AABB (axes-aligned-bounding-box) in the object's local "
          "coordinate frame, AABB is represented in [xmin, xmax, ymin, ymax, zmin, zmax]");

  py::class_<BoundingBox2dData>(
      m, "BoundingBox2dData", "a simple struct to represent a 2D bounding box for an instance")
      .def(py::init<>())
      .def_readwrite("box_range", &BoundingBox2dData::boxRange, "[xmin, xmax, ymin, ymax]")
      .def_readwrite(
          "visibility_ratio",
          &BoundingBox2dData::visibilityRatio,
          "visibilityRatio calculated by occlusion between objects. "
          "visibilityRatio = 1: an object is not occluded by any other object "
          "visibilityRatio = 0: an object is fully occluded by other objects");

  py::class_<SegmentationData>(m, "SegmentationData", "A class to represent segmentation image")
      .def(py::init<>())
      .def("get_height", &SegmentationData::getHeight)
      .def("get_width", &SegmentationData::getWidth)
      .def("is_valid", &SegmentationData::isValid)
      .def(
          "get_visualizable",
          &SegmentationData::getVisualizable,
          "Get stored image as 3-channel uint8 format")
      .def(
          "to_numpy_array",
          [](const SegmentationData& self) -> tools::image::PyArrayVariant {
            return tools::image::toPyArrayVariant(self.imageVariant().value());
          })
      .def(
          "at",
          [](const SegmentationData& self, int x, int y) -> uint64_t {
            return std::get<projectaria::tools::image::ImageU64>(self.imageVariant().value())(x, y);
          },
          py::arg("x"),
          py::arg("y"));

  py::class_<DepthData>(m, "DepthData", "A class to represent depth image")
      .def(py::init<>())
      .def("get_height", &DepthData::getHeight)
      .def("get_width", &DepthData::getWidth)
      .def("is_valid", &DepthData::isValid)
      .def(
          "get_visualizable",
          &DepthData::getVisualizable,
          "Get stored image as 3-channel uint8 format")
      .def(
          "to_numpy_array",
          [](const DepthData& self) -> tools::image::PyArrayVariant {
            return tools::image::toPyArrayVariant(self.imageVariant().value());
          })
      .def(
          "at",
          [](const DepthData& self, int x, int y) {
            return std::get<projectaria::tools::image::ImageU16>(self.imageVariant().value())(x, y);
          },
          py::arg("x"),
          py::arg("y"));

  py::class_<SyntheticData>(m, "SyntheticData", "A class to represent synthetic image")
      .def(py::init<>())
      .def("get_height", &SyntheticData::getHeight)
      .def("get_width", &SyntheticData::getWidth)
      .def("is_valid", &SyntheticData::isValid)
      .def(
          "get_visualizable",
          &SyntheticData::getVisualizable,
          "Get stored image as an image variant")
      .def(
          "to_numpy_array",
          [](const SyntheticData& self) -> tools::image::PyArrayVariant {
            return tools::image::toPyArrayVariant(self.imageVariant().value());
          })
      .def(
          "at",
          [](const SyntheticData& self, int x, int y, int channel)
              -> tools::image::PixelValueVariant {
            return tools::image::at(self.imageVariant().value(), x, y, channel);
          },
          py::arg("x"),
          py::arg("y"),
          py::arg("channel") = 0);

  py::class_<SkeletonFrame>(
      m, "SkeletonFrame", "A simple struct to represent a frame of skeleton data")
      .def(py::init<>())
      .def_readwrite(
          "joints",
          &SkeletonFrame::joints,
          "a vector representing the joint locations of a skeleton")
      .def_readwrite(
          "markers",
          &SkeletonFrame::markers,
          "a vector representing optitrack markers of a skeleton");

  py::class_<AriaImageDataWithDt>(m, "AriaImageDataWithDt", "query result containing Aria image")
      .def("data", &AriaImageDataWithDt::data, py::return_value_policy::reference_internal)
      .def("dt_ns", &AriaImageDataWithDt::dtNs)
      .def("is_valid", &AriaImageDataWithDt::isValid);

  py::class_<Aria3dPoseDataWithDt>(
      m, "Aria3dPoseDataWithDt", "query result containing Aria device pose")
      .def("data", &Aria3dPoseDataWithDt::data, py::return_value_policy::reference_internal)
      .def("dt_ns", &Aria3dPoseDataWithDt::dtNs)
      .def("is_valid", &Aria3dPoseDataWithDt::isValid);

  py::class_<BoundingBox3dDataWithDt>(
      m, "BoundingBox3dDataWithDt", "query result containing an instance 3D bounding box")
      .def("data", &BoundingBox3dDataWithDt::data, py::return_value_policy::reference_internal)
      .def("dt_ns", &BoundingBox3dDataWithDt::dtNs)
      .def("is_valid", &BoundingBox3dDataWithDt::isValid);

  py::class_<BoundingBox2dDataWithDt>(
      m, "BoundingBox2dDataWithDt", "query result containing an instance 2D bounding box")
      .def("data", &BoundingBox2dDataWithDt::data, py::return_value_policy::reference_internal)
      .def("dt_ns", &BoundingBox2dDataWithDt::dtNs)
      .def("is_valid", &BoundingBox2dDataWithDt::isValid);

  py::class_<SegmentationDataWithDt>(
      m, "SegmentationDataWithDt", "query result containing segmentation image")
      .def("data", &SegmentationDataWithDt::data, py::return_value_policy::reference_internal)
      .def("dt_ns", &SegmentationDataWithDt::dtNs)
      .def("is_valid", &SegmentationDataWithDt::isValid);

  py::class_<DepthDataWithDt>(m, "DepthDataWithDt", "query result containing depth image")
      .def("data", &DepthDataWithDt::data, py::return_value_policy::reference_internal)
      .def("dt_ns", &DepthDataWithDt::dtNs)
      .def("is_valid", &DepthDataWithDt::isValid);

  py::class_<SyntheticDataWithDt>(
      m, "SyntheticDataWithDt", "query result containing synthetic image")
      .def("data", &SyntheticDataWithDt::data, py::return_value_policy::reference_internal)
      .def("dt_ns", &SyntheticDataWithDt::dtNs)
      .def("is_valid", &SyntheticDataWithDt::isValid);

  py::class_<EyeGazeWithDt>(m, "EyeGazeWithDt", "query result containing eye gaze data")
      .def("data", &EyeGazeWithDt::data, py::return_value_policy::reference_internal)
      .def("dt_ns", &EyeGazeWithDt::dtNs)
      .def("is_valid", &EyeGazeWithDt::isValid);

  py::class_<SkeletonFrameWithDt>(
      m, "SkeletonFrameWithDt", "query result containing skeleton frame data")
      .def("data", &SkeletonFrameWithDt::data, py::return_value_policy::reference_internal)
      .def("dt_ns", &SkeletonFrameWithDt::dtNs)
      .def("is_valid", &SkeletonFrameWithDt::isValid);

  // Skeleton Provider
  py::class_<AriaDigitalTwinSkeletonProvider>(
      m,
      "AriaDigitalTwinSkeletonProvider",
      "Class for loading and accessing skeleton marker and joint information from an ADT sequence. "
      "Motive (the software running Optitrack) generates a frame of marker positions for each"
      "capture corresponding to each bodysuit in the scene. It then estimates the person's joint"
      "positions for each of these marker frames. ADT converts these measurements to the Scene frame to"
      "be consistent with all ground truth data and this class allows the user to load and query that"
      "data. We provide a separate class from the main AriaDigitalTwinDataProvider to separate out the"
      "skeleton loading and allow users to call this API without loading all other ADT data.")
      .def(py::init<const std::string&>())
      .def(
          "get_skeleton_by_timestamp_ns",
          &AriaDigitalTwinSkeletonProvider::getSkeletonByTimestampNs,
          "Gets a skeleton frame by timestamp",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_joint_connections",
          &AriaDigitalTwinSkeletonProvider::getJointConnections,
          "get the connections between joint IDs. (e.g, head id -> neck id)")
      .def(
          "get_joint_labels",
          &AriaDigitalTwinSkeletonProvider::getJointLabels,
          "get the labels associated with each joint where the i^th element in the vector corresponds "
          "to joint Id i")
      .def(
          "get_marker_labels",
          &AriaDigitalTwinSkeletonProvider::getMarkerLabels,
          "get the labels associated with each marker where the i^th element in the vector corresponds "
          "to maker Id i");

  // Data path and dataPath Provider
  py::class_<AriaDigitalTwinDataPaths>(
      m,
      "AriaDigitalTwinDataPaths",
      "A struct that includes the file paths of all ADT data files for one sequence of one device.")
      .def(py::init<>())
      .def_readwrite(
          "sequence_name",
          &AriaDigitalTwinDataPaths::sequenceName,
          "Name of sequence that these files belong to")
      .def_readwrite("aria_vrs_filepath", &AriaDigitalTwinDataPaths::ariaVrsFilePath, "Aria vrs")
      .def_readwrite(
          "aria_trajectory_filepath",
          &AriaDigitalTwinDataPaths::ariaTrajectoryFilePath,
          "Aria 6DoF pose trajectory")
      .def_readwrite(
          "object_trajectories_filepath",
          &AriaDigitalTwinDataPaths::objectTrajectoriesFilePath,
          "object 6Dof pose trajectory")
      .def_readwrite(
          "object_boundingbox_3d_filepath",
          &AriaDigitalTwinDataPaths::objectBoundingBox3dFilePath,
          "axis-aligned bounding box (AABB) of objects in its local coordinate frame")
      .def_readwrite(
          "boundingboxes_2d_filepath",
          &AriaDigitalTwinDataPaths::boundingBoxes2dFilePath,
          "2D object bounding boxes for all cameras, stored as <cameraId, filePath>")
      .def_readwrite(
          "segmentations_filepath",
          &AriaDigitalTwinDataPaths::segmentationsFilePath,
          "2D segmentation maps")
      .def_readwrite(
          "depth_images_filepath", &AriaDigitalTwinDataPaths::depthImagesFilePath, "depth maps")
      .def_readwrite(
          "synthetic_vrs_filepath",
          &AriaDigitalTwinDataPaths::syntheticVrsFilePath,
          "synthetic file")
      .def_readwrite(
          "eyegazes_filepath", &AriaDigitalTwinDataPaths::eyeGazesFilePath, "eye gaze file")
      .def_readwrite(
          "skeletons_filepaths",
          &AriaDigitalTwinDataPaths::skeletonsFilePaths,
          "skeleton files: skeletonId -> filepath")
      .def_readwrite(
          "skeleton_metadata_filepath",
          &AriaDigitalTwinDataPaths::skeletonMetaDataFilePath,
          "skeleton metadata file")
      .def_readwrite(
          "metadata_filepath",
          &AriaDigitalTwinDataPaths::metaDataFilePath,
          "data collection metadata file")
      .def_readwrite(
          "instances_filepath",
          &AriaDigitalTwinDataPaths::instancesFilePath,
          "instances file, a.k.a. object instance information file")
      .def_readwrite("mps", &AriaDigitalTwinDataPaths::mps, "MPS data paths")
      .def("__str__", [](const AriaDigitalTwinDataPaths& d) { return d.toString(); });

  py::class_<AriaDigitalTwinDataPathsProvider>(
      m,
      "AriaDigitalTwinDataPathsProvider",
      " This class is to load all data file paths from ADT data structure given a sequence path. "
      " This class supports both v1.X dataset versions as well as v2.X dataset versions (and beyond) "
      " which have different formats: "
      " v1.X: Each ADT collection sequence may contain more than one Aria device wearers. The data "
      " associated with each Aria device is called a subsequence: "
      " ├── sequencePath        "
      " │   ├── subsequence1_Name "
      " │   │   ├── 2d_bounding_box.csv "
      " │   │   ├── 2d_bounding_box_with_skeleton.csv "
      " │   │   ├── 3d_bounding_box.csv "
      " │   │   ├── Skeleton_C.json "
      " │   │   ├── skeleton_aria_association.json "
      " │   │   ├── aria_trajectory.csv "
      " │   │   ├── depth_images.vrs "
      " │   │   ├── depth_images_with_skeleton.vrs "
      " │   │   ├── eyegaze.csv "
      " │   │   ├── instances.csv "
      " │   │   ├── scene_objects.csv "
      " │   │   ├── segmentations.vrs "
      " │   │   ├── segmentations_with_skeleton.vrs "
      " │   │   └── video.vrs "
      " │   ├── subsequence2_Name "
      " │   │   ├── 2d_bounding_box.csv "
      " │   │   ├── ... "
      " │   └── metadata.json "
      " v2.X and beyond: We have removed the concept of subsequence. Each sequence can only contain one "
      " Aria recording, and concurrent recordings can be fetched by looking the field in the metadata "
      " file. This means we have the following file structure: "
      " ├── sequencePath        "
      " │   ├── 2d_bounding_box.csv "
      " │   ├── 2d_bounding_box_with_skeleton.csv "
      " │   ├── 3d_bounding_box.csv "
      " │   ├── Skeleton_C.json "
      " │   ├── skeleton_aria_association.json "
      " │   ├── aria_trajectory.csv "
      " │   ├── depth_images.vrs "
      " │   ├── depth_images_with_skeleton.vrs "
      " │   ├── eyegaze.csv "
      " │   ├── instances.csv "
      " │   ├── scene_objects.csv "
      " │   ├── segmentations.vrs "
      " │   ├── segmentations_with_skeleton.vrs "
      " │   └── video.vrs "
      " │   └── metadata.json ")
      .def(py::init<const std::string&>())
      .def(
          "get_datapaths",
          &AriaDigitalTwinDataPathsProvider::getDataPaths,
          "retrieve the DataPaths for this sequene. "
          "If loading a sequence that has version < 2.0 and has multiple subsequences, this will return "
          "the data paths associated with the first device serial",
          py::arg("skeleton_flag") = false)
      .def(
          "get_datapaths_by_device_num",
          &AriaDigitalTwinDataPathsProvider::getDataPathsByDeviceNum,
          "retrieve the DataPaths from a device based on its index. "
          "DEPRECATION NOTE: With dataset versions 2.0 and beyond, "
          "this function has been deprecated since there is only one device per sequence. "
          "If you are using this on older data, it will still work.If using on new data, "
          "it will only work if deviceNum is 0. ",
          py::arg("device_num"),
          py::arg("skeleton_flag") = false)
      .def(
          "get_datapaths_by_device_serial",
          &AriaDigitalTwinDataPathsProvider::getDataPathsByDeviceSerial,
          "retrieve the DataPaths from a device based on its serial number. "
          "DEPRECATION NOTE: With dataset versions 2.0 and beyond, this function has been "
          "deprecated since there is only one device per sequence. This function will still "
          "work with old or newer data as long as you are querying with the correct serial "
          "associated with this sequence.",
          py::arg("device_serial"),
          py::arg("skeleton_flag") = false)
      .def(
          "get_device_serial_number",
          &AriaDigitalTwinDataPathsProvider::getDeviceSerialNumber,
          "get the device serial number for this sequence. "
          "If loading a sequence that has version < 2.0 and has multiple subsequences, "
          "this will return the first device serial")
      .def(
          "get_device_serial_numbers",
          &AriaDigitalTwinDataPathsProvider::getDeviceSerialNumbers,
          "get all device serial numbers in the recording sequence."
          "DEPRECATION NOTE: With dataset versions 2.0 and beyond, this function has been "
          "deprecated since there is only one device per sequence. This function will still "
          "work with old or newer data, however, we recommend using getDeviceSerialNumber "
          "instead for newer data & @return a const reference to a vector of string")
      .def(
          "get_scene_name",
          &AriaDigitalTwinDataPathsProvider::getSceneName,
          "get the scene name of the recording sequence")
      .def(
          "get_num_skeletons",
          &AriaDigitalTwinDataPathsProvider::getNumSkeletons,
          "get the number of skeletons in the current sequence")
      .def(
          "is_multi_person",
          &AriaDigitalTwinDataPathsProvider::isMultiPerson,
          "check if the sequence is a multi-person sequence")
      .def(
          "get_concurrent_sequence_name",
          &AriaDigitalTwinDataPathsProvider::getConcurrentSequenceName,
          "return string with name of sequence collected at the same time (if multi-sequence), return null otherwise");

  // Bind AriaDigitalTwinDataProvider
  // Bind instance info
  py::enum_<InstanceType>(m, "InstanceType")
      .value("UNKNOWN", InstanceType::Unknown)
      .value("OBJECT", InstanceType::Object)
      .value("HUMAN", InstanceType::Human)
      .export_values();

  py::enum_<RigidityType>(m, "RigidityType")
      .value("UNKNOWN", RigidityType::Unknown)
      .value("RIGID", RigidityType::Rigid)
      .value("DEFORMABLE", RigidityType::Deformable)
      .export_values();

  py::enum_<MotionType>(m, "MotionType")
      .value("UNKNOWN", MotionType::Unknown)
      .value("STATIC", MotionType::Static)
      .value("DYNAMIC", MotionType::Dynamic)
      .export_values();

  py::class_<RotationalSymmetryAxis>(
      m,
      "RotationalSymmetryAxis",
      "A struct representing the rotational symmetry axis of an instance")
      .def_readwrite("axis", &RotationalSymmetryAxis::axis)
      .def_readwrite("angle_degree", &RotationalSymmetryAxis::angleDegree)
      .def("__repr__", [](const RotationalSymmetryAxis& d) { return d.toString(); });

  py::class_<RotationalSymmetry>(
      m,
      "RotationalSymmetry",
      "A struct representing the rotational symmetry properties of an instance")
      .def_readwrite(
          "is_annotated",
          &RotationalSymmetry::isAnnotated,
          "if false, it means we have not annotated the instance, the isSymmetry is unknown")
      .def_readwrite(
          "axes", &RotationalSymmetry::axes, "all rotational symmetrical axes of an instance")
      .def("__repr__", [](const RotationalSymmetry& d) { return d.toString(); });

  py::class_<CanonicalPose>(
      m,
      "CanonicalPose",
      "Canonical pose defines the transformation so that prototypes in the category "
      "face a consistent direction")
      .def_readwrite("up_vector", &CanonicalPose::upVector)
      .def_readwrite("front_vector", &CanonicalPose::frontVector)
      .def("__repr__", [](const InstanceInfo& d) { return d.toString(); });

  py::class_<InstanceInfo>(
      m,
      "InstanceInfo",
      "A struct that represents the information of an instance, where an instance can "
      "either be a human or an object.")
      .def_readwrite(
          "id",
          &InstanceInfo::id,
          "Unique ID assigned to every instance. This is what should be used to query for this instance in your dataset, e.g. 8831369073541652")
      .def_readwrite(
          "name",
          &InstanceInfo::name,
          "Unique name given to this instance. This is usually a name that well describes the instance, e.g. CloudNightLight_1")
      .def_readwrite(
          "prototype_name",
          &InstanceInfo::prototypeName,
          "Prototype that this instance belongs to. A prototype is a set of objects sharing the same shape and textures, e.g. CloudNightLight")
      .def_readwrite(
          "category",
          &InstanceInfo::category,
          "Category that the prototype belongs to. This is some name that well describes a group of related prototypes, e.g. light")
      .def_readwrite("category_uid", &InstanceInfo::categoryUid, "Unique ID for the category")
      .def_readwrite(
          "instance_type",
          &InstanceInfo::instanceType,
          "Type of instance, one of {Object, Human, Unknown} ")
      .def_readwrite(
          "motion_type",
          &InstanceInfo::motionType,
          "Type of instance motion, one of {Static, Dynamic, Unknown}")
      .def_readwrite(
          "rigidity_type",
          &InstanceInfo::rigidityType,
          "Type of instance rigidity, one of {Rigid, Deformable, Unknown}")
      .def_readwrite(
          "rotational_symmetry",
          &InstanceInfo::rotationalSymmetry,
          "Rotational symmetry of the object. For each rotational symmetry, we define the rotation axis and its minimal rotation angle.")
      .def_readwrite(
          "canonical_pose",
          &InstanceInfo::canonicalPose,
          "Canonical pose defines the transformation so that prototypes in the category face a consistent direction")
      .def_readwrite(
          "associated_device_serial",
          &InstanceInfo::associatedDeviceSerial,
          "Device serial associated with this object. This is only applicable to Humans, and will be NONE if the human is not wearing an Aria device")
      .def("__repr__", [](const InstanceInfo& d) { return d.toString(); });

  py::class_<AriaDigitalTwinDataProvider>(
      m,
      "AriaDigitalTwinDataProvider",
      "This is the core data loader that should provide all the data access you will need for"
      "an ADT sequence. Note that each sequence may contain multiple devices, you should create one"
      "`AriaDigitalTwinDataProvider` instance for each device.")
      .def(py::init<const AriaDigitalTwinDataPaths&>())
      .def(
          "get_aria_device_capture_timestamps_ns",
          &AriaDigitalTwinDataProvider::getAriaDeviceCaptureTimestampsNs,
          "Get all timestamps (in ns) of all observations of an Aria sensor, in"
          " `TimeDomain::DeviceTime`.",
          py::arg("stream_id"))
      .def(
          "get_aria_image_by_timestamp_ns",
          &AriaDigitalTwinDataProvider::getAriaImageByTimestampNs,
          "Query an Aria camera image by timestamp",
          py::arg("device_time_stamp_ns"),
          py::arg("stream_id"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_aria_camera_calibration",
          &AriaDigitalTwinDataProvider::getAriaCameraCalibration,
          "Get the camera calibration of an Aria camera, including intrinsics, distortion params,"
          "and projection functions.",
          py::arg("stream_id"))
      .def(
          "get_aria_transform_device_camera",
          &AriaDigitalTwinDataProvider::getAria_T_Device_Camera,
          "Get the pose of an Aria camera in the device coordinate frame.",
          py::arg("stream_id"))
      .def(
          "get_device_time_from_timecode_ns",
          &AriaDigitalTwinDataProvider::getDeviceTimeFromTimecodeNs,
          "ADT uses Timecode to synchronize multiple Aria devices. Use this function to convert a"
          " timestamp from `TimeDomain::TimeCode` to `TimeDomain::DeviceTime`. See"
          " `adt_multiperson_tutorial.ipynb` for usage example.",
          py::arg("timecode_ns"))
      .def(
          "get_timecode_from_device_time_ns",
          &AriaDigitalTwinDataProvider::getTimecodeFromDeviceTimeNs,
          "ADT uses Timecode to synchronize multiple Aria devices. Use this function to convert a",
          " timestamp from `TimeDomain::DeviceTime` to `TimeDomain::TimeCode`. See",
          " `adt_multiperson_tutorial.ipynb` for usage example.",
          py::arg("device_time_ns"))

      .def(
          "get_aria_3d_pose_by_timestamp_ns",
          &AriaDigitalTwinDataProvider::getAria3dPoseByTimestampNs,
          "Query the device pose of the Aria unit in the trajectory, by timestamp.",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_object_3d_boundingboxes_by_timestamp_ns",
          &AriaDigitalTwinDataProvider::getObject3dBoundingBoxesByTimestampNs,
          "Query object 3D bounding boxes by timestamp.",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_object_2d_boundingboxes_by_timestamp_ns",
          &AriaDigitalTwinDataProvider::getObject2dBoundingBoxesByTimestampNs,
          "Query 2D object bounding boxes by timestamp, in the view of a given camera.",
          py::arg("device_timestamp_ns"),
          py::arg("stream_id"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_skeleton_2d_boundingboxes_by_timestamp_ns",
          &AriaDigitalTwinDataProvider::getSkeleton2dBoundingBoxesByTimestampNs,
          "Query 2D skeleton bounding boxes by timestamp, in the view of a given camera.",
          py::arg("device_timestamp_ns"),
          py::arg("stream_id"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_segmentation_image_by_timestamp_ns",
          &AriaDigitalTwinDataProvider::getSegmentationImageByTimestampNs,
          "Query a camera's segmentation image by timestamp",
          py::arg("device_timestamp_ns"),
          py::arg("stream_id"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_depth_image_by_timestamp_ns",
          &AriaDigitalTwinDataProvider::getDepthImageByTimestampNs,
          "Query a camera's depth image by timestamp",
          py::arg("device_timestamp_ns"),
          py::arg("stream_id"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_synthetic_image_by_timestamp_ns",
          &AriaDigitalTwinDataProvider::getSyntheticImageByTimestampNs,
          "Query a camera's synthetic image by timestamp",
          py::arg("device_timestamp_ns"),
          py::arg("stream_id"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_eyegaze_by_timestamp_ns",
          &AriaDigitalTwinDataProvider::getEyeGazeByTimestampNs,
          "Query eye gaze by timestamp. The eye gaze is in Central-Pupil-Frame (CPF).",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_skeleton_by_timestamp_ns",
          &AriaDigitalTwinDataProvider::getSkeletonByTimestampNs,
          "Query the skeleton frame by timestamp for a specific skeleton.",
          py::arg("device_timeStamp_ns"),
          py::arg("instance_id"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def("has_aria_data", &AriaDigitalTwinDataProvider::hasAriaData)
      .def("has_aria_3d_poses", &AriaDigitalTwinDataProvider::hasAria3dPoses)
      .def("has_object_3d_boundingboxes", &AriaDigitalTwinDataProvider::hasObject3dBoundingboxes)
      .def(
          "has_instance_2d_boundingboxes", &AriaDigitalTwinDataProvider::hasInstance2dBoundingBoxes)
      .def("has_segmentation_images", &AriaDigitalTwinDataProvider::hasSegmentationImages)
      .def("has_depth_images", &AriaDigitalTwinDataProvider::hasDepthImages)
      .def("has_synthetic_images", &AriaDigitalTwinDataProvider::hasSyntheticImages)
      .def("has_eyegaze", &AriaDigitalTwinDataProvider::hasEyeGaze)
      .def("has_skeleton", &AriaDigitalTwinDataProvider::hasSkeleton)
      .def("has_instances_info", &AriaDigitalTwinDataProvider::hasInstancesInfo)
      .def("has_mps", &AriaDigitalTwinDataProvider::hasMps)
      .def(
          "get_instance_ids",
          &AriaDigitalTwinDataProvider::getInstanceIds,
          "get all instances in a sequence. An instance can be any type of InstanceType")
      .def(
          "has_instance_id",
          &AriaDigitalTwinDataProvider::hasInstanceId,
          "query if an instance exists in current data.",
          py::arg("instance_id"))
      .def(
          "get_instance_info_by_id",
          &AriaDigitalTwinDataProvider::getInstanceInfoById,
          "get instance information by instance id.",
          py::arg("instance_id"))
      .def(
          "get_object_ids",
          &AriaDigitalTwinDataProvider::getObjectIds,
          "get all instance ids as a vector whose `InstanceType == OBJECT`.")
      .def(
          "get_skeleton_ids",
          &AriaDigitalTwinDataProvider::getSkeletonIds,
          "get all instance ids as a vector whose `InstanceType == HUMAN`.")
      .def(
          "get_start_time_ns",
          &AriaDigitalTwinDataProvider::getStartTimeNs,
          "get the start time of the Aria poses in nanoseconds")
      .def(
          "get_end_time_ns",
          &AriaDigitalTwinDataProvider::getEndTimeNs,
          "get the end time of the Aria poses in nanoseconds")
      .def(
          "raw_data_provider_ptr",
          &AriaDigitalTwinDataProvider::rawDataProviderPtr,
          "get a pointer to the raw data provider")
      .def(
          "segmentation_data_provider_ptr",
          &AriaDigitalTwinDataProvider::segmentationDataProviderPtr,
          "get a pointer to the segmentation data provider")
      .def(
          "depth_data_provider_ptr",
          &AriaDigitalTwinDataProvider::depthDataProviderPtr,
          "get a pointer to the depth data provider")
      .def(
          "synthetic_data_provider_ptr",
          &AriaDigitalTwinDataProvider::syntheticDataProviderPtr,
          "get a pointer to the synthetic data provider")
      .def(
          "mps_data_provider_ptr",
          &AriaDigitalTwinDataProvider::mpsDataProviderPtr,
          "return the MPS data provider")
      .def(
          "get_skeleton_provider",
          &AriaDigitalTwinDataProvider::getSkeletonProvider,
          "return the skeleton provider for a given human instance",
          py::arg("instance_id"));

  // Bind ADT Data Provider helper functions
  m.def(
      "get_interpolated_aria_3d_pose_at_timestamp_ns",
      &getInterpolatedAria3dPoseAtTimestampNs,
      "helper function to return an interpolated Aria3dPose given a query timestamp",
      py::arg("provider"),
      py::arg("device_time_stamp_ns"));
  m.def(
      "get_interpolated_object_3d_boundingboxes_at_timestamp_ns",
      &getInterpolatedObject3dBoundingBoxesAtTimestampNs,
      "helper function to return an interpolated object 3D bounding box given a query timestamp",
      py::arg("provider"),
      py::arg("device_time_stamp_ns"));

  // Bind utility functions
  m.def(
      "bbox2d_to_image_coordinates",
      &bbox2dToImageCoordinates,
      "helper function to convert a 2D bounding box [xmin, xmax, ymin, ymax] to the 4 image "
      "coordinates [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]",
      py::arg("bbox"));
  m.def(
      "bbox2d_to_image_line_coordinates",
      &bbox2dToImageLineCoordinates,
      "helper function to convert a 2D bounding box [xmin, xmax, ymin, ymax] to the 5 "
      "coordinates that draw the lines in the image completing the full bounding box "
      "[(x1, y1), (x2, y2), (x3, y3), (x4, y4), (x1, y1)]",
      py::arg("bbox"));
  m.def(
      "bbox3d_to_coordinates",
      &bbox3dToCoordinates,
      "helper function to convert a 3d bounding box [xmin, xmax, ymin, ymax, zmin, zmax] "
      "to the 8 corner coordinates in the object frame [b1, b2, b3, b4, t1, t2, t3, t4] "
      "where b is for bottom and t is for top",
      py::arg("bbox"));
  m.def(
      "bbox3d_to_line_coordinates",
      &bbox3dToLineCoordinates,
      "helper function to convert a 3d bounding box [xmin, xmax, ymin, ymax, zmin, zmax] "
      "to the 16 coordinates that draw the lines completing the full bounding box "
      "[b1, b2, b3, b4, b1, t1, t2, t3, t4, t1, t2, b2, b3, t3, t4, b4] where b is for "
      "bottom and t is for top",
      py::arg("bbox"));
  m.def("is_dataset_corrupt", [](const std::string& seq) {
    return kCorruptDatasets.find(seq) != kCorruptDatasets.end();
  });
}

} // namespace projectaria::dataset::adt
