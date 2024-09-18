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

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sophus/se3.hpp>

#include "EyeGazeFormat.h"
#include "GlobalPointCloudFormat.h"
#include "HandTrackingFormat.h"
#include "MpsDataPathsFormat.h"
#include "OnlineCalibrationFormat.h"
#include "PointObservationFormat.h"
#include "StaticCameraCalibrationFormat.h"
#include "TrajectoryFormat.h"

#include "EyeGazeReader.h"
#include "GlobalPointCloudReader.h"
#include "HandTrackingReader.h"
#include "MpsDataPathsProvider.h"
#include "MpsDataProvider.h"
#include "OnlineCalibrationsReader.h"
#include "PointObservationReader.h"
#include "StaticCameraCalibrationReader.h"
#include "TrajectoryReaders.h"

namespace py = pybind11;

namespace projectaria::tools::mps {

void exportMps(py::module& m) {
  // For submodule documentation, see: projectaria_tools/projectaria_tools/core/mps.py

  // gaze vergence fields
  py::class_<EyeGazeVergence>(m, "EyeGazeVergence")
      .def_readwrite("left_yaw", &EyeGazeVergence::left_yaw)
      .def_readwrite("right_yaw", &EyeGazeVergence::right_yaw)
      .def_readwrite("left_yaw_low", &EyeGazeVergence::left_yaw_low)
      .def_readwrite("right_yaw_low", &EyeGazeVergence::right_yaw_low)
      .def_readwrite("left_yaw_high", &EyeGazeVergence::left_yaw_high)
      .def_readwrite("right_yaw_high", &EyeGazeVergence::right_yaw_high)
      .def_readwrite("tx_left_eye", &EyeGazeVergence::tx_left_eye)
      .def_readwrite("ty_left_eye", &EyeGazeVergence::ty_left_eye)
      .def_readwrite("tz_left_eye", &EyeGazeVergence::tz_left_eye)
      .def_readwrite("tx_right_eye", &EyeGazeVergence::tx_right_eye)
      .def_readwrite("ty_right_eye", &EyeGazeVergence::ty_right_eye)
      .def_readwrite("tz_right_eye", &EyeGazeVergence::tz_right_eye);

  // gaze
  py::class_<EyeGaze>(m, "EyeGaze", "An object representing single Eye gaze output.")
      .def_readwrite(
          "tracking_timestamp",
          &EyeGaze::trackingTimestamp,
          "Timestamp of the eye tracking camera frame in device time domain.")
      .def_readwrite(
          "yaw", &EyeGaze::yaw, "Eye gaze yaw angle (horizontal) in radians in CPF frame.")
      .def_readwrite(
          "vergence",
          &EyeGaze::vergence,
          "Additional fields related to vergence (new model output).")
      .def_readwrite(
          "pitch", &EyeGaze::pitch, "Eye gaze pitch angle (vertical) in radians in CPF frame.")
      .def_readwrite(
          "depth",
          &EyeGaze::depth,
          "Depth in meters of the 3D eye gaze point in CPF frame. A value of 0 indicates that the depth is not available.")
      .def_readwrite(
          "yaw_low",
          &EyeGaze::yaw_low,
          "[yaw_low, yaw_high] represent the confidence interval of the eye gaze yaw. The `yaw` is in the [yaw_low, yaw_high] interval but not necessarily in the middle.")
      .def_readwrite(
          "pitch_low",
          &EyeGaze::pitch_low,
          "[pitch_low, pitch_high] represents the confidence interval of the eye gaze pitch. The `pitch` is in the [pitch_low, pitch_high] interval but not necessarily in the middle.")
      .def_readwrite(
          "yaw_high",
          &EyeGaze::yaw_high,
          "[yaw_low, yaw_high] represent the confidence interval of the eye gaze yaw. The `yaw` is in the [yaw_low, yaw_high] interval but not necessarily in the middle.")
      .def_readwrite(
          "pitch_high",
          &EyeGaze::pitch_high,
          "[pitch_low, pitch_high] represents the confidence interval of the eye gaze pitch. The `pitch` is in the [pitch_low, pitch_high] interval but not necessarily in the middle.")
      .def_readwrite(
          "session_uid",
          &EyeGaze::session_uid,
          "`unique id` for the calibration session. If there are multiple in-session calibrations in the recording, each segment will have a different `session_uid`.")
      .def("__repr__", [](EyeGaze const& self) { return fmt::to_string(self); });

  m.def(
      "read_eyegaze",
      &readEyeGaze,
      py::arg("path"),
      R"docdelimiter(Read Eye Gaze from the eye gaze output generated via MPS.
  Parameters
  __________
  path: Path to the eye gaze csv file.

  )docdelimiter");
  m.def(
      "get_eyegaze_point_at_depth",
      &getEyeGazePointAtDepth,
      py::arg("yaw_rads"),
      py::arg("pitch_rads"),
      py::arg("depth_m"),
      R"docdelimiter( Given the yaw and pitch angles of the eye gaze and a depth, return the gaze 3D point in CPF frame.
  Parameters
  __________
  yaw_rads: Yaw angle in radians in CPF frame.
  pitch_rads: Pitch angle in radians in CPF frame.
  depth_m: Depth of the point in meters.
  )docdelimiter");

  m.def(
      "get_gaze_intersection_point",
      &getGazeIntersectionPoint,
      py::arg("left_yaw_rads"),
      py::arg("right_yaw_rads"),
      py::arg("pitch_rads"),
      R"docdelimiter( Given the left and right yaw angles and common pitch get the intersection point in 3D in CPF frame.
  Parameters
  __________
  left_yaw_rads: Left Yaw angle in radians in CPF frame.
  right_yaw_rads: Right Yaw angle in radians in CPF frame.
  pitch_rads: Pitch angle in radians in CPF frame.
  )docdelimiter");

  m.def(
      "compute_depth_and_combined_gaze_direction",
      &computeDepthAndCombinedGazeDirection,
      py::arg("left_yaw_rads"),
      py::arg("right_yaw_rads"),
      py::arg("pitch_rads"),
      R"docdelimiter( Given the left and right yaw angles and common pitch get the combined gaze angles and depth in CPF frame.
  Parameters
  __________
  left_yaw_rads: Left Yaw angle in radians in CPF frame.
  right_yaw_rads: Right Yaw angle in radians in CPF frame.
  pitch_rads: Pitch angle in radians in CPF frame.
  )docdelimiter");

  m.def(
      "get_gaze_vectors",
      &getGazeVectors,
      py::arg("left_yaw_rads"),
      py::arg("right_yaw_rads"),
      py::arg("pitch_rads"),
      R"docdelimiter( Given the left and right yaw angles and common pitch get the left and right gaze vectors from their respective origins in XYZ CPF frame.
  Parameters
  __________
  left_yaw_rads: Left Yaw angle in radians in CPF frame.
  right_yaw_rads: Right Yaw angle in radians in CPF frame.
  pitch_rads: Pitch angle in radians in CPF frame.
  )docdelimiter");

  // trajectory
  py::class_<ClosedLoopTrajectoryPose>(
      m,
      "ClosedLoopTrajectoryPose",
      R"docdelimiter(
          Closed loop trajectory is the pose estimation output by our mapping process, in an arbitrary
  gravity aligned world coordinate frame. The estimation includes pose and dynamics (translational
  and angular velocities).

  Closed loop trajectories are fully bundle adjusted with detected loop closures, reducing the VIO
  drift which is present in the open loop trajectories. However, due to the loop closure
  correction, the “relative” and “local” trajectory accuracy within a short time span (i.e.
  seconds) might be worse compared to open loop trajectories.

  In some datasets we also share and use this format for trajectory pose ground truth from
  simulation or Optitrack
    )docdelimiter")
      .def_readwrite(
          "tracking_timestamp",
          &ClosedLoopTrajectoryPose::trackingTimestamp,
          "Timestamp for the pose in device time domain.")
      .def_readwrite(
          "utc_timestamp", &ClosedLoopTrajectoryPose::utcTimestamp, "UTC timestamp for the pose.")
      .def_readwrite(
          "transform_world_device",
          &ClosedLoopTrajectoryPose::T_world_device,
          "Transformation from this device to world coordinate frame")
      .def_readwrite(
          "device_linear_velocity_device",
          &ClosedLoopTrajectoryPose::deviceLinearVelocity_device,
          "Translational velocity of device coordinate frame in device frame")
      .def_readwrite(
          "angular_velocity_device",
          &ClosedLoopTrajectoryPose::angularVelocity_device,
          "Angular velocity of device coordinate frame in device frame")
      .def_readwrite(
          "quality_score",
          &ClosedLoopTrajectoryPose::qualityScore,
          "A float between [0, 1] which describes how good the pose and dynamics are, the higher score the estimation has higher quality.")
      .def_readwrite(
          "gravity_world",
          &ClosedLoopTrajectoryPose::gravity_world,
          "Earth gravity vector in world frame. This vector is pointing toward the ground, and includes gravitation and centrifugal forces from earth rotation.")
      .def_readwrite(
          "graph_uid",
          &ClosedLoopTrajectoryPose::graphUid,
          "Unique identifier of the world coordinate frame. When the graphUid is the same, poses, velocities and point clouds are defined in the same coordinate frame.")
      .def("__repr__", [](ClosedLoopTrajectoryPose const& self) { return fmt::to_string(self); });

  py::class_<OpenLoopTrajectoryPose>(
      m,
      "OpenLoopTrajectoryPose",
      R"docdelimiter(
        Open loop trajectory is the odometry estimation output by the visual-inertial odometry (VIO), in
        an arbitrary odometry coordinate frame. The estimation includes pose and dynamics (translational
        and angular velocities).

        The open loop trajectory has good “relative” and “local” accuracy: the relative transformation
        between two frames is accurate when the time span between two frames is short (within a few
        minutes). However, the open loop trajectory has increased drift error accumulated over time spent
        and travel distance. Consider using closed loop trajectory if you are looking for trajectory
        without drift error.
    )docdelimiter")
      .def_readwrite(
          "tracking_timestamp",
          &OpenLoopTrajectoryPose::trackingTimestamp,
          "Timestamp for the pose in device time domain.")
      .def_readwrite(
          "utc_timestamp", &OpenLoopTrajectoryPose::utcTimestamp, "UTC timestamp for the pose.")
      .def_readwrite(
          "transform_odometry_device",
          &OpenLoopTrajectoryPose::T_odometry_device,
          "Transformation from this device to an arbitrary odometry coordinate frame.")
      .def_readwrite(
          "device_linear_velocity_odometry",
          &OpenLoopTrajectoryPose::deviceLinearVelocity_odometry,
          "Translational velocity of device coordinate frame in odometry frame.")
      .def_readwrite(
          "angular_velocity_device",
          &OpenLoopTrajectoryPose::angularVelocity_device,
          "Angular velocity of device coordinate frame in device frame.")
      .def_readwrite(
          "quality_score",
          &OpenLoopTrajectoryPose::qualityScore,
          "A float between [0, 1] which describes how good the pose and dynamics are, the higher score the estimation has higher quality.")
      .def_readwrite(
          "gravity_odometry",
          &OpenLoopTrajectoryPose::gravity_odometry,
          "Earth gravity vector in odometry frame. This vector is pointing toward the ground, and includes gravitation and centrifugal forces from earth rotation.")
      .def_readwrite(
          "session_uid",
          &OpenLoopTrajectoryPose::sessionUid,
          "Unique identifier of the odometry coordinate frame. When the session_uid is the same, poses and velocities are defined in the same coordinate frame.")
      .def("__repr__", [](OpenLoopTrajectoryPose const& self) { return fmt::to_string(self); });
  m.def(
      "read_open_loop_trajectory",
      &readOpenLoopTrajectory,
      py::arg("path"),
      R"docdelimiter(Read Open loop trajectory.
  Parameters
  __________
  path: Path to the open loop trajectory csv file. Usually named 'open_loop_trajectory.csv'
  )docdelimiter");

  m.def(
      "read_closed_loop_trajectory",
      &readClosedLoopTrajectory,
      py::arg("path"),
      R"docdelimiter(Read Closed loop trajectory.
  Parameters
  __________
  path: Path to the closed loop trajectory csv file. Usually named 'closed_loop_trajectory.csv'
  )docdelimiter");

  // online calibrations
  py::class_<OnlineCalibration>(m, "OnlineCalibration")
      .def_readwrite(
          "tracking_timestamp",
          &OnlineCalibration::trackingTimestamp,
          "Timestamp for the calibration sample in device time domain.")
      .def_readwrite(
          "utc_timestamp",
          &OnlineCalibration::utcTimestamp,
          "UTC Timestamp of the device image capture")
      .def_readwrite(
          "camera_calibs", &OnlineCalibration::cameraCalibs, "Online estimated camera calibrations")
      .def_readwrite(
          "imu_calibs", &OnlineCalibration::imuCalibs, "Online estimated IMU calibrations")
      .def("__repr__", [](OnlineCalibration const& self) { return fmt::to_string(self); })
      .def(
          "get_camera_calib",
          &OnlineCalibration::getCameraCalib,
          "Helper function to get the camera calibration of a specific camera label",
          py::arg("label"));

  m.def(
      "read_online_calibration",
      &readOnlineCalibration,
      py::arg("path"),
      R"docdelimiter(Read estimated online calibrations.
  Parameters
  __________
  path: Path to the online calibration jsonl file. Usually named 'online_calibration.jsonl'
  )docdelimiter");

  // compression mode
  py::enum_<StreamCompressionMode>(m, "StreamCompressionMode", "Stream compression mode")
      .value("NONE", StreamCompressionMode::NONE, "No compression")
      .value("GZIP", StreamCompressionMode::GZIP, "GZIP compression");

  // point cloud
  py::class_<GlobalPointPosition>(m, "GlobalPointPosition")
      .def_readwrite(
          "uid", &GlobalPointPosition::uid, "A unique identifier of this point within this map")
      .def_readwrite(
          "graph_uid",
          &GlobalPointPosition::graphUid,
          "Unique identifier of the world coordinate frame")
      .def_readwrite(
          "position_world",
          &GlobalPointPosition::position_world,
          "The position of this point relative the world coordinate frame")
      .def_readwrite(
          "inverse_distance_std",
          &GlobalPointPosition::inverseDistanceStd,
          "Standard deviation of the inverse distance estimate")
      .def_readwrite(
          "distance_std",
          &GlobalPointPosition::distanceStd,
          "Standard deviation of distance estimate")
      .def("__repr__", [](const GlobalPointPosition& self) { return fmt::to_string(self); });

  m.def(
      "read_global_point_cloud",
      [](const std::string& path, StreamCompressionMode& mode) -> GlobalPointCloud {
        auto warnings = pybind11::module::import("warnings");
        warnings.attr("warn")(
            "readGlobalPointCloud(path, mode) is deprecated, use readGlobalPointCloud(path) instead.");

        return readGlobalPointCloud(path);
      },
      py::arg("path"),
      py::arg("compression"),
      R"docdelimiter(Read global point cloud.
  Parameters
  __________
  path: Path to the global point cloud file. Usually named 'global_pointcloud.csv.gz'
  compression: Stream compression mode for reading csv file.
  )docdelimiter");

  m.def(
      "read_global_point_cloud",
      [](const std::string& path) -> GlobalPointCloud { return readGlobalPointCloud(path); },
      py::arg("path"),
      R"docdelimiter(Read global point cloud.
  Parameters
  __________
  path: Path to the global point cloud file. Usually named 'global_pointcloud' with a '.csv' or '.csv.gz'
  )docdelimiter");

  // point observations
  py::class_<PointObservation>(m, "PointObservation", "2D observations of the point")
      .def_readwrite(
          "point_uid",
          &PointObservation::pointUid,
          "A unique identifier of this point within this map")
      .def_readwrite(
          "frame_capture_timestamp",
          &PointObservation::frameCaptureTimestamp,
          "Board timestamp of the host frame’s center of exposure")
      .def_readwrite(
          "camera_serial",
          &PointObservation::cameraSerial,
          "The serial number of the camera which observes this point")
      .def_readwrite(
          "uv",
          &PointObservation::uv,
          "The observed measurement (pixels) of the point in the observing frame’s camera")
      .def("__repr__", [](const PointObservation& self) { return fmt::to_string(self); });

  m.def(
      "read_point_observations",
      [](const std::string& path, StreamCompressionMode& mode) -> PointObservations {
        auto warnings = pybind11::module::import("warnings");
        warnings.attr("warn")(
            "readPointObservations(path, mode) is deprecated, use readPointObservations(path) instead.");

        return readPointObservations(path);
      },
      py::arg("path"),
      py::arg("compression"),
      R"docdelimiter(Read point observations.
  Parameters
  __________
  path: Path to the point observations file. Usually named 'semidense_observations.csv.gz'
  compression: Stream compression mode for reading csv file.
  )docdelimiter");

  m.def(
      "read_point_observations",
      [](const std::string& path) -> PointObservations { return readPointObservations(path); },
      py::arg("path"),
      R"docdelimiter(Read point observations.
  Parameters
  __________
  path: Path to the point observations file. Usually named 'semidense_observations' with a '.csv' or '.csv.gz'
  )docdelimiter");

  // static camera calibrations
  py::class_<StaticCameraCalibration>(
      m,
      "StaticCameraCalibration",
      "Static camera intrinsic calibration and extrinsics in the world frame")
      .def_readwrite(
          "camera_uid",
          &StaticCameraCalibration::cameraUid,
          "Static camera's unique identifier, currently we are using path of the video file")
      .def_readwrite(
          "graph_uid", &StaticCameraCalibration::graphUid, "UID of the world coordinate frame")
      .def_readwrite(
          "transform_world_cam",
          &StaticCameraCalibration::T_world_cam,
          "Static camera's pose in world frame")
      .def_readwrite("width", &StaticCameraCalibration::width, "image size")
      .def_readwrite("height", &StaticCameraCalibration::height, "image size")
      .def_readwrite(
          "intrinsics_type",
          &StaticCameraCalibration::intrinsicsType,
          "Intrinsics type string. Currently the only intrinsics type supported is 'KANNALABRANDTK3', a.k.a OpenCV fisheye model")
      .def_readwrite(
          "intrinsics", &StaticCameraCalibration::intrinsics, "cam intrinsic calibration params")
      .def_readwrite(
          "start_frame_idx",
          &StaticCameraCalibration::startFrameIdx,
          "The start frame number from the video when the camera is stationary and camera pose result is applicable. Not available, when the pose is applicable to the whole video")
      .def_readwrite(
          "end_frame_idx",
          &StaticCameraCalibration::endFrameIdx,
          "The end frame number from the video when the camera is stationary and camera pose result is applicable. Not available, when the pose is applicable to the whole video")
      .def_readwrite(
          "quality",
          &StaticCameraCalibration::quality,
          "Quality of the reloc. -1 means no quality information available, 1 means good quality, 0 means bad quality")
      .def("__repr__", [](const StaticCameraCalibration& self) { return fmt::to_string(self); });

  m.def(
      "read_static_camera_calibrations",
      &readStaticCameraCalibrations,
      py::arg("path"),
      R"docdelimiter(Read static camera calibrations.
  Parameters
  __________
  path: Path to the static camera calibrations file.
  )docdelimiter");

  // MPS data paths provider
  py::class_<MpsDataPaths>(
      m, "MpsDataPaths", "A struct that includes the file paths of all MPS data for a sequence.")
      .def(py::init<>())
      .def_readwrite("slam", &MpsDataPaths::slam, "MPS SLAM file paths")
      .def_readwrite("eyegaze", &MpsDataPaths::eyegaze, "MPS eyegaze file paths")
      .def_readwrite("hand_tracking", &MpsDataPaths::handTracking, "MPS hand tracking file paths")
      .def_readwrite("root", &MpsDataPaths::root, "MPS root directory path")
      .def("__repr__", [](MpsDataPaths const& self) { return fmt::to_string(self); });

  py::class_<MpsSlamDataPaths>(
      m,
      "MpsSlamDataPaths",
      "A struct that includes the file paths of all MPS SLAM data for a VRS sequence processed by MPS.")
      .def_readwrite(
          "closed_loop_trajectory",
          &MpsSlamDataPaths::closedLoopTrajectory,
          "Closed loop trajectory")
      .def_readwrite(
          "open_loop_trajectory", &MpsSlamDataPaths::openLoopTrajectory, "Open loop trajectory")
      .def_readwrite("semidense_points", &MpsSlamDataPaths::semidensePoints, "Semidense points")
      .def_readwrite(
          "semidense_observations",
          &MpsSlamDataPaths::semidenseObservations,
          "Semidense point observations")
      .def_readwrite(
          "online_calibrations",
          &MpsSlamDataPaths::onlineCalibration,
          "Online calibration results from SLAM")
      .def_readwrite("summary", &MpsSlamDataPaths::summary, "SLAM summary")
      .def("__repr__", [](MpsSlamDataPaths const& self) { return fmt::to_string(self); });

  py::class_<MpsEyegazeDataPaths>(
      m,
      "MpsEyegazeDataPaths",
      "A struct that includes the file paths of all MPS eye gaze data for a sequence.")
      .def_readwrite(
          "general_eyegaze",
          &MpsEyegazeDataPaths::generalEyegaze,
          "General (non-calibrated) eyegaze results")
      .def_readwrite(
          "personalized_eyegaze",
          &MpsEyegazeDataPaths::personalizedEyegaze,
          "Personalized (calibrated) eyegaze results")
      .def_readwrite("summary", &MpsEyegazeDataPaths::summary, "Eyegaze summary")
      .def("__repr__", [](MpsEyegazeDataPaths const& self) { return fmt::to_string(self); });

  py::class_<HandTrackingDataPaths>(
      m,
      "HandTrackingDataPaths",
      "A struct that includes the file paths of all MPS Hand Tracking data for a VRS sequence processed by MPS.")
      .def_readwrite(
          "wrist_and_palm_poses", &HandTrackingDataPaths::wristAndPalmPoses, "Wrist and palm poses")
      .def_readwrite("summary", &HandTrackingDataPaths::summary, "Hand Tracking summary")
      .def("__repr__", [](HandTrackingDataPaths const& self) { return fmt::to_string(self); });

  py::class_<MpsDataPathsProvider>(
      m,
      "MpsDataPathsProvider",
      "This class is allows you to get all MPS data paths associated with an Aria sequence. \n"
      "Note that all Aria open datasets will have MPS results which fit the format specified in this data provider.\n"
      "Use this data provider to avoid breaking changes in your code due to changes in MPS files\n")
      .def(py::init<const std::string&>())
      .def("get_data_paths", &MpsDataPathsProvider::getDataPaths, "Get the resulting data paths");

  py::class_<MpsDataProvider, std::shared_ptr<MpsDataProvider>>(
      m,
      "MpsDataProvider",
      "This class is to load all MPS data given an MpsDataPaths object, and also provide all API needed "
      "to query that data. NOTE: to minimize disk usage, this data provider only loads data from disk "
      "after that data type is first queried.\n")
      .def(py::init<const MpsDataPaths&>())
      .def(
          "has_general_eyegaze",
          &MpsDataProvider::hasGeneralEyeGaze,
          "Check if general eye gaze data is available in the MPS data paths")
      .def(
          "has_personalized_eyegaze",
          &MpsDataProvider::hasPersonalizedEyeGaze,
          "Check if personalized eye gaze data is available in the MPS data paths")
      .def(
          "has_open_loop_poses",
          &MpsDataProvider::hasOpenLoopPoses,
          "Check if open loop poses are available in the MPS data paths")
      .def(
          "has_closed_loop_poses",
          &MpsDataProvider::hasClosedLoopPoses,
          "Check if closed loop poses are available in the MPS data paths")
      .def(
          "has_online_calibrations",
          &MpsDataProvider::hasOnlineCalibrations,
          "Check if online calibrations are available in the MPS data paths")
      .def(
          "has_semidense_point_cloud",
          &MpsDataProvider::hasSemidensePointCloud,
          "Check if semidense point cloud data is available in the MPS data paths")
      .def(
          "has_semidense_observations",
          &MpsDataProvider::hasSemidenseObservations,
          "Check if semidense observations are available in the MPS data paths")
      .def(
          "has_wrist_and_palm_poses",
          &MpsDataProvider::hasWristAndPalmPoses,
          "Check if wrist and palm poses are available in the MPS data paths")
      .def(
          "get_general_eyegaze",
          &MpsDataProvider::getGeneralEyeGaze,
          "Query MPS for general EyeGaze at a specific timestamp. This will throw an exception if "
          "general eye gaze data is not available. Check for data availability first using: "
          "`has_general_eyegaze()`",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_personalized_eyegaze",
          &MpsDataProvider::getPersonalizedEyeGaze,
          "Query MPS for personalized EyeGaze at a specific timestamp. This will throw an "
          "exception if personalized eye gaze data is not available. Check for data availability "
          "first using `has_personalized_eyegaze()`",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_open_loop_pose",
          &MpsDataProvider::getOpenLoopPose,
          "Query MPS for OpenLoopTrajectoryPose at a specific timestamp. This will throw an "
          "exception if open loop trajectory data is not available. Check for data availability "
          "first using `has_open_loop_poses()`",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_closed_loop_pose",
          &MpsDataProvider::getClosedLoopPose,
          "Query MPS for ClosedLoopTrajectoryPose at a specific timestamp. This will throw an "
          "exception if open loop trajectory data is not available. Check for data availability "
          "first using `has_closed_loop_poses()`",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get interpolated closed loop pose",
          &MpsDataProvider::getInterpolatedClosedLoopPose,
          "Query MPS for ClosedLoopTrajectoryPose at a specific timestamp."
          "interpolate between two poses if the timestamp is not exactly matched,"
          "and return None if query time is out of trajectory's time range.",
          py::arg("device_timestamp_ns"))
      .def(
          "get_online_calibration",
          &MpsDataProvider::getOnlineCalibration,
          "Query MPS for OnlineCalibration at a specific timestamp. This will throw an exception "
          "if online calibration data is not available. Check for data availability first "
          "using `has_online_calibrations()`",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_rgb_corrected_closed_loop_pose",
          &MpsDataProvider::getRgbCorrectedClosedLoopPose,
          py::return_value_policy::reference_internal,
          "Get the corrected rgb frame pose based on the online calibration.",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_rgb_corrected_timestamp_ns",
          &MpsDataProvider::getRgbCorrectedTimestampNs,
          py::return_value_policy::reference_internal,
          "Get the corrected rgb frame timestamp based on the online calibration.",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_semidense_point_cloud",
          &MpsDataProvider::getSemidensePointCloud,
          py::return_value_policy::reference_internal,
          "Get the MPS semidense point cloud. This will throw an exception if the point cloud is "
          "not available. Check for data availability first using 'has_semidense_point_cloud()'")
      .def(
          "get_semidense_observations",
          &MpsDataProvider::getSemidenseObservations,
          py::return_value_policy::reference_internal,
          "Get the MPS point observations. This will throw an exception if the observations are "
          "not available. Check for data availability first using 'has_semidense_observations()'")
      .def(
          "get_wrist_and_palm_pose",
          &MpsDataProvider::getWristAndPalmPose,
          py::return_value_policy::reference_internal,
          "Get the MPS wrist and palm pose. This will throw an exception if the wrist and palm "
          "poses are not available. Check for data availability first using "
          "'has_wrist_and_palm_poses()'");

  py::module hand_tracking = m.def_submodule("hand_tracking");

  py::class_<WristAndPalmPose>(
      hand_tracking,
      "WristAndPalmPose",
      "An object representing WristAndPalmPose output at a single timestamp.")
      .def_readwrite(
          "tracking_timestamp",
          &WristAndPalmPose::trackingTimestamp,
          "The timestamp of the pose estimate in device time domain.")
      .def_readwrite(
          "left_hand",
          &WristAndPalmPose::leftHand,
          "Left hand pose estimate, or None if no valid pose was found.")
      .def_readwrite(
          "right_hand",
          &WristAndPalmPose::rightHand,
          "Right hand pose estimate, or None if no valid pose was found.")
      .def("__repr__", [](WristAndPalmPose const& self) { return fmt::to_string(self); });

  py::class_<WristAndPalmPose::OneSide>(
      hand_tracking,
      "WristAndPalmPose.OneSide",
      "An object representing WristAndPalmPose output for one side of the body.")
      .def_readwrite(
          "confidence",
          &WristAndPalmPose::OneSide::confidence,
          "Tracking confidence score for this hand.")
      .def_readwrite(
          "wrist_position_device",
          &WristAndPalmPose::OneSide::wristPosition_device,
          "Position of the wrist joint in device frame.")
      .def_readwrite(
          "palm_position_device",
          &WristAndPalmPose::OneSide::palmPosition_device,
          "Position of the palm joint in device frame.")
      .def_readwrite(
          "wrist_and_palm_normal_device",
          &WristAndPalmPose::OneSide::wristAndPalmNormal_device,
          "Normal for each of the wrist and palm joint in device frame.")
      .def("__repr__", [](WristAndPalmPose::OneSide const& self) { return fmt::to_string(self); });

  py::class_<WristAndPalmPose::OneSide::WristAndPalmNormals>(
      hand_tracking,
      "WristAndPalmPose.OneSide.WristAndPalmNormals",
      "An object representing WristAndPalmNormals output for one side of the body.")
      .def_readwrite(
          "wrist_normal_device",
          &WristAndPalmPose::OneSide::WristAndPalmNormals::wristNormal_device,
          "Normal for wrist joint in device frame.")
      .def_readwrite(
          "palm_normal_device",
          &WristAndPalmPose::OneSide::WristAndPalmNormals::palmNormal_device,
          "Normal for palm joint in device frame.")
      .def("__repr__", [](WristAndPalmPose::OneSide::WristAndPalmNormals const& self) {
        return fmt::to_string(self);
      });

  hand_tracking.def(
      "read_wrist_and_palm_poses",
      &readWristAndPalmPoses,
      py::arg("path"),
      R"docdelimiter(Read Wrist and Palm poses from the hand tracking output generated via MPS.
  Parameters
  __________
  path: Path to the wrist and palm poses csv file.

  )docdelimiter");
}

} // namespace projectaria::tools::mps
