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
#include "EyeGazeReader.h"
#include "GlobalPointCloudFormat.h"
#include "GlobalPointCloudReader.h"
#include "OnlineCalibrationFormat.h"
#include "PointObservationFormat.h"
#include "PointObservationReader.h"
#include "StaticCameraCalibrationFormat.h"
#include "StaticCameraCalibrationReader.h"
#include "TrajectoryFormat.h"
#include "TrajectoryReaders.h"
#include "onlineCalibrationReader.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace projectaria::tools::mps {

void exportMps(py::module& m) {
  // For submodule documentation, see: projectaria_tools/projectaria_tools/core/mps.py

  // gaze
  py::class_<EyeGaze>(m, "EyeGaze", "An object representing single Eye gaze output.")
      .def_readwrite(
          "tracking_timestamp",
          &EyeGaze::trackingTimestamp,
          "Timestamp of the eye tracking camera frame in device time domain.")
      .def_readwrite(
          "yaw", &EyeGaze::yaw, "Eye gaze yaw angle (horizontal) in radians in CPF frame.")
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
      "path"_a,
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
      "path"_a,
      R"docdelimiter(Read Open loop trajectory.
  Parameters
  __________
  path: Path to the open loop trajectory csv file. Usually named 'open_loop_trajectory.csv'

  )docdelimiter");
  m.def(
      "read_closed_loop_trajectory",
      &readClosedLoopTrajectory,
      "path"_a,
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
      .def("__repr__", [](OnlineCalibration const& self) { return fmt::to_string(self); });

  m.def(
      "read_online_calibration",
      &readOnlineCalibration,
      "path"_a,
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
      &readGlobalPointCloud,
      "path"_a,
      "compression"_a,
      R"docdelimiter(Read global point cloud.
  Parameters
  __________
  path: Path to the global point cloud file. Usually named 'global_pointcloud.csv.gz'
  compression: Stream compression mode for reading csv file.

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
      &readPointObservations,
      "path"_a,
      "compression"_a,
      R"docdelimiter(Read point observations.
  Parameters
  __________
  path: Path to the point observations file. Usually named 'semidense_observations.csv.gz'
  compression: Stream compression mode for reading csv file.

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
      .def("__repr__", [](const StaticCameraCalibration& self) { return fmt::to_string(self); });

  m.def(
      "read_static_camera_calibrations",
      &readStaticCameraCalibrations,
      "path"_a,
      R"docdelimiter(Read static camera calibrations.
  Parameters
  __________
  path: Path to the static camera calibrations file.

  )docdelimiter");
}

} // namespace projectaria::tools::mps
