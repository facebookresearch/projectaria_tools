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

#include <fstream>

#include <calibration/CameraCalibrationFormat.h>
#include <calibration/DeviceCalibration.h>
#include <calibration/DeviceVersion.h>
#include <calibration/ImuMagnetometerCalibrationFormat.h>
#include <calibration/loader/DeviceCalibrationJson.h>
#include <calibration/utility/Distort.h>
#include <image/utility/ColorCorrect.h>
#include <image/utility/Devignetting.h>

#include <fmt/format.h>
#include <sophus/se3.hpp>

#include "ImageDataHelper.h"

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace projectaria::tools::calibration {

namespace py = pybind11;

namespace {

inline void declareDeviceVersion(py::module& m) {
  py::enum_<DeviceVersion>(
      m, "DeviceVersion", "A enum class that represents the version of the device: Gen1, Gen2.")
      .value("NotValid", DeviceVersion::NotValid)
      .value("Gen1", DeviceVersion::Gen1)
      .value("Gen2", DeviceVersion::Gen2)
      .export_values()
      .def(
          py::pickle(
              [](const DeviceVersion& version) { // __getstate__
                return static_cast<int>(version);
              },
              [](int state) { // __setstate__
                return static_cast<DeviceVersion>(state);
              }));
  m.def("get_name", &getName, "Get the name from device version", py::arg("deviceVersion"));
  m.def(
      "from_device_class_name",
      &fromDeviceClassName,
      "Get the device version from a class name in calibration",
      py::arg("name"));
}

inline void declareCameraCalibration(py::module& m) {
  using namespace projectaria::tools::calibration;

  py::class_<CameraProjection>(
      m,
      "CameraProjection",
      "A struct to represent a camera projection instance, which is basically camera intrinsics."
      " This struct stores the intrinsics parameters internally.")
      .def(py::init<>(), "Default constructor, creates an empty CameraProjection instance.")
      .def(
          py::init<const CameraProjection::ModelType&, const Eigen::VectorXd&>(),
          R"pbdoc(Constructor with a list of parameters for CameraProjection.
          Args:
            type: The type of projection model, e.g. ModelType::Linear.
            projection_params: The projection parameters.
          )pbdoc")
      .def("model_name", &CameraProjection::modelName)
      .def("projection_params", &CameraProjection::projectionParams)
      .def(
          "project",
          &CameraProjection::project,
          py::arg("point_in_camera"),
          py::arg("jacobian_wrt_point") = nullptr,
          "projects a 3d world point in the camera space to a 2d pixel in the image space."
          " No checks performed in this process.")
      .def(
          "unproject",
          &CameraProjection::unproject,
          py::arg("camera_pixel"),
          "unprojects a 2d pixel in the image space to a 3d world point in homogenous coordinate.",
          " No checks performed in this process.")
      .def(
          "get_principal_point",
          &CameraProjection::getPrincipalPoint,
          "returns principal point location as {cx, cy}.")
      .def(
          "get_focal_lengths",
          &CameraProjection::getFocalLengths,
          "returns focal lengths as {fx, fy}.")
      .def(
          py::pickle(
              [](const CameraProjection& projection) { // __getstate__
                return py::make_tuple(projection.modelName(), projection.projectionParams());
              },
              [](py::tuple t) { // __setstate__
                if (t.size() != 2) {
                  throw std::runtime_error("Invalid state!");
                }

                auto modelType = t[0].cast<CameraProjection::ModelType>();
                auto projectionParams = t[1].cast<Eigen::VectorXd>();
                return CameraProjection(modelType, projectionParams);
              }));

  py::enum_<CameraProjection::ModelType>(
      m,
      "CameraModelType",
      "Enum that represents the type of camera projection model. See Linear.h, Spherical.h,"
      " KannalaBrandtK3.h and FisheyeRadTanThinPrism.h for details.")
      .value(
          "KANNALA_BRANDT_K3",
          CameraProjection::ModelType::KannalaBrandtK3,
          "Spherical + polynomial radial distortion up to 9-th order.")
      .value(
          "FISHEYE624",
          CameraProjection::ModelType::Fisheye624,
          "Spherical + polynomial radial distortion up to 11-th order + tangential distortion.")
      .value(
          "SPHERICAL",
          CameraProjection::ModelType::Spherical,
          "Spherical projection, linear in angular space.")
      .value(
          "LINEAR",
          CameraProjection::ModelType::Linear,
          "Linear pinhole projection, unit plane points and camera pixels are linearly related.")
      .value(
          "FISHEYE62",
          CameraProjection::ModelType::Fisheye62,
          "Spherical + polynomial radial distortion up to 11-th order.")
      .export_values()
      .def(
          py::pickle(
              [](const CameraProjection::ModelType& type) { // __getstate__
                return static_cast<int>(type);
              },
              [](int state) { // __setstate__
                return static_cast<CameraProjection::ModelType>(state);
              }));

  py::class_<CameraCalibration>(
      m,
      "CameraCalibration",
      "A class that provides APIs for camera calibration, including extrinsics, intrinsics, and projection.")
      .def(py::init<>())
      .def(
          py::init<
              const std::string&,
              const CameraProjection::ModelType&,
              const Eigen::VectorXd&,
              const Sophus::SE3d&,
              const int,
              const int,
              const std::optional<double>,
              const double,
              const std::string&>(),
          R"pbdoc(Constructor with a list of parameters for CameraCalibration.
  Args:
    label: The label of the camera, e.g. "camera-slam-left".
    projection_model_type The type of camera projection model, e.g. ModelType::Linear
    projection_params: The projection parameters.
    T_Device_Camera: The extrinsics of camera in Device frame.
    image_width: Width of camera image.
    image_height: Height of camera image.
    maybe_valid_radius: [optional] radius of a circular mask that represents the valid area on
            the camera's sensor plane. Pixels out of this circular region are considered invalid. Setting
            this to None means the entire sensor plane is valid.
    max_solid_angle: an angle theta representing the FOV cone of the camera. Rays out of
            [-theta, +theta] will be rejected during projection
    serial_number: Serial number of the camera.)pbdoc")
      .def(
          py::init<
              const std::string&,
              const CameraProjection::ModelType&,
              const Eigen::VectorXd&,
              const Sophus::SE3d&,
              const int,
              const int,
              const std::optional<double>,
              const double,
              const std::string&,
              const double>(),
          R"pbdoc(Constructor with a list of parameters for CameraCalibration.
  Args:
    label: The label of the camera, e.g. "camera-slam-left".
    projection_model_type The type of camera projection model, e.g. ModelType::Linear
    projection_params: The projection parameters.
    T_Device_Camera: The extrinsics of camera in Device frame.
    image_width: Width of camera image.
    image_height: Height of camera image.
    maybe_valid_radius: [optional] radius of a circular mask that represents the valid area on
            the camera's sensor plane. Pixels out of this circular region are considered invalid. Setting
            this to None means the entire sensor plane is valid.
    max_solid_angle: an angle theta representing the FOV cone of the camera. Rays out of
            [-theta, +theta] will be rejected during projection
    serial_number: Serial number of the camera.
    time_offset_sec_device_camera: time offset in second between the camera mid exposure time and the capture
    timestamp.)pbdoc")
      .def(
          py::init<
              const std::string&,
              const CameraProjection::ModelType&,
              const Eigen::VectorXd&,
              const Sophus::SE3d&,
              const int,
              const int,
              const std::optional<double>,
              const double,
              const std::string&,
              const double,
              const std::optional<double>>(),
          R"pbdoc(Constructor with a list of parameters for CameraCalibration.
  Args:
    label: The label of the camera, e.g. "camera-slam-left".
    projection_model_type The type of camera projection model, e.g. ModelType::Linear
    T_Device_Camera: The extrinsics of camera in Device frame.
    projection_params: The projection parameters.
    image_width: Width of camera image.
    image_height: Height of camera image.
    maybe_valid_radius: [optional] radius of a circular mask that represents the valid area on
            the camera's sensor plane. Pixels out of this circular region are considered invalid. Setting
            this to None means the entire sensor plane is valid.
    max_solid_angle: an angle theta representing the FOV cone of the camera. Rays out of
            [-theta, +theta] will be rejected during projection
    serial_number: Serial number of the camera.
    time_offset_sec_device_camera: time offset in second between the camera mid exposure time and the capture
    timestamp.
    maybe_readout_time_sec: readout time in second to read from the first pixel to the last pixel.)pbdoc")
      .def("get_label", &CameraCalibration::getLabel)
      .def("get_serial_number", &CameraCalibration::getSerialNumber)
      .def("get_transform_device_camera", &CameraCalibration::getT_Device_Camera)
      .def("get_image_size", &CameraCalibration::getImageSize)
      .def("get_max_solid_angle", &CameraCalibration::getMaxSolidAngle)
      .def("get_valid_radius", &CameraCalibration::getValidRadius)
      .def("get_time_offset_sec_device_camera", &CameraCalibration::getTimeOffsetSecDeviceCamera)
      .def("get_readout_time_sec", &CameraCalibration::getReadOutTimeSec)
      .def(
          "is_visible",
          &CameraCalibration::isVisible,
          py::arg("camera_pixel"),
          "Function to check whether a pixel is within the valid area of the sensor plane.")
      .def(
          "model_name",
          [](const CameraCalibration& self) -> CameraProjection::ModelType {
            auto warnings = pybind11::module::import("warnings");
            warnings.attr("warn")(
                "model_name(stream_id) is deprecated, use get_model_name() instead.");
            return self.modelName();
          })
      .def("get_model_name", &CameraCalibration::modelName)
      .def("get_principal_point", &CameraCalibration::getPrincipalPoint)
      .def("get_focal_lengths", &CameraCalibration::getFocalLengths)
      .def(
          "projection_params",
          [](const CameraCalibration& self) -> Eigen::VectorXd {
            auto warnings = pybind11::module::import("warnings");
            warnings.attr("warn")(
                "projection_params() is deprecated, use get_projection_params() instead.");
            return self.projectionParams();
          })
      .def("get_projection_params", &CameraCalibration::projectionParams)
      .def(
          "project_no_checks",
          &CameraCalibration::projectNoChecks,
          py::arg("point_in_camera"),
          "Function to project a 3d point (in camera frame) to a 2d camera pixel location. In this"
          " function, no check is performed.")
      .def(
          "project",
          &CameraCalibration::project,
          py::arg("point_in_camera"),
          "Function to project a 3d point (in camera frame) to a 2d camera pixel location, with a"
          " number of validity checks to ensure the point is visible.")
      .def(
          "unproject_no_checks",
          &CameraCalibration::unprojectNoChecks,
          py::arg("camera_pixel"),
          "Function to unproject a 2d pixel location to a 3d ray in camera frame. In this function,"
          " no check is performed.")
      .def(
          "unproject",
          &CameraCalibration::unproject,
          py::arg("camera_pixel"),
          "Function to unproject a 2d pixel location to a 3d ray, in camera frame, with a number of"
          " validity checks to ensure the unprojection is valid.")
      .def(
          "rescale",
          &CameraCalibration::rescale,
          py::arg("new_resolution"),
          py::arg("scale"),
          py::arg("origin_offset") = Eigen::Vector2d{0, 0},
          "Obtain a new camera calibration after translation and scaling transform from the original "
          "camera calibration. <br> transform is done in the order of (1) shift -> (2) scaling. <\br>"
          "Note that assymetric cropping is allowed")
      .def(
          py::pickle(
              [](const CameraCalibration& calib) { // __getstate__
                return py::make_tuple(
                    calib.getLabel(),
                    calib.modelName(),
                    calib.projectionParams(),
                    calib.getT_Device_Camera(),
                    calib.getImageSize(),
                    calib.getValidRadius(),
                    calib.getMaxSolidAngle(),
                    calib.getSerialNumber(),
                    calib.getTimeOffsetSecDeviceCamera(),
                    calib.getReadOutTimeSec());
              },
              [](py::tuple t) { // __setstate__
                if (t.size() != 10) {
                  throw std::runtime_error("Invalid state!");
                }

                return CameraCalibration(
                    t[0].cast<std::string>(),
                    t[1].cast<CameraProjection::ModelType>(),
                    t[2].cast<Eigen::VectorXd>(),
                    t[3].cast<Sophus::SE3d>(),
                    t[4].cast<std::pair<int, int>>().first,
                    t[4].cast<std::pair<int, int>>().second,
                    t[5].cast<std::optional<double>>(),
                    t[6].cast<double>(),
                    t[7].cast<std::string>(),
                    t[8].cast<double>(),
                    t[9].cast<std::optional<double>>());
              }))
      .def("__repr__", [](const CameraCalibration& self) { return fmt::to_string(self); });

  m.def(
      "get_linear_camera_calibration",
      getLinearCameraCalibration,
      "Function to create a simple Linear camera calibration object from some parameters.",
      py::arg("image_width"),
      py::arg("image_height"),
      py::arg("focal_length"),
      py::arg("label") = "",
      py::arg("T_Device_Camera") = Sophus::SE3d{},
      py::arg("time_offset_sec_device_camera") = 0.0);

  m.def(
      "get_spherical_camera_calibration",
      getSphericalCameraCalibration,
      "Function to create a simple Spherical camera calibration object from some parameters.",
      py::arg("image_width"),
      py::arg("image_height"),
      py::arg("focal_length"),
      py::arg("label") = "",
      py::arg("T_Device_Camera") = Sophus::SE3d{},
      py::arg("time_offset_sec_device_camera") = 0.0);

  m.def(
      "rotate_camera_calib_cw90deg",
      rotateCameraCalibCW90Deg,
      "Rotate CameraCalibration (Linear model only) clock-wise for 90 degrees (Upright view)",
      py::arg("camera_calibration"));
}

inline void declareLinearRectificationModel(py::module& m) {
  py::class_<LinearRectificationModel3d>(
      m,
      "LinearRectificationModel3d",
      "A class that contains imu and mag intrinsics rectification model.")
      .def(py::init<const Eigen::Matrix3d&, const Eigen::Vector3d&>())
      .def(
          "get_rectification",
          &LinearRectificationModel3d::getRectification,
          "Get the rectification matrix. ")
      .def("get_bias", &LinearRectificationModel3d::getBias, "Get the bias vector.")
      .def(py::pickle( // __getstate__
          [](const LinearRectificationModel3d& model) {
            return py::make_tuple(model.getRectification(), model.getBias());
          },
          [](py::tuple t) { // __setstate__
            if (t.size() != 2) {
              throw std::runtime_error("Invalid state!");
            }

            return LinearRectificationModel3d(
                t[0].cast<Eigen::Matrix3d>(), t[1].cast<Eigen::Vector3d>());
          }));
}

inline void declareImuCalibration(py::module& m) {
  py::class_<ImuCalibration>(
      m,
      "ImuCalibration",
      "A class representing an IMU calibration model, including both accelerometer and gyroscope."
      " We assume the accelerometer and gyroscope for each IMU are co-located and thus they share the same extrinsic.")
      .def(
          py::init<
              const std::string&,
              const Eigen::Matrix3d&,
              const Eigen::Vector3d&,
              const Eigen::Matrix3d&,
              const Eigen::Vector3d&,
              const Sophus::SE3d&>())
      .def("get_label", &ImuCalibration::getLabel)
      .def(
          "raw_to_rectified_accel",
          &ImuCalibration::rawToRectifiedAccel,
          py::arg("raw"),
          "convert from imu sensor readout to actual acceleration: "
          "rectified = rectificationMatrix.inv() * (raw - bias).")
      .def(
          "rectified_to_raw_accel",
          &ImuCalibration::rectifiedToRawAccel,
          py::arg("rectified"),
          "simulate imu accel sensor readout from actual acceleration:"
          " raw = rectificationMatrix * rectified + bias.")
      .def(
          "raw_to_rectified_gyro",
          &ImuCalibration::rawToRectifiedGyro,
          py::arg("raw"),
          "convert from imu sensor readout to actual angular velocity: "
          "rectified = rectificationMatrix.inv() * (raw - bias).")
      .def(
          "rectified_to_raw_gyro",
          &ImuCalibration::rectifiedToRawGyro,
          py::arg("rectified"),
          "simulate imu gyro sensor readout from actual angular velocity: "
          " raw = rectificationMatrix * rectified + bias.")
      .def(
          "get_accel_model",
          &ImuCalibration::getAccelModel,
          "Get accelerometer intrinsics model that contains rectification matrix and bias vector.")
      .def(
          "get_gyro_model",
          &ImuCalibration::getGyroModel,
          "Get gyroscope intrinsics model that contains rectification matrix and bias vector.")
      .def("get_transform_device_imu", &ImuCalibration::getT_Device_Imu)
      .def(
          py::pickle(
              [](const ImuCalibration& calib) { // __getstate__
                return py::make_tuple(
                    calib.getLabel(),
                    calib.getAccelModel().getRectification(),
                    calib.getAccelModel().getBias(),
                    calib.getGyroModel().getRectification(),
                    calib.getGyroModel().getBias(),
                    calib.getT_Device_Imu());
              },
              [](py::tuple t) { // __setstate__
                if (t.size() != 6) {
                  throw std::runtime_error("Invalid state!");
                }

                return ImuCalibration(
                    t[0].cast<std::string>(),
                    t[1].cast<Eigen::Matrix3d>(),
                    t[2].cast<Eigen::Vector3d>(),
                    t[3].cast<Eigen::Matrix3d>(),
                    t[4].cast<Eigen::Vector3d>(),
                    t[5].cast<Sophus::SE3d>());
              }))
      .def("__repr__", [](const ImuCalibration& self) { return fmt::to_string(self); });
}

inline void declareMagnetometerCalibration(py::module& m) {
  py::class_<MagnetometerCalibration>(
      m,
      "MagnetometerCalibration",
      "A class representing a magnetometer calibration model, including only the intrinsics of"
      " the magnetometer.")
      .def(py::init<const std::string&, const Eigen::Matrix3d&, const Eigen::Vector3d&>())
      .def("get_label", &MagnetometerCalibration::getLabel)
      .def(
          "raw_to_rectified",
          &MagnetometerCalibration::rawToRectified,
          py::arg("raw"),
          "convert from mag sensor readout to actual magnetic field, "
          "rectified = rectificationMatrix.inv() * (raw - bias).")
      .def(
          "rectified_to_raw",
          &MagnetometerCalibration::rectifiedToRaw,
          py::arg("rectified"),
          "simulate mag sensor readout from actual magnetic field"
          " raw = rectificationMatrix * rectified + bias.")
      .def("get_model", &MagnetometerCalibration::getModel, "Get magnetometer intrinsics model.")
      .def(
          py::pickle(
              [](const MagnetometerCalibration& self) { // __getstate__
                return py::make_tuple(
                    self.getLabel(), self.getModel().getRectification(), self.getModel().getBias());
              },
              [](py::tuple t) { // __setstate__
                if (t.size() != 3) {
                  throw std::runtime_error("Invalid state!");
                }

                return MagnetometerCalibration(
                    t[0].cast<std::string>(),
                    t[1].cast<Eigen::Matrix3d>(),
                    t[2].cast<Eigen::Vector3d>());
              }));
}

inline void declareBarometerCalibration(py::module& m) {
  py::class_<BarometerCalibration>(m, "BarometerCalibration")
      .def(py::init<const std::string&, double, double>())
      .def("get_label", &BarometerCalibration::getLabel)
      .def("raw_to_rectified", &BarometerCalibration::rawToRectified, py::arg("raw"))
      .def("rectified_to_raw", &BarometerCalibration::rectifiedToRaw, py::arg("rectified"))
      .def("get_slope", &BarometerCalibration::getSlope)
      .def("get_offset_pa", &BarometerCalibration::getOffsetPa)
      .def(
          py::pickle(
              [](const BarometerCalibration& self) { // __getstate__
                return py::make_tuple(self.getLabel(), self.getSlope(), self.getOffsetPa());
              },
              [](py::tuple t) { // __setstate__
                if (t.size() != 3) {
                  throw std::runtime_error("Invalid state!");
                }

                return BarometerCalibration(
                    t[0].cast<std::string>(), t[1].cast<double>(), t[2].cast<double>());
              }));
}

inline void declareMicrophoneCalibration(py::module& m) {
  py::class_<MicrophoneCalibration>(m, "MicrophoneCalibration")
      .def(py::init<>())
      .def(py::init<const std::string&, double>())
      .def("get_label", &MicrophoneCalibration::getLabel)
      .def("raw_to_rectified", &MicrophoneCalibration::rawToRectified, py::arg("raw"))
      .def("rectified_to_raw", &MicrophoneCalibration::rectifiedToRaw, py::arg("rectified"))
      .def("get_d_sensitivity_1k_dbv", &MicrophoneCalibration::getDSensitivity1KDbv)
      .def(
          py::pickle(
              [](const MicrophoneCalibration& calib) {
                return py::make_tuple(calib.getLabel(), calib.getDSensitivity1KDbv());
              },
              [](py::tuple t) {
                if (t.size() != 2) {
                  throw std::runtime_error("Invalid state!");
                }

                return MicrophoneCalibration(t[0].cast<std::string>(), t[1].cast<double>());
              }));
}

inline void declareSensorCalibration(py::module& m) {
  declareCameraCalibration(m);
  declareLinearRectificationModel(m);
  declareImuCalibration(m);
  declareMagnetometerCalibration(m);
  declareBarometerCalibration(m);
  declareMicrophoneCalibration(m);

  py::enum_<SensorCalibrationType>(m, "SensorCalibrationType")
      .value("NOT_VALID", SensorCalibrationType::NotValid)
      .value("CAMERA_CALIBRATION", SensorCalibrationType::CameraCalibration)
      .value("IMU_CALIBRATION", SensorCalibrationType::ImuCalibration)
      .value("MAGNETOMETER_CALIBRATION", SensorCalibrationType::MagnetometerCalibration)
      .value("BAROMETER_CALIBRATION", SensorCalibrationType::BarometerCalibration)
      .value("MICROPHONE_CALIBRATION", SensorCalibrationType::MicrophoneCalibration)
      .value("ARIA_ET_CALIBRATION", SensorCalibrationType::AriaEtCalibration)
      .value("ARIA_MIC_CALIBRATION", SensorCalibrationType::AriaMicCalibration)
      .export_values()
      .export_values()
      .def(
          py::pickle(
              [](const SensorCalibrationType& type) { return static_cast<int>(type); },
              [](int state) { return static_cast<SensorCalibrationType>(state); }));

  py::class_<SensorCalibration>(
      m,
      "SensorCalibration",
      "An adaptor class to access an arbitrary sensor's calibration, which is a python `enum` of"
      " {CameraCalibration, ImuCalibration, MagnetometerCalibration, BarometerCalibration,"
      " MicrophoneCalibration, AriaEtCalibration, AriaMicCalibration}")
      .def(py::init<>())
      .def(py::init<const SensorCalibration::SensorCalibrationVariant&>())
      .def(
          "camera_calibration",
          &SensorCalibration::cameraCalibration,
          "Try to get the SensorCalibration as a CameraCalibration. Will throw if sensor type does not match.")
      .def(
          "imu_calibration",
          &SensorCalibration::imuCalibration,
          "Try to get the SensorCalibration as a ImuCalibration. Will throw if sensor type does not match.")
      .def(
          "magnetometer_calibration",
          &SensorCalibration::magnetometerCalibration,
          "Try to get the SensorCalibration as a MagnetometerCalibration. Will throw if sensor type does not match.")
      .def(
          "barometer_calibration",
          &SensorCalibration::barometerCalibration,
          "Try to get the SensorCalibration as a BarometerCalibration. Will throw if sensor type does not match.")
      .def(
          "microphone_calibration",
          &SensorCalibration::microphoneCalibration,
          "Try to get the SensorCalibration as a MicrophoneCalibration. Will throw if sensor type does not match.")
      .def(
          "aria_et_calibration",
          &SensorCalibration::ariaEtCalibration,
          "Try to get the SensorCalibration as a AriaEtCalibration. Will throw if sensor type does not match.")
      .def(
          "aria_mic_calibration",
          &SensorCalibration::ariaMicCalibration,
          "Try to get the SensorCalibration as a AriaMicCalibration. Will throw if sensor type does not match.")
      .def(
          "sensor_calibration_type",
          &SensorCalibration::sensorCalibrationType,
          "get the type of this sensor calibration as an enum.")
      .def(
          py::pickle(
              [](const SensorCalibration& calib) {
                switch (calib.sensorCalibrationType()) {
                  case SensorCalibrationType::CameraCalibration:
                    return py::make_tuple(
                        SensorCalibrationType::CameraCalibration, calib.cameraCalibration());
                  case SensorCalibrationType::ImuCalibration:
                    return py::make_tuple(
                        SensorCalibrationType::ImuCalibration, calib.imuCalibration());
                  case SensorCalibrationType::MagnetometerCalibration:
                    return py::make_tuple(
                        SensorCalibrationType::MagnetometerCalibration,
                        calib.magnetometerCalibration());
                  case SensorCalibrationType::BarometerCalibration:
                    return py::make_tuple(
                        SensorCalibrationType::BarometerCalibration, calib.barometerCalibration());
                  case SensorCalibrationType::MicrophoneCalibration:
                    return py::make_tuple(
                        SensorCalibrationType::MicrophoneCalibration,
                        calib.microphoneCalibration());
                  case SensorCalibrationType::AriaEtCalibration:
                    return py::make_tuple(
                        SensorCalibrationType::AriaEtCalibration, calib.ariaEtCalibration());
                  case SensorCalibrationType::AriaMicCalibration:
                    return py::make_tuple(
                        SensorCalibrationType::AriaMicCalibration, calib.ariaMicCalibration());
                  default:
                    throw std::runtime_error("Unsupported calibration type");
                }
              },
              [](py::tuple t) {
                if (t.size() != 2) {
                  throw std::runtime_error("Invalid state!");
                }

                auto type = t[0].cast<SensorCalibrationType>();
                switch (type) {
                  case SensorCalibrationType::CameraCalibration:
                    return SensorCalibration(t[1].cast<CameraCalibration>());
                  case SensorCalibrationType::ImuCalibration:
                    return SensorCalibration(t[1].cast<ImuCalibration>());
                  case SensorCalibrationType::MagnetometerCalibration:
                    return SensorCalibration(t[1].cast<MagnetometerCalibration>());
                  case SensorCalibrationType::BarometerCalibration:
                    return SensorCalibration(t[1].cast<BarometerCalibration>());
                  case SensorCalibrationType::MicrophoneCalibration:
                    return SensorCalibration(t[1].cast<MicrophoneCalibration>());
                  case SensorCalibrationType::AriaEtCalibration:
                    return SensorCalibration(t[1].cast<AriaEtCalibration>());
                  case SensorCalibrationType::AriaMicCalibration:
                    return SensorCalibration(t[1].cast<AriaMicCalibration>());
                  default:
                    throw std::runtime_error("Unsupported calibration type");
                }
              }));
}

inline void declareDeviceCalibration(py::module& m) {
  py::class_<DeviceCadExtrinsics>(
      m, "DeviceCadExtrinsics", "This class retrieves fixed CAD extrinsics values for Aria Device")
      .def(py::init<>())
      .def(
          py::init<const DeviceVersion&, const std::string&, const std::string&>(),
          "Construct for Cad extrinsics based on device version, device sub type, and origin label, where the label of the origin (`Device` coordinate frame) sensor,"
          "e.g. camera-slam-left")
      .def(
          py::init<const std::string&, const std::string&>(),
          "[Deprecated! Please use: DeviceCadExtrinsics(deviceVersion, deviceTypeType, deviceLabel)] Construct for Cad extrinsics for Aria Gen1 only. Input: device sub type, and origin label, where the label of the origin (`Device` coordinate frame) sensor,"
          "e.g. camera-slam-left")
      .def(
          py::pickle(
              [](const DeviceCadExtrinsics& cad) {
                return py::make_tuple(
                    cad.getDeviceVersion(), cad.getDeviceSubType(), cad.getOriginSensorLabel());
              },
              [](py::tuple t) {
                if (t.size() != 3) {
                  throw std::runtime_error("Invalid state!");
                }

                return DeviceCadExtrinsics(
                    t[0].cast<DeviceVersion>(), t[1].cast<std::string>(), t[2].cast<std::string>());
              }));

  py::class_<DeviceCalibration>(
      m,
      "DeviceCalibration",
      "A class to store and access calibration information of a device,"
      " including: camera, imu, magnetometer, barometer, and microphones.")
      .def(
          py::init<
              const std::map<std::string, CameraCalibration>&,
              const std::map<std::string, ImuCalibration>&,
              const std::map<std::string, MagnetometerCalibration>&,
              const std::map<std::string, BarometerCalibration>&,
              const std::map<std::string, MicrophoneCalibration>&,
              const DeviceCadExtrinsics&,
              const std::string&,
              const std::string&,
              const DeviceVersion&>(),
          R"pbdoc(Constructor that composes a collection of sensor calibrations into a DeviceCalibration"
            " @param camera_calibs: map of <label, CameraCalibration>"
            " @param imu_calibs: map of <label, ImuCalibration>"
            " @param magnetometer_calibs: map of <label, MagnetometerCalibration>"
            " @param barometer_calibs: map of <label, BarometerCalibration>"
            " @param microphone_calibs: map of <label, MicrophoneCalibration>"
            " @param device_cad_extrinsics: a struct representing the CAD extrinsics info of the device sensors."
            " @param device_subtype: the subtype of the device. For Aria, this would be 'DVT-S' or 'DVT-L'."
            " @param origin_label: the label identifying the origin of the calibration extrinsics, which needs"
            " to be a sensor within this device. This is basically the 'Device' frame in `T_Device_Sensor`."
            " @param device_version: the version of the device, {Gen1(default), Gen2})pbdoc",
          py::arg("camera_calibs") = std::map<std::string, CameraCalibration>(),
          py::arg("imu_calibs") = std::map<std::string, ImuCalibration>(),
          py::arg("magnetometer_calibs") = std::map<std::string, MagnetometerCalibration>(),
          py::arg("barometer_calibs") = std::map<std::string, BarometerCalibration>(),
          py::arg("microphone_calibs") = std::map<std::string, MicrophoneCalibration>(),
          py::arg("device_cad_extrinsics") = DeviceCadExtrinsics(),
          py::arg("device_subtype") = std::string(),
          py::arg("origin_label") = std::string(),
          py::arg("device_version") = DeviceVersion::Gen1)
      .def(
          "get_all_labels",
          &DeviceCalibration::getAllLabels,
          "returns all labels for all the sensors.")
      .def(
          "get_camera_labels",
          &DeviceCalibration::getCameraLabels,
          "returns all labels for cameras.")
      .def("get_imu_labels", &DeviceCalibration::getImuLabels, "returns all labels for imus.")
      .def(
          "get_magnetometer_labels",
          &DeviceCalibration::getMagnetometerLabels,
          "returns all labels for magnetometers.")
      .def(
          "get_barometer_labels",
          &DeviceCalibration::getBarometerLabels,
          "returns all labels for barometers.")
      .def(
          "get_audio_labels",
          &DeviceCalibration::getAudioLabels,
          "returns all labels for calibrated audio sensors, including microphones and speakers.")
      .def(
          "get_microphone_labels",
          &DeviceCalibration::getMicrophoneLabels,
          "returns all labels for calibrated microphones.")
      .def(
          "get_speaker_labels",
          &DeviceCalibration::getSpeakerLabels,
          "returns all labels for calibrated speakers.")
      .def(
          "get_sensor_calib",
          &DeviceCalibration::getSensorCalib,
          py::arg("label"),
          "returns a sensor calibration by its label. Will return None if label does not exist"
          " in device calibration.")
      .def(
          "get_camera_calib",
          &DeviceCalibration::getCameraCalib,
          py::arg("label"),
          "returns a camera calibration by its label. Will return None if label does not exist"
          " in device calibration.")
      .def(
          "get_imu_calib",
          &DeviceCalibration::getImuCalib,
          py::arg("label"),
          "returns a imu calibration by its label. Will return None if label does not exist"
          " in device calibration.")
      .def(
          "get_magnetometer_calib",
          &DeviceCalibration::getMagnetometerCalib,
          py::arg("label"),
          "returns a magnetometer calibration by its label. Will return None if label does not exist"
          " in device calibration.")
      .def(
          "get_barometer_calib",
          &DeviceCalibration::getBarometerCalib,
          py::arg("label"),
          "returns a barometer calibration by its label. Will return None if label does not exist"
          " in device calibration.")
      .def(
          "get_microphone_calib",
          &DeviceCalibration::getMicrophoneCalib,
          py::arg("label"),
          "returns a microphone calibration by its label. Will return None if label does not exist"
          " in device calibration.")
      .def(
          "get_aria_et_camera_calib",
          &DeviceCalibration::getAriaEtCameraCalib,
          "returns an array-of-2 of CameraCalibration representing left and right ET camera calibrations for an Aria device."
          " Will return None if device is not Aria, or it does not contain the valid ET camera.")
      .def(
          "get_aria_microphone_calib",
          &DeviceCalibration::getAriaMicrophoneCalib,
          "returns an array-of-7 of mic calibration for an Aria device. "
          "Will return None if device is not Aria, or it does not contain the valid mic calibrations.")
      .def(
          "get_device_version",
          &DeviceCalibration::getDeviceVersion,
          "Get the version of device, e.g. Gen1 or Gen2.")
      .def(
          "get_device_subtype",
          &DeviceCalibration::getDeviceSubtype,
          "Get the subtype of device. For Aria, this is 'DVT-S' or 'DVT-L' to indicate the size of"
          " the Aria unit.")
      .def(
          "get_transform_device_cpf",
          &DeviceCalibration::getT_Device_Cpf,
          "returns relative pose between device frame (anchored to a particular sensor defined by"
          " `origin_label`) and CPF (central pupil frame), where CPF is a virtual coordinate frame defined"
          " in CAD model.")
      .def(
          "get_transform_device_sensor",
          &DeviceCalibration::getT_Device_Sensor,
          py::arg("label"),
          py::arg("get_cad_value") = false,
          "returns calibrated `T_Device_Sensor` given a label."
          " You can return the CAD extrinsics value by specifying `get_cad_value = True`.")
      .def(
          "get_transform_cpf_sensor",
          &DeviceCalibration::getT_Cpf_Sensor,
          py::arg("label"),
          py::arg("get_cad_value") = false,
          "returns calibrated sensor extrinsics in CPF frame given a label."
          " You can return the CAD extrinsics value by specifying `get_cad_value = True`.")
      .def(
          "get_origin_label",
          &DeviceCalibration::getOriginLabel,
          "obtain the definition of Origin (or Device in T_Device_Sensor).")
      .def(
          "set_devignetting_mask_folder_path",
          &DeviceCalibration::setDevignettingMaskFolderPath,
          py::arg("mask_folder_path"),
          "Set the devignetting mask folder path.")
      .def(
          "load_devignetting_mask",
          [](DeviceCalibration& self, const std::string& label) {
            projectaria::tools::image::ManagedImage3F32 matrix = self.loadDevignettingMask(label);
            return image::toPyArrayVariant(matrix);
          },
          py::arg("label"),
          "Load devignetting mask corresponding to the label and return as numpy array")
      .def(
          py::pickle(
              [](const DeviceCalibration& calib) {
                return py::make_tuple(
                    calib.getCameraCalibs(),
                    calib.getImuCalibs(),
                    calib.getMagnetometerCalibs(),
                    calib.getBarometerCalibs(),
                    calib.getMicrophoneCalibs(),
                    calib.getDeviceCadExtrinsics(),
                    calib.getDeviceSubtype(),
                    calib.getOriginLabel(),
                    calib.getDeviceVersion());
              },
              [](py::tuple t) {
                if (t.size() != 9) {
                  throw std::runtime_error("Invalid state!");
                }

                return DeviceCalibration(
                    t[0].cast<std::map<std::string, CameraCalibration>>(),
                    t[1].cast<std::map<std::string, ImuCalibration>>(),
                    t[2].cast<std::map<std::string, MagnetometerCalibration>>(),
                    t[3].cast<std::map<std::string, BarometerCalibration>>(),
                    t[4].cast<std::map<std::string, MicrophoneCalibration>>(),
                    t[5].cast<DeviceCadExtrinsics>(),
                    t[6].cast<std::string>(),
                    t[7].cast<std::string>(),
                    t[8].cast<DeviceVersion>());
              }));

  m.def(
      "device_calibration_from_json",
      [&](const std::string& filepath) {
        std::ifstream fin{filepath};
        if (!fin.good()) {
          throw std::invalid_argument(fmt::format("Could not open {}", filepath));
        }
        std::string jsonStr(
            (std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
        return deviceCalibrationFromJson(jsonStr);
      },
      "Load calibration from json.");

  m.def(
      "device_calibration_from_json_string",
      &deviceCalibrationFromJson,
      "Load calibration from json string.");

  m.def(
      "device_calibration_to_json_string",
      &deviceCalibrationToJson,
      "Export device calibration to json string");
}

template <typename T>
void declareDistortByCalibration(py::module& m) {
  m.def(
      "distort_by_calibration",
      [](py::array_t<T> arraySrc,
         const CameraCalibration& dstCalib,
         const CameraCalibration& srcCalib,
         const image::InterpolationMethod method) {
        py::buffer_info info = arraySrc.request();

        size_t imageWidth = arraySrc.shape()[1];
        size_t imageHeight = arraySrc.shape()[0];
        bool isRgb = arraySrc.ndim() == 3 && arraySrc.shape()[2] == 3;
        constexpr int MaxVal = image::DefaultImageValTraits<T>::maxValue;

        if (!isRgb) {
          image::Image<T, MaxVal> imageSrc((T*)info.ptr, imageWidth, imageHeight);
          return image::toPyArrayVariant(
              distortByCalibration(image::ImageVariant{imageSrc}, dstCalib, srcCalib, method));
        } else {
          if constexpr (std::is_same<T, uint8_t>::value) {
            image::Image3U8 imageSrc((Eigen::Vector3<T>*)info.ptr, imageWidth, imageHeight);
            return image::toPyArrayVariant(
                distortByCalibration(image::ImageVariant{imageSrc}, dstCalib, srcCalib, method));
          } else {
            throw std::runtime_error("Type is not uint8_t but has 3 channels.");
          }
        }
      },
      py::arg("arraySrc"),
      py::arg("dstCalib"),
      py::arg("srcCalib"),
      py::arg("method") = image::InterpolationMethod::Bilinear,
      "Distorts an input image to swap its underlying image distortion model.");

  m.def(
      "distort_by_calibration_and_apply_rotation",
      [](py::array_t<T> arraySrc,
         const CameraCalibration& dstCalib,
         const CameraCalibration& srcCalib,
         const Sophus::SO3d& so3_srcCalib_dstCalib,
         const image::InterpolationMethod method) {
        py::buffer_info info = arraySrc.request();

        size_t imageWidth = arraySrc.shape()[1];
        size_t imageHeight = arraySrc.shape()[0];
        bool isRgb = arraySrc.ndim() == 3 && arraySrc.shape()[2] == 3;
        constexpr int MaxVal = image::DefaultImageValTraits<T>::maxValue;

        if (!isRgb) {
          image::Image<T, MaxVal> imageSrc((T*)info.ptr, imageWidth, imageHeight);
          return image::toPyArrayVariant(distortByCalibrationAndApplyRotation(
              image::ImageVariant{imageSrc}, dstCalib, srcCalib, so3_srcCalib_dstCalib, method));
        } else {
          if constexpr (std::is_same<T, uint8_t>::value) {
            image::Image3U8 imageSrc((Eigen::Vector3<T>*)info.ptr, imageWidth, imageHeight);
            return image::toPyArrayVariant(distortByCalibrationAndApplyRotation(
                image::ImageVariant{imageSrc}, dstCalib, srcCalib, so3_srcCalib_dstCalib, method));
          } else {
            throw std::runtime_error("Type is not uint8_t but has 3 channels.");
          }
        }
      },
      py::arg("arraySrc"),
      py::arg("dstCalib"),
      py::arg("srcCalib"),
      py::arg("so3_srcCalib_dstCalib"),
      py::arg("method") = image::InterpolationMethod::Bilinear,
      "Distorts an input image to swap its underlying image distortion model, while applying a rotation to the camera ray. This can be used for stereo rectification.");

  m.def(
      "distort_depth_by_calibration",
      [](py::array_t<T> arraySrc,
         const CameraCalibration& dstCalib,
         const CameraCalibration& srcCalib) {
        py::buffer_info info = arraySrc.request();

        size_t imageWidth = arraySrc.shape()[1];
        size_t imageHeight = arraySrc.shape()[0];
        bool isThreeChannel = arraySrc.ndim() == 3 && arraySrc.shape()[2] == 3;
        constexpr int MaxVal = image::DefaultImageValTraits<T>::maxValue;

        if (!isThreeChannel) {
          image::Image<T, MaxVal> imageSrc((T*)info.ptr, imageWidth, imageHeight);
          return image::toPyArrayVariant(
              distortDepthByCalibration(image::ImageVariant{imageSrc}, dstCalib, srcCalib));
        } else {
          throw std::runtime_error("Depth image should not have 3 channels.");
        }
      },
      "Distorts an input depth image using InterpolationMethod::Bilinear to swap its underlying image distortion model.");

  m.def(
      "distort_label_by_calibration",
      [](py::array_t<T> arraySrc,
         const CameraCalibration& dstCalib,
         const CameraCalibration& srcCalib) {
        py::buffer_info info = arraySrc.request();

        size_t imageWidth = arraySrc.shape()[1];
        size_t imageHeight = arraySrc.shape()[0];
        bool isThreeChannel = arraySrc.ndim() == 3 && arraySrc.shape()[2] == 3;
        constexpr int MaxVal = image::DefaultImageValTraits<T>::maxValue;

        if (!isThreeChannel) {
          image::Image<T, MaxVal> imageSrc((T*)info.ptr, imageWidth, imageHeight);
          return image::toPyArrayVariant(
              distortLabelMaskByCalibration(image::ImageVariant{imageSrc}, dstCalib, srcCalib));
        } else if constexpr (std::is_same<T, uint8_t>::value) {
          image::Image3U8 imageSrc((Eigen::Vector3<T>*)info.ptr, imageWidth, imageHeight);
          return image::toPyArrayVariant(
              distortLabelMaskByCalibration(image::ImageVariant{imageSrc}, dstCalib, srcCalib));
        } else {
          throw std::runtime_error("Type is not uint8_t but has 3 channels.");
        }
      },
      "Distorts an input image label using InterpolationMethod::NearestNeighbor to swap its underlying image distortion model.");
}

inline void declareDistortByCalibrationAll(py::module& m) {
  declareDistortByCalibration<uint8_t>(m);
  declareDistortByCalibration<float>(m);
  declareDistortByCalibration<uint16_t>(m);
  declareDistortByCalibration<uint64_t>(m);
}

template <typename T>
inline void declareDevignetting(py::module& m) {
  m.def(
      "devignetting",
      [](py::array_t<T> srcImage, const py::array_t<float>& devignettingMask) {
        size_t srcImageWidth = srcImage.shape()[1];
        size_t srcImageHeight = srcImage.shape()[0];
        constexpr int MaxVal = image::DefaultImageValTraits<T>::maxValue;

        py::buffer_info maskBuf = devignettingMask.request();
        if (maskBuf.ndim != 3 || maskBuf.shape[2] != 3) {
          throw std::runtime_error(
              "Input array for devignetting mask must have shape (height, width, 3)");
        }
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            eigenMatrix(
                static_cast<float*>(maskBuf.ptr),
                maskBuf.shape[0],
                maskBuf.shape[1] * maskBuf.shape[2]);
        image::ManagedImage3F32 convertedDevignettingMask(maskBuf.shape[1], maskBuf.shape[0]);
        std::memcpy(
            convertedDevignettingMask.data(), eigenMatrix.data(), maskBuf.size * sizeof(float));

        bool isRgb = srcImage.ndim() == 3 && srcImage.shape()[2] == 3;
        if (isRgb) {
          if constexpr (std::is_same<T, uint8_t>::value) {
            image::Image3U8 imageSrc(
                (Eigen::Vector3<T>*)srcImage.request().ptr, srcImageWidth, srcImageHeight);
            return image::toPyArrayVariant(
                devignetting(image::ImageVariant{imageSrc}, convertedDevignettingMask));
          } else {
            throw std::runtime_error("Type is not uint8_t but has 3 channels.");
          }
        } else {
          image::Image<T, MaxVal> imageSrc(
              (T*)srcImage.request().ptr, srcImageWidth, srcImageHeight);
          return image::toPyArrayVariant(
              devignetting(image::ImageVariant{imageSrc}, convertedDevignettingMask));
        }
      },
      py::arg("src_image"),
      py::arg("devignetting_mask"),
      "Devignetting image with devignetting mask");
}

inline void declareDevignettingAll(py::module& m) {
  declareDevignetting<uint8_t>(m);
  declareDevignetting<uint16_t>(m);
  declareDevignetting<uint64_t>(m);
  declareDevignetting<float>(m);
}

template <typename T>
inline void declareColorCorrect(py::module& m) {
  m.def(
      "color_correct",
      [](py::array_t<T> srcImage, DeviceVersion deviceVersion = DeviceVersion::Gen1) {
        // set default device version to 1 (Gen1) to avoid breaking existing code
        py::buffer_info info = srcImage.request();
        size_t imageWidth = srcImage.shape()[1];
        size_t imageHeight = srcImage.shape()[0];

        bool isRgb = srcImage.ndim() == 3 && srcImage.shape()[2] == 3;
        if (isRgb) {
          if constexpr (std::is_same<T, uint8_t>::value) {
            image::Image3U8 imageSrc((Eigen::Vector3<T>*)info.ptr, imageWidth, imageHeight);
            return image::toPyArrayVariant(
                colorCorrect(image::ImageVariant{imageSrc}, deviceVersion));
          } else {
            throw std::runtime_error("Type is not uint8_t but has 3 channels.");
          }
        } else {
          throw std::runtime_error(
              "Invalid input image depth, an image with 3 channel is expected");
        }
      },
      py::arg("src_image"),
      py::arg("device_version"),
      "Correct color distorted image in old Aria recordings");
}

inline void declareColorCorrectAll(py::module& m) {
  declareColorCorrect<uint8_t>(m);
  declareColorCorrect<uint16_t>(m);
  declareColorCorrect<uint64_t>(m);
  declareColorCorrect<float>(m);
}

} // namespace

inline void exportDeviceCalibration(py::module& m) {
  // For submodule documentation, see: projectaria_tools/projectaria_tools/core/calibration.py
  declareDeviceVersion(m);
  declareSensorCalibration(m);
  declareDeviceCalibration(m);
  declareDistortByCalibrationAll(m);
  declareDevignettingAll(m);
  declareColorCorrectAll(m);
}
} // namespace projectaria::tools::calibration
