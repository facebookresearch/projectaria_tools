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

#include <optional>
#include <string>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <calibration/camera_projections/CameraProjection.h>

namespace projectaria::tools::calibration {

/**
 * @brief A class that provides APIs for camera calibration, including extrinsics, intrinsics, and
 * projection
 */
class CameraCalibration {
 public:
  /**
   * @brief Default constructor, creates an empty CameraCalibration instance.
   */
  CameraCalibration() = default;

  /**
   * @brief Default copy constructor.
   */
  CameraCalibration(const CameraCalibration&) = default;

  /**
   * @brief Default move constructor.
   */
  CameraCalibration(CameraCalibration&&) = default;

  /**
   * @brief Default copy assignment.
   */
  CameraCalibration& operator=(const CameraCalibration&) = default;

  /**
   * @brief Default move assignment.
   */
  CameraCalibration& operator=(CameraCalibration&&) = default;

  /**
   * @brief Constructor with a list of parameters for CameraCalibration.
   * @param label The label of the camera, e.g. "camera-slam-left".
   * @param projectionModelType The type of camera projection model, e.g. ModelType::Linear
   * @param projectionParams The projection parameters
   * @param T_Device_Camera The extrinsics of camera in Device frame.
   * @param imageWidth Width of camera image.
   * @param imageHeight Height of camera image.
   * @param maybeValidRadius [optional] radius of a circular mask that represents the valid area on
   * the camera's sensor plane. Pixels out of this circular region are considered invalid. Setting
   * this to nullopt means the entire sensor plane is valid.
   * @param maxSolidAngle an angle theta representing the FOV cone of the camera. Rays out of
   * [-theta, +theta] will be rejected during projection.
   * @param serialNumber The serial number of the camera
   * @param timeOffsetSecDeviceCamera time offset between device and camera. Online calibration may
   * estimate this value. correctedCaptureTime = captureTimestampNs - TimeOffsetSecDeviceCamera
   * @param readOutTimeSec Optional readout time setting from reading the first pixel to last pixel
   * which is useful to handle the rolling shutter effect.
   */
  CameraCalibration(
      const std::string& label,
      const CameraProjection::ModelType& projectionModelType,
      const Eigen::VectorXd& projectionParams,
      const Sophus::SE3d& T_Device_Camera,
      int imageWidth,
      int imageHeight,
      std::optional<double> maybeValidRadius,
      double maxSolidAngle,
      const std::string& serialNumber = "",
      double timeOffsetSecDeviceCamera = 0.0,
      const std::optional<double>& maybeReadOutTimeSec = {});

  std::string getLabel() const;
  std::string getSerialNumber() const;
  Sophus::SE3d getT_Device_Camera() const;
  Eigen::Vector2i getImageSize() const;
  double getMaxSolidAngle() const;
  std::optional<double> getValidRadius() const;
  double getTimeOffsetSecDeviceCamera() const;
  double& getTimeOffsetSecDeviceCameraMut();
  std::optional<double> getReadOutTimeSec() const;
  std::optional<double>& getReadOutTimeSecMut();
  /**
   * @brief Function to check whether a pixel is within the valid area of the sensor plane.
   */
  bool isVisible(const Eigen::Vector2d& cameraPixel) const;

  CameraProjection::ModelType modelName() const; // return KB3 or Fisheye624
  Eigen::Vector2d getPrincipalPoint() const; // return optical center
  Eigen::Vector2d getFocalLengths() const; // return focal length in x and y
  const Eigen::VectorXd& projectionParams() const; // return full calibration parameters
  Eigen::VectorXd& projectionParamsMut();
  int numParameters() const; // return number of parameters
  int numProjectionParameters() const; // return number of projection parameters
  int numDistortionParameters() const; // return number of distortion parameters

  /**
   * @brief Function to project a 3d point (in camera frame) to a 2d camera pixel location. In this
   * function, no check is performed.
   * @param pointInCamera 3d point in camera frame.
   * @param jacobianWrtPoint Optional, if not null, will store the Jacobian of the projection
   * with respect to the point in camera space.
   * @param jacobianWrtParams Optional, if not null, will store the Jacobian of the projection
   * with respect to the projection parameters.
   * @return 2d pixel location in image plane.
   */
  Eigen::Vector2d projectNoChecks(
      const Eigen::Vector3d& pointInCamera,
      Eigen::Ref<Eigen::Matrix<double, 2, 3>> jacobianWrtPoint = NullRef(),
      Eigen::Ref<Eigen::Matrix<double, 2, Eigen::Dynamic>> jacobianWrtParams = NullRef()) const;

  /**
   * @brief Function to project a 3d point (in camera frame) to a 2d camera pixel location, with a
   * number of validity checks to ensure the point is visible.
   * @param pointInCamera 3d point in camera frame.
   * @param jacobianWrtPoint Optional, if not null, will store the Jacobian of the projection
   * with respect to the point in camera space.
   * @param jacobianWrtParams Optional, if not null, will store the Jacobian of the projection
   * with respect to the projection parameters.
   * @return 2d pixel location in image plane. If any of the check failed, return std::nullopt.
   */
  std::optional<Eigen::Vector2d> project(
      const Eigen::Vector3d& pointInCamera,
      Eigen::Ref<Eigen::Matrix<double, 2, 3>> jacobianWrtPoint = NullRef(),
      Eigen::Ref<Eigen::Matrix<double, 2, Eigen::Dynamic>> jacobianWrtParams = NullRef()) const;

  /**
   * @brief Function to unproject a 2d pixel location to a 3d ray in camera frame. In this function,
   * no check is performed.
   * @param cameraPixel 2d pixel location in image plane.
   * @return 3d ray, in camera frame.
   */
  Eigen::Vector3d unprojectNoChecks(const Eigen::Vector2d& cameraPixel) const;

  /**
   * @brief Function to unproject a 2d pixel location to a 3d ray, in camera frame, with a number of
   * validity checks to ensure the unprojection is valid.
   * @param cameraPixel 2d pixel location in image plane.
   * @return 3d ray, in camera frame. If any of the check failed, return std::nullopt
   */
  std::optional<Eigen::Vector3d> unproject(const Eigen::Vector2d& cameraPixel) const;

  /**
   * @brief Obtain a new camera calibration after translation + scaling transform from the original
   * camera calibration. <br> transform is done in the order of (1) shift -> (2) scaling.
   * @param newResolution The resolution of the new camera calibration.
   * @param scale The scaling factor to apply in step (2).
   * @param originOffset The offset to apply in step (1). By default the value is {0, 0}.
   */
  CameraCalibration rescale(
      const Eigen::Vector2i& newResolution,
      double scale,
      const Eigen::Vector2d& originOffset = {0.0, 0.0}) const;

 private:
  std::string label_;
  CameraProjection projectionModel_;
  Sophus::SE3d T_Device_Camera_;
  int imageWidth_;
  int imageHeight_;

  // If the lens is Fisheye and does not cover the entire sensor, we are
  // storing here the radial valid image area.
  // Notes:
  //  - This circle radius is defined from the principal point.
  //  - If radius is < 0, it means you have full sensor data.
  std::optional<double> maybeValidRadius_;
  double maxSolidAngle_;
  std::string serialNumber_;
  double timeOffsetSecDeviceCamera_ = 0.0;

  // For rolling shutter camera, this could be useful to handling the rolling shutter effect.
  std::optional<double> maybeReadOutTimeSec_ = 0.0;
};

/**
 * @brief Function to create a simple Linear camera calibration object from some parameters.
 * @param imageWidth Width of the camera in pixels.
 * @param imageHeight Height of the camera in pixels.
 * @param focalLength Focal length of the camera in pixels.
 * @param label Label of the camera, Default value is empty string.
 * @param T_Device_Camera Pose of the Camera Calibration (Should be the same as the original
 * camera's pose before undistortion).
 * @param timeOffsetSecDeviceCamera time offset between device and camera. Online calibration may
 * estimate this value. correctedCaptureTime = captureTimestampNs - TimeOffsetSecDeviceCamera
 */
CameraCalibration getLinearCameraCalibration(
    int imageWidth,
    int imageHeight,
    double focalLength,
    const std::string& label = "",
    const Sophus::SE3d& T_Device_Camera = Sophus::SE3d{},
    double timeOffsetSecDeviceCamera = 0.0);
/**
 * @brief Function to create a simple Spherical camera calibration object from some parameters.
 * @param imageWidth Width of the camera in pixels.
 * @param imageHeight Height of the camera in pixels.
 * @param focalLength Focal length of the camera in pixels.
 * @param label Label of the camera, Default value is empty string.
 * @param T_Device_Camera Pose of the Camera Calibration (Should be the same as the original
 * camera's pose before undistortion).
 * @param timeOffsetSecDeviceCamera time offset between device and camera. Online calibration may
 * estimate this value. correctedCaptureTime = captureTimestampNs - TimeOffsetSecDeviceCamera
 */
CameraCalibration getSphericalCameraCalibration(
    int imageWidth,
    int imageHeight,
    double focalLength,
    const std::string& label = "",
    const Sophus::SE3d& T_Device_Camera = Sophus::SE3d{},
    double timeOffsetSecDeviceCamera = 0.0);

CameraCalibration rotateCameraCalibCW90Deg(const CameraCalibration& camCalib);
} // namespace projectaria::tools::calibration
