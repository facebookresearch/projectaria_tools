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

#include <cmath>
#include <type_traits>

#include <fmt/format.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>

#include <data_provider/data_types/Uuid.h>

namespace projectaria::tools::data_provider {

using FrontendSessionUid = Uuid<struct FrontendSessionTag>;
using FrameSetId = uint32_t;

/// The output status of VIO
enum class VioStatus : uint8_t {
  VALID = 0,
  FILTER_NOT_INITIALIZED,
  INVALID,
};

enum class TrackingQuality : uint8_t {
  UNKNOWN = 0,
  GOOD,
  BAD,
  UNRECOVERABLE,
};

// Visual tracking quality class
//
// This class is used to identify whether the "visual-only" part of the tracking is good.
// This flag is different from the VIO tracking quality, since VIO uses visual + inertial, and
// possibly other information for tracking, so it's likely when visual tracking is lost, VIO still
// tracks with IMU only (integration for seconds, and TLIO for longer). In this case "visual-only"
// tracking is bad, but VIO tracking is good.
enum class VisualTrackingQuality : uint8_t {
  UNKNOWN = 0,
  BAD,
  GOOD,
};

/********************  VIO online calibration data structures *************/
// Camera Ocal
template <typename Scalar>
struct ProjectionModelParameters {
  /// The type of projection model.
  std::string type;

  /// Projection intrinsics within the context of VIO.
  /// Usual order: fx,[fy], cx, cy, distortion parameters
  std::vector<Scalar> intrinsics;

  /// The readout time of the image, in sec. Zero for Global-shutter cameras
  Scalar readoutTimeSec = -1;

  /// Squared radius of the circle that contains valid pixels
  Scalar maxRadiusSquared = -1;

  // image size, pixels
  struct {
    int16_t width;
    int16_t height;
  } imageSize = {-1, -1};
};

// A helper function for unit conversion.
template <typename T>
constexpr auto degreesToRadians(const T& degree) {
  static_assert(
      std::is_floating_point<T>::value,
      "Unit conversion should only be used with floating-point types.");
  return degree * M_PI / T{180};
}

// Imu Ocal
template <typename Scalar>
struct ImuMeasurementModelParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // the scale of each of the gyro axes
  Eigen::Vector3<Scalar> gyroScaleVec;

  // the scale of each of the accel axes
  Eigen::Vector3<Scalar> accelScaleVec;

  // additive bias, m/sec^2
  Eigen::Vector3<Scalar> accelBiasMSec2;

  // additive bias, rad/sec
  Eigen::Vector3<Scalar> gyroBiasRadSec;

  // nonorthogonality of accel axes. Each row of the matrix must be unit norm
  Eigen::Matrix3<Scalar> accelNonorth;

  // nonorthogonality of gyro axes. Each row of the matrix must be unit norm
  Eigen::Matrix3<Scalar> gyroNonorth;

  // gyro g sensitivity (implied units are rad/sec per (m/sec^2))
  Eigen::Matrix3<Scalar> gyroGSensitivityRadSecPerMSec2;

  // The time offsets of the recorded timestamps. This means that if we record a measurement at t,
  // this actually corresponds to the omega/specific force at t-time_offset.
  // If we estimate the time offset wrt. a reference clock (e.g. camera timestamps), we reconstruct
  // the timestamps
  //   tReference = tAccel - dtReferenceAccel
  //   tAccel = tReference + dtReferenceAccel
  Scalar dtReferenceAccelSec;
  Scalar dtReferenceGyroSec;

  // IMU from BodyImu (i.e., the base device frame that we use for navigation) transform
  // IT IS ASSUMED THAT THE FIRST IMU IS THE BODY IMU
  Sophus::SE3<Scalar> T_Imu_BodyImu;

  // nominal sample period, seconds
  Scalar nominalSamplingPeriodSec;

  // saturation threshold for the accel, absolute value of the measurement along each axis, in
  // m/sec^2
  Scalar accelSaturationThresholdMSec2;
  // saturation threshold for the gyro, absolute value of the measurement along each axis, in
  // rad/sec
  Scalar gyroSaturationThresholdRadSec;

  ImuMeasurementModelParameters() {
    // set default values
    reset();
  }

  void reset() {
    // default values
    gyroNonorth = Eigen::Matrix3<Scalar>::Identity();
    accelNonorth = Eigen::Matrix3<Scalar>::Identity();

    accelBiasMSec2.setZero();
    gyroBiasRadSec.setZero();
    gyroGSensitivityRadSecPerMSec2.setZero();

    gyroScaleVec.setConstant(1.0);
    accelScaleVec.setConstant(1.0);

    // values for typical IMU with 1kHz sample rate
    dtReferenceAccelSec = Scalar(0.0);
    dtReferenceGyroSec = Scalar(0.0);

    nominalSamplingPeriodSec = Scalar(0.001);

    // We use a slightly lower value than 4g here.
    accelSaturationThresholdMSec2 = Scalar(38.0);
    // Default is 2k deg/sec. Use a slightly lower value here.
    gyroSaturationThresholdRadSec = degreesToRadians(Scalar(1900.));
  }

  template <typename NewScalar>
  std::conditional_t<
      std::is_same<Scalar, NewScalar>::value,
      const ImuMeasurementModelParameters<NewScalar>&,
      ImuMeasurementModelParameters<NewScalar>>
  cast() const {
    if constexpr (std::is_same<Scalar, NewScalar>::value) {
      return *this;
    } else {
      ImuMeasurementModelParameters<NewScalar> result;
      // Gyroscope scale / bias
      result.gyroScaleVec = gyroScaleVec.template cast<NewScalar>();
      result.gyroBiasRadSec = gyroBiasRadSec.template cast<NewScalar>();

      // Accelerometer scale / bias
      result.accelScaleVec = accelScaleVec.template cast<NewScalar>();
      result.accelBiasMSec2 = accelBiasMSec2.template cast<NewScalar>();

      result.accelNonorth = accelNonorth.template cast<NewScalar>();
      result.gyroNonorth = gyroNonorth.template cast<NewScalar>();
      result.gyroGSensitivityRadSecPerMSec2 =
          gyroGSensitivityRadSecPerMSec2.template cast<NewScalar>();

      result.dtReferenceAccelSec = static_cast<NewScalar>(dtReferenceAccelSec);
      result.dtReferenceGyroSec = static_cast<NewScalar>(dtReferenceGyroSec);

      // IMU from Rig transform
      result.T_Imu_BodyImu = T_Imu_BodyImu.template cast<NewScalar>();

      result.nominalSamplingPeriodSec = static_cast<NewScalar>(nominalSamplingPeriodSec);

      result.accelSaturationThresholdMSec2 = static_cast<NewScalar>(accelSaturationThresholdMSec2);
      result.gyroSaturationThresholdRadSec = static_cast<NewScalar>(gyroSaturationThresholdRadSec);

      return result;
    }
  }

  inline void getCompensatedImuMeasurement(
      const Scalar& uncompensatedGyroRadSecAverage,
      const Scalar& uncompensatedAccelMSec2Average,
      Eigen::Vector3<Scalar>& compensatedGyroRadSec,
      Eigen::Vector3<Scalar>& compensatedAccelMSec2) const {
    const Eigen::Matrix3<Scalar> accelScaleMat = getAccelScaleMat();
    const Eigen::Matrix3<Scalar> accelScaleInv = accelScaleMat.inverse();
    const Eigen::Matrix3<Scalar> gyroScaleInv = getGyroScaleMat().inverse();

    Eigen::Vector3<Scalar> gyroLinear = uncompensatedGyroRadSecAverage;

    Eigen::Vector3<Scalar> accelLinear = uncompensatedAccelMSec2Average;

    // compensated gyro first
    compensatedGyroRadSec.noalias() = gyroScaleInv * gyroLinear -
        gyroGSensitivityRadSecPerMSec2 * uncompensatedAccelMSec2Average - gyroBiasRadSec;

    // start with the uncompensated signal
    Eigen::Vector3<Scalar> tempAccel = accelLinear;

    compensatedAccelMSec2.noalias() = accelScaleInv * tempAccel - accelBiasMSec2;
  }

  // function computing the reference timestamp corresponding to an Imu measurement timestamp
  [[nodiscard]] int64_t referenceTimestampCorrespondingToImuTimestamp(int64_t imuTimestamp) const {
    return imuTimestamp - dtReferenceGyroSec;
  }

  // function computing the imu timestamp corresponding to the reference timestamp
  // referenceTimestampUs. Identical to the gyro timestamp.
  [[nodiscard]] int64_t imuTimestampCorrespondingToReferenceTimestamp(
      int64_t referenceTimestamp) const {
    return referenceTimestamp + dtReferenceGyroSec;
  }

  // function computing the accelerometer timestamp corresponding to the reference timestamp
  // referenceTimestampUs
  [[nodiscard]] int64_t accelTimestampCorrespondingToReferenceTimestamp(
      int64_t referenceTimestamp) const {
    return referenceTimestamp + dtReferenceAccelSec;
  }

  void setScaleMatrices(
      const Eigen::Matrix3<Scalar>& gyroScaleMat,
      const Eigen::Matrix3<Scalar>& accelScaleMat) {
    for (int i = 0; i < 3; i++) {
      gyroScaleVec(i) = gyroScaleMat.row(i).norm();
      gyroNonorth.row(i) = gyroScaleMat.row(i).normalized();

      accelScaleVec(i) = accelScaleMat.row(i).norm();
      accelNonorth.row(i) = accelScaleMat.row(i).normalized();
    }

    // make sure the accel nonorthogonality is upper triangular
    XR_DEV_CHECK_EQ(accelNonorth(1, 0), Scalar(0.0));
    XR_DEV_CHECK_EQ(accelNonorth(2, 0), Scalar(0.0));
    XR_DEV_CHECK_EQ(accelNonorth(2, 1), Scalar(0.0));
  }

  [[nodiscard]] Eigen::Matrix3<Scalar> getAccelScaleMat() const {
    return accelScaleVec.asDiagonal() * accelNonorth;
  }

  [[nodiscard]] Eigen::Matrix3<Scalar> getGyroScaleMat() const {
    return gyroScaleVec.asDiagonal() * gyroNonorth;
  }
  void check() const {
    XR_DEV_CHECK_GT(nominalSamplingPeriodSec, 0);
    XR_DEV_CHECK_LT(
        nominalSamplingPeriodSec,
        1); // we assume anything longer than 1 sec sample period is wrong...
    XR_DEV_CHECK_GT(accelSaturationThresholdMSec2, 0);
    XR_DEV_CHECK_GT(gyroSaturationThresholdRadSec, 0);
  }

  void print() const {
    XR_LOGCI(
        "ImuMeasurementModelParameters", "nominalSamplingPeriodSec: {}", nominalSamplingPeriodSec);

    XR_LOGCI(
        "ImuMeasurementModelParameters",
        "accelSaturationThresholdMSec2: {}",
        accelSaturationThresholdMSec2);
    XR_LOGCI(
        "ImuMeasurementModelParameters",
        "gyroSaturationThresholdRadSec: {}",
        gyroSaturationThresholdRadSec);

    XR_LOGCI("ImuMeasurementModelParameters", "dtReferenceAccelSec:  {}", dtReferenceAccelSec);
    XR_LOGCI("ImuMeasurementModelParameters", "dtReferenceGyroSec:  {}", dtReferenceGyroSec);

    XR_LOGCI("ImuMeasurementModelParameters", "gyroBiasRadSec:  [{}]", gyroBiasRadSec.transpose());
    XR_LOGCI("ImuMeasurementModelParameters", "accelBiasMSec2:  [{}]", accelBiasMSec2.transpose());

    XR_LOGCI("ImuMeasurementModelParameters", "gyroScaleVec:  [{}]", gyroScaleVec.transpose());
    XR_LOGCI("ImuMeasurementModelParameters", "accelScaleVec:  [{}]", accelScaleVec.transpose());

    XR_LOGCI("ImuMeasurementModelParameters", "gyroNonorth: {}", gyroNonorth);
    XR_LOGCI("ImuMeasurementModelParameters", "accelNonorthv: {}", accelNonorth);

    XR_LOGCI("ImuMeasurementModelParameters", "Gyro scale matrix: {}", getGyroScaleMat());
    XR_LOGCI("ImuMeasurementModelParameters", "Accel scale matrix: {}", getAccelScaleMat());

    XR_LOGCI(
        "ImuMeasurementModelParameters",
        "gyroGSensitivityRadSecPerMSec2 {}",
        gyroGSensitivityRadSecPerMSec2);

    XR_LOGCI(
        "ImuMeasurementModelParameters",
        "T_Imu_BodyImu.rotationMatrix:  {}",
        T_Imu_BodyImu.rotationMatrix());
    XR_LOGCI(
        "ImuMeasurementModelParameters",
        "T_Imu_BodyImu.translation:  [{}]",
        T_Imu_BodyImu.translation().transpose());
  }

  void checkAndPrint() const {
    check();
    print();
  }
};

struct OnlineCalibState {
  // camera intrinsic, camera model parameters. The vector size is the number of cameras in the
  // system.
  std::vector<ProjectionModelParameters<float>> camParameters;
  // T_camera_imu extrinsics
  std::vector<Sophus::SE3f> T_Cam_BodyImu;
  // time offset between the reference and the camera in nanosecond
  std::vector<int64_t> dt_Ref_Cam;
  // IMU state. The vector size is the number of IMUs in the system.
  std::vector<ImuMeasurementModelParameters<float>> imuModelParameters;

  OnlineCalibState() = default;

  OnlineCalibState(const OnlineCalibState& other) = default;

  OnlineCalibState& operator=(const OnlineCalibState& other) {
    if (this != &other) {
      camParameters = other.camParameters;
      T_Cam_BodyImu = other.T_Cam_BodyImu;
      dt_Ref_Cam = other.dt_Ref_Cam;
      imuModelParameters = other.imuModelParameters;
    }
    return *this;
  }
  void reset() {
    camParameters.clear();
    T_Cam_BodyImu.clear();
    dt_Ref_Cam.clear();
    imuModelParameters.clear();
  }

  [[nodiscard]] int numCameras() const {
    // Consistency check.
    if (camParameters.size() != T_Cam_BodyImu.size()) {
      throw std::runtime_error("online calib camera size not matching T_Cam_BodyImu size");
    }
    if (camParameters.size() != dt_Ref_Cam.size()) {
      throw std::runtime_error("online calib camera size not matching dt_Ref_Cam size");
    }
    return camParameters.size();
  }
};

// utils to convert status/tracking quality enum to string for log
std::string toString(VioStatus status);
std::string toString(TrackingQuality trackingQuality);
std::string toString(VisualTrackingQuality trackingQuality);

} // namespace projectaria::tools::data_provider
