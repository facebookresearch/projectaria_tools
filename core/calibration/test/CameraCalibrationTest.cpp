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

#include <calibration/CameraCalibration.h>
#include <calibration/camera_projections/FisheyeRadTanThinPrism.h>
#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <stdexcept>

using namespace projectaria::tools::calibration;

namespace {

constexpr double kTolerance = 1e-12;

Eigen::VectorXd makeLinearParams() {
  Eigen::VectorXd params(LinearProjection::kNumParams);
  params << 100.0, 120.0, 4.0, 3.0;
  return params;
}

CameraCalibration makeLinearCalibration(
    const std::optional<double> validRadius = std::nullopt,
    const double maxSolidAngle = M_PI,
    const Sophus::SE3d& T_Device_Camera = Sophus::SE3d{}) {
  return CameraCalibration{
      "camera",
      CameraProjection::ModelType::Linear,
      makeLinearParams(),
      T_Device_Camera,
      10,
      8,
      validRadius,
      maxSolidAngle,
      "serial",
      0.25,
      0.01};
}

Eigen::VectorXd makeFisheye624Params() {
  Eigen::VectorXd params(Fisheye624::kNumParams);
  params << 10.0, 4.0, 3.0, 0.01, -0.02, 0.03, -0.04, 0.05, -0.06, 0.07, -0.08, 0.09, -0.10, 0.11,
      -0.12;
  return params;
}

void expectVectorNear(
    const Eigen::VectorXd& actual,
    const Eigen::VectorXd& expected,
    const double tolerance = kTolerance) {
  ASSERT_EQ(actual.size(), expected.size());
  EXPECT_TRUE(actual.isApprox(expected, tolerance))
      << "actual: " << actual.transpose() << "\nexpected: " << expected.transpose();
}

} // namespace

TEST(CameraCalibrationTest, ConstructorPreservesMetadataAndMutableState) {
  const Eigen::Vector3d translation{1.0, 2.0, 3.0};
  const Sophus::SE3d T_Device_Camera{Eigen::Matrix3d::Identity(), translation};
  CameraCalibration camera = makeLinearCalibration(5.5, 1.2, T_Device_Camera);

  EXPECT_EQ(camera.getLabel(), "camera");
  EXPECT_EQ(camera.getSerialNumber(), "serial");
  EXPECT_EQ(camera.getImageSize(), Eigen::Vector2i(10, 8));
  EXPECT_DOUBLE_EQ(camera.getMaxSolidAngle(), 1.2);
  EXPECT_EQ(camera.getValidRadius(), std::optional<double>(5.5));
  EXPECT_DOUBLE_EQ(camera.getTimeOffsetSecDeviceCamera(), 0.25);
  EXPECT_EQ(camera.getReadOutTimeSec(), std::optional<double>(0.01));
  EXPECT_TRUE(camera.getT_Device_Camera().matrix().isApprox(T_Device_Camera.matrix()));

  camera.getTimeOffsetSecDeviceCameraMut() = -0.5;
  camera.getReadOutTimeSecMut() = 0.02;
  camera.getT_Device_CameraMut().translation() = Eigen::Vector3d{-1.0, -2.0, -3.0};

  EXPECT_DOUBLE_EQ(camera.getTimeOffsetSecDeviceCamera(), -0.5);
  EXPECT_EQ(camera.getReadOutTimeSec(), std::optional<double>(0.02));
  EXPECT_TRUE(
      camera.getT_Device_Camera().translation().isApprox(Eigen::Vector3d(-1.0, -2.0, -3.0)));
}

TEST(CameraCalibrationTest, ProjectionModelAccessorsExposeIntrinsics) {
  CameraCalibration camera = makeLinearCalibration();

  EXPECT_EQ(camera.modelName(), CameraProjection::ModelType::Linear);
  expectVectorNear(camera.projectionParams(), makeLinearParams());
  EXPECT_EQ(camera.numParameters(), LinearProjection::kNumParams);
  EXPECT_EQ(camera.numProjectionParameters(), LinearProjection::kNumParams);
  EXPECT_EQ(camera.numDistortionParameters(), 0);
  EXPECT_EQ(camera.getFocalLengths(), Eigen::Vector2d(100.0, 120.0));
  EXPECT_EQ(camera.getPrincipalPoint(), Eigen::Vector2d(4.0, 3.0));

  camera.projectionParamsMut()[LinearProjection::kFocalXIdx] = 150.0;
  EXPECT_EQ(camera.getFocalLengths(), Eigen::Vector2d(150.0, 120.0));
}

TEST(CameraCalibrationTest, IsVisibleUsesContinuousImageBoundsAndOptionalMask) {
  const CameraCalibration fullSensorCamera = makeLinearCalibration();

  EXPECT_TRUE(fullSensorCamera.isVisible(Eigen::Vector2d(-0.5, -0.5)));
  EXPECT_TRUE(fullSensorCamera.isVisible(Eigen::Vector2d(9.5, 7.5)));
  EXPECT_FALSE(fullSensorCamera.isVisible(Eigen::Vector2d(-0.51, 3.0)));
  EXPECT_FALSE(fullSensorCamera.isVisible(Eigen::Vector2d(4.0, 7.51)));

  const CameraCalibration maskedCamera = makeLinearCalibration(2.0);
  EXPECT_TRUE(maskedCamera.isVisible(Eigen::Vector2d(6.0, 3.0)));
  EXPECT_FALSE(maskedCamera.isVisible(Eigen::Vector2d(6.01, 3.0)));
}

TEST(CameraCalibrationTest, ProjectReturnsPixelOnlyWhenPointPassesConeAndImageChecks) {
  const CameraCalibration camera = makeLinearCalibration(3.0, 0.75);

  const std::optional<Eigen::Vector2d> visiblePixel =
      camera.project(Eigen::Vector3d(0.01, 0.0, 1.0));
  ASSERT_TRUE(visiblePixel.has_value());
  EXPECT_TRUE(visiblePixel->isApprox(Eigen::Vector2d(5.0, 3.0), kTolerance));

  EXPECT_FALSE(camera.project(Eigen::Vector3d(1.0, 0.0, 1.0)).has_value());
  EXPECT_FALSE(camera.project(Eigen::Vector3d(0.0, 0.05, 1.0)).has_value());
  EXPECT_FALSE(camera.project(Eigen::Vector3d(0.1, 0.0, -1.0)).has_value());
}

TEST(CameraCalibrationTest, ProjectNoChecksComputesJacobiansForLinearModel) {
  const CameraCalibration camera = makeLinearCalibration();
  Eigen::Matrix<double, 2, 3> jacobianWrtPoint;
  Eigen::Matrix<double, 2, Eigen::Dynamic> jacobianWrtParams(2, camera.numParameters());

  const Eigen::Vector2d pixel =
      camera.projectNoChecks(Eigen::Vector3d(0.2, 0.2, 2.0), jacobianWrtPoint, jacobianWrtParams);

  EXPECT_TRUE(pixel.isApprox(Eigen::Vector2d(14.0, 15.0), kTolerance));
  Eigen::Matrix<double, 2, 3> expectedPointJacobian;
  expectedPointJacobian << 50.0, 0.0, -5.0, 0.0, 60.0, -6.0;
  EXPECT_TRUE(jacobianWrtPoint.isApprox(expectedPointJacobian, kTolerance));
  Eigen::Matrix<double, 2, 4> expectedParamsJacobian;
  expectedParamsJacobian << 0.1, 0.0, 1.0, 0.0, 0.0, 0.1, 0.0, 1.0;
  EXPECT_TRUE(jacobianWrtParams.isApprox(expectedParamsJacobian, kTolerance));
}

TEST(CameraCalibrationTest, UnprojectChecksVisibilityBeforeReturningRay) {
  const CameraCalibration camera = makeLinearCalibration(2.0);

  const std::optional<Eigen::Vector3d> ray = camera.unproject(Eigen::Vector2d(5.0, 3.0));
  ASSERT_TRUE(ray.has_value());
  EXPECT_TRUE(ray->isApprox(Eigen::Vector3d(0.01, 0.0, 1.0), kTolerance));

  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(6.01, 3.0)).has_value());
  EXPECT_FALSE(camera.unproject(Eigen::Vector2d(10.0, 3.0)).has_value());
  EXPECT_TRUE(camera.unprojectNoChecks(Eigen::Vector2d(10.0, 3.0))
                  .isApprox(Eigen::Vector3d(0.06, 0.0, 1.0), kTolerance));
}

TEST(CameraCalibrationTest, RescaleTransformsIntrinsicsMaskAndResolutionWithoutMutatingOriginal) {
  const CameraCalibration camera = makeLinearCalibration(6.0);

  const CameraCalibration rescaled =
      camera.rescale(Eigen::Vector2i(5, 4), 0.5, Eigen::Vector2d(1.0, 2.0));

  Eigen::VectorXd expectedParams(LinearProjection::kNumParams);
  expectedParams << 50.0, 60.0, 1.25, 0.25;
  expectVectorNear(rescaled.projectionParams(), expectedParams);
  EXPECT_EQ(rescaled.getImageSize(), Eigen::Vector2i(5, 4));
  EXPECT_EQ(rescaled.getValidRadius(), std::optional<double>(3.0));

  expectVectorNear(camera.projectionParams(), makeLinearParams());
  EXPECT_EQ(camera.getImageSize(), Eigen::Vector2i(10, 8));
  EXPECT_EQ(camera.getValidRadius(), std::optional<double>(6.0));
}

TEST(CameraCalibrationTest, FactoryHelpersCreateCenteredLinearAndSphericalCameras) {
  const Sophus::SE3d T_Device_Camera{
      Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
      Eigen::Vector3d(0.1, 0.2, 0.3)};

  const CameraCalibration linearCamera =
      getLinearCameraCalibration(640, 480, 320.0, "linear", T_Device_Camera, 0.123);
  const CameraCalibration sphericalCamera =
      getSphericalCameraCalibration(640, 480, 320.0, "spherical", T_Device_Camera, -0.25);

  Eigen::VectorXd expectedParams(LinearProjection::kNumParams);
  expectedParams << 320.0, 320.0, 319.5, 239.5;
  EXPECT_EQ(linearCamera.modelName(), CameraProjection::ModelType::Linear);
  EXPECT_EQ(linearCamera.getSerialNumber(), "LinearCameraCalibration");
  EXPECT_EQ(linearCamera.getLabel(), "linear");
  EXPECT_EQ(linearCamera.getImageSize(), Eigen::Vector2i(640, 480));
  EXPECT_DOUBLE_EQ(linearCamera.getTimeOffsetSecDeviceCamera(), 0.123);
  expectVectorNear(linearCamera.projectionParams(), expectedParams);

  EXPECT_EQ(sphericalCamera.modelName(), CameraProjection::ModelType::Spherical);
  EXPECT_EQ(sphericalCamera.getSerialNumber(), "SphericalCameraCalibration");
  EXPECT_EQ(sphericalCamera.getLabel(), "spherical");
  EXPECT_DOUBLE_EQ(sphericalCamera.getTimeOffsetSecDeviceCamera(), -0.25);
  expectVectorNear(sphericalCamera.projectionParams(), expectedParams);
  EXPECT_TRUE(sphericalCamera.getT_Device_Camera().matrix().isApprox(T_Device_Camera.matrix()));
}

TEST(CameraCalibrationTest, RotateLinearCameraUpdatesIntrinsicsExtrinsicsAndResolution) {
  const Sophus::SE3d T_Device_Camera{
      Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitX()).toRotationMatrix(),
      Eigen::Vector3d(1.0, 2.0, 3.0)};
  const CameraCalibration camera = makeLinearCalibration(5.0, 1.5, T_Device_Camera);

  const CameraCalibration rotated = rotateCameraCalibCW90Deg(camera);

  Eigen::VectorXd expectedParams(LinearProjection::kNumParams);
  expectedParams << 120.0, 100.0, 4.0, 4.0;
  expectVectorNear(rotated.projectionParams(), expectedParams);
  EXPECT_EQ(rotated.getImageSize(), Eigen::Vector2i(8, 10));
  EXPECT_EQ(rotated.getValidRadius(), std::optional<double>(5.0));
  EXPECT_EQ(rotated.getLabel(), "camera");
  EXPECT_EQ(rotated.getSerialNumber(), "serial");
  EXPECT_DOUBLE_EQ(rotated.getTimeOffsetSecDeviceCamera(), 0.25);

  const Sophus::SE3d expectedT_Device_Camera = T_Device_Camera * Sophus::SE3d::rotZ(M_PI / -2.0);
  EXPECT_TRUE(
      rotated.getT_Device_Camera().matrix().isApprox(expectedT_Device_Camera.matrix(), kTolerance));
}

TEST(CameraCalibrationTest, RotateFisheye624CameraTransformsDistortionLayout) {
  const CameraCalibration camera{
      "fisheye",
      CameraProjection::ModelType::Fisheye624,
      makeFisheye624Params(),
      Sophus::SE3d{},
      10,
      8,
      4.0,
      M_PI,
      "fisheye-serial",
      0.0};

  const CameraCalibration rotated = rotateCameraCalibCW90Deg(camera);

  Eigen::VectorXd expectedParams(Fisheye624::kNumParams);
  expectedParams << 10.0, 4.0, 4.0, 0.01, -0.02, 0.03, -0.04, 0.05, -0.06, 0.08, 0.07, -0.11, 0.12,
      0.09, -0.10;
  expectVectorNear(rotated.projectionParams(), expectedParams);
  EXPECT_EQ(rotated.getImageSize(), Eigen::Vector2i(8, 10));
  EXPECT_EQ(rotated.modelName(), CameraProjection::ModelType::Fisheye624);
  EXPECT_EQ(rotated.getValidRadius(), std::optional<double>(4.0));
}

TEST(CameraCalibrationTest, RotateRejectsUnsupportedProjectionModels) {
  const CameraCalibration sphericalCamera =
      getSphericalCameraCalibration(640, 480, 320.0, "spherical");

  EXPECT_THROW(rotateCameraCalibCW90Deg(sphericalCamera), std::runtime_error);
}
