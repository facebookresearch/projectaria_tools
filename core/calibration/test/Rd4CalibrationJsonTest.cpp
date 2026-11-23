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

#include <calibration/loader/SensorCalibrationJson.h>

#include <gtest/gtest.h>

#include <stdexcept>

using namespace projectaria::tools::calibration;

namespace {

CameraCalibration makeRd4Camera() {
  Eigen::VectorXd params(FisheyeCubicPlusRd4::kNumParams);
  params << 201.7, 201.6, 200.2, 199.9, -0.02, -0.1, 0.3, -0.6, 0.6, -0.35, 0.08, -5e-4, 5e-4,
      -8e-4, 8e-4, 4e-4, 4e-4, -1e-3, -7e-5, -2e-4, 2e-4, -4e-4, -2e-5, -4.6e-4, 9e-4, -2.4e-4;
  return {
      "test-camera",
      CameraProjection::ModelType::FisheyeCubicPlusRd4,
      params,
      Sophus::SE3d(
          Sophus::SO3d::exp(Eigen::Vector3d(0.1, -0.2, 0.3)), Eigen::Vector3d(0.06, 0.002, -0.015)),
      400,
      400,
      190.0,
      1.16,
      "test-serial",
      0.001};
}

} // namespace

TEST(Rd4CalibrationJsonTest, RoundTripsWithCanonicalModelName) {
  const auto camera = makeRd4Camera();
  const auto json = cameraCalibrationToJson(camera);
  EXPECT_EQ(json.at("Projection").at("Name"), "FisheyeCubicPlusRd4<6>");

  const CameraConfigBuilder defaults(DeviceVersion::Gen2);
  const auto parsed = parseCameraCalibrationFromJson(json, defaults);
  EXPECT_EQ(parsed.modelName(), CameraProjection::ModelType::FisheyeCubicPlusRd4);
  EXPECT_TRUE(parsed.projectionParams().isApprox(camera.projectionParams(), 1e-12));
  EXPECT_TRUE(parsed.getT_Device_Camera().matrix().isApprox(camera.getT_Device_Camera().matrix()));
  EXPECT_EQ(parsed.getImageSize(), camera.getImageSize());
  EXPECT_EQ(parsed.getValidRadius(), camera.getValidRadius());
  EXPECT_DOUBLE_EQ(parsed.getMaxSolidAngle(), camera.getMaxSolidAngle());
  EXPECT_DOUBLE_EQ(parsed.getTimeOffsetSecDeviceCamera(), camera.getTimeOffsetSecDeviceCamera());
  EXPECT_EQ(parsed.getLabel(), camera.getLabel());
  EXPECT_EQ(parsed.getSerialNumber(), camera.getSerialNumber());
}

TEST(Rd4CalibrationJsonTest, RejectsIncorrectParameterCount) {
  auto json = cameraCalibrationToJson(makeRd4Camera());
  json["Projection"]["Params"].erase(0);
  EXPECT_THROW(
      parseCameraCalibrationFromJson(json, CameraConfigBuilder(DeviceVersion::Gen2)),
      std::runtime_error);
}
