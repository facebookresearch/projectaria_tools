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

#include <calibration/camera_projections/CameraProjection.h>
#include <gtest/gtest.h>

#define DEFAULT_LOG_CHANNEL "CameraProjectionTest"
using namespace projectaria::tools::calibration;
static const std::unordered_map<CameraProjection::ModelType, Eigen::VectorXd>
    kTestProjectionParams = {
        {CameraProjection::ModelType::Linear,
         Eigen::VectorXd::Map(
             std::array<double, 4>{
                 {556.9648709428538, 555.9836002606175, 319.5000821079618, 239.500905501914}}
                 .data(),
             4)},
        {CameraProjection::ModelType::KannalaBrandtK3,
         Eigen::VectorXd::Map(
             std::array<double, 8>{{556.9648709428538,
                                    555.9836002606175,
                                    319.5000821079618,
                                    239.500905501914,
                                    0.03981131761869532,
                                    -0.03980438501559722,
                                    0.1244291105385938,
                                    -0.2780730001905606}}
                 .data(),
             8)},
        {CameraProjection::ModelType::Fisheye624,
         Eigen::VectorXd::Map(
             std::array<double, 15>{{242.0503165415735,
                                     319.8053191211108,
                                     240.9679064324467,
                                     -0.02689165420917155,
                                     0.1019919163316408,
                                     -0.07180862612716554,
                                     0.01245898233008245,
                                     0.001242086400994195,
                                     -0.0004149914187376608,
                                     0.0005499786009926172,
                                     0.0005819661606400815,
                                     -0.001232007804086031,
                                     -0.0003868334860997226,
                                     -0.0005963449586486949,
                                     1.071922498768078e-05}}
                 .data(),
             15)},
        {CameraProjection::ModelType::Spherical,
         Eigen::VectorXd::Map(std::array<double, 4>{{800.0, 800.0, 640.0, 360.0}}.data(), 4)}};
static const Eigen::Matrix<float, 3, 1> kPtInCameraFloat(1.0f, 1.0f, 5.0f);
static const Eigen::Matrix<double, 3, 1> kPtInCameraDouble(1.0, 1.0, 5.0);
static const Eigen::Matrix<float, 2, 1> kCameraPixelFloat(340.0f, 240.0f);
static const Eigen::Matrix<double, 2, 1> kCameraPixelDouble(340.0, 240.0);
static constexpr double kDesiredPrecision = 1e-5;

void compareEigenVector(
    const Eigen::VectorXf& vectorInFloat,
    const Eigen::VectorXd& vectorInDouble,
    const std::string& testNamePrefix) {
  double differenceNorm = (vectorInFloat.cast<double>() - vectorInDouble).norm();
  bool are_close = differenceNorm <=
      kDesiredPrecision * std::min(vectorInFloat.cast<double>().norm(), vectorInDouble.norm());
  EXPECT_TRUE(are_close);
  if (!are_close) {
    fmt::print(
        "{}: differenceNorm / minNorm: {:.5f}\n",
        testNamePrefix,
        differenceNorm / std::min(vectorInFloat.cast<double>().norm(), vectorInDouble.norm()));
  }
}

void testCastForSinglePair(
    const CameraProjectionTemplated<double>& cameraProjectionDouble,
    const CameraProjectionTemplated<float>& cameraProjectionFloat,
    const std::string& testNamePrefix) {
  compareEigenVector(
      cameraProjectionFloat.projectionParams(),
      cameraProjectionDouble.projectionParams(),
      testNamePrefix + "projectionParams");

  compareEigenVector(
      cameraProjectionFloat.project(kPtInCameraFloat),
      cameraProjectionDouble.project(kPtInCameraDouble),
      testNamePrefix + "project");

  compareEigenVector(
      cameraProjectionFloat.unproject(kCameraPixelFloat),
      cameraProjectionDouble.unproject(kCameraPixelDouble),
      testNamePrefix + "unproject");
}

TEST(CameraProjectionTest, FloatDoubleParamsComparison) {
  for (const auto& [modelType, projectionParamsDouble] : kTestProjectionParams) {
    CameraProjectionTemplated<double> cameraProjectionDouble(modelType, projectionParamsDouble);
    CameraProjectionTemplated<float> cameraProjectionFloat = cameraProjectionDouble.cast<float>();
    testCastForSinglePair(cameraProjectionDouble, cameraProjectionFloat, "DtoF, ");

    CameraProjectionTemplated<float> cameraProjectionFloat2(
        static_cast<typename CameraProjectionTemplated<float>::ModelType>(modelType),
        projectionParamsDouble.cast<float>());
    CameraProjectionTemplated<double> cameraProjectionDouble2 =
        cameraProjectionFloat2.cast<double>();
    testCastForSinglePair(cameraProjectionDouble2, cameraProjectionFloat2, "FtoD, ");
  }
}

TEST(CameraProjectionTest, Jacobian) {
  // Set up problem
  Eigen::Matrix<double, 2, 3> dProj_dPointCam;
  const double max_norm = 1e-4;

  for (const auto& [modelType, projectionParams] : kTestProjectionParams) {
    // Skip spherical model, since we can't compute Jacobian
    if (modelType == CameraProjection::ModelType::Spherical) {
      continue;
    }

    CameraProjectionTemplated<double> cameraProjectionDouble(modelType, projectionParams);

    // Project at X and compute Jacobian
    Eigen::Matrix<double, 2, 1> fX0 =
        cameraProjectionDouble.project(kPtInCameraDouble, &dProj_dPointCam);

    // Check Jacobian with finite difference approximation
    double eps = 1e-8;
    Eigen::Matrix<double, 2, 3> J(fX0.rows(), kPtInCameraDouble.rows());

    for (int i = 0; i < J.cols(); ++i) {
      Eigen::Matrix<double, 3, 1> xp = kPtInCameraDouble;
      Eigen::Matrix<double, 3, 1> xn = kPtInCameraDouble;

      xp[i] += eps;
      xn[i] -= eps;

      const Eigen::Matrix<double, 2, 1> fxp = cameraProjectionDouble.project(xp);
      const Eigen::Matrix<double, 2, 1> fxn = cameraProjectionDouble.project(xn);

      J.col(i) = (fxp - fxn) / (2 * eps);
    }

    // We expect the approximate derivative to be close to the actual derivative
    double norm = (J - dProj_dPointCam).norm();
    EXPECT_NEAR(norm, 0.0, max_norm);
  }
}
