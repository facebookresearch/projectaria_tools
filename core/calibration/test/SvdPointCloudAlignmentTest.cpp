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

#include <calibration/SvdPointCloudAlignment.h>

#include <cmath>
#include <random>

#include <gtest/gtest.h>

using namespace projectaria::tools::calibration;

namespace {

std::vector<Eigen::Vector3d> makeTestPoints() {
  return {
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
      {1.0, 1.0, 0.0},
      {0.0, 1.0, 1.0},
  };
}

std::vector<Eigen::Vector3d> transformPoints(
    const Sophus::SE3d& T,
    const std::vector<Eigen::Vector3d>& points) {
  std::vector<Eigen::Vector3d> result;
  result.reserve(points.size());
  for (const auto& p : points) {
    result.push_back(T * p);
  }
  return result;
}

} // namespace

TEST(SvdPointCloudAlignmentTest, IdentityTransform) {
  const auto points = makeTestPoints();
  const auto result = alignPointCloudsToRigidTransform(points, points);
  ASSERT_TRUE(result.has_value());

  // Should be identity
  const Sophus::SE3d identity;
  EXPECT_LT((result->matrix() - identity.matrix()).norm(), 1e-10);
}

TEST(SvdPointCloudAlignmentTest, KnownRigidTransform) {
  const auto points_A = makeTestPoints();

  // Create a known rigid transform: 45 degree rotation around Z + translation
  const Eigen::AngleAxisd rotation(M_PI / 4.0, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d translation(0.1, -0.2, 0.3);
  const Sophus::SE3d T_B_A(rotation.toRotationMatrix(), translation);

  const auto points_B = transformPoints(T_B_A, points_A);
  const auto result = alignPointCloudsToRigidTransform(points_A, points_B);
  ASSERT_TRUE(result.has_value());

  // Verify recovered transform matches
  EXPECT_LT((result->matrix() - T_B_A.matrix()).norm(), 1e-10);
}

TEST(SvdPointCloudAlignmentTest, PureTranslation) {
  const auto points_A = makeTestPoints();

  const Eigen::Vector3d translation(0.05, -0.03, 0.01);
  const Sophus::SE3d T_B_A(Eigen::Matrix3d::Identity(), translation);

  const auto points_B = transformPoints(T_B_A, points_A);
  const auto result = alignPointCloudsToRigidTransform(points_A, points_B);
  ASSERT_TRUE(result.has_value());

  EXPECT_LT((result->translation() - translation).norm(), 1e-10);
}

TEST(SvdPointCloudAlignmentTest, PureRotation) {
  const auto points_A = makeTestPoints();

  const Eigen::AngleAxisd rotation(0.1, Eigen::Vector3d(1, 1, 1).normalized());
  const Sophus::SE3d T_B_A(rotation.toRotationMatrix(), Eigen::Vector3d::Zero());

  const auto points_B = transformPoints(T_B_A, points_A);
  const auto result = alignPointCloudsToRigidTransform(points_A, points_B);
  ASSERT_TRUE(result.has_value());

  EXPECT_LT((result->matrix() - T_B_A.matrix()).norm(), 1e-10);
}

TEST(SvdPointCloudAlignmentTest, WithNoise) {
  const auto points_A = makeTestPoints();

  const Eigen::AngleAxisd rotation(0.05, Eigen::Vector3d::UnitY());
  const Eigen::Vector3d translation(0.001, -0.002, 0.001);
  const Sophus::SE3d T_B_A(rotation.toRotationMatrix(), translation);

  auto points_B = transformPoints(T_B_A, points_A);

  // Add small noise
  std::mt19937 gen(42);
  std::normal_distribution<double> noise(0.0, 1e-4);
  for (auto& p : points_B) {
    p += Eigen::Vector3d(noise(gen), noise(gen), noise(gen));
  }

  const auto result = alignPointCloudsToRigidTransform(points_A, points_B);
  ASSERT_TRUE(result.has_value());

  // Result should be close but not exact due to noise
  EXPECT_LT((result->translation() - translation).norm(), 1e-3);
  EXPECT_LT((result->rotationMatrix() - T_B_A.rotationMatrix()).norm(), 1e-3);
}

TEST(SvdPointCloudAlignmentTest, TooFewPoints) {
  std::vector<Eigen::Vector3d> points_A = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
  std::vector<Eigen::Vector3d> points_B = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
  EXPECT_FALSE(alignPointCloudsToRigidTransform(points_A, points_B).has_value());
}

TEST(SvdPointCloudAlignmentTest, SizeMismatch) {
  std::vector<Eigen::Vector3d> points_A = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  std::vector<Eigen::Vector3d> points_B = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
  EXPECT_FALSE(alignPointCloudsToRigidTransform(points_A, points_B).has_value());
}

TEST(SvdPointCloudAlignmentTest, CollinearPoints) {
  std::vector<Eigen::Vector3d> points_A = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  std::vector<Eigen::Vector3d> points_B = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  EXPECT_FALSE(alignPointCloudsToRigidTransform(points_A, points_B).has_value());
}

TEST(SvdPointCloudAlignmentTest, EmptyInput) {
  std::vector<Eigen::Vector3d> empty;
  EXPECT_FALSE(alignPointCloudsToRigidTransform(empty, empty).has_value());
}
