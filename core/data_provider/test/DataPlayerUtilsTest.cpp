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

#include <data_provider/players/DataPlayerUtils.h>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace projectaria::tools::data_provider;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr float kNaNf = std::numeric_limits<float>::quiet_NaN();

// (x, y, z, w) = (1, 2, 3, 4) / sqrt(30); distinct components catch any xyzw -> wxyz mix-up.
const double kRotationNorm = std::sqrt(30.);
const vrs::Point4Dd kRotationXyzw{
    1. / kRotationNorm,
    2. / kRotationNorm,
    3. / kRotationNorm,
    4. / kRotationNorm};
const vrs::Point3Dd kTranslation{1., 2., 3.};

const vrs::Point4Df kRotationXyzwf{
    static_cast<float>(kRotationXyzw[0]),
    static_cast<float>(kRotationXyzw[1]),
    static_cast<float>(kRotationXyzw[2]),
    static_cast<float>(kRotationXyzw[3])};
const vrs::Point3Df kTranslationf{1.f, 2.f, 3.f};

template <typename T>
Eigen::Matrix<T, 3, 3> expectedRotationMatrix() {
  Eigen::Matrix<T, 3, 3> rotation;
  rotation << 4, -20, 22, 28, 10, 4, -10, 20, 20;
  return rotation / T(30);
}

std::optional<Sophus::SE3f> toSE3f(
    const vrs::Point4Dd& rotation,
    const vrs::Point3Dd& translation) {
  return populateToSE3<double, float>(rotation, translation);
}

// The instantiation HandPosePlayer uses.
std::optional<Sophus::SE3d> toSE3d(
    const vrs::Point4Df& rotation,
    const vrs::Point3Df& translation) {
  return populateToSE3<float, double>(rotation, translation);
}

} // namespace

TEST(DataPlayerUtils, populateToSE3KeepsAValidPose) {
  const auto pose = toSE3f(kRotationXyzw, kTranslation);
  ASSERT_TRUE(pose.has_value());
  EXPECT_TRUE(pose->rotationMatrix().isApprox(expectedRotationMatrix<float>()));
  EXPECT_TRUE(pose->translation().isApprox(Eigen::Vector3f{1.f, 2.f, 3.f}));
}

TEST(DataPlayerUtils, populateToSE3RejectsNaNRotation) {
  const auto pose = toSE3f(vrs::Point4Dd{kNaN, kNaN, kNaN, kNaN}, kTranslation);
  EXPECT_FALSE(pose.has_value());
}

TEST(DataPlayerUtils, populateToSE3RejectsNaNTranslation) {
  const auto pose = toSE3f(kRotationXyzw, vrs::Point3Dd{kNaN, kNaN, kNaN});
  EXPECT_FALSE(pose.has_value());
}

TEST(DataPlayerUtils, populateToSE3RejectsZeroRotation) {
  const auto pose = toSE3f(vrs::Point4Dd{0., 0., 0., 0.}, kTranslation);
  EXPECT_FALSE(pose.has_value());
}

// A rotation normalizable in double precision but not in float must be rejected
// for a float SE3, whose epsilon is the larger of the two.
TEST(DataPlayerUtils, populateToSE3RejectsRotationBelowTheOutputEpsilon) {
  const double tinyNorm = static_cast<double>(Sophus::Constants<float>::epsilon()) / 2.;
  const vrs::Point4Dd rotation{0., 0., 0., tinyNorm};
  const auto floatPose = toSE3f(rotation, kTranslation);
  EXPECT_FALSE(floatPose.has_value());
  const auto doublePose = populateToSE3<double>(rotation, kTranslation);
  EXPECT_TRUE(doublePose.has_value());
}

TEST(DataPlayerUtils, populateToSE3KeepsAValidPoseFromFloatInput) {
  const auto pose = toSE3d(kRotationXyzwf, kTranslationf);
  ASSERT_TRUE(pose.has_value());
  EXPECT_TRUE(pose->rotationMatrix().isApprox(expectedRotationMatrix<double>(), 1e-6));
  EXPECT_TRUE(pose->translation().isApprox(Eigen::Vector3d{1., 2., 3.}));
}

TEST(DataPlayerUtils, populateToSE3RejectsNaNFloatRotation) {
  const auto pose = toSE3d(vrs::Point4Df{kNaNf, kNaNf, kNaNf, kNaNf}, kTranslationf);
  EXPECT_FALSE(pose.has_value());
}

TEST(DataPlayerUtils, populateToSE3RejectsNaNFloatTranslation) {
  const auto pose = toSE3d(kRotationXyzwf, vrs::Point3Df{kNaNf, kNaNf, kNaNf});
  EXPECT_FALSE(pose.has_value());
}

// The mirror of the case above: the same float rotation is normalizable as a
// double but not as a float, so only the double output keeps it.
TEST(DataPlayerUtils, populateToSE3FromFloatInputUsesTheOutputEpsilon) {
  const vrs::Point4Df rotation{0.f, 0.f, 0.f, Sophus::Constants<float>::epsilon() / 2.f};
  EXPECT_TRUE(toSE3d(rotation, kTranslationf).has_value());
  EXPECT_FALSE(populateToSE3<float>(rotation, kTranslationf).has_value());
}
