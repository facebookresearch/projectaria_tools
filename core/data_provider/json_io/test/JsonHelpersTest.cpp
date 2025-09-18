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

#include <data_provider/json_io/JsonHelpers.h>
#include <data_provider/json_io/test/TestCompareUtils.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::json;

TEST(JsonIOTest, EigenFixedVector3fRoundTrip) {
  // Ground-truth fixed-size vector
  Eigen::Matrix<float, 3, 1> gtVector;
  gtVector << 1.5f, -2.0f, 3.25f;

  // Serialize to JSON
  nlohmann::json jsonVector = projectaria::tools::json::eigenVectorToJson<float, 3>(gtVector);

  // Deserialize back
  Eigen::Matrix<float, 3, 1> resultVector =
      projectaria::tools::json::eigenVectorFromJson<float, 3>(jsonVector);

  EXPECT_TRUE(isClose(gtVector, resultVector));
}

TEST(JsonIOTest, EigenDynamicVectorRoundTrip) {
  // Ground-truth dynamic-size vector
  Eigen::VectorXf gtVector(4);
  gtVector << 0.1f, 0.2f, 0.3f, 0.4f;

  // Expected values for size check
  std::vector<float> gtValues = {0.1f, 0.2f, 0.3f, 0.4f};

  // Serialize to JSON
  nlohmann::json jsonDynamicVector =
      projectaria::tools::json::eigenVectorToJson<float, Eigen::Dynamic>(gtVector);

  // Deserialize back
  Eigen::VectorXf resultVector =
      projectaria::tools::json::eigenVectorFromJson<float>(jsonDynamicVector);

  ASSERT_EQ(gtVector.size(), gtValues.size());
  EXPECT_TRUE(isClose(gtVector, resultVector));
}

TEST(JsonIOTest, EigenMatrix3x3fRoundTrip) {
  // Ground-truth 3×3 matrix
  Eigen::Matrix3f gtMatrix;
  gtMatrix << 1, 2, 3, 4, 5, 6, 7, 8, 9;

  // Serialize to JSON
  nlohmann::json jsonMatrix = projectaria::tools::json::eigenMatrixToJson<float, 3, 3>(gtMatrix);

  // Deserialize back
  Eigen::Matrix3f resultMatrix =
      projectaria::tools::json::eigenMatrixFromJson<float, 3, 3>(jsonMatrix);

  EXPECT_TRUE(isClose(gtMatrix, resultMatrix));
}

TEST(JsonIOTest, SophusSE3fRoundTrip) {
  // Ground-truth SE3 pose
  Eigen::Quaternionf gtQuat(
      Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(1.0f, Eigen::Vector3f::UnitY()));
  Eigen::Vector3f gtTranslation(0.5f, -1.0f, 2.0f);
  Sophus::SE3f gtPose(gtQuat, gtTranslation);

  // Serialize to JSON
  nlohmann::json jsonPose = projectaria::tools::json::se3ToJson<float>(gtPose);

  // Deserialize back
  Sophus::SE3f resultPose = projectaria::tools::json::se3FromJson<float>(jsonPose);

  EXPECT_TRUE(isClose(gtPose, resultPose));
}
