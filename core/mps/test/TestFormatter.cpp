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

#include <mps/HandTrackingFormat.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace projectaria::tools::mps;
using ::testing::MatchesRegex;

TEST(format_oneSideWristAndPalmPose, formattedMessageIsCorrectForDefaultPose) {
  WristAndPalmPose::OneSide oneSideWristAndPalmPose;
  std::string fmtStr = fmt::format("{}", oneSideWristAndPalmPose);
  std::cout << fmtStr << std::endl;
  EXPECT_THAT(
      fmtStr, MatchesRegex("^WristAndPalmPose::OneSide\\(confidence: .*, wrist: .*, palm: .*\\)"));
}

TEST(format_oneSideWristAndPalmPose, formattedMessageIsCorrectForBothNormals) {
  WristAndPalmPose::OneSide oneSideWristAndPalmPose;
  // Now contains both palm and wrist normals
  oneSideWristAndPalmPose.wristAndPalmNormal_device =
      WristAndPalmPose::OneSide::WristAndPalmNormals();
  oneSideWristAndPalmPose.wristAndPalmNormal_device->palmNormal_device = Eigen::Vector3d(0, 1, 0);
  oneSideWristAndPalmPose.wristAndPalmNormal_device->wristNormal_device = Eigen::Vector3d(0, 0, 1);
  std::string fmtStr = fmt::format("{}", oneSideWristAndPalmPose);
  std::cout << fmtStr << std::endl;
  EXPECT_THAT(
      fmtStr,
      MatchesRegex(
          "^WristAndPalmPose::OneSide\\(confidence: .*, wrist: .*, palm: .*, palmNormal: .*, wristNormal: .*\\)"));
}
