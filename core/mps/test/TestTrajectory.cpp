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

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <mps/TrajectoryFormat.h>
#include <mps/TrajectoryReaders.h>

using namespace projectaria::tools::mps;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x))

static const std::string testDataFolder = XSTRING(TEST_FOLDER);

static const std::string testClosedLoopFile =
    testDataFolder + "mps_sample/trajectory/closed_loop_trajectory.csv";
static const std::string testOpenLoopFile =
    testDataFolder + "mps_sample/trajectory/open_loop_trajectory.csv";

TEST(mps_closed_loop_valid_file, reader) {
  const auto trajectoryValues = readClosedLoopTrajectory(testClosedLoopFile);
  EXPECT_EQ(trajectoryValues.size(), 11517);
  for (const auto& trajectoryValue : trajectoryValues) {
    EXPECT_EQ(trajectoryValue.graphUid, "ea7288b8-b6a0-012d-8156-257aba6bd796");
    EXPECT_DOUBLE_EQ(trajectoryValue.gravity_world.z(), -9.81);
  }
}

TEST(mps_closed_loop_invalid_file, reader) {
  const auto trajectoryValues = readClosedLoopTrajectory("");
  EXPECT_TRUE(trajectoryValues.empty());
}

TEST(mps_open_loop_valid_file, reader) {
  const auto trajectoryValues = readOpenLoopTrajectory(testOpenLoopFile);
  EXPECT_FALSE(trajectoryValues.empty());
}

TEST(mps_open_loop_invalid_file, reader) {
  const auto trajectoryValues = readOpenLoopTrajectory("");
  EXPECT_TRUE(trajectoryValues.empty());
}
