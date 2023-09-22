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

#include <mps/EyeGazeReader.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::mps;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x))

static const std::string testDataFolder = XSTRING(TEST_FOLDER);

TEST(mps_eyegaze_valid_file, reader) {
  const auto eyegazeValues = readEyeGaze(testDataFolder + "mps_sample/eye_gaze/eyegaze.csv");
  EXPECT_TRUE(eyegazeValues.size() > 0);
  EXPECT_EQ("", eyegazeValues[0].session_uid); // They should be empty
}

TEST(mps_eyegaze_valid_file_with_session_uid, reader) {
  const auto eyegazeValues =
      readEyeGaze(testDataFolder + "mps_sample/eye_gaze/generalized_eye_gaze.csv");
  EXPECT_TRUE(eyegazeValues.size() > 0);
  EXPECT_NE("", eyegazeValues[0].session_uid); // They should be NON empty
}

TEST(mps_eyegaze_invalid_file, reader) {
  const auto eyegazeValues = readEyeGaze("");
  EXPECT_TRUE(eyegazeValues.empty());
}
