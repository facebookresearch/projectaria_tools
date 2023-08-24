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

#include <mps/GlobalPointCloudReader.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::mps;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x))

static const std::string testDataFolder = XSTRING(TEST_FOLDER);

TEST(mps_globalpointcloud_valid_file, reader) {
  const auto globalpointcloudValues = readGlobalPointCloud(
      testDataFolder + "mps_sample/trajectory/global_points.csv.gz", StreamCompressionMode::GZIP);
  EXPECT_TRUE(globalpointcloudValues.size() > 0);
}

TEST(mps_globalpointcloud_valid_file_invalid_mode, reader) {
  const auto globalpointcloudValues = readGlobalPointCloud(
      testDataFolder + "mps_sample/trajectory/global_points.csv.gz", StreamCompressionMode::NONE);
  EXPECT_TRUE(globalpointcloudValues.empty());
}

TEST(mps_globalpointcloud_invalid_file, reader) {
  const auto globalpointcloudValues = readGlobalPointCloud("", StreamCompressionMode::GZIP);
  EXPECT_TRUE(globalpointcloudValues.empty());
}
