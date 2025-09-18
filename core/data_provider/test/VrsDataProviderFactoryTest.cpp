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

#include <data_provider/VrsDataProvider.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::data_provider;

#define STRING(x) #x
#define GEN1_STRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"
#define GEN2_STRING(x) std::string(STRING(x)) + "aria_gen2_unit_test_sequence.vrs"

static const std::string ariaGen1TestDataPath = GEN1_STRING(TEST_FOLDER);
static const std::string ariaGen2TestDataPath = GEN2_STRING(TEST_FOLDER_GEN2);

TEST(VrsDataProvider, Factory) {
  std::shared_ptr<VrsDataProvider> providerGen1 = createVrsDataProvider(ariaGen1TestDataPath);
  EXPECT_TRUE(providerGen1);
  EXPECT_EQ(providerGen1->getDeviceVersion(), projectaria::tools::calibration::DeviceVersion::Gen1);
  std::shared_ptr<VrsDataProvider> providerGen2 = createVrsDataProvider(ariaGen2TestDataPath);
  EXPECT_TRUE(providerGen2);
  EXPECT_EQ(providerGen2->getDeviceVersion(), projectaria::tools::calibration::DeviceVersion::Gen2);
  EXPECT_FALSE(createVrsDataProvider("123"));
}
