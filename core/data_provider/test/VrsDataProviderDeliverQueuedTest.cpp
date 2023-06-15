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
#define XSTRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"

static const std::string ariaTestDataPath = XSTRING(TEST_FOLDER);

#define DEFAULT_LOG_CHANNEL "TestLog"
#include <logging/Checks.h>
#include <logging/Log.h>

TEST(VrsDataProvider, deliverQueuedSensorData) {
  auto provider = createVrsDataProvider(ariaTestDataPath);

  // play all data
  int64_t lastDeviceTime = -1;
  int numData = 0;
  for (const auto& sensorData : provider->deliverQueuedSensorData()) {
    int64_t currentDeviceTime = sensorData.getTimeNs(TimeDomain::DeviceTime);
    // validate that device time are played in order
    EXPECT_GE(currentDeviceTime, lastDeviceTime);

    lastDeviceTime = currentDeviceTime;
    ++numData;
  }
  EXPECT_GE(numData, 1);

  // validate that all data are played
  int numDataExpected = 0;
  for (const auto& streamId : provider->getAllStreams()) {
    numDataExpected += provider->getNumData(streamId);
  }
  EXPECT_EQ(numData, numDataExpected);
}
