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
#include <image/ImageVariant.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::calibration;
using namespace projectaria::tools::image;

#define STRING(x) #x
#define GEN1_STRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"

static const std::string ariaGen1TestDataPath = GEN1_STRING(TEST_FOLDER);

TEST(VrsDataProvider, colorCorrection) {
  // Color correction is only enabled for Gen1
  auto provider = createVrsDataProvider(ariaGen1TestDataPath);

  std::string cameraLabel = "camera-rgb";
  std::optional<vrs::StreamId> maybeStreamId = provider->getStreamIdFromLabel(cameraLabel);
  EXPECT_TRUE(maybeStreamId.has_value());
  const auto streamId = maybeStreamId.value();

  // Repeatedly query the same frame should return the same image data
  provider->setColorCorrection(true);
  ImageDataAndRecord firstQueryResult = provider->getImageDataByIndex(streamId, 0);
  auto firstImageVariant = firstQueryResult.first.imageVariant();
  EXPECT_TRUE(firstImageVariant.has_value());
  int x = 500, y = 500;
  const uint8_t firstImageValue = std::get<uint8_t>(at(firstImageVariant.value(), x, y, 0));

  for (int i = 0; i < 10; i++) {
    ImageDataAndRecord repeatedQueryResult = provider->getImageDataByIndex(streamId, 0);
    auto repeatedImageVariant = repeatedQueryResult.first.imageVariant();
    EXPECT_TRUE(repeatedImageVariant.has_value());

    const uint8_t repeatedImageValue = std::get<uint8_t>(at(repeatedImageVariant.value(), x, y, 0));

    EXPECT_EQ(firstImageValue, repeatedImageValue);
  }
}
