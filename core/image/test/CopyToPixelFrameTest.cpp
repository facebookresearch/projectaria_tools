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

#include <gtest/gtest.h>
#include <image/CopyToPixelFrame.h>
#include <image/ImageVariant.h>
#include <vrs/utils/PixelFrame.h>
using namespace projectaria::tools::image;
TEST(ManagedImage, CopyToPixelFrameTest) {
  // Unit test for copyToPixelFrame function
  constexpr int rows = 10;
  constexpr int cols = 9;
  ManagedImage<uint8_t> image(cols, rows);
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      image(col, row) = 42;
    }
  }
  ManagedImageVariant managedImageVariant = std::move(image);
  // Create a PixelFrame with the same size
  vrs::utils::PixelFrame pixelFrame(vrs::PixelFormat::GREY8, cols, rows, cols);
  vrs::utils::PixelFrame originalPixelFrame = pixelFrame;
  copyToPixelFrame(managedImageVariant, pixelFrame);

  // Verify that the data in PixelFrame matches the data from ManagedImage
  const uint8_t* pixelFrameData = pixelFrame.data<uint8_t>();
  for (size_t i = 0; i < pixelFrame.getBuffer().size(); ++i) {
    EXPECT_EQ(pixelFrameData[i], 42);
  }

  // Compare metadata of original and modified PixelFrame
  EXPECT_EQ(originalPixelFrame.getWidth(), pixelFrame.getWidth());
  EXPECT_EQ(originalPixelFrame.getHeight(), pixelFrame.getHeight());
  EXPECT_EQ(originalPixelFrame.getStride(), pixelFrame.getStride());
  EXPECT_EQ(originalPixelFrame.getPixelFormat(), pixelFrame.getPixelFormat());
}
