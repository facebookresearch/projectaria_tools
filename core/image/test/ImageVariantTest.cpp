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

#include <image/ImageVariant.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::image;

template <typename T>
void TestImageSizing(const int expectedChannels) {
  constexpr int width = 17;
  constexpr int height = 41;
  projectaria::tools::image::ImageVariant variant =
      projectaria::tools::image::Image<T>(nullptr, width, height);

  EXPECT_EQ(width, getWidth(variant));
  EXPECT_EQ(height, getHeight(variant));
  EXPECT_EQ(expectedChannels, getChannel(variant));
}

TEST(ImageVariant, Sizing) {
  TestImageSizing<uint8_t>(1);
  TestImageSizing<Eigen::Matrix<uint8_t, 2, 1>>(2);
  TestImageSizing<Eigen::Matrix<uint8_t, 3, 1>>(3);
  TestImageSizing<Eigen::Matrix<uint8_t, 4, 1>>(4);
  TestImageSizing<uint16_t>(1);
  TestImageSizing<Eigen::Matrix<uint16_t, 2, 1>>(2);
  TestImageSizing<Eigen::Matrix<uint16_t, 3, 1>>(3);
  TestImageSizing<float>(1);
  TestImageSizing<Eigen::Matrix<float, 2, 1>>(2);
  TestImageSizing<Eigen::Matrix<float, 3, 1>>(3);
  TestImageSizing<Eigen::Matrix<float, 4, 1>>(4);
}
