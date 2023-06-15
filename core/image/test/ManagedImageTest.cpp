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

#include <image/ManagedImage.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::image;

TEST(ManagedImage, Constructor) {
  projectaria::tools::image::ManagedImage<float> imageEmpty;
  EXPECT_EQ(imageEmpty.width(), 0);
  EXPECT_EQ(imageEmpty.height(), 0);
  EXPECT_EQ(imageEmpty.sizeBytes(), 0);
  EXPECT_FALSE(imageEmpty.isValid());

  projectaria::tools::image::ManagedImage<float> image(10, 9);
  EXPECT_EQ(image.width(), 10);
  EXPECT_EQ(image.height(), 9);
  EXPECT_EQ(image.sizeBytes(), 9 * 10 * sizeof(float));
  EXPECT_TRUE(image.isValid());
}
