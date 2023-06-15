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

#include <image/Image.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::image;

TEST(Image, Constructor) {
  // The test image data is held as an eigen matrix for convenience. We use a non-square 2x4 image
  // to ensure that the width and height are correctly set.
  Eigen::Matrix<uint8_t, 2, 4, Eigen::RowMajor> image_data;
  image_data << 1, 2, 3, 4, 5, 6, 7, 8;
  const size_t image_data_num_bytes = image_data.rows() * image_data.cols() * sizeof(uint8_t);
  const size_t image_data_pitch = image_data.cols() * sizeof(uint8_t);
  const Eigen::Vector2i image_dim(image_data.cols(), image_data.rows());

  // Empty constructor
  std::unique_ptr<Image<uint8_t>> image = std::make_unique<Image<uint8_t>>();
  EXPECT_EQ(image->width(), 0);
  EXPECT_EQ(image->height(), 0);
  EXPECT_EQ(image->sizeBytes(), 0);
  EXPECT_FALSE(image->isValid());

  // Constructor with data pointer and width and height.
  image = std::make_unique<Image<uint8_t>>(image_data.data(), image_data.cols(), image_data.rows());
  EXPECT_EQ(image->width(), image_data.cols());
  EXPECT_EQ(image->height(), image_data.rows());
  EXPECT_EQ(image->sizeBytes(), image_data_num_bytes);
  EXPECT_TRUE(image->isValid());

  // Constructor with data pointer and width and pitch.
  image = std::make_unique<Image<uint8_t>>(
      image_data.data(), image_data.cols(), image_data.rows(), image_data_pitch);
  EXPECT_EQ(image->width(), image_data.cols());
  EXPECT_EQ(image->height(), image_data.rows());
  EXPECT_EQ(image->sizeBytes(), image_data_num_bytes);
  EXPECT_TRUE(image->isValid());
}
