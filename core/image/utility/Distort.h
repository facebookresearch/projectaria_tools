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

#pragma once
#include <functional>
#include <optional>

#include <image/ImageVariant.h>

namespace projectaria::tools::image {
/**
 * @brief Distorts an input image
 * @param srcVariant the input image
 * @param inverseWarp the 2d mapping from a pixel in the output image to a pixel in the input image
 * @param imageSize the size of the output image
 */
image::ManagedImageVariant distortImageVariant(
    const image::ImageVariant& srcVariant,
    const std::function<std::optional<Eigen::Vector2f>(const Eigen::Vector2f&)>& inverseWarp,
    const Eigen::Vector2i& imageSize);

} // namespace projectaria::tools::image
