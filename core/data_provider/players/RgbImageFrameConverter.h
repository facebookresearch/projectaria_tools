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

#include <cmath>

#include <vrs/RecordFormat.h>
#include <vrs/utils/PixelFrame.h>
#include <xprs/xprs.h>

namespace projectaria::tools::data_provider {

/** @brief A helper function to convert RGB frame from YUV420 to RGB format, using Ocean library */
void convertDecodedYuv420ToRgb8(
    const vrs::utils::PixelFrame& inputFrame,
    vrs::utils::PixelFrame& outConvertedFrame,
    uint32_t w,
    uint32_t h);

/** @brief A helper function to convert RGB frame from YUV420 to RGB format, using Ocean library.
 Overload with output data type being std::vector<uint8_t>. */
void convertDecodedYuv420ToRgb8(
    const xprs::Frame& inputXprsFrame,
    std::vector<uint8_t>& outConvertedFrame,
    uint32_t w,
    uint32_t h);
} // namespace projectaria::tools::data_provider
