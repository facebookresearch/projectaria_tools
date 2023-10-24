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
#include <calibration/CameraCalibration.h>
#include <image/ImageVariant.h>
#include <image/utility/Distort.h>

namespace projectaria::tools::calibration {
/**
 * @brief Distorts an input image to swap its underlying image distortion model
 * @param srcVariant the input image
 * @param dstCalib the calibration model of the output image
 * @param srcCalib the calibration model of the input image
 * @param method the interpolation method (Bilinear, NearestNeighbor)
 */
image::ManagedImageVariant distortByCalibration(
    const image::ImageVariant& srcVariant,
    const CameraCalibration& dstCalib,
    const CameraCalibration& srcCalib,
    const image::InterpolationMethod method = image::InterpolationMethod::Bilinear);

/**
 * @brief Distorts an input depth image using InterpolationMethod::Bilinear to swap its underlying
 * image distortion model
 * @param srcVariant the input image
 * @param dstCalib the calibration model of the output image
 * @param srcCalib the calibration model of the input image
 */
image::ManagedImageVariant distortDepthByCalibration(
    const image::ImageVariant& srcVariant,
    const CameraCalibration& dstCalib,
    const CameraCalibration& srcCalib);

/**
 * @brief Distorts an input label mask image using InterpolationMethod::NearestNeighbor to swap its
 * underlying image distortion model
 * @param srcVariant the input image
 * @param dstCalib the calibration model of the output image
 * @param dstCalib the calibration model of the input image
 */
image::ManagedImageVariant distortLabelMaskByCalibration(
    const image::ImageVariant& srcVariant,
    const CameraCalibration& dstCalib,
    const CameraCalibration& srcCalib);

} // namespace projectaria::tools::calibration
