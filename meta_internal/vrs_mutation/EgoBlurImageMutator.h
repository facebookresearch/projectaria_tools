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

#include "ImageMutationFilterCopier.h"

namespace EgoBlur {

struct EgoBlurImageMutator : public vrs::utils::UserDefinedImageMutator {
  std::string egoBlurFaceModelPath_;

  explicit EgoBlurImageMutator(const std::string& egoBlurFaceModelPath)
      : egoBlurFaceModelPath_(egoBlurFaceModelPath) {}

  bool operator()(
      double /*timestamp*/,
      const vrs::StreamId& streamId,
      vrs::utils::PixelFrame* frame) override {
    if (!frame) {
      return false;
    }

    // Convert the Frame to a Pytorch Tensor...
    // frame->getWidth(), frame->getHeight()

    if (streamId.getNumericName().find("214") != std::string::npos) {
      // RGB

      // Inference

      // Copy back results into the frame
      // std::memcpy(
      //     frame->wdata(),
      //     std::get<ManagedImage3U8>(undistortedImage).data(),
      //     frame->getWidth() * frame->getStride());
    } else {
      // Gray

      // Inference

      // Copy back results into the frame
      // std::memcpy(
      //     frame->wdata(),
      //     std::get<ManagedImageU8>(undistortedImage).data(),
      //     frame->getWidth() * frame->getHeight());
    }
    return true;
  }
};

} // namespace EgoBlur
