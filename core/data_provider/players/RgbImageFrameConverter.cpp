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

#include <data_provider/players/RgbImageFrameConverter.h>

#include <ocean/base/Frame.h>
#include <ocean/base/WorkerPool.h>
#include <ocean/cv/FrameConverter.h>
#include <ocean/cv/FrameConverterY_U_V12.h>

namespace projectaria::tools::data_provider {

namespace {
// Ocean::Worker is initialized when converting large files
Ocean::Worker* getWorkers(uint32_t pixels) {
  return pixels >= 640 * 480 ? Ocean::WorkerPool::get().scopedWorker()() : nullptr;
}

void convertToRgbAndCopyToRawPtr(
    const Ocean::FrameType& sourceFrameType,
    Ocean::Frame::PlaneInitializers<uint8_t>& sourcePlaneInitializers,
    uint8_t* outConvertedPtr,
    uint32_t w,
    uint32_t h) {
  Ocean::Frame sourceFrame(sourceFrameType, sourcePlaneInitializers);
  const Ocean::FrameType targetFrameType(
      w, h, Ocean::FrameType::FORMAT_RGB24, Ocean::FrameType::ORIGIN_UPPER_LEFT);
  Ocean::Frame targetFrame(
      targetFrameType, reinterpret_cast<void*>(outConvertedPtr), Ocean::Frame::CM_USE_KEEP_LAYOUT);
  if (!Ocean::CV::FrameConverter::Comfort::convertAndCopy(
          sourceFrame, targetFrame, getWorkers(w * h))) {
    const std::string errorMessage = "Ocean: Convert to rgb and copy failed";
    throw std::runtime_error(errorMessage);
  }
  if (targetFrame.isPlaneOwner()) {
    const std::string errorMessage = "Error: target frame is still plane owner";
    throw std::runtime_error(errorMessage);
  }
}

} // namespace

void convertDecodedYuv420ToRgb8(
    const vrs::utils::PixelFrame& inputFrame,
    vrs::utils::PixelFrame& outConvertedFrame,
    uint32_t w,
    uint32_t h) {
  const Ocean::FrameType sourceFrameType(
      w, h, Ocean::FrameType::FORMAT_Y_U_V12, Ocean::FrameType::ORIGIN_UPPER_LEFT);
  const uint8_t* yPlane = inputFrame.rdata();
  const uint8_t* uPlane = yPlane + inputFrame.getPlaneStride(0) * inputFrame.getPlaneHeight(0);
  const uint8_t* vPlane = uPlane + inputFrame.getPlaneStride(1) * inputFrame.getPlaneHeight(1);

  Ocean::Frame::PlaneInitializers<uint8_t> sourcePlaneInitializers = {
      {yPlane,
       Ocean::Frame::CM_USE_KEEP_LAYOUT,
       static_cast<uint32_t>(inputFrame.getPlaneStride(0) - w)},
      {uPlane,
       Ocean::Frame::CM_USE_KEEP_LAYOUT,
       static_cast<uint32_t>(inputFrame.getPlaneStride(1) - (w + 1) / 2)},
      {vPlane,
       Ocean::Frame::CM_USE_KEEP_LAYOUT,
       static_cast<uint32_t>(inputFrame.getPlaneStride(2) - (w + 1) / 2)},
  };
  convertToRgbAndCopyToRawPtr(
      sourceFrameType, sourcePlaneInitializers, outConvertedFrame.wdata(), w, h);
}

void convertDecodedYuv420ToRgb8(
    const xprs::Frame& inputXprsFrame,
    std::vector<uint8_t>& outConvertedFrame,
    uint32_t w,
    uint32_t h) {
  const Ocean::FrameType sourceFrameType(
      w, h, Ocean::FrameType::FORMAT_Y_U_V12, Ocean::FrameType::ORIGIN_UPPER_LEFT);
  Ocean::Frame::PlaneInitializers<uint8_t> sourcePlaneInitializers = {
      {inputXprsFrame.planes[0],
       Ocean::Frame::CM_USE_KEEP_LAYOUT,
       static_cast<uint32_t>(inputXprsFrame.stride[0] - w)},
      {inputXprsFrame.planes[1],
       Ocean::Frame::CM_USE_KEEP_LAYOUT,
       static_cast<uint32_t>(inputXprsFrame.stride[1] - (w + 1) / 2)},
      {inputXprsFrame.planes[2],
       Ocean::Frame::CM_USE_KEEP_LAYOUT,
       static_cast<uint32_t>(inputXprsFrame.stride[2] - (w + 1) / 2)},
  };
  convertToRgbAndCopyToRawPtr(
      sourceFrameType, sourcePlaneInitializers, outConvertedFrame.data(), w, h);
}

void convertDecodedNv12ToRgb8(
    const xprs::Frame& xprsFrame,
    std::vector<uint8_t>& outConvertedFrame,
    uint32_t w,
    uint32_t h) {
  Ocean::FrameType sourceFrameType(
      w, h, Ocean::FrameType::FORMAT_Y_UV12, Ocean::FrameType::ORIGIN_UPPER_LEFT); // NV12
  Ocean::Frame::PlaneInitializers<uint8_t> sourcePlaneInitializers = {
      {xprsFrame.planes[0],
       Ocean::Frame::CM_USE_KEEP_LAYOUT,
       static_cast<uint32_t>(xprsFrame.stride[0] - xprsFrame.width)},
      {xprsFrame.planes[1],
       Ocean::Frame::CM_USE_KEEP_LAYOUT,
       static_cast<uint32_t>(xprsFrame.stride[1] - xprsFrame.width)},
  };
  convertToRgbAndCopyToRawPtr(
      sourceFrameType, sourcePlaneInitializers, outConvertedFrame.data(), w, h);
}

} // namespace projectaria::tools::data_provider
