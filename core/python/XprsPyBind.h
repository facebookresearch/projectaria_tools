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

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <fmt/core.h>
#include <sstream>
#include <type_traits>

#include <data_provider/ErrorHandler.h>
#include <data_provider/SensorData.h>
#include <data_provider/players/RgbImageFrameConverter.h>
#include <xprs/xprsDecoder.h>

namespace projectaria::tools::data_provider {

namespace py = pybind11;
namespace {

inline bool xprsFrameToImageData(xprs::Frame& xprsFrame, ImageData& imageData) {
  if ((imageData.getPixelFormat() == vrs::PixelFormat::GREY8) &&
      (xprsFrame.fmt == xprs::PixelFormat::YUV420P || xprsFrame.fmt == xprs::PixelFormat::NV12)) {
    xprsFrame.fmt = xprs::PixelFormat::GRAY8;
  }
  const uint32_t width = xprsFrame.width;
  const uint32_t height = xprsFrame.height;
  const uint32_t stride = xprsFrame.stride[0];

  // For grayscale images, just copy
  if (xprsFrame.fmt == xprs::PixelFormat::GRAY8) {
    imageData.pixelFrame->getBuffer().resize(static_cast<size_t>(width) * height);
    for (int i = 0; i < height; ++i) {
      std::copy_n(
          xprsFrame.planes[0] + i * stride,
          width,
          imageData.pixelFrame->getBuffer().begin() + i * width);
    }
    return true;
  }

  // SW decoder returns YUV420P, HW decoder returns NV12 for RGB image
  if (xprsFrame.fmt == xprs::PixelFormat::YUV420P) {
    std::vector<uint8_t> outDecodedFrame;
    outDecodedFrame.resize(static_cast<size_t>(width) * height * 3);
    convertDecodedYuv420ToRgb8(xprsFrame, outDecodedFrame, width, height);

    // Copy to pixel frame
    auto outputPixelFrame =
        std::make_shared<vrs::utils::PixelFrame>(vrs::PixelFormat::RGB8, width, height, width * 3);

    std::copy(
        outDecodedFrame.begin(), outDecodedFrame.end(), outputPixelFrame->getBuffer().begin());
    imageData.pixelFrame = std::move(outputPixelFrame);

    return true;
  } else {
    fmt::print("Unsupported pixel format\n");
    return false;
  }
}

void declareXprsDecoding(py::module& module) {
  using namespace xprs;
  py::enum_<VideoCodecFormat>(module, "VideoCodecFormat")
      .value("H264", VideoCodecFormat::H264)
      .value("H265", VideoCodecFormat::H265)
      .value("VP9", VideoCodecFormat::VP9)
      .export_values();

  py::enum_<XprsResult>(module, "XprsResult")
      .value("OK", XprsResult::OK)
      .value("ERR_NOT_INITIALIZED", XprsResult::ERR_NOT_INITIALIZED)
      .value("ERR_INVALID_FRAME", XprsResult::ERR_INVALID_FRAME)
      .value("ERR_NO_FRAME", XprsResult::ERR_NO_FRAME)
      .value("ERR_FFMPEG", XprsResult::ERR_FFMPEG)
      .value("ERR_SYSTEM", XprsResult::ERR_SYSTEM)
      .value("ERR_MUX_FAILURE", XprsResult::ERR_MUX_FAILURE)
      .export_values();

  py::class_<VideoCodec>(module, "VideoCodec")
      .def(py::init<>())
      .def_readwrite("format", &VideoCodec::format)
      .def_readwrite("implementation_name", &VideoCodec::implementationName)
      .def_readwrite("hw_accel", &VideoCodec::hwAccel);

  py::class_<IVideoDecoder, std::shared_ptr<IVideoDecoder>>(module, "IVideoDecoder")
      .def("init", &IVideoDecoder::init)
      .def(
          "decode_oss_frame",
          [](std::shared_ptr<IVideoDecoder> self, ImageData& imageData) -> bool {
            Frame xprsFrame;
            Buffer buffer = {
                imageData.pixelFrame->getBuffer().size(),
                imageData.pixelFrame->wdata(),
            };
            XprsResult xprsRes = self->decodeFrame(xprsFrame, buffer);
            if (xprsRes != XprsResult::OK) {
              fmt::print("Failed to decode frame using xprs\n");
              return false;
            }
            // convert to image data
            return xprsFrameToImageData(xprsFrame, imageData);
          });

  module.def(
      "createDecoder",
      [](const VideoCodec& codec) {
        IVideoDecoder* decoder = createDecoder(codec);
        return std::shared_ptr<IVideoDecoder>(decoder);
      },
      py::arg("codec"),
      py::return_value_policy::take_ownership);
}

} // namespace

inline void exportXprs(py::module& m) {
  declareXprsDecoding(m);
}

} // namespace projectaria::tools::data_provider
