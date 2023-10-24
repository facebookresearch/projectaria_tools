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

#include "ImageDataHelper.h"

#include <image/ImageVariant.h>
#include <image/utility/Debayer.h>
#include <image/utility/Distort.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <cstdint>

namespace projectaria::tools::image {
namespace py = pybind11;

template <class ImageType>
void declare_image(py::module& module, const std::string& typestr) {
  py::class_<ImageType>(module, typestr.c_str())
      .def(py::init<>())
      .def("get_width", &ImageType::width, "Returns the number of columns")
      .def("get_height", &ImageType::height, "Returns the number of rows")
      .def(
          "to_numpy_array",
          [](const ImageType& self) -> PyArrayVariant {
            PyArrayVariantVisitor visitor;
            return visitor(self);
          },
          "Converts to numpy array")
      .def(
          "at",
          [](const ImageType& self, int x, int y, int channel) -> PixelValueVariant {
            return at(self, x, y, channel);
          },
          py::arg("x"),
          py::arg("y"),
          py::arg("channel") = 0,
          "Returns the pixel at (x, y, channel)");
}

void declare_debayer(py::module& module) {
  module.def(
      "debayer",
      [](py::array_t<uint8_t> arraySrc) {
        py::buffer_info info = arraySrc.request();

        size_t imageWidth = arraySrc.shape()[1];
        size_t imageHeight = arraySrc.shape()[0];
        if (arraySrc.ndim() != 2) {
          throw std::runtime_error("Can only debayer grayscale data");
        }
        image::ImageU8 imageRaw((uint8_t*)info.ptr, imageWidth, imageHeight);
        image::ManagedImage3U8 imageDebayered = image::debayer(imageRaw);

        image::PyArrayVariantVisitor visitor;
        return visitor(imageDebayered);
      },
      "Debayer and also correct color by preset color calibration");
}

void declare_interpolationMethod(py::module& module) {
  py::enum_<InterpolationMethod>(module, "InterpolationMethod", "Image interpolation method.")
      // image type
      .value("NEAREST_NEIGHBOR", InterpolationMethod::NearestNeighbor)
      .value("BILINEAR", InterpolationMethod::Bilinear)
      .export_values();
}

inline void exportImage(py::module& m) {
  // For submodule documentation, see: projectaria_tools/projectaria_tools/core/image.py

  // imageData
  declare_image<projectaria::tools::image::ManagedImage3U8>(m, "ManagedImage3U8");
  declare_image<projectaria::tools::image::ManagedImageU8>(m, "ManagedImageU8");
  declare_image<projectaria::tools::image::ManagedImage<float>>(m, "ManagedImageF32");
  declare_image<projectaria::tools::image::ManagedImage<uint16_t>>(m, "ManagedImageU16");
  declare_image<projectaria::tools::image::ManagedImage<uint64_t>>(m, "ManagedImageU64");

  declare_image<projectaria::tools::image::Image3U8>(m, "Image3U8");
  declare_image<projectaria::tools::image::ImageU8>(m, "ImageU8");
  declare_image<projectaria::tools::image::Image<float>>(m, "ImageF32");
  declare_image<projectaria::tools::image::Image<uint16_t>>(m, "ImageU16");
  declare_image<projectaria::tools::image::Image<uint64_t>>(m, "ImageU64");

  declare_debayer(m);

  declare_interpolationMethod(m);
}
} // namespace projectaria::tools::image
