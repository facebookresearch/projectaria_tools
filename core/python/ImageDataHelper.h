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

#include <image/ImageVariant.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace projectaria::tools::image {
namespace py = pybind11;

using PyArrayVariant = std::variant<
    py::array_t<float>,
    py::array_t<uint8_t>,
    py::array_t<uint16_t>,
    py::array_t<uint64_t>,
    py::array_t<Eigen::half>>;

inline PyArrayVariant toPyArrayVariant(const ImageVariant& imageVariant);
inline PyArrayVariant toPyArrayVariant(const ManagedImageVariant& imageVariant);

//////////////////////////////////////////////////////////////////////////////////////////////
// Implementation below
inline py::array_t<float> matrix2fToNumpy(const Eigen::MatrixXf& matrix) {
  size_t rows = matrix.rows();
  size_t cols = matrix.cols();
  return py::array_t<float>(
      {rows, cols}, // shape
      {cols * sizeof(float), sizeof(float)}, // strides
      matrix.data() // data pointer
  );
}

inline Eigen::MatrixXf numpyToMatrix2f(const py::array_t<float>& array) {
  // Request a buffer descriptor from the NumPy array
  py::buffer_info buf = array.request();
  if (buf.ndim != 2) {
    throw std::runtime_error("Input should be a 2D NumPy array");
  }
  if (buf.format != py::format_descriptor<float>::format()) {
    throw std::runtime_error("Input should be a NumPy array of type float");
  }
  size_t rows = buf.shape[0];
  size_t cols = buf.shape[1];
  Eigen::MatrixXf matrix = Eigen::Map<Eigen::MatrixXf>(static_cast<float*>(buf.ptr), rows, cols);
  return matrix;
}

struct PyArrayVariantVisitor {
  template <class T, int M>
  PyArrayVariant operator()(const projectaria::tools::image::Image<T, M>& image) const {
    size_t width = image.width();
    size_t height = image.height();
    size_t stride = image.pitch();
    return {py::array_t<T>({height, width}, {stride, sizeof(T)}, (T*)(image.data()))};
  }
  template <class T, int M, int D>
  PyArrayVariant operator()(
      const projectaria::tools::image::Image<Eigen::Matrix<T, D, 1, 0, D, 1>, M>& image) const {
    size_t width = image.width();
    size_t height = image.height();
    size_t stride = image.pitch();
    size_t channel = D;
    return {py::array_t<T>(
        {height, width, channel}, {stride, channel * sizeof(T), sizeof(T)}, (T*)(image.data()))};
  }
};

inline PyArrayVariant toPyArrayVariant(const ImageVariant& imageVariant) {
  return std::visit(PyArrayVariantVisitor(), imageVariant);
}

inline PyArrayVariant toPyArrayVariant(const ManagedImageVariant& imageVariant) {
  return std::visit(PyArrayVariantVisitor(), imageVariant);
}

} // namespace projectaria::tools::image
