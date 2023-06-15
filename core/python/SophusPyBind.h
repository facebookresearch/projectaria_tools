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

#include <sophus/se3.hpp>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace sophus {

namespace py = pybind11;

inline void exportSophus(py::module& m) {
  m.doc() = "A minimal pybind11 binding for Sophus module.";

  py::class_<Sophus::SE3d>(m, "SE3d")
      .def(py::init<>())
      .def(
          "rotation_matrix",
          [](const Sophus::SE3d& self) { return self.rotationMatrix(); },
          "Returns 3x3 rotation matrix")
      .def(
          "quaternion",
          [](const Sophus::SE3d& self) {
            auto& q = self.so3().unit_quaternion();
            return std::vector<double>{q.w(), q.x(), q.y(), q.z()};
          },
          "Return quaternion as [w x y z]")
      .def(
          "translation",
          [](const Sophus::SE3d& self) { return self.translation(); },
          "returns 3x1 translation vector")
      .def(
          "matrix",
          [](const Sophus::SE3d& self) { return self.matrix(); },
          "Returns 3x4 transform matrix.")
      .def(
          "inverse",
          [](const Sophus::SE3d& self) { return self.inverse(); },
          "Returns the inverse of the SE3d transform");
}
} // namespace sophus
