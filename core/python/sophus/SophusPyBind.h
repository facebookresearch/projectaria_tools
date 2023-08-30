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

#include "SO3PyBind.h"

#include <sstream>
// By default, Sophus calls std::abort when a pre-condition fails. Register a handler that raises
// an exception so we don't crash the Python process.
#ifdef SOPHUS_DISABLE_ENSURES
#undef SOPHUS_DISABLE_ENSURES
#endif
#ifndef SOPHUS_ENABLE_ENSURE_HANDLER
#define SOPHUS_ENABLE_ENSURE_HANDLER
#endif

namespace sophus {
inline void
ensureFailed(char const* function, char const* file, int line, char const* description) {
  std::stringstream message;
  message << "'SOPHUS_ENSURE' failed in function '" << function << "', on line '" << line
          << "' of file '" << file << "'. Full description:" << std::endl
          << description;
  throw std::domain_error(message.str());
}

inline void exportSophus(pybind11::module& module) {
  exportSO3Group<double>(module, "SO3");

  pybind11::class_<Sophus::SE3d>(module, "SE3d")
      .def(pybind11::init<>())
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
          "Returns the inverse of the SE3d transform")
      .def("__repr__", [](const Sophus::SE3d& self) { return fmt::to_string(self); });
}

} // namespace sophus
