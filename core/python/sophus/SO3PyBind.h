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

#include <sophus/common.hpp>
#include <sophus/so3.hpp>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace sophus {

// In python, we choose to export our Sophus::SO3 as a vector of SO3 objects by binding the cpp
// object `SO3Group` defined below. This is because numerical code in python tends to work with
// array of values to get efficient program. This approach is inspired by
// scipy.spatial.transform.Rotation.
template <typename Scalar>
class SO3Group : public std::vector<Sophus::SO3<Scalar>> {
 public:
  // The empty constructor is not accessible from python.
  // Python always create at least one identity element (like c++ sophus)
  SO3Group() = default;
  // implicit copy conversion from a Sophus::SO3 value
  /* implicit */ SO3Group(const Sophus::SO3<Scalar>& in) {
    this->push_back(in);
  }
};
} // namespace sophus

// The following caster makes so that, even if we wrap SO3Group in python, those can be
// implicitly converted to the c++ Sophus::SO3 object at boundaries between languages.
// This is so we can pass python SO3 object to c++ function as if they were regular 1-element
// Sophus::SO3 object. This simplifies binding the rest of c++ code. This implicit cast fails if the
// python object is not a 1-element SO3 object.
// NOTE: this caster makes a copy, so can't not be used for passing a reference of a SO3 element to
// a c++ function.
namespace pybind11 {
namespace detail {
template <>
struct type_caster<Sophus::SO3<double>> {
 public:
  PYBIND11_TYPE_CASTER(Sophus::SO3<double>, _("SO3"));

  // converting from python -> c++ type
  bool load(handle src, bool /*convert*/) {
    try {
      sophus::SO3Group<double>& ref = src.cast<sophus::SO3Group<double>&>();
      if (ref.size() != 1) {
        throw std::domain_error(fmt::format(
            "A element of size 1 is required here. Input has {} elements.", ref.size()));
      }
      value = ref[0];
      return true;
    } catch (const pybind11::cast_error&) {
      return false; // Conversion failed
    }
  }

  // converting from c++ -> python type
  static handle cast(Sophus::SO3<double> src, return_value_policy policy, handle parent) {
    return type_caster_base<sophus::SO3Group<double>>::cast(sophus::SO3Group(src), policy, parent);
  }
};
} // namespace detail
} // namespace pybind11

namespace sophus {
template <typename Scalar>
using PybindSO3Group = pybind11::class_<SO3Group<Scalar>>;

template <typename Scalar>
PybindSO3Group<Scalar> exportSO3Group(pybind11::module& module, const std::string& name) {
  PybindSO3Group<Scalar> type(module, name.c_str());
  type.def(
      pybind11::init([]() {
        SO3Group<Scalar> ret;
        ret.push_back({});
        return ret;
      }),
      " Default Constructor initializing a group containing 1 identity element");
  type.def(pybind11::init<const Sophus::SO3<Scalar>&>(), "Copy constructor from single element");

  type.def_static(
      "exp",
      [](const Eigen::Matrix<Scalar, Eigen::Dynamic, 3>& rotvecs) -> SO3Group<Scalar> {
        SO3Group<Scalar> output;
        output.reserve(rotvecs.rows());
        for (size_t i = 0; i < rotvecs.rows(); ++i) {
          output.emplace_back(Sophus::SO3<Scalar>::exp(rotvecs.row(i)));
        }
        return output;
      },
      "Create rotations from rotations vectors of size Nx3 in rad");

  type.def_static(
      "from_quat",
      [](const Scalar& w, const Eigen::Matrix<Scalar, 3, 1>& xyz) -> SO3Group<Scalar> {
        Eigen::Matrix<Scalar, 4, 1> mat(w, xyz[0], xyz[1], xyz[2]);
        if (std::fabs(mat.norm() - 1.0) > Sophus::Constants<Scalar>::epsilon()) {
          throw std::runtime_error("The norm of the quaternion is not 1");
        }
        Eigen::Quaternion quat(mat[0], mat[1], mat[2], mat[3]);
        return {Sophus::SO3<Scalar>(quat)};
      },
      "Create a rotation from a quaternion as w, [x, y, z]");

  type.def_static(
      "from_quat",
      [](const std::vector<Scalar>& x_vec,
         const Eigen::Matrix<Scalar, -1, 3>& xyz_vec) -> SO3Group<Scalar> {
        if (x_vec.size() != xyz_vec.rows()) {
          throw std::runtime_error(fmt::format(
              "Size of the real and imagery part is not the same: {} {}",
              x_vec.size(),
              xyz_vec.rows()));
        }
        SO3Group<Scalar> output;
        output.reserve(x_vec.size());
        for (size_t i = 0; i < x_vec.size(); ++i) {
          Eigen::Matrix<Scalar, 4, 1> mat(x_vec[i], xyz_vec(i, 0), xyz_vec(i, 1), xyz_vec(i, 2));
          if (std::fabs(mat.norm() - 1.0) > Sophus::Constants<Scalar>::epsilon()) {
            throw std::runtime_error(
                fmt::format("The norm of the quaternion is not 1 for quaternion {}", mat));
          }
          Eigen::Quaternion quat(mat[0], mat[1], mat[2], mat[3]);
          output.push_back(Sophus::SO3<Scalar>(quat));
        }
        return output;
      },
      "Create rotations from a list of quaternions as w_vec: Nx1, xyz_vec: Nx3");

  type.def_static("from_matrix", [](const Eigen::Matrix<Scalar, 3, 3>& matrix) -> SO3Group<Scalar> {
    return Sophus::SO3<Scalar>::fitToSO3(matrix);
  });
  type.def_static(
      "from_matrix",
      [](const std::vector<Eigen::Matrix<Scalar, 3, 3>>& matrices) -> SO3Group<Scalar> {
        SO3Group<Scalar> output;
        output.reserve(matrices.size());
        for (size_t i = 0; i < matrices.size(); ++i) {
          output.push_back(Sophus::SO3<Scalar>::fitToSO3(matrices[i]));
        }
        return output;
      });

  type.def(
      "to_quat",
      [](const SO3Group<Scalar>& rotations) -> Eigen::Matrix<Scalar, Eigen::Dynamic, 4> {
        auto quaternions = Eigen::Matrix<Scalar, Eigen::Dynamic, 4>(rotations.size(), 4);
        for (size_t i = 0; i < rotations.size(); ++i) {
          quaternions.row(i) = Eigen::Matrix<Scalar, 1, 4>{
              rotations[i].unit_quaternion().w(),
              rotations[i].unit_quaternion().x(),
              rotations[i].unit_quaternion().y(),
              rotations[i].unit_quaternion().z(),
          };
        }
        return quaternions;
      },
      "Return quaternion as Nx4 vectors with the order [w x y z].");

  type.def(
      "to_matrix",
      [](const SO3Group<Scalar>& rotations) -> std::vector<Eigen::Matrix<Scalar, 3, 3>> {
        std::vector<Eigen::Matrix<Scalar, 3, 3>> output;
        output.reserve(rotations.size());

        for (size_t i = 0; i < rotations.size(); i++) {
          output.push_back(rotations[i].matrix());
        }
        return output;
      },
      "Convert an array of SO3 into an array of rotation matrices");

  type.def(
      "log",
      [](const SO3Group<Scalar>& rotations) -> Eigen::Matrix<Scalar, Eigen::Dynamic, 3> {
        auto output = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>(rotations.size(), 3);
        for (size_t i = 0; i < rotations.size(); ++i) {
          output.row(i) = rotations[i].log();
        }
        return output;
      },
      "Return the rotations vector representation by taking the log operator.");

  type.def(
      "inverse",
      [](const SO3Group<Scalar>& rotations) -> SO3Group<Scalar> {
        SO3Group<Scalar> result;
        for (size_t i = 0; i < rotations.size(); ++i) {
          result = rotations[i].inverse();
        }
        return result;
      },
      "Compute the inverse of the rotations.");

  type.def("__copy__", [](const SO3Group<Scalar>& rotations) -> SO3Group<Scalar> {
    return rotations; // copy is done with the std::vector copy constructor
  });
  type.def("__str__", [](const SO3Group<Scalar>& rotations) -> std::string {
    return fmt::format("sophus.SO3 (x{})", rotations.size());
  });
  type.def("__len__", [](const SO3Group<Scalar>& rotations) { return rotations.size(); });
  type.def("__repr__", [](const SO3Group<Scalar>& rotations) -> std::string {
    std::stringstream stream;
    stream << fmt::format("SO3 (wxyz) (x{})\n[", rotations.size());
    for (const auto& r : rotations) {
      stream << fmt::format(
          "[{}, {}, {}, {}],\n",
          r.unit_quaternion().w(),
          r.unit_quaternion().x(),
          r.unit_quaternion().y(),
          r.unit_quaternion().z());
    }
    // replace last to previous characters
    stream.seekp(-2, stream.cur);
    stream << "]";
    return stream.str();
  });

  type.def(
      "__matmul__",
      [](const SO3Group<Scalar>& rotations, const SO3Group<Scalar>& other) -> SO3Group<Scalar> {
        if (other.size() == 0 || rotations.size() == 0) {
          throw std::domain_error("Both operand should have size greater than 0");
        }
        SO3Group<Scalar> result;
        if (other.size() == 1) {
          result.reserve(rotations.size());
          for (size_t i = 0; i < rotations.size(); ++i) {
            result.push_back(rotations[i] * other[0]);
          }
        } else if (rotations.size() == 1) {
          result.reserve(other.size());
          for (size_t i = 0; i < other.size(); ++i) {
            result.push_back(rotations[0] * other[i]);
          }
        } else {
          throw std::domain_error(
              "Only allows rotations of size 1 to N (or N to 1) multiplication.");
        }
        return result;
      });
  type.def("__imatmul__", [](SO3Group<Scalar>& rotations, const SO3Group<Scalar>& other) {
    if (rotations.size() == 0 || other.size() == 0) {
      throw std::domain_error("Both operand should have size greater than 0");
    }

    if (rotations.size() == 1) {
      for (size_t i = 0; i < other.size(); ++i) {
        rotations[0] = rotations[0] * other[i];
      }
    } else if (other.size() == 1) {
      for (size_t i = 0; i < rotations.size(); ++i) {
        rotations[i] = rotations[i] * other[0];
      }
    } else {
      throw std::domain_error("Only allows rotations of size 1 to N (or N to 1) multiplication.");
    }

    return rotations;
  });

  type.def(
      "__matmul__",
      [](const SO3Group<Scalar>& rotations, const Eigen::Matrix<Scalar, 3, Eigen::Dynamic>& matrix)
          -> Eigen::Matrix<Scalar, 3, Eigen::Dynamic> {
        if (matrix.cols() == 0 || rotations.size() == 0) {
          throw std::domain_error("Both operand should have size greater than 0");
        }
        if (rotations.size() != 1) {
          throw std::domain_error("Number of rotations must be 1.");
        }

        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> result(3, matrix.cols());
        for (size_t i = 0; i < matrix.cols(); ++i) {
          result.col(i) = rotations[0] * matrix.col(i);
        }
        return result;
      });

  type.def(
      "__getitem__", [](const SO3Group<Scalar>& rotations, const size_t index) -> SO3Group<Scalar> {
        if (index < 0 || index >= rotations.size()) {
          throw std::out_of_range("Index out of range");
        }
        return rotations[index];
      });

  return type;
}

} // namespace sophus
