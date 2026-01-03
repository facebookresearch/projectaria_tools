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

#include <stdexcept>

#include <data_provider/ErrorHandler.h>
#include <fmt/format.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include "nlohmann/json.hpp"

namespace projectaria::tools::json {
// Templated function to parse fixed size Eigen::Vector from/to JSON
template <typename Scalar, int Size>
Eigen::Matrix<Scalar, Size, 1> eigenVectorFromJson(const nlohmann::json& jsonObj) {
  Eigen::Matrix<Scalar, Size, 1> vec;
  for (int i = 0; i < Size; ++i) {
    vec(i) = jsonObj[i];
  }
  return vec;
}
template <typename Scalar, int Size>
nlohmann::json eigenVectorToJson(const Eigen::Matrix<Scalar, Size, 1>& vec) {
  int vectorSize = Size == Eigen::Dynamic ? vec.rows() : Size;
  nlohmann::json jsonObj = nlohmann::json::array();
  for (int i = 0; i < vectorSize; ++i) {
    jsonObj.push_back(vec(i));
  }
  return jsonObj;
}

// Specialization for dynamic size
template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1> eigenVectorFromJson(const nlohmann::json& jsonObj) {
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> vec(jsonObj.size());
  for (size_t i = 0; i < jsonObj.size(); ++i) {
    vec(i) = jsonObj[i].get<Scalar>();
  }
  return vec;
}

// Templated function to parse Eigen::Matrix to/from JSON
template <typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Rows, Cols> eigenMatrixFromJson(const nlohmann::json& jsonObj) {
  Eigen::Matrix<Scalar, Rows, Cols> mat;
  for (int i = 0; i < Rows; ++i) {
    mat.row(i) = eigenVectorFromJson<Scalar, Cols>(jsonObj[i]).transpose();
  }
  return mat;
}
template <typename Scalar, int Rows, int Cols>
nlohmann::json eigenMatrixToJson(const Eigen::Matrix<Scalar, Rows, Cols>& mat) {
  nlohmann::json jsonObj = nlohmann::json::array();
  for (int i = 0; i < Rows; ++i) {
    jsonObj.push_back(eigenVectorToJson<Scalar, Cols>(mat.row(i).transpose()));
  }
  return jsonObj;
}

// Templated function to parse SE3 from/to JSON
template <typename Scalar>
Sophus::SE3<Scalar> se3FromJson(const nlohmann::json& jsonObj) {
  Eigen::Vector3<Scalar> translation = eigenVectorFromJson<Scalar, 3>(jsonObj["Translation"]);
  data_provider::checkAndThrow(
      jsonObj["UnitQuaternion"].size() == 2,
      fmt::format(
          "Expects UnitQuaternion to have two components, actual size: {}", jsonObj.size()));
  Scalar qReal = jsonObj["UnitQuaternion"][0].get<Scalar>();
  Eigen::Vector3<Scalar> qImag = eigenVectorFromJson<Scalar, 3>(jsonObj["UnitQuaternion"][1]);
  Eigen::Quaternion<Scalar> rotation(qReal, qImag.x(), qImag.y(), qImag.z());
  return Sophus::SE3<Scalar>(rotation, translation);
}
template <typename Scalar>
nlohmann::json se3ToJson(const Sophus::SE3<Scalar>& se3) {
  nlohmann::json jsonObj;
  jsonObj["Translation"] = eigenVectorToJson<Scalar, 3>(se3.translation());
  jsonObj["UnitQuaternion"] = nlohmann::json::array();
  jsonObj["UnitQuaternion"].push_back(se3.unit_quaternion().w());
  jsonObj["UnitQuaternion"].push_back(eigenVectorToJson<Scalar, 3>(se3.unit_quaternion().vec()));
  return jsonObj;
}

} // namespace projectaria::tools::json
