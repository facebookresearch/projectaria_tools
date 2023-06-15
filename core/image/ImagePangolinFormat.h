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

#include "ImageVariant.h"

namespace projectaria::tools::image {

// @brief Returns the pangolin format string of an image type
// @tparam Img ImageClass type
template <typename Img>
inline std::string ImageTypeName() {
  return "NotRecognized";
}

// Following Pangolin format string conventions
template <>
inline std::string ImageTypeName<Image<uint8_t>>() {
  return "GRAY8";
}
template <>
inline std::string ImageTypeName<Image3U8>() {
  return "RGB24";
}
template <>
inline std::string ImageTypeName<Image4U8>() {
  return "RGBA32";
}
template <>
inline std::string ImageTypeName<ImageU10>() {
  return "GRAY10";
}
template <>
inline std::string ImageTypeName<ImageU12>() {
  return "GRAY12";
}
template <>
inline std::string ImageTypeName<ImageU16>() {
  return "GRAY16LE";
}
template <>
inline std::string ImageTypeName<Image2U10>() {
  return "2U10";
}
template <>
inline std::string ImageTypeName<Image2U12>() {
  return "2U12";
}
template <>
inline std::string ImageTypeName<Image2U16>() {
  return "2U16";
}
template <>
inline std::string ImageTypeName<Image3U10>() {
  return "RGB30";
}
template <>
inline std::string ImageTypeName<Image3U12>() {
  return "RGB36";
}
template <>
inline std::string ImageTypeName<Image3U16>() {
  return "RGB48";
}
template <>
inline std::string ImageTypeName<Image<float>>() {
  return "GRAY32F";
}
template <>
inline std::string ImageTypeName<Image<Eigen::Vector2f>>() {
  return "2F";
}
template <>
inline std::string ImageTypeName<Image<Eigen::Vector3f>>() {
  return "RGB3F";
}
template <>
inline std::string ImageTypeName<Image<Eigen::Vector4f>>() {
  return "RGBA4F";
}

inline std::string getPangolinFormatString(const ImageVariant& imageVariant) {
  return std::visit(
      [](const auto& img) { return ImageTypeName<typename std::decay<decltype(img)>::type>(); },
      imageVariant);
}
} // namespace projectaria::tools::image
