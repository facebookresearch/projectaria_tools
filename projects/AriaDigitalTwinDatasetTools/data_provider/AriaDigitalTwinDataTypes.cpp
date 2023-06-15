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

#include "AriaDigitalTwinDataTypes.h"

#include <cstdint>
#include <fstream>

#include <vrs/DiskFile.h>

namespace projectaria::dataset::adt {
// depth clipp at 7m = 255 for visualization
constexpr double kClippedAtMm = 7000;

projectaria::tools::image::ManagedImage3U8 SegmentationData::getVisualizable() const {
  if (!isValid()) {
    throw std::runtime_error(
        "trying to access invalid ImageGtData, please use isValid() before access the data");
  }
  // Read into image
  projectaria::tools::image::ManagedImage3U8 imageColored(getWidth(), getHeight());
  auto imageSegmentation =
      std::get<projectaria::tools::image::Image<uint64_t>>(imageVariant().value());

  for (int j = 0; j < getHeight(); ++j) {
    for (int i = 0; i < getWidth(); ++i) {
      const uint64_t value = imageSegmentation(i, j);
      imageColored(i, j)(0) = static_cast<uint8_t>(value % 256 + 0.5);
      imageColored(i, j)(1) = static_cast<uint8_t>(243 * value % 256 + 0.5);
      imageColored(i, j)(2) = static_cast<uint8_t>(17 * value % 256 + 0.5);
    }
  }
  return imageColored;
}

projectaria::tools::image::ManagedImageU8 DepthData::getVisualizable() const {
  if (!isValid()) {
    throw std::runtime_error(
        "trying to access invalid ImageGtData, please use isValid() before access the data");
  }
  // Read into image
  projectaria::tools::image::ManagedImageU8 imageGray(getWidth(), getHeight());
  auto imageDepth = std::get<projectaria::tools::image::Image<uint16_t>>(imageVariant().value());
  for (int j = 0; j < getHeight(); ++j) {
    for (int i = 0; i < getWidth(); ++i) {
      const double value = imageDepth(i, j);
      uint8_t valueMapped = 0;
      if (value >= kClippedAtMm) {
        valueMapped = 255;
      } else {
        valueMapped = value / kClippedAtMm * 255.0;
      }
      imageGray(i, j) = static_cast<uint16_t>(valueMapped);
    }
  }
  return imageGray;
}

std::string RotationalSymmetryAxis::toString() const {
  return fmt::format("axis: [{}, {}, {}], angleDegree: {}", axis[0], axis[1], axis[2], angleDegree);
}

std::string RotationalSymmetry::toString() const {
  std::string result;
  result = fmt::format("isAnnotated = {},  axes = [", isAnnotated);
  for (const auto& axis : axes) {
    result += "{" + axis.toString() + "},";
  }
  result += "]";
  return result;
}

std::string CanonicalPose::toString() const {
  std::string result;
  result = fmt::format(
      "up_vector = [{}, {}, {}], front_vector = [{}, {}, {}]",
      upVector[0],
      upVector[1],
      upVector[2],
      frontVector[0],
      frontVector[1],
      frontVector[2]);
  return result;
}

std::string InstanceInfo::toString() const {
  std::string result;
  result += "instance id: " + std::to_string(id) + "\n";
  result += "instance name: " + name + "\n";
  result += "prototype name: " + prototypeName + "\n";
  result += "category: " + category + "\n";
  result += "category uid: " + std::to_string(categoryUid) + "\n";
  result += "instance type: ";
  switch (instanceType) {
    case InstanceType::Unknown:
      result += "Unknown\n";
      break;
    case InstanceType::Object:
      result += "Object\n";
      break;
    case InstanceType::Human:
      result += "Human\n";
      break;
  }
  result += "motion type: ";
  switch (motionType) {
    case MotionType::Unknown:
      result += "Unknown\n";
      break;
    case MotionType::Dynamic:
      result += "Dynamic\n";
      break;
    case MotionType::Static:
      result += "Static\n";
      break;
  }
  result += "rigidity type: ";
  switch (rigidityType) {
    case RigidityType::Unknown:
      result += "Unknown\n";
      break;
    case RigidityType::Rigid:
      result += "Rigid\n";
      break;
    case RigidityType::Deformable:
      result += "Deformable\n";
      break;
  }

  result += "rotational symmetry: " + rotationalSymmetry.toString() + "\n";
  result += "canonical pose: " + canonicalPose.toString() + "\n";
  result += "associated aria (only for human type): " + associatedDeviceSerial + "\n";

  return result;
}

} // namespace projectaria::dataset::adt
