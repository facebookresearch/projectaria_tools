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

#include "HandTracking.h"
#include <sophus/interpolate.hpp>
#include <algorithm>
#include <stdexcept>

namespace projectaria::tools::mps {

namespace {

constexpr int kMaxAllowedInterpolationGapMs = 100;

/**
 * @brief Helper function to interpolate between two HandTrackingResult::OneSide objects.
 * Performs linear interpolation on landmarks, takes minimum confidence, interpolates SE3d using
 * Sophus, estimates palm normal from interpolated landmarks, and calculates wrist normal from
 * interpolated SE3d.
 */
HandTrackingResult::OneSide interpolateOneSide(
    const HandTrackingResult::OneSide& side1,
    const HandTrackingResult::OneSide& side2,
    double alpha,
    HANDEDNESS handedness) {
  HandTrackingResult::OneSide interpolatedSide;

  // Take the minimum confidence
  interpolatedSide.confidence = std::min(side1.confidence, side2.confidence);

  // Interpolate landmark positions (3D points)
  for (size_t i = 0; i < kNumHandLandmarks; ++i) {
    interpolatedSide.landmarkPositions_device[i] =
        (1.0 - alpha) * side1.landmarkPositions_device[i] +
        alpha * side2.landmarkPositions_device[i];
  }

  // Interpolate SE3d transformation using Sophus
  interpolatedSide.T_Device_Wrist =
      Sophus::interpolate(side1.T_Device_Wrist, side2.T_Device_Wrist, alpha);

  // Estimate palm normal from interpolated landmarks
  Eigen::Vector3d interpolatedPalmNormal =
      estimatePalmNormal(interpolatedSide.landmarkPositions_device, handedness);

  // Calculate wrist normal as -Y direction of interpolated T_Device_Wrist
  Eigen::Vector3d interpolatedWristNormal =
      -1.0 * interpolatedSide.T_Device_Wrist.rotationMatrix().col(1);

  // Set the interpolated palm normal and wrist normal
  interpolatedSide.wristAndPalmNormal_device = HandTrackingResult::OneSide::WristAndPalmNormals{
      interpolatedPalmNormal, interpolatedWristNormal};

  return interpolatedSide;
}

} // anonymous namespace

std::optional<HandTrackingResult> interpolateHandTrackingResult(
    const HandTrackingResult& handPose1,
    const HandTrackingResult& handPose2,
    double alpha,
    std::chrono::microseconds timestamp) {
  // Validate alpha is in valid range
  if (alpha < 0.0 || alpha > 1.0) {
    throw std::invalid_argument("Alpha must be between 0.0 and 1.0");
  }

  // Extract timestamps from the results
  auto timestamp1 = handPose1.trackingTimestamp;
  auto timestamp2 = handPose2.trackingTimestamp;

  // Ensure handPose1 has the earlier timestamp (swap if needed)
  if (timestamp1 > timestamp2) {
    // If timestamps are reversed, swap inputs and adjust alpha
    return interpolateHandTrackingResult(handPose2, handPose1, 1.0 - alpha, timestamp);
  }

  // Check 100ms threshold - return nullopt if time difference exceeds 100ms
  auto timeDifference = timestamp2 - timestamp1;
  constexpr auto maxAllowedDifference = std::chrono::milliseconds(kMaxAllowedInterpolationGapMs);
  if (timeDifference > maxAllowedDifference) {
    return std::nullopt;
  }

  // Validate that the provided timestamp is consistent with the alpha value
  if (timestamp1 != timestamp2) { // Only check if timestamps are different
    auto totalDuration = timestamp2 - timestamp1;
    auto expectedTimestamp = timestamp1 +
        std::chrono::microseconds(static_cast<int64_t>(
            alpha * static_cast<double>(totalDuration.count())));

    // Allow small tolerance for floating point precision (±1 microsecond)
    auto tolerance = std::chrono::microseconds(1);
    if (std::abs((timestamp - expectedTimestamp).count()) > tolerance.count()) {
      throw std::invalid_argument(
          "Provided timestamp is inconsistent with alpha value and input timestamps");
    }
  }

  HandTrackingResult interpolatedResult;

  // Set the interpolated timestamp
  interpolatedResult.trackingTimestamp = timestamp;

  // Interpolate left hand only if both results have left hand data
  if (handPose1.leftHand.has_value() && handPose2.leftHand.has_value()) {
    interpolatedResult.leftHand = interpolateOneSide(
        handPose1.leftHand.value(), handPose2.leftHand.value(), alpha, HANDEDNESS::LEFT);
  }
  // If either result is missing left hand data, leftHand remains nullopt

  // Interpolate right hand only if both results have right hand data
  if (handPose1.rightHand.has_value() && handPose2.rightHand.has_value()) {
    interpolatedResult.rightHand = interpolateOneSide(
        handPose1.rightHand.value(), handPose2.rightHand.value(), alpha, HANDEDNESS::RIGHT);
  }
  // If either result is missing right hand data, rightHand remains nullopt

  return interpolatedResult;
}

std::optional<HandTrackingResult> interpolateHandTrackingResult(
    const HandTrackingResult& handPose1,
    const HandTrackingResult& handPose2,
    std::chrono::microseconds targetTimestamp) {
  // Extract timestamps from the results
  auto timestamp1 = handPose1.trackingTimestamp;
  auto timestamp2 = handPose2.trackingTimestamp;

  // Ensure handPose1 has the earlier timestamp
  if (timestamp1 > timestamp2) {
    // If timestamps are reversed, swap the inputs and call recursively
    return interpolateHandTrackingResult(handPose2, handPose1, targetTimestamp);
  }

  // Check 100ms threshold - return nullopt if time difference exceeds 100ms
  auto timeDifference = timestamp2 - timestamp1;
  constexpr auto maxAllowedDifference = std::chrono::milliseconds(100);
  if (timeDifference > maxAllowedDifference) {
    return std::nullopt;
  }

  // Validate that target timestamp is within the valid range
  if (targetTimestamp < timestamp1 || targetTimestamp > timestamp2) {
    throw std::invalid_argument(
        "Target timestamp must be between handPose1 and handPose2 timestamps");
  }

  // Handle edge cases where target timestamp equals one of the input timestamps
  if (targetTimestamp == timestamp1) {
    return handPose1;
  }
  if (targetTimestamp == timestamp2) {
    return handPose2;
  }

  // Calculate alpha based on temporal relationship
  auto totalDuration = timestamp2 - timestamp1;
  auto targetOffset = targetTimestamp - timestamp1;

  // Convert to double for interpolation calculation
  double alpha =
      static_cast<double>(targetOffset.count()) / static_cast<double>(totalDuration.count());

  // Use the existing alpha-based interpolation function
  return interpolateHandTrackingResult(handPose1, handPose2, alpha, targetTimestamp);
}

Eigen::Vector3d estimatePalmNormal(const Landmarks& landmarks, HANDEDNESS handedness) {
  constexpr auto kWristIndex = static_cast<uint8_t>(HandLandmark::WRIST);
  constexpr auto kIndexProximalIndex = static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL);
  constexpr auto kPinkyProximalIndex = static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL);

  const Eigen::Vector3d& wristLocation = landmarks[kWristIndex];
  const Eigen::Vector3d& indexProximalLocation = landmarks[kIndexProximalIndex];
  const Eigen::Vector3d& pinkyProximalLocation = landmarks[kPinkyProximalIndex];

  // Calculate palm normal using cross product
  Eigen::Vector3d palmNormal = (pinkyProximalLocation - wristLocation)
                                   .cross(indexProximalLocation - wristLocation)
                                   .normalized();

  // For right hand, flip the normal
  if (handedness == HANDEDNESS::RIGHT) {
    palmNormal *= -1.0;
  }

  return palmNormal;
}

} // namespace projectaria::tools::mps
