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

#include <mps/HandTracking.h>

#include <gtest/gtest.h>

using namespace projectaria::tools::mps;

// Helper to create test HandTrackingResult
HandTrackingResult createTestHandTrackingSample(
    std::chrono::microseconds timestamp,
    bool hasLeft = false,
    bool hasRight = false,
    double confidence = 0.8) {
  HandTrackingResult result;
  result.trackingTimestamp = timestamp;

  auto createHand = [confidence]() {
    HandTrackingResult::OneSide hand;
    hand.confidence = confidence;

    // Create non-collinear landmarks for proper palm normal calculation
    for (size_t i = 0; i < kNumHandLandmarks; ++i) {
      // Default pattern for most landmarks
      hand.landmarkPositions_device[i] = Eigen::Vector3d(i * 0.1, i * 0.05, i * 0.02);
    }

    // Set key landmarks to form a proper triangle for palm normal calculation
    constexpr auto kWristIndex = static_cast<uint8_t>(HandLandmark::WRIST);
    constexpr auto kIndexProximalIndex = static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL);
    constexpr auto kPinkyProximalIndex = static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL);

    hand.landmarkPositions_device[kWristIndex] = Eigen::Vector3d(0, 0, 0); // WRIST at origin
    hand.landmarkPositions_device[kIndexProximalIndex] =
        Eigen::Vector3d(0.1, 0, 0); // INDEX_PROXIMAL on X-axis
    hand.landmarkPositions_device[kPinkyProximalIndex] =
        Eigen::Vector3d(0, 0.1, 0); // PINKY_PROXIMAL on Y-axis

    return hand;
  };

  if (hasLeft) {
    result.leftHand = createHand();
  }
  if (hasRight) {
    result.rightHand = createHand();
  }
  return result;
}

TEST(HandTrackingInterpolation, BasicAlphaInterpolation) {
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(1000), true, true, 0.9);
  auto r2 = createTestHandTrackingSample(std::chrono::microseconds(2000), true, true, 0.5);

  auto optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(1500));

  ASSERT_TRUE(optResult.has_value());
  auto result = optResult.value();
  EXPECT_EQ(result.trackingTimestamp, std::chrono::microseconds(1500));
  EXPECT_TRUE(result.leftHand.has_value());
  EXPECT_TRUE(result.rightHand.has_value());
  EXPECT_EQ(result.leftHand->confidence, 0.5); // min(0.9, 0.5)
}

TEST(HandTrackingInterpolation, MissingHandsSetToNullopt) {
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(1000), true, true);
  auto r2 =
      createTestHandTrackingSample(std::chrono::microseconds(2000), false, true); // missing left

  auto optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(1500));

  ASSERT_TRUE(optResult.has_value());
  auto result = optResult.value();
  EXPECT_FALSE(result.leftHand.has_value()); // nullopt when one missing
  EXPECT_TRUE(result.rightHand.has_value()); // present in both
}

TEST(HandTrackingInterpolation, TimestampBasedInterpolation) {
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(1000), true, false, 0.8);
  auto r2 =
      createTestHandTrackingSample(std::chrono::microseconds(50000), true, false, 0.6); // 49ms diff

  auto optResult = interpolateHandTrackingResult(r1, r2, std::chrono::microseconds(25500));

  ASSERT_TRUE(optResult.has_value());
  auto result = optResult.value();
  EXPECT_EQ(result.trackingTimestamp, std::chrono::microseconds(25500));
  EXPECT_TRUE(result.leftHand.has_value());
  EXPECT_EQ(result.leftHand->confidence, 0.6); // min(0.8, 0.6)
}

TEST(HandTrackingInterpolation, TimestampOutOfRange) {
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(1000), true, true);
  auto r2 = createTestHandTrackingSample(std::chrono::microseconds(2000), true, true);

  EXPECT_THROW(
      interpolateHandTrackingResult(r1, r2, std::chrono::microseconds(500)), std::invalid_argument);

  EXPECT_THROW(
      interpolateHandTrackingResult(r1, r2, std::chrono::microseconds(2500)),
      std::invalid_argument);
}

TEST(HandTrackingInterpolation, EdgeCases) {
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(1000), true, true);
  auto r2 = createTestHandTrackingSample(std::chrono::microseconds(2000), true, true);

  // Exact timestamps should return original results
  auto optResult1 = interpolateHandTrackingResult(r1, r2, std::chrono::microseconds(1000));
  ASSERT_TRUE(optResult1.has_value());
  EXPECT_EQ(optResult1->trackingTimestamp, std::chrono::microseconds(1000));

  auto optResult2 = interpolateHandTrackingResult(r1, r2, std::chrono::microseconds(2000));
  ASSERT_TRUE(optResult2.has_value());
  EXPECT_EQ(optResult2->trackingTimestamp, std::chrono::microseconds(2000));

  // Reversed timestamps should auto-swap
  auto optReversed = interpolateHandTrackingResult(r2, r1, std::chrono::microseconds(1500));
  ASSERT_TRUE(optReversed.has_value());
  EXPECT_EQ(optReversed->trackingTimestamp, std::chrono::microseconds(1500));
}

TEST(HandTrackingInterpolation, AlphaValidation) {
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(1000), true, true);
  auto r2 = createTestHandTrackingSample(std::chrono::microseconds(2000), true, true);

  // Test invalid alpha values
  EXPECT_THROW(
      interpolateHandTrackingResult(r1, r2, -0.1, std::chrono::microseconds(1500)),
      std::invalid_argument);

  EXPECT_THROW(
      interpolateHandTrackingResult(r1, r2, 1.1, std::chrono::microseconds(1500)),
      std::invalid_argument);
}

TEST(HandTrackingInterpolation, TimestampAlphaConsistency) {
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(1000), true, true);
  auto r2 = createTestHandTrackingSample(std::chrono::microseconds(3000), true, true);

  // Test inconsistent timestamp with alpha
  // Alpha 0.5 should correspond to timestamp 2000, not 1800
  EXPECT_THROW(
      interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(1800)),
      std::invalid_argument);

  // Test consistent timestamp with alpha (should succeed)
  auto optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(2000));
  EXPECT_TRUE(optResult.has_value());

  // Test tolerance boundary (±1 microsecond should be allowed)
  optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(2001));
  EXPECT_TRUE(optResult.has_value());

  optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(1999));
  EXPECT_TRUE(optResult.has_value());

  // Test beyond tolerance (should fail)
  EXPECT_THROW(
      interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(2002)),
      std::invalid_argument);
}

TEST(HandTrackingInterpolation, TimeDifference100msThreshold) {
  // Test exactly at 100ms threshold (should succeed)
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(0), true, true);
  auto r2 = createTestHandTrackingSample(std::chrono::microseconds(100000), true, true); // 100ms

  auto optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(50000));
  EXPECT_TRUE(optResult.has_value());

  // Test just below 100ms threshold (should succeed)
  r1 = createTestHandTrackingSample(std::chrono::microseconds(0), true, true);
  r2 = createTestHandTrackingSample(std::chrono::microseconds(99999), true, true); // 99.999ms

  optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(49999));
  EXPECT_TRUE(optResult.has_value());

  // Test above 100ms threshold (should return nullopt)
  r1 = createTestHandTrackingSample(std::chrono::microseconds(0), true, true);
  r2 = createTestHandTrackingSample(std::chrono::microseconds(100001), true, true); // 100.001ms

  optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(50000));
  EXPECT_FALSE(optResult.has_value());

  // Test timestamp-based function with time difference > 100ms
  optResult = interpolateHandTrackingResult(r1, r2, std::chrono::microseconds(50000));
  EXPECT_FALSE(optResult.has_value());
}

TEST(HandTrackingInterpolation, InterpolatedPalmNormals) {
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(1000), true, true);
  auto r2 = createTestHandTrackingSample(std::chrono::microseconds(2000), true, true);

  auto optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(1500));

  ASSERT_TRUE(optResult.has_value());
  auto result = optResult.value();

  // Check that palm normals were estimated and set
  EXPECT_TRUE(result.leftHand.has_value());
  EXPECT_TRUE(result.rightHand.has_value());
  EXPECT_TRUE(result.leftHand->wristAndPalmNormal_device.has_value());
  EXPECT_TRUE(result.rightHand->wristAndPalmNormal_device.has_value());

  // Check that palm normals are normalized vectors (length ≈ 1.0)
  auto leftPalmNormal = result.leftHand->wristAndPalmNormal_device->palmNormal_device;
  auto rightPalmNormal = result.rightHand->wristAndPalmNormal_device->palmNormal_device;

  EXPECT_NEAR(leftPalmNormal.norm(), 1.0, 1e-6);
  EXPECT_NEAR(rightPalmNormal.norm(), 1.0, 1e-6);

  // For left and right hands with the same landmark pattern,
  // palm normals should point in opposite directions
  EXPECT_LT(leftPalmNormal.dot(rightPalmNormal), 0.0);
}

TEST(HandTrackingInterpolation, SE3TransformationInterpolation) {
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(1000), true, false);
  auto r2 = createTestHandTrackingSample(std::chrono::microseconds(2000), true, false);

  // Set different SE3 transformations for testing interpolation
  Sophus::SE3d t1(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 0));
  Sophus::SE3d t2(Eigen::Quaterniond::Identity(), Eigen::Vector3d(2, 0, 0));

  r1.leftHand->T_Device_Wrist = t1;
  r2.leftHand->T_Device_Wrist = t2;

  auto optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(1500));

  ASSERT_TRUE(optResult.has_value());
  auto result = optResult.value();

  // Check that SE3 transformation was interpolated (should be halfway)
  EXPECT_TRUE(result.leftHand.has_value());
  auto interpolatedTransform = result.leftHand->T_Device_Wrist;

  // Translation should be interpolated (halfway between (0,0,0) and (2,0,0))
  Eigen::Vector3d expectedTranslation(1, 0, 0);
  EXPECT_TRUE(interpolatedTransform.translation().isApprox(expectedTranslation, 1e-6));
}

TEST(HandTrackingInterpolation, WristNormalFromInterpolatedSE3) {
  auto r1 = createTestHandTrackingSample(std::chrono::microseconds(1000), true, false);
  auto r2 = createTestHandTrackingSample(std::chrono::microseconds(2000), true, false);

  // Set SE3 transformations with different orientations
  Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q2 = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

  r1.leftHand->T_Device_Wrist = Sophus::SE3d(q1, Eigen::Vector3d::Zero());
  r2.leftHand->T_Device_Wrist = Sophus::SE3d(q2, Eigen::Vector3d::Zero());

  auto optResult = interpolateHandTrackingResult(r1, r2, 0.5, std::chrono::microseconds(1500));

  ASSERT_TRUE(optResult.has_value());
  auto result = optResult.value();

  EXPECT_TRUE(result.leftHand.has_value());
  EXPECT_TRUE(result.leftHand->wristAndPalmNormal_device.has_value());

  // Check that wrist normal is calculated from interpolated SE3 (-Y direction)
  auto interpolatedTransform = result.leftHand->T_Device_Wrist;
  Eigen::Vector3d expectedWristNormal = -1.0 * interpolatedTransform.rotationMatrix().col(1);
  auto actualWristNormal = result.leftHand->wristAndPalmNormal_device->wristNormal_device;

  EXPECT_TRUE(actualWristNormal.isApprox(expectedWristNormal, 1e-6));
  EXPECT_NEAR(actualWristNormal.norm(), 1.0, 1e-6); // Should be normalized
}

TEST(EstimatePalmNormal, BasicFunctionality) {
  // Create sample landmarks with simple geometric arrangement
  Landmarks landmarks;
  landmarks.fill(Eigen::Vector3d::Zero());

  // Set key landmarks for palm normal calculation
  landmarks[static_cast<uint8_t>(HandLandmark::WRIST)] = Eigen::Vector3d(0, 0, 0);
  landmarks[static_cast<uint8_t>(HandLandmark::INDEX_PROXIMAL)] = Eigen::Vector3d(1, 0, 0);
  landmarks[static_cast<uint8_t>(HandLandmark::PINKY_PROXIMAL)] = Eigen::Vector3d(0, 1, 0);

  // Test left hand
  auto leftPalmNormal = estimatePalmNormal(landmarks, HANDEDNESS::LEFT);
  EXPECT_NEAR(leftPalmNormal.norm(), 1.0, 1e-6);

  // Test right hand (should be negated)
  auto rightPalmNormal = estimatePalmNormal(landmarks, HANDEDNESS::RIGHT);
  EXPECT_NEAR(rightPalmNormal.norm(), 1.0, 1e-6);

  // Left and right should be opposite
  EXPECT_NEAR(leftPalmNormal.dot(rightPalmNormal), -1.0, 1e-6);
}
