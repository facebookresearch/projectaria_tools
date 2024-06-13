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

#include <gtest/gtest.h>
#include <sophus/average.hpp>

#include "AriaDigitalTwinDataPathsProvider.h"
#include "AriaDigitalTwinDataProvider.h"
#include "AriaStreamIds.h"

using namespace projectaria::dataset::adt;
using namespace projectaria::tools::data_provider;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x))

static const std::string adtTestDataPath = XSTRING(TEST_FOLDER);
static constexpr double kTolerance = 1e-5;
static constexpr int kNumImagesInTestData = 3;

namespace {

// Helper function to check if two SE3's are close enough
bool SE3IsApproximatelyEqual(const Sophus::SE3d& lhs, const Sophus::SE3d& rhs, double tolerance) {
  bool equalFlag = true;
  equalFlag &= (lhs.translation() - rhs.translation()).isMuchSmallerThan(tolerance);
  equalFlag &= std::abs(lhs.angleX() - rhs.angleX()) < tolerance;
  equalFlag &= std::abs(lhs.angleY() - rhs.angleY()) < tolerance;
  equalFlag &= std::abs(lhs.angleZ() - rhs.angleZ()) < tolerance;

  return equalFlag;
}

/*
 * Helper class to test Aria query APIs for a given stream id in AdtDataProvider
 */
class AriaQueryApiTester {
 public:
  AriaQueryApiTester(
      const std::shared_ptr<AriaDigitalTwinDataProvider>& provider,
      const std::string& label,
      const vrs::StreamId& streamId,
      const size_t expectedNumSamples)
      : provider_(provider),
        label_(label),
        streamId_(streamId),
        expectedNumSamples_(expectedNumSamples) {}

  void run();

 private:
  std::shared_ptr<AriaDigitalTwinDataProvider> provider_;
  std::string label_;
  vrs::StreamId streamId_;
  size_t expectedNumSamples_;
};

void AriaQueryApiTester::run() {
  fmt::print("Raw API testing for stream {} \n", streamId_.getNumericName());

  // Test calibration label
  const auto camCalib = provider_->getAriaCameraCalibration(streamId_);
  EXPECT_TRUE(camCalib.has_value());
  EXPECT_EQ(camCalib->getLabel(), label_);

  // Test timestamp vec querying
  const auto timestampVec = provider_->getAriaDeviceCaptureTimestampsNs(streamId_);
  EXPECT_EQ(timestampVec.size(), expectedNumSamples_);

  // Test image getter APIs over all samples except first and last
  for (size_t iSample = 1; iSample < expectedNumSamples_ - 1; ++iSample) {
    const int64_t sampleDeviceTimestamp = timestampVec.at(iSample);

    // These should all return the exact same image as rgbImage
    auto sampleImageWithDt = provider_->getAriaImageByTimestampNs(
        sampleDeviceTimestamp, streamId_, TimeQueryOptions::Closest);
    EXPECT_TRUE(sampleImageWithDt.isValid());
    EXPECT_EQ(sampleImageWithDt.dtNs(), 0);

    const int64_t dtBefore = (sampleDeviceTimestamp - timestampVec.at(iSample - 1)) / 2 - 1;
    sampleImageWithDt = provider_->getAriaImageByTimestampNs(
        sampleDeviceTimestamp - dtBefore, streamId_, TimeQueryOptions::Closest);
    EXPECT_TRUE(sampleImageWithDt.isValid());
    EXPECT_EQ(sampleImageWithDt.dtNs(), dtBefore);

    const int64_t dtAfter = (timestampVec.at(iSample + 1) - sampleDeviceTimestamp) / 2 - 1;
    sampleImageWithDt = provider_->getAriaImageByTimestampNs(
        sampleDeviceTimestamp + dtAfter, streamId_, TimeQueryOptions::Closest);
    EXPECT_TRUE(sampleImageWithDt.isValid());
    EXPECT_EQ(sampleImageWithDt.dtNs(), -dtAfter);

    // Test time conversion round trip from DeviceTime -> Timecode -> DeviceTime
    const int64_t timecodeTimestamp = provider_->getTimecodeFromDeviceTimeNs(sampleDeviceTimestamp);
    const int64_t roundtripDeviceTimestamp =
        provider_->getDeviceTimeFromTimecodeNs(timecodeTimestamp);
    EXPECT_NEAR(
        sampleDeviceTimestamp, roundtripDeviceTimestamp, 20); /* allow for numerical difference */
  }
}

/*
 * Helper class to test instance query APIs in AdtDataProvider
 */
class InstanceQueryApiTester {
 public:
  InstanceQueryApiTester(
      const std::shared_ptr<AriaDigitalTwinDataProvider>& provider,
      const size_t expectedNumInstances,
      const size_t expectedNumObjects,
      const size_t expectedNumSkeletons,
      const std::vector<InstanceId>& expectedInstances)
      : provider_(provider),
        numInstances_(expectedNumInstances),
        numObjects_(expectedNumObjects),
        numSkeletons_(expectedNumSkeletons),
        instances_(expectedInstances) {}

  void run();

 private:
  std::shared_ptr<AriaDigitalTwinDataProvider> provider_;
  size_t numInstances_;
  size_t numObjects_;
  size_t numSkeletons_;
  std::vector<InstanceId> instances_;
};

void InstanceQueryApiTester::run() {
  fmt::print("Instance query API testing");

  auto instanceIds = provider_->getInstanceIds();
  EXPECT_EQ(instanceIds.size(), numInstances_);

  auto objectIds = provider_->getObjectIds();
  EXPECT_EQ(objectIds.size(), numObjects_);

  auto skeletonIds = provider_->getSkeletonIds();
  EXPECT_EQ(skeletonIds.size(), numSkeletons_);

  EXPECT_EQ(numInstances_, numObjects_ + numSkeletons_);

  // Check the list of instances are in this dataset
  for (const auto& instanceId : instances_) {
    EXPECT_TRUE(provider_->hasInstanceId(instanceId));
  }
}

class InterpolationFunctionTester {
 public:
  InterpolationFunctionTester(const std::shared_ptr<AriaDigitalTwinDataProvider>& provider)
      : provider_(provider) {}

  void run();

 private:
  std::shared_ptr<AriaDigitalTwinDataProvider> provider_;
};

void InterpolationFunctionTester::run() {
  fmt::print("Interpolation function testing");

  // Locate a device timestamp in the middle of the sequence
  int64_t sampleTimestamp = provider_->getStartTimeNs() / 2 + provider_->getEndTimeNs() / 2;

  // find the aria pose sample that is before and after this timestamp
  auto ariaPoseBefore =
      provider_->getAria3dPoseByTimestampNs(sampleTimestamp, TimeQueryOptions::Before);
  EXPECT_TRUE(ariaPoseBefore.isValid());
  auto ariaPoseAfter =
      provider_->getAria3dPoseByTimestampNs(sampleTimestamp + 1, TimeQueryOptions::After);
  EXPECT_TRUE(ariaPoseAfter.isValid());
  int64_t beforeTimestamp = sampleTimestamp + ariaPoseBefore.dtNs();
  int64_t afterTimestamp = sampleTimestamp + 1 + ariaPoseAfter.dtNs();
  EXPECT_GE(afterTimestamp, beforeTimestamp);

  // Take the middle timestamp for test, could be off by 1 because of integer type
  int64_t middleTimestamp = (beforeTimestamp + afterTimestamp) / 2;

  // Query interpolation function with timestamp == frame_time
  auto interpPoseBefore = getInterpolatedAria3dPoseAtTimestampNs(*provider_, beforeTimestamp);
  auto interpPoseAfter = getInterpolatedAria3dPoseAtTimestampNs(*provider_, afterTimestamp);
  EXPECT_TRUE(interpPoseBefore.isValid());
  EXPECT_TRUE(interpPoseAfter.isValid());
  EXPECT_EQ(interpPoseBefore.dtNs(), 0);
  EXPECT_EQ(interpPoseAfter.dtNs(), 0);

  EXPECT_TRUE(SE3IsApproximatelyEqual(
      interpPoseBefore.data().T_Scene_Device, ariaPoseBefore.data().T_Scene_Device, kTolerance));
  EXPECT_TRUE(SE3IsApproximatelyEqual(
      interpPoseAfter.data().T_Scene_Device, ariaPoseAfter.data().T_Scene_Device, kTolerance));

  // Query interpolation with middle timestamp, it should
  auto interpPoseMiddle = getInterpolatedAria3dPoseAtTimestampNs(*provider_, middleTimestamp);
  EXPECT_TRUE(interpPoseMiddle.isValid());
  EXPECT_EQ(interpPoseMiddle.dtNs(), 0);
  const auto maybeAvgPose = Sophus::average(std::vector<Sophus::SE3d>{
      ariaPoseBefore.data().T_Scene_Device, ariaPoseAfter.data().T_Scene_Device});
  EXPECT_TRUE(maybeAvgPose);
  EXPECT_TRUE(SE3IsApproximatelyEqual(*maybeAvgPose, interpPoseMiddle.data().T_Scene_Device, 1e-2));
}

} // namespace

TEST(AdtDataProvider, AriaQueryAPI) {
  // Construct a ADT data provider from the test data path
  const auto dataPathsProvider = AriaDigitalTwinDataPathsProvider(adtTestDataPath);
  const auto maybeDataPaths = dataPathsProvider.getDataPaths(true);
  EXPECT_TRUE(maybeDataPaths.has_value());

  std::shared_ptr<AriaDigitalTwinDataProvider> provider =
      std::make_shared<AriaDigitalTwinDataProvider>(maybeDataPaths.value());
  // Test RGB and slam streams
  auto rgbTest =
      AriaQueryApiTester(provider, "camera-rgb", kRgbCameraStreamId, kNumImagesInTestData);
  rgbTest.run();
  auto slamLeftTest = AriaQueryApiTester(
      provider, "camera-slam-left", kSlamLeftCameraStreamId, kNumImagesInTestData);
  slamLeftTest.run();
  auto slamRightTest = AriaQueryApiTester(
      provider, "camera-slam-right", kSlamRightCameraStreamId, kNumImagesInTestData);
  slamRightTest.run();
}

TEST(AdtDataProvider, DataProviderPtrAndFlagApi) {
  // Construct a ADT data provider from the test data path
  const auto dataPathsProvider = AriaDigitalTwinDataPathsProvider(adtTestDataPath);
  const auto maybeDataPaths = dataPathsProvider.getDataPaths(true);
  EXPECT_TRUE(maybeDataPaths.has_value());

  std::shared_ptr<AriaDigitalTwinDataProvider> provider =
      std::make_shared<AriaDigitalTwinDataProvider>(maybeDataPaths.value());
  EXPECT_NE(provider->rawDataProviderPtr(), nullptr);
  EXPECT_NE(provider->segmentationDataProviderPtr(), nullptr);
  EXPECT_NE(provider->depthDataProviderPtr(), nullptr);
  EXPECT_NE(provider->syntheticDataProviderPtr(), nullptr);

  // Check flags
  EXPECT_TRUE(provider->hasAriaData());
  EXPECT_TRUE(provider->hasAria3dPoses());
  EXPECT_TRUE(provider->hasDepthImages());
  EXPECT_TRUE(provider->hasSegmentationImages());
  EXPECT_TRUE(provider->hasEyeGaze());
  EXPECT_TRUE(provider->hasInstance2dBoundingBoxes());
  EXPECT_TRUE(provider->hasInstancesInfo());
  EXPECT_TRUE(provider->hasSkeleton());
  EXPECT_TRUE(provider->hasSyntheticImages());
}

TEST(AdtDataProvider, InstanceQueryAPI) {
  // Construct a ADT data provider from the test data path
  const auto dataPathsProvider = AriaDigitalTwinDataPathsProvider(adtTestDataPath);
  const auto maybeDataPaths = dataPathsProvider.getDataPaths(true);
  EXPECT_TRUE(maybeDataPaths.has_value());

  std::shared_ptr<AriaDigitalTwinDataProvider> provider =
      std::make_shared<AriaDigitalTwinDataProvider>(maybeDataPaths.value());
  auto instanceQueryTester = InstanceQueryApiTester(
      provider,
      350 /*instances*/,
      349 /*objects*/,
      1 /*skeleton*/,
      {4213328128795167,
       7379153972126671,
       6243788802362822,
       4560452377344604,
       8471018366305712,
       5577751545657322} /* a list of hand-picked object ids in the dataset */);
  instanceQueryTester.run();
}

TEST(AdtDataProvider, InterpolationTest) {
  // Construct a ADT data provider from the test data path
  const auto dataPathsProvider = AriaDigitalTwinDataPathsProvider(adtTestDataPath);
  const auto maybeDataPaths = dataPathsProvider.getDataPaths();
  EXPECT_TRUE(maybeDataPaths.has_value());

  std::shared_ptr<AriaDigitalTwinDataProvider> provider =
      std::make_shared<AriaDigitalTwinDataProvider>(maybeDataPaths.value());
  EXPECT_NE(provider, nullptr);

  auto interpolationTester = InterpolationFunctionTester(provider);
  interpolationTester.run();
}

TEST(AdtDataProvider, Mps) {
  // Construct a ADT data provider from the test data path
  const auto dataPathsProvider = AriaDigitalTwinDataPathsProvider(adtTestDataPath);
  const auto maybeDataPaths = dataPathsProvider.getDataPaths();
  EXPECT_TRUE(maybeDataPaths.has_value());

  auto adtProvider = std::make_shared<AriaDigitalTwinDataProvider>(maybeDataPaths.value());
  EXPECT_NE(adtProvider, nullptr);
  auto mpsProvider = adtProvider->mpsDataProviderPtr();
  EXPECT_NE(mpsProvider, nullptr);
  EXPECT_TRUE(mpsProvider->hasGeneralEyeGaze());
  EXPECT_FALSE(mpsProvider->hasPersonalizedEyeGaze());
  EXPECT_FALSE(mpsProvider->hasOpenLoopPoses());
  EXPECT_FALSE(mpsProvider->hasClosedLoopPoses());
  EXPECT_FALSE(mpsProvider->hasOnlineCalibrations());
  EXPECT_FALSE(mpsProvider->hasSemidensePointCloud());
  EXPECT_FALSE(mpsProvider->hasSemidenseObservations());

  auto maybeEyeGaze = mpsProvider->getGeneralEyeGaze(0);
  EXPECT_TRUE(maybeEyeGaze.has_value());
}
