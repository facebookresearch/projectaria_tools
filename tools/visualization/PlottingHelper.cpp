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

#include "PlottingHelper.h"

#include <pangolin/gl/gldraw.h>

#include "PangolinColor.h"

using namespace projectaria::tools::mps;
using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::calibration;

namespace projectaria::tools::viz {

const Eigen::Matrix3d kSimpleCam =
    (Eigen::Matrix3d() << 245, 0, 250, 0, 245, 250, 0, 0, 1).finished();
const Eigen::Matrix3d kSimpleCamInv = kSimpleCam.inverse();

const std::vector<std::vector<HandLandmark>> kHandSkeletonOrders = {
    // Palm shape
    {HandLandmark::WRIST,
     HandLandmark::THUMB_INTERMEDIATE,
     HandLandmark::INDEX_PROXIMAL,
     HandLandmark::MIDDLE_PROXIMAL,
     HandLandmark::RING_PROXIMAL,
     HandLandmark::PINKY_PROXIMAL,
     HandLandmark::WRIST,
     HandLandmark::PALM_CENTER},
    // Thumb line
    {HandLandmark::WRIST,
     HandLandmark::THUMB_INTERMEDIATE,
     HandLandmark::THUMB_DISTAL,
     HandLandmark::THUMB_FINGERTIP},
    // Index line
    {HandLandmark::WRIST,
     HandLandmark::INDEX_PROXIMAL,
     HandLandmark::INDEX_INTERMEDIATE,
     HandLandmark::INDEX_DISTAL,
     HandLandmark::INDEX_FINGERTIP},
    // Middle line
    {HandLandmark::WRIST,
     HandLandmark::MIDDLE_PROXIMAL,
     HandLandmark::MIDDLE_INTERMEDIATE,
     HandLandmark::MIDDLE_DISTAL,
     HandLandmark::MIDDLE_FINGERTIP},
    // Ring line
    {HandLandmark::WRIST,
     HandLandmark::RING_PROXIMAL,
     HandLandmark::RING_INTERMEDIATE,
     HandLandmark::RING_DISTAL,
     HandLandmark::RING_FINGERTIP},
    // Pinky line
    {HandLandmark::WRIST,
     HandLandmark::PINKY_PROXIMAL,
     HandLandmark::PINKY_INTERMEDIATE,
     HandLandmark::PINKY_DISTAL,
     HandLandmark::PINKY_FINGERTIP}};

namespace {

// A helper function to draw GL camera as frustum
void drawCamera(int width, int height, const Sophus::SE3d& T_World_Camera, double scale) {
  pangolin::glSetFrameOfReference(T_World_Camera.matrix());
  pangolin::glDrawFrustum(
      kSimpleCamInv(0, 2),
      kSimpleCamInv(1, 2),
      kSimpleCamInv(0, 0),
      kSimpleCamInv(1, 1),
      width,
      height,
      scale);
  pangolin::glUnsetFrameOfReference();
}

// A helper function to generate skeleton segments for a hand, given its landmark names
std::vector<Eigen::Vector2d> createHandSkeleton2DSegmentsFromLandmarks(
    const std::vector<std::optional<Eigen::Vector2d>>& landmarks,
    const std::vector<HandLandmark>& landmarkNameVec) {
  std::vector<Eigen::Vector2d> segments;
  for (const auto& landmarkName : landmarkNameVec) {
    const auto& maybePt = landmarks.at(static_cast<uint8_t>(landmarkName));
    if (maybePt.has_value()) {
      segments.push_back(maybePt.value());
    }
  }
  return segments;
}

std::vector<std::vector<Eigen::Vector2d>> createHandSkeleton2d(
    const std::vector<std::optional<Eigen::Vector2d>>& handmarkers2d) {
  std::vector<std::vector<Eigen::Vector2d>> handSkeleton;

  for (const auto& landmarkNameVec : kHandSkeletonOrders) {
    handSkeleton.push_back(
        createHandSkeleton2DSegmentsFromLandmarks(handmarkers2d, landmarkNameVec));
  }

  return handSkeleton;
}

std::vector<std::vector<Eigen::Vector3d>> createHandSkeleton3d(
    const std::array<Eigen::Vector3d, kNumHandLandmarks>& handMarkers3d) {
  std::vector<std::vector<Eigen::Vector3d>> handSkeleton;
  // Loop over all skeleton segments. In 3D no need to check for std::nullopt
  for (const auto& landmarkNameVec : kHandSkeletonOrders) {
    std::vector<Eigen::Vector3d> segments;
    std::transform(
        landmarkNameVec.begin(),
        landmarkNameVec.end(),
        std::back_inserter(segments),
        [&](const auto& landmarkName) {
          return handMarkers3d.at(static_cast<uint8_t>(landmarkName));
        });
    handSkeleton.push_back(std::move(segments));
  }
  return handSkeleton;
}

/**
 * Draws a sphere at the specified center with the given radius.
 *
 * @param center The center of the sphere.
 * @param radius The radius of the sphere.
 * @param horizontalResolution The number of slices to divide the sphere into horizontally (default:
 * 8).
 * @param verticalResolution The number of stacks to divide the sphere into vertically (default: 8).
 */
void drawSphere(
    const Eigen::Vector3f& center,
    float radius,
    int horizontalResolution = 8,
    int verticalResolution = 8) {
  // Lambda function to uniformly sample points on a unit sphere
  auto sampleUnitSphere = [](float thetaFraction, float phiFraction) {
    // Calculate spherical coordinates
    float theta = M_PI * thetaFraction; // polar angle
    float phi = 2 * M_PI * phiFraction; // azimuthal angle
    // Convert to Cartesian coordinates
    return Eigen::Vector3f(
        std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
  };
  // Set polygon mode to line for wireframe rendering
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  // Draw bottom cap
  glBegin(GL_TRIANGLE_FAN);
  {
    // Center point of the cap
    Eigen::Vector3f normal = sampleUnitSphere(0.0f, 0.0f);
    Eigen::Vector3f vertex = center + normal * radius;
    glNormal3fv(normal.data());
    glVertex3fv(vertex.data());
    // Outer ring of the cap
    for (int indexHorizontal = 0; indexHorizontal <= horizontalResolution; indexHorizontal++) {
      float phiFraction = indexHorizontal / (float)horizontalResolution;
      Eigen::Vector3f currentNormal = sampleUnitSphere(1.0f / verticalResolution, phiFraction);
      Eigen::Vector3f currentVertex = center + normal * radius;
      glNormal3fv(currentNormal.data());
      glVertex3fv(currentVertex.data());
    }
  }
  glEnd();
  // Draw middle sections
  for (int stackIndex = 1; stackIndex < verticalResolution - 1; stackIndex++) {
    glBegin(GL_QUAD_STRIP);
    {
      // Current stack fraction
      float thetaFraction = stackIndex / (float)verticalResolution;
      // Iterate over each slice in the current stack
      for (int indexHorizontal = 0; indexHorizontal <= horizontalResolution; indexHorizontal++) {
        float phiFraction = indexHorizontal / (float)horizontalResolution;
        // Sample two adjacent points on the unit sphere
        Eigen::Vector3f normal0 = sampleUnitSphere(thetaFraction, phiFraction);
        Eigen::Vector3f normal1 =
            sampleUnitSphere(thetaFraction + (1.0f / verticalResolution), phiFraction);
        // Calculate corresponding vertices on the sphere
        Eigen::Vector3f vertex0 = center + normal0 * radius;
        Eigen::Vector3f vertex1 = center + normal1 * radius;
        // Emit vertices and normals
        glNormal3fv(normal0.data());
        glVertex3fv(vertex0.data());
        glNormal3fv(normal1.data());
        glVertex3fv(vertex1.data());
      }
    }
    glEnd();
  }
  // Draw top cap
  glBegin(GL_TRIANGLE_FAN);
  {
    // Center point of the cap
    Eigen::Vector3f normal = sampleUnitSphere(1.0f, 0.0f);
    Eigen::Vector3f vertex = center + normal * radius;
    glNormal3fv(normal.data());
    glVertex3fv(vertex.data());
    // Outer ring of the cap
    for (int indexHorizontal = 0; indexHorizontal <= horizontalResolution; indexHorizontal++) {
      float phiFraction = indexHorizontal / (float)horizontalResolution;
      Eigen::Vector3f currentNormal =
          sampleUnitSphere(1.0f - (1.0f / verticalResolution), phiFraction);
      Eigen::Vector3f currentVertex = center + normal * radius;
      glNormal3fv(currentNormal.data());
      glVertex3fv(currentVertex.data());
    }
  }
  glEnd();
  // Reset polygon mode to fill
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

} // namespace

void plotProjectedEyeGaze(
    const data_provider::OnDeviceEyeGazeData& eyeGazeData,
    const calibration::CameraCalibration& camCalib,
    const Sophus::SE3d& T_Cpf_Camera,
    const std::string& camLabel) {
  // Project spatial gaze point into the current camera image
  const Eigen::Vector3d spatialGazePointInCpf =
      eyeGazeData.spatial_gaze_point_in_cpf.cast<double>();
  const auto maybePixelLocation = camCalib.project(T_Cpf_Camera.inverse() * spatialGazePointInCpf);
  // Skip if projection is invalid
  if (!maybePixelLocation.has_value()) {
    return;
  }

  // Render eyegaze point on image view
  // Set plotting ratio, which should be higher for RGB camera due to its higher resolution
  double eyegazeMarkerSize = camLabel == "camera-rgb" ? 25.0 : 8.0;
  setPlotColor("orchid", 0.8);
  pangolin::glDrawCircle(maybePixelLocation.value(), eyegazeMarkerSize);
}

void plotEyeGazeIn3dView(
    const data_provider::OnDeviceEyeGazeData& eyeGazeData,
    const Sophus::SE3d& T_World_Cpf) {
  pangolin::glSetFrameOfReference(T_World_Cpf.matrix());

  if (eyeGazeData.combined_gaze_valid) {
    // Plot combined gaze origin
    const Eigen::Vector3d combinedGazeOriginCpf =
        eyeGazeData.combined_gaze_origin_in_cpf.cast<double>();
    Eigen::Vector3d boxSize(3e-3, 3e-3, 3e-3);

    Eigen::AlignedBox3d gazeOriginBox(
        combinedGazeOriginCpf - boxSize, combinedGazeOriginCpf + boxSize);

    setPlotColor("medium_slate_blue");
    pangolin::glDrawAlignedBox(gazeOriginBox);

    // Plot combined gaze direction
    Eigen::Vector3d combinedGazeVector =
        getUnitVectorFromYawPitch(eyeGazeData.yaw, eyeGazeData.pitch);
    Eigen::Vector3d gazeEndPoint = combinedGazeOriginCpf + combinedGazeVector * eyeGazeData.depth;

    setPlotColor("pale_green");
    glLineWidth(2.0);
    pangolin::glDrawLineStrip<double, 3>(
        std::vector<Eigen::Vector3d>{combinedGazeOriginCpf, gazeEndPoint});
  }

  if (eyeGazeData.spatial_gaze_point_valid) {
    // Plot spatial gaze point
    const Eigen::Vector3d spatialGaze = eyeGazeData.spatial_gaze_point_in_cpf.cast<double>();
    Eigen::Vector3d boxSize(3e-3, 3e-3, 3e-3);

    Eigen::AlignedBox3d spatialGazePointBox(spatialGaze - boxSize, spatialGaze + boxSize);

    setPlotColor("orchid");
    pangolin::glDrawAlignedBox(spatialGazePointBox);
  }

  pangolin::glUnsetFrameOfReference();
}

void plotProjectedSingleHandPose(
    const OnDeviceHandPoseData::OneSide& singleHand,
    const CameraCalibration& camCalib,
    const std::string& camLabel,
    HANDEDNESS handedness) {
  // Project hand markers into current camera image
  std::vector<std::optional<Eigen::Vector2d>> projectedHandMarkers;
  for (const auto& markerInDevice : singleHand.landmarkPositions_device) {
    const std::optional<Eigen::Vector2d> maybeProjectedMarker =
        camCalib.project(camCalib.getT_Device_Camera().inverse() * markerInDevice);
    projectedHandMarkers.push_back(maybeProjectedMarker);
  }

  // Create hand skeleton
  const auto handSkeleton = createHandSkeleton2d(projectedHandMarkers);

  // Plot hand markers
  double handMarkerPlotSize = camLabel == "camera-rgb" ? 9.0 : 4.0;
  if (handedness == HANDEDNESS::LEFT) {
    setPlotColor("orange_red", 0.7);
  } else {
    setPlotColor("yellow", 0.7);
  }
  for (const auto& maybeMarkerInCamera : projectedHandMarkers) {
    if (maybeMarkerInCamera.has_value()) {
      pangolin::glDrawCircle(maybeMarkerInCamera.value(), handMarkerPlotSize);
    }
  }

  // Plot hand skeleton
  setPlotColor("green", 0.8);
  for (const auto& handSkeletonSegment : handSkeleton) {
    pangolin::glDrawLineStrip(handSkeletonSegment);
  }
}

void plotProjectedHandPose(
    const OnDeviceHandPoseData& handPoseData,
    const CameraCalibration& camCalib,
    const std::string& camLabel) {
  if (handPoseData.leftHand.has_value()) {
    plotProjectedSingleHandPose(
        handPoseData.leftHand.value(), camCalib, camLabel, HANDEDNESS::LEFT);
  }

  if (handPoseData.rightHand.has_value()) {
    plotProjectedSingleHandPose(
        handPoseData.rightHand.value(), camCalib, camLabel, HANDEDNESS::RIGHT);
  }
}

void plotSingleHandPoseIn3dView(
    const OnDeviceHandPoseData::OneSide& singleHand,
    const Sophus::SE3d& T_World_Device,
    HANDEDNESS handedness) {
  // Create hand skeleton from hand markers
  const Landmarks& handMarkersInDevice = singleHand.landmarkPositions_device;
  const auto handSkeletonInDevice = createHandSkeleton3d(handMarkersInDevice);

  // Draw hand markers
  pangolin::glSetFrameOfReference(T_World_Device.matrix());

  switch (handedness) {
    case HANDEDNESS::LEFT:
      setPlotColor("orange_red");
      break;
    case HANDEDNESS::RIGHT:
      setPlotColor("yellow");
      break;
  }
  for (const auto& handMarker : handMarkersInDevice) {
    drawSphere(handMarker.cast<float>(), 5e-3);
  }

  // Draw hand skeleton)
  setPlotColor("green");
  glLineWidth(1.5);
  for (const auto& skeletonSegment : handSkeletonInDevice) {
    pangolin::glDrawLineStrip(skeletonSegment);
  }

  pangolin::glUnsetFrameOfReference();
}

void plotHandPoseIn3dView(
    const data_provider::OnDeviceHandPoseData& handPoseData,
    const Sophus::SE3d& T_World_Device) {
  if (handPoseData.leftHand.has_value()) {
    plotSingleHandPoseIn3dView(handPoseData.leftHand.value(), T_World_Device, HANDEDNESS::LEFT);
  }

  if (handPoseData.rightHand.has_value()) {
    plotSingleHandPoseIn3dView(handPoseData.rightHand.value(), T_World_Device, HANDEDNESS::RIGHT);
  }
}

void plotAriaGlassOutline(
    const calibration::DeviceCalibration& deviceCalib,
    const Sophus::SE3d& T_World_Device) {
  // Find the sensor labels at left and right glass corners
  const auto deviceVersion = deviceCalib.getDeviceVersion();
  std::string leftCornerLabel;
  std::string rightCornerLabel;
  std::vector<std::string> allSlamLabels;
  switch (deviceVersion) {
    case DeviceVersion::Gen1:
      leftCornerLabel = "camera-slam-left";
      rightCornerLabel = "camera-slam-right";
      allSlamLabels = {"camera-slam-left", "camera-slam-right"};
      break;
    case DeviceVersion::Gen2:
      leftCornerLabel = "slam-front-left";
      rightCornerLabel = "slam-front-right";
      allSlamLabels = {"slam-front-left", "slam-front-right", "slam-side-left", "slam-side-right"};
      break;
    default:
      throw std::runtime_error(
          fmt::format("Unsupported device version: {}", getName(deviceVersion)));
  }

  const std::vector<std::string> glassOutlineSensors = {
      "mic5",
      leftCornerLabel,
      "mic2",
      "mic1",
      "baro0",
      "mic1",
      leftCornerLabel,
      rightCornerLabel,
      "mic0",
      "baro0",
      rightCornerLabel,
      "mic6",
  };

  std::vector<Eigen::Vector3d> glassOutline;
  for (const auto& sensorLabel : glassOutlineSensors) {
    // For mic, mag, and baro, use CAD values because their factory calibration does not contain
    // extrinsics
    bool useCadValue = false;
    if (sensorLabel.find("mic") != std::string::npos ||
        sensorLabel.find("mag") != std::string::npos ||
        sensorLabel.find("baro") != std::string::npos) {
      useCadValue = true;
    }
    const auto maybeT_Device_Sensor = deviceCalib.getT_Device_Sensor(sensorLabel, useCadValue);

    if (!maybeT_Device_Sensor.has_value()) {
      throw std::runtime_error(fmt::format("Cannot find sensor label: {}", sensorLabel));
    }
    glassOutline.push_back(maybeT_Device_Sensor.value().translation());
  }

  pangolin::glSetFrameOfReference(T_World_Device.matrix());
  pangolin::glDrawAxis(0.05);
  setPlotColor("beige");
  pangolin::glDrawLineStrip(glassOutline);

  // Draw slam and RGB cameras
  setPlotColor("forest_green");
  for (const auto& slamLabel : allSlamLabels) {
    const auto& maybeT_Device_Cam = deviceCalib.getT_Device_Sensor(slamLabel);
    if (maybeT_Device_Cam.has_value()) {
      drawCamera(512, 512, maybeT_Device_Cam.value(), 5e-3);
    }
  }
  setPlotColor("plum");
  const auto& maybeT_Device_Cam = deviceCalib.getT_Device_Sensor("camera-rgb");
  if (maybeT_Device_Cam.has_value()) {
    drawCamera(512, 512, maybeT_Device_Cam.value(), 1.5e-2);
  }

  pangolin::glUnsetFrameOfReference();
}

} // namespace projectaria::tools::viz
