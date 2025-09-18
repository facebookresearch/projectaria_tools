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

#include <pangolin/display/image_view.h>
#include <pangolin/pangolin.h>

#include <data_provider/VrsDataProvider.h>
#include <vrs/StreamId.h>

#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace projectaria::tools::viz {

// A helper function to plot EyeGaze data in projected camera image view
void plotProjectedEyeGaze(
    const data_provider::OnDeviceEyeGazeData& eyeGazeData,
    const calibration::CameraCalibration& camCalib,
    const Sophus::SE3d& T_Cpf_Camera,
    const std::string& camLabel);

// A helper function to plot EyeGaze data in 3D
void plotEyeGazeIn3dView(
    const data_provider::OnDeviceEyeGazeData& eyeGazeData,
    const Sophus::SE3d& T_World_Cpf);

// A helper function to plot single hand data in projected camera image view
void plotProjectedSingleHandPose(
    const data_provider::OnDeviceHandPoseData::OneSide& singleHand,
    const calibration::CameraCalibration& camCalib,
    const std::string& camLabel,
    mps::HANDEDNESS handness);

// A helper function to plot HandPose data in projected camera image view
void plotProjectedHandPose(
    const data_provider::OnDeviceHandPoseData& handPoseData,
    const calibration::CameraCalibration& camCalib,
    const std::string& camLabel);

// A helper function to plot HandPose data in 3D
void plotHandPoseIn3dView(
    const data_provider::OnDeviceHandPoseData& handPoseData,
    const Sophus::SE3d& T_World_Device);

// A helper function to plot an Aria glass frame in 3D
void plotAriaGlassOutline(
    const calibration::DeviceCalibration& deviceCalib,
    const Sophus::SE3d& T_World_Device);

} // namespace projectaria::tools::viz
