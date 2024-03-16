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
#include <mps/StaticCameraCalibrationReader.h>

#include <cstdio>
#include <filesystem>
#include <fstream>

using namespace projectaria::tools::mps;

static const std::string StaticCameraCalibrationFileTemplate =
    R"(cam_uid,graph_uid,tx_world_cam,ty_world_cam,tz_world_cam,qx_world_cam,qy_world_cam,qz_world_cam,qw_world_cam,image_width,image_height,intrinsics_type,intrinsics_0,intrinsics_1,intrinsics_2,intrinsics_3,intrinsics_4,intrinsics_5,intrinsics_6,intrinsics_7,start_frame_idx,end_frame_idx,quality
 cam01,40084458-f219-16ca-0c3c-8bd94ee06e6c,1.691723,0.545741,-0.116511,-0.437056,-0.661141,0.501553,0.346871,3840,2160,KANNALABRANDTK3,1761.275757,1761.275757,1920.000000,1080.000000,0.036902,0.057131,-0.057154,0.018458,-1,-1,1)";

TEST(mps_static_camera_calibration, valid_file) {
  const std::string staticCameraCalibrationTemporaryFilename = std::tmpnam(nullptr);
  std::ofstream file(staticCameraCalibrationTemporaryFilename);
  file << StaticCameraCalibrationFileTemplate;
  file.close();
  const auto staticCalibrations =
      readStaticCameraCalibrations(staticCameraCalibrationTemporaryFilename);
  EXPECT_FALSE(staticCalibrations.empty());
  std::filesystem::remove(staticCameraCalibrationTemporaryFilename);
}

TEST(mps_static_camera_calibration, invalid_file) {
  const auto staticCalibrations = readStaticCameraCalibrations("");
  EXPECT_TRUE(staticCalibrations.empty());
}
