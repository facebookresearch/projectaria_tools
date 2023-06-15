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

#include "StaticCameraCalibrationReader.h"

#include <logging/Checks.h>

#ifndef CSV_IO_NO_THREAD
#define CSV_IO_NO_THREAD
#endif
#include <fast-cpp-csv-parser/csv.h>

#include <iostream>

namespace projectaria::tools::mps {

constexpr std::array<const char*, 22> kStaticCameraCalibrationHeader = {
    "cam_uid",         "graph_uid",       "tx_world_cam", "ty_world_cam", "tz_world_cam",
    "qx_world_cam",    "qy_world_cam",    "qz_world_cam", "qw_world_cam", "image_width",
    "image_height",    "intrinsics_type", "intrinsics_0", "intrinsics_1", "intrinsics_2",
    "intrinsics_3",    "intrinsics_4",    "intrinsics_5", "intrinsics_6", "intrinsics_7",
    "start_frame_idx", "end_frame_idx",
};

StaticCameraCalibrations readStaticCameraCalibrations(const std::string& fileName) {
  StaticCameraCalibrations poses;

  io::CSVReader<kStaticCameraCalibrationHeader.size()> csv(fileName);
  // Read in the CSV header
  // allow extra column for future-proof forward compatibility
  const auto readHeader = [&](auto&&... args) {
    csv.read_header(io::ignore_extra_column, args...);
  };
  std::apply(readHeader, kStaticCameraCalibrationHeader);

  std::string cam_uid;
  std::string graph_uid;
  Eigen::Vector3d t_world_cam;
  Eigen::Quaterniond q_world_cam;
  std::string intrinsics_type;
  int width, height;
  Eigen::Matrix<float, 8, 1> intrinsics;
  int start_frame_idx, end_frame_idx;

  while (csv.read_row(
      cam_uid,
      graph_uid,
      t_world_cam.x(),
      t_world_cam.y(),
      t_world_cam.z(),
      q_world_cam.x(),
      q_world_cam.y(),
      q_world_cam.z(),
      q_world_cam.w(),
      width,
      height,
      intrinsics_type,
      intrinsics[0],
      intrinsics[1],
      intrinsics[2],
      intrinsics[3],
      intrinsics[4],
      intrinsics[5],
      intrinsics[6],
      intrinsics[7],
      start_frame_idx,
      end_frame_idx)) {
    auto& pose = poses.emplace_back();
    pose.cameraUid = cam_uid;
    pose.graphUid = graph_uid;
    pose.T_world_cam = Sophus::SE3d(q_world_cam, t_world_cam);
    pose.width = width;
    pose.height = height;

    XR_CHECK_EQ(intrinsics_type, "KANNALABRANDTK3", "only KB3 type is supported today");
    pose.intrinsicsType = intrinsics_type;
    pose.intrinsics = intrinsics;

    XR_CHECK(
        (start_frame_idx == -1 && start_frame_idx == -1) ||
            (start_frame_idx >= 0 && end_frame_idx >= 0 && start_frame_idx <= end_frame_idx),
        "start and end frame indices are invalid");
    if (start_frame_idx >= 0) {
      pose.startFrameIdx = start_frame_idx;
    }
    if (end_frame_idx >= 0) {
      pose.endFrameIdx = end_frame_idx;
    }
  }

  std::cout << "Loaded #StaticCameraCalibration data: " << poses.size() << std::endl;

  return poses;
}

} // namespace projectaria::tools::mps
