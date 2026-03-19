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

#include <optional>
#include <vector>

#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace projectaria::tools::calibration {

/**
 * @brief Computes the least-squares rigid body transform between two 3D point sets
 * using SVD-based alignment (Umeyama's method, PAMI 1991).
 *
 * Given corresponding point pairs (points_A[i], points_B[i]), finds the rigid transform
 * T_B_A such that points_B[i] ≈ T_B_A * points_A[i] in a least-squares sense.
 *
 * @param points_A Points in coordinate frame A.
 * @param points_B Corresponding points in coordinate frame B. Must be same size as points_A.
 * @return Upon success, SE3 transform T_B_A. Returns std::nullopt if:
 *   - fewer than 3 point pairs are provided
 *   - point sets have different sizes
 *   - points are collinear or degenerate
 */
std::optional<Sophus::SE3d> alignPointCloudsToRigidTransform(
    const std::vector<Eigen::Vector3d>& points_A,
    const std::vector<Eigen::Vector3d>& points_B);

} // namespace projectaria::tools::calibration
