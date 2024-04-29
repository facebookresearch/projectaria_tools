# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List

import numpy as np
from _core_pybinds.calibration import CameraCalibration, DeviceCalibration
from _core_pybinds.mps import (
    ClosedLoopTrajectoryPose,
    EyeGaze,
    get_eyegaze_point_at_depth,
    GlobalPointPosition,
    hand_tracking,
)
from _core_pybinds.sophus import SE3


def bisection_timestamp_search(timed_data, query_timestamp_ns: int) -> int:
    """
    Binary search helper function, assuming that timed_data is sorted by the field names 'tracking_timestamp'
    Returns index of the element closest to the query timestamp else returns None if not found (out of time range)
    """
    # Deal with border case
    if timed_data and len(timed_data) > 1:
        first_timestamp = timed_data[0].tracking_timestamp.total_seconds() * 1e9
        last_timestamp = timed_data[-1].tracking_timestamp.total_seconds() * 1e9
        if query_timestamp_ns <= first_timestamp:
            return None
        elif query_timestamp_ns >= last_timestamp:
            return None
    # If this is safe we perform the Bisection search
    start = 0
    end = len(timed_data) - 1
    while start < end:
        mid = (start + end) // 2
        mid_timestamp = timed_data[mid].tracking_timestamp.total_seconds() * 1e9
        if mid_timestamp == query_timestamp_ns:
            return mid
        if mid_timestamp < query_timestamp_ns:
            start = mid + 1
        else:
            end = mid - 1
    return start


def get_nearest_eye_gaze(eye_gazes: List[EyeGaze], query_timestamp_ns: int) -> EyeGaze:
    """
    Helper function to get nearest eye gaze for a timestamp (ns)
    Return the closest or equal timestamp eye_gaze information that can be found, returns None if not found (out of time range)
    """
    bisection_index = bisection_timestamp_search(eye_gazes, query_timestamp_ns)
    if bisection_index is None:
        return None
    return eye_gazes[bisection_index]


def get_nearest_pose(
    mps_trajectory: List[ClosedLoopTrajectoryPose], query_timestamp_ns: int
) -> ClosedLoopTrajectoryPose:
    """
    Helper function to get nearest pose for a timestamp (ns)
    Return the closest or equal timestamp pose information that can be found, returns None if not found (out of time range)
    """
    bisection_index = bisection_timestamp_search(mps_trajectory, query_timestamp_ns)
    if bisection_index is None:
        return None
    return mps_trajectory[bisection_index]


def get_nearest_wrist_and_palm_pose(
    wirst_and_palm_poses: List[hand_tracking.WristAndPalmPose], query_timestamp_ns: int
) -> hand_tracking.WristAndPalmPose:
    """
    Helper function to get nearest wrist and palm pose for a timestamp (ns)
    Return the closest or equal timestamp wrist and palm pose that can be found, returns None if not found (out of time range)
    """
    bisection_index = bisection_timestamp_search(
        wirst_and_palm_poses, query_timestamp_ns
    )
    if bisection_index is None:
        return None
    return wirst_and_palm_poses[bisection_index]


def filter_points_from_confidence(
    raw_points: List[GlobalPointPosition],
    threshold_invdep: float = 0.001,
    threshold_dep: float = 0.05,
) -> List[GlobalPointPosition]:
    """
    Filter the point cloud by inv depth and depth
    """
    filtered_points = [None] * len(raw_points)
    j = 0
    for pt in raw_points:
        if (
            pt.inverse_distance_std < threshold_invdep
            and pt.distance_std < threshold_dep
        ):
            filtered_points[j] = pt
            j = j + 1
    return filtered_points[:j]


def filter_points_from_count(
    raw_points: List[GlobalPointPosition], max_point_count: int = 500_000
) -> List[GlobalPointPosition]:
    """
    Filter the point cloud by count (random points are sampled from the initial set)
    """
    if len(raw_points) > max_point_count:
        print(
            f"For performance reason we reduce number of points from {len(raw_points)} to {max_point_count}."
        )
        sampled = np.random.choice(len(raw_points), max_point_count, replace=False)
        return (np.stack(raw_points)[sampled]).tolist()
    return raw_points


def get_gaze_vector_reprojection(
    eye_gaze: EyeGaze,
    stream_id_label: str,
    device_calibration: DeviceCalibration,
    camera_calibration: CameraCalibration,
    depth_m: float = 1.0,
    make_upright: bool = False,
) -> np.ndarray:
    """
    Helper function to project a eye gaze output onto a given image and its calibration, assuming specified fixed depth
    """
    gaze_center_in_cpf = get_eyegaze_point_at_depth(
        eye_gaze.yaw, eye_gaze.pitch, depth_m
    )
    # these changes are needed to ensure gaze projections are always using CAD extrinsics
    transform_device_cpf_cad = (
        device_calibration.get_transform_device_cpf()
    )  # this is always CAD
    transform_device_camera_cad = device_calibration.get_transform_device_sensor(
        stream_id_label, True
    )  # this will ensure T_device_camera is CAD value
    # if we want to project on an upright image we will rotate T_device_camera by 90deg cw
    if make_upright:
        transform_camera_cw90 = SE3.from_matrix(
            np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        )
        transform_device_camera_cad = (
            transform_device_camera_cad @ transform_camera_cw90
        )
    transform_camera_cpf_cad = (
        transform_device_camera_cad.inverse() @ transform_device_cpf_cad
    )

    gaze_center_in_camera = transform_camera_cpf_cad @ gaze_center_in_cpf
    gaze_center_in_pixels = camera_calibration.project(gaze_center_in_camera)
    return gaze_center_in_pixels
