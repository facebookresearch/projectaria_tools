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

from typing import Callable, List, Optional, Tuple, Union

import numpy as np

import rerun as rr

from projectaria_tools.core import calibration, data_provider, mps
from projectaria_tools.core.calibration import (
    CameraCalibration,
    DeviceCalibration,
    DeviceVersion,
)
from projectaria_tools.core.mps.utils import (
    filter_points_from_confidence,
    filter_points_from_count,
    get_gaze_vector_reprojection,
    get_nearest_eye_gaze,
    get_nearest_hand_tracking_result,
    get_nearest_pose,
    get_nearest_wrist_and_palm_pose,
)
from projectaria_tools.core.sensor_data import SensorData, SensorDataType, TimeDomain
from projectaria_tools.core.sophus import SE3
from projectaria_tools.core.stream_id import StreamId
from projectaria_tools.utils.rerun_helpers import (
    AriaGlassesOutline,
    create_hand_skeleton_from_landmarks,
    ToTransform3D,
)
from tqdm import tqdm


WRIST_PALM_TIME_DIFFERENCE_THRESHOLD_NS: int = 2e8
WRIST_PALM_COLOR: List[int] = [255, 64, 0]
NORMAL_VIS_LEN = 0.03  # in meters
NORMAL_VIS_LEN_2D = 120.0  # in pixels
HAND_LANDMARK_COLORS: Tuple[List[int]] = ([255, 165, 0], [255, 255, 0])


def get_camera_projection_from_device_point(
    point: np.ndarray, camera_calibration: CameraCalibration
) -> Optional[np.ndarray]:
    T_device_camera = camera_calibration.get_transform_device_camera()
    return camera_calibration.project(T_device_camera.inverse() @ point)


def log_device_trajectory(trajectory_files: List[str]) -> None:
    #
    # Log device trajectory (reduce sample count for display)
    #
    print("Loading and logging trajectory(ies)...")
    trajectory_list_size = len(trajectory_files)
    i = 0
    for trajectory_file in trajectory_files:
        print(f"Loading: {trajectory_file}")
        # First try to load as closed loop trajectory
        trajectory_data = mps.read_closed_loop_trajectory(trajectory_file)
        device_trajectory = [
            it.transform_world_device.translation()[0] for it in trajectory_data
        ][0::80]

        # If not a closed loop trajectory, try to load as open loop trajectory
        if len(device_trajectory) == 0:
            print(
                "Cannot load a closed loop trajectory. Now trying to load as open loop trajectory"
            )
            trajectory_data = mps.read_open_loop_trajectory(trajectory_file)
            device_trajectory = [
                it.transform_odometry_device.translation()[0] for it in trajectory_data
            ][0::80]

        entity_path = (
            "world/device_trajectory"
            if trajectory_list_size == 1
            else f"world/device_trajectory_{i}"
        )
        rr.log(
            entity_path,
            rr.LineStrips3D(device_trajectory, radii=0.008),
            static=True,
        )
        print(f"Showing: {trajectory_file} as {entity_path}")
        i += 1


def log_point_clouds(points_files: List[str]) -> None:
    #
    # Log Point Cloud(s) (reduce point count for display)
    #
    print("Loading and logging point cloud(s)...")
    point_cloud_list_size = len(points_files)
    i = 0
    for points_file in points_files:
        points_data = mps.read_global_point_cloud(points_file)
        # Filter out low confidence points
        points_data = filter_points_from_confidence(points_data)
        # Down sample points
        points_data_down_sampled = filter_points_from_count(
            points_data, 500_000 if point_cloud_list_size == 1 else 20_000
        )
        # Retrieve point position
        point_positions = [it.position_world for it in points_data_down_sampled]

        entity_path = (
            "world/points" if point_cloud_list_size == 1 else f"world/points_{i}"
        )
        rr.log(
            entity_path,
            rr.Points3D(point_positions, radii=0.006),
            static=True,
        )
        print(f"Showing: {points_file} as {entity_path}")
        i += 1


def log_RGB_camera_calibration(
    rgb_camera_calibration: CameraCalibration,
    rgb_stream_label: str,
    down_sampling_factor: int,
) -> None:
    #
    # Log RGB camera calibration
    #
    rr.log(
        f"world/device/{rgb_stream_label}",
        rr.Pinhole(
            resolution=[
                rgb_camera_calibration.get_image_size()[0] / down_sampling_factor,
                rgb_camera_calibration.get_image_size()[1] / down_sampling_factor,
            ],
            focal_length=float(
                rgb_camera_calibration.get_focal_lengths()[0] / down_sampling_factor
            ),
        ),
        static=True,
    )


def log_Aria_glasses_outline(
    device_calibration: DeviceCalibration,
) -> None:
    #
    # Log Aria Glasses outline
    #
    aria_glasses_point_outline = AriaGlassesOutline(device_calibration)
    rr.log(
        "world/device/glasses_outline",
        rr.LineStrips3D([aria_glasses_point_outline]),
        static=True,
    )


def log_camera_pose(
    trajectory_data: List[
        Union[mps.ClosedLoopTrajectoryPose, mps.OpenLoopTrajectoryPose]
    ],
    device_time_ns: int,
    rgb_camera_calibration: CameraCalibration,
    rgb_stream_label: str,
) -> None:
    # Camera pose
    if trajectory_data:
        pose_info = get_nearest_pose(trajectory_data, device_time_ns)
        if pose_info:
            if isinstance(pose_info, mps.OpenLoopTrajectoryPose):
                T_world_device = pose_info.transform_odometry_device
            else:
                T_world_device = pose_info.transform_world_device
            T_device_camera = rgb_camera_calibration.get_transform_device_camera()
            rr.log(
                "world/device",
                ToTransform3D(T_world_device, False),
            )
            rr.log(
                f"world/device/{rgb_stream_label}",
                ToTransform3D(T_device_camera, False),
            )


def log_RGB_image(
    data: SensorData,
    down_sampling_factor: int,
    jpeg_quality: int,
    rgb_stream_label: str,
    postprocess_image: Callable[[np.ndarray], np.ndarray] = lambda img: img,
) -> None:
    if data.sensor_data_type() == SensorDataType.IMAGE:
        img = data.image_data_and_record()[0].to_numpy_array()
        img = postprocess_image(img)
        if down_sampling_factor > 1:
            img = img[::down_sampling_factor, ::down_sampling_factor]

        # Note: We configure the QUEUE to return only RGB image, so we are sure this image is corresponding to a RGB frame
        rr.log(
            f"world/device/{rgb_stream_label}",
            rr.Image(img).compress(jpeg_quality=jpeg_quality),
        )


def log_eye_gaze(
    eyegaze_data: List[mps.EyeGaze],
    device_time_ns: int,
    T_device_CPF: SE3,
    rgb_stream_label: str,
    device_calibration: DeviceCalibration,
    rgb_camera_calibration: CameraCalibration,
    down_sampling_factor: int,
    make_upright: bool = False,
) -> None:
    #
    # Eye Gaze (vector and image reprojection)
    #
    logged_eyegaze: bool = False
    if eyegaze_data:
        eye_gaze = get_nearest_eye_gaze(eyegaze_data, device_time_ns)
        if eye_gaze:
            # If depth available use it, else use a proxy (1 meter depth along the EyeGaze ray)
            depth_m = eye_gaze.depth or 1.0
            gaze_vector_in_cpf = mps.get_eyegaze_point_at_depth(
                eye_gaze.yaw, eye_gaze.pitch, depth_m
            )
            # Move EyeGaze vector to CPF coordinate system for visualization
            rr.log(
                "world/device/eye-gaze",
                rr.Arrows3D(
                    origins=[T_device_CPF @ [0, 0, 0]],
                    vectors=[T_device_CPF @ gaze_vector_in_cpf],
                    colors=[[255, 0, 255]],
                ),
            )
            gaze_projection = get_gaze_vector_reprojection(
                eye_gaze=eye_gaze,
                stream_id_label=rgb_stream_label,
                device_calibration=device_calibration,
                camera_calibration=rgb_camera_calibration,
                depth_m=depth_m,
                make_upright=make_upright,
            )
            if gaze_projection is not None:
                rr.log(
                    f"world/device/{rgb_stream_label}/eye-gaze_projection",
                    rr.Points2D(
                        gaze_projection / down_sampling_factor,
                        radii=4,
                    ),
                )
                logged_eyegaze = True
            # Else (eye gaze projection is outside the image or behind the image plane)
    if not logged_eyegaze:
        rr.log("world/device/eye-gaze", rr.Clear.flat())
        rr.log(
            f"world/device/{rgb_stream_label}/eye-gaze_projection",
            rr.Clear.flat(),
        )


def log_hand_tracking(
    wrist_and_palm_poses: List[mps.hand_tracking.WristAndPalmPose],
    device_time_ns: int,
    rgb_camera_calibration: CameraCalibration,
    rgb_stream_label: str,
    down_sampling_factor: int,
) -> None:
    #
    # Hand Tracking (wrist and palm 3D pose and image projections)
    #
    logged_hand_tracking_2D_points: bool = False
    logged_hand_tracking_2D_links: bool = False
    logged_hand_tracking_3D: bool = False
    logged_hand_tracking_2D_normal_links: bool = False
    if wrist_and_palm_poses:
        wrist_and_palm_pose = get_nearest_wrist_and_palm_pose(
            wrist_and_palm_poses, device_time_ns
        )
        wrist_points: List[np.array] = []
        palm_points: List[np.array] = []
        wrist_normals: List[np.array] = []
        palm_normals: List[np.array] = []
        if (
            wrist_and_palm_pose
            and np.abs(
                wrist_and_palm_pose.tracking_timestamp.total_seconds() * 1e9
                - device_time_ns
            )
            < WRIST_PALM_TIME_DIFFERENCE_THRESHOLD_NS
        ):
            for one_side_pose in [
                wrist_and_palm_pose.right_hand,
                wrist_and_palm_pose.left_hand,
            ]:
                if one_side_pose and one_side_pose.confidence > 0:
                    wrist_points.append(one_side_pose.wrist_position_device)
                    palm_points.append(one_side_pose.palm_position_device)

                    # Then collect optional wrist and palm normals
                    if one_side_pose.wrist_and_palm_normal_device is not None:
                        wrist_normals.append(
                            one_side_pose.wrist_and_palm_normal_device.wrist_normal_device
                        )
                        palm_normals.append(
                            one_side_pose.wrist_and_palm_normal_device.palm_normal_device
                        )
        if wrist_points and palm_points:
            # Log wrist and palm 3D points
            rr.log(
                "world/device/wrist-and-palm/points",
                rr.Points3D(
                    np.concatenate([wrist_points, palm_points]),
                    radii=0.01,
                    colors=[WRIST_PALM_COLOR],
                ),
            )
            rr.log(
                "world/device/wrist-and-palm/links",
                rr.LineStrips3D(
                    np.stack([wrist_points, palm_points], axis=1),
                    colors=[WRIST_PALM_COLOR],
                ),
            )
            logged_hand_tracking_3D = True

        if wrist_normals:
            # Log wrist 3D normals
            rr.log(
                "world/device/wrist-and-palm/wrist_normals",
                rr.Arrows3D(
                    origins=wrist_points,
                    vectors=np.asarray(wrist_normals) * NORMAL_VIS_LEN,
                    colors=[WRIST_PALM_COLOR],
                ),
            )

        if palm_normals:
            # Log palm 3D normals
            rr.log(
                "world/device/wrist-and-palm/palm_normals",
                rr.Arrows3D(
                    origins=palm_points,
                    vectors=np.asarray(palm_normals) * NORMAL_VIS_LEN,
                    colors=[WRIST_PALM_COLOR],
                ),
            )

        # Log wrist and palm 3D point projections on the image
        wrist_pixels = [
            get_camera_projection_from_device_point(wrist_point, rgb_camera_calibration)
            for wrist_point in wrist_points
        ]
        palm_pixels = [
            get_camera_projection_from_device_point(palm_point, rgb_camera_calibration)
            for palm_point in palm_points
        ]
        wrist_and_palm_points_2d = []
        wrist_and_palm_line_strips_2d = []
        for wrist_pixel, palm_pixel in zip(wrist_pixels, palm_pixels):
            wrist_and_palm_points_2d += [
                p / down_sampling_factor
                for p in [wrist_pixel, palm_pixel]
                if p is not None
            ]
            if wrist_pixel is not None and palm_pixel is not None:
                wrist_and_palm_line_strips_2d += [
                    [p / down_sampling_factor for p in [wrist_pixel, palm_pixel]]
                ]

        if wrist_and_palm_points_2d:
            rr.log(
                f"world/device/{rgb_stream_label}/wrist-and-palm_projection/points",
                rr.Points2D(
                    wrist_and_palm_points_2d,
                    radii=4,
                    colors=[WRIST_PALM_COLOR],
                ),
            )
            logged_hand_tracking_2D_points = True

        if wrist_and_palm_line_strips_2d:
            rr.log(
                f"world/device/{rgb_stream_label}/wrist-and-palm_projection/link",
                rr.LineStrips2D(
                    wrist_and_palm_line_strips_2d,
                    colors=[WRIST_PALM_COLOR],
                ),
            )
            logged_hand_tracking_2D_links = True

        # Log wrist and palm 3D normals projections on the image
        if len(wrist_normals) > 0 and len(palm_normals) > 0:
            wrist_normal_tip_pixels = [
                get_camera_projection_from_device_point(
                    wrist_point + wrist_normal * NORMAL_VIS_LEN, rgb_camera_calibration
                )
                for wrist_point, wrist_normal in zip(wrist_points, wrist_normals)
            ]
            palm_normal_tip_pixels = [
                get_camera_projection_from_device_point(
                    palm_point + palm_normal * NORMAL_VIS_LEN, rgb_camera_calibration
                )
                for palm_point, palm_normal in zip(palm_points, palm_normals)
            ]
            wrist_and_palm_normal_arrows_2d = []
            for hand_i, (wrist_pixel, palm_pixel) in enumerate(
                zip(wrist_pixels, palm_pixels)
            ):
                # normal tip vector projected to image can be used as arrow direction for the vis
                if (
                    wrist_normal_tip_pixels[hand_i] is not None
                    and wrist_pixel is not None
                ):
                    wrist_normal_in_2d = wrist_normal_tip_pixels[hand_i] - wrist_pixel
                    wrist_normal_in_2d /= np.linalg.norm(wrist_normal_in_2d)
                    wrist_normal_in_2d *= NORMAL_VIS_LEN_2D / down_sampling_factor
                    wrist_and_palm_normal_arrows_2d.append(wrist_normal_in_2d)
                if (
                    palm_normal_tip_pixels[hand_i] is not None
                    and palm_pixel is not None
                ):
                    palm_normal_in_2d = palm_normal_tip_pixels[hand_i] - palm_pixel
                    palm_normal_in_2d /= np.linalg.norm(palm_normal_in_2d)
                    palm_normal_in_2d *= NORMAL_VIS_LEN_2D / down_sampling_factor
                    wrist_and_palm_normal_arrows_2d.append(palm_normal_in_2d)

            if wrist_and_palm_normal_arrows_2d:
                rr.log(
                    f"world/device/{rgb_stream_label}/wrist-and-palm_projection/normals",
                    rr.Arrows2D(
                        origins=wrist_and_palm_points_2d,
                        vectors=wrist_and_palm_normal_arrows_2d,
                        colors=[WRIST_PALM_COLOR],
                    ),
                )
                logged_hand_tracking_2D_normal_links = True

    if not logged_hand_tracking_3D:
        rr.log("world/device/wrist-and-palm", rr.Clear.recursive())

    if not logged_hand_tracking_2D_points:
        # If no points were found, recursively clear the 2D projections
        rr.log(
            f"world/device/{rgb_stream_label}/wrist-and-palm_projection",
            rr.Clear.recursive(),
        )
    else:
        if not logged_hand_tracking_2D_links:
            # If only the points links are missing, clear the links
            rr.log(
                f"world/device/{rgb_stream_label}/wrist-and-palm_projection/link",
                rr.Clear.recursive(),
            )
        if not logged_hand_tracking_2D_normal_links:
            # If only the normal links are missing, clear the links
            rr.log(
                f"world/device/{rgb_stream_label}/wrist-and-palm_projection/normals",
                rr.Clear.recursive(),
            )


def log_single_hand_landmarks(
    hand_landmarks_in_device: List[np.array],
    hand_label: str,
    rgb_camera_calibration: CameraCalibration,
    rgb_stream_label: str,
    down_sampling_factor: int,
) -> None:
    handedness = 0 if hand_label == "left" else 1
    ################# Plot single hand in 3D views #################
    # Plot 3D hand markers and skeleton
    hand_skeleton_3d = create_hand_skeleton_from_landmarks(hand_landmarks_in_device)
    rr.log(
        f"world/device/landmarks/points_{hand_label}",
        rr.Points3D(
            positions=hand_landmarks_in_device,
            colors=[[HAND_LANDMARK_COLORS[handedness]]],
            radii=5e-3,
        ),
    )
    rr.log(
        f"world/device/landmarks/landmark_links_{hand_label}",
        rr.LineStrips3D(
            hand_skeleton_3d,
            colors=[[HAND_LANDMARK_COLORS[handedness]]],
        ),
    )

    ################# Plot single hand in 2D camera views #################
    # project into camera frame, and also create line segments
    hand_landmarks_in_camera = []
    for pt_in_device in hand_landmarks_in_device:
        pt_in_camera = (
            rgb_camera_calibration.get_transform_device_camera().inverse()
            @ pt_in_device
        )
        pixel = rgb_camera_calibration.project(pt_in_camera)
        hand_landmarks_in_camera.append(
            (pixel / down_sampling_factor) if (pixel is not None) else None
        )

    # Create hand skeleton in 2D image space
    hand_skeleton = create_hand_skeleton_from_landmarks(hand_landmarks_in_camera)

    # Remove "None" markers from hand joints in camera. This is intentionally done AFTER the hand skeleton creation
    hand_landmarks_in_camera = list(
        filter(lambda x: x is not None, hand_landmarks_in_camera)
    )

    rr.log(
        f"world/device/{rgb_stream_label}/landmarks_projection/points_{hand_label}",
        rr.Points2D(
            hand_landmarks_in_camera,
            radii=2,
            colors=[HAND_LANDMARK_COLORS[handedness]],
        ),
    )
    rr.log(
        f"world/device/{rgb_stream_label}/landmarks_projection/link_{hand_label}",
        rr.LineStrips2D(
            hand_skeleton,
            colors=[HAND_LANDMARK_COLORS[handedness]],
        ),
    )


def log_wrist_and_palm(
    wrist_points: List[np.array],
    palm_points: List[np.array],
    wrist_normals: List[np.array],
    palm_normals: List[np.array],
    rgb_camera_calibration: CameraCalibration,
    rgb_stream_label: str,
    down_sampling_factor: int,
) -> None:
    ## Draw wrist and palm 3D points we collected from both hands above.
    if wrist_points and palm_points:
        # Log wrist and palm 3D points
        rr.log(
            "world/device/wrist-and-palm/points",
            rr.Points3D(
                np.concatenate([wrist_points, palm_points]),
                radii=0.005,
                colors=[WRIST_PALM_COLOR],
            ),
        )
        rr.log(
            "world/device/wrist-and-palm/links",
            rr.LineStrips3D(
                np.stack([wrist_points, palm_points], axis=1),
                colors=[WRIST_PALM_COLOR],
            ),
        )
    ## Draw wrist and palm normals we collected from both hands above.
    if wrist_normals:
        rr.log(
            "world/device/wrist-and-palm/wrist_normals",
            rr.Arrows3D(
                origins=wrist_points,
                vectors=np.asarray(wrist_normals) * NORMAL_VIS_LEN,
                colors=[WRIST_PALM_COLOR],
            ),
        )
    if palm_normals:
        rr.log(
            "world/device/wrist-and-palm/palm_normals",
            rr.Arrows3D(
                origins=palm_points,
                vectors=np.asarray(palm_normals) * NORMAL_VIS_LEN,
                colors=[WRIST_PALM_COLOR],
            ),
        )

    # Log wrist and palm landmarks projected on the image
    wrist_pixels = [
        get_camera_projection_from_device_point(wrist_point, rgb_camera_calibration)
        for wrist_point in wrist_points
    ]
    palm_pixels = [
        get_camera_projection_from_device_point(palm_point, rgb_camera_calibration)
        for palm_point in palm_points
    ]
    wrist_and_palm_points_2d = []
    wrist_and_palm_line_strips_2d = []
    for wrist_pixel, palm_pixel in zip(wrist_pixels, palm_pixels):
        wrist_and_palm_points_2d += [
            p / down_sampling_factor for p in [wrist_pixel, palm_pixel] if p is not None
        ]
        if wrist_pixel is not None and palm_pixel is not None:
            wrist_and_palm_line_strips_2d += [
                [p / down_sampling_factor for p in [wrist_pixel, palm_pixel]]
            ]

    if wrist_and_palm_points_2d:
        rr.log(
            f"world/device/{rgb_stream_label}/wrist-and-palm_projection/points",
            rr.Points2D(
                wrist_and_palm_points_2d,
                radii=2,
                colors=[WRIST_PALM_COLOR],
            ),
        )
    if wrist_and_palm_line_strips_2d:
        rr.log(
            f"world/device/{rgb_stream_label}/wrist-and-palm_projection/link",
            rr.LineStrips2D(
                wrist_and_palm_line_strips_2d,
                colors=[WRIST_PALM_COLOR],
            ),
        )

    # Log wrist and palm normals projected on the image
    if len(wrist_normals) > 0 and len(palm_normals) > 0:
        wrist_normal_tip_pixels = [
            get_camera_projection_from_device_point(
                wrist_point + wrist_normal * NORMAL_VIS_LEN, rgb_camera_calibration
            )
            for wrist_point, wrist_normal in zip(wrist_points, wrist_normals)
        ]
        palm_normal_tip_pixels = [
            get_camera_projection_from_device_point(
                palm_point + palm_normal * NORMAL_VIS_LEN, rgb_camera_calibration
            )
            for palm_point, palm_normal in zip(palm_points, palm_normals)
        ]
        wrist_and_palm_normal_arrows_2d = []
        for hand_i, (wrist_pixel, palm_pixel) in enumerate(
            zip(wrist_pixels, palm_pixels)
        ):
            # normal tip vector projected to image can be used as arrow direction for the vis
            if wrist_normal_tip_pixels[hand_i] is not None and wrist_pixel is not None:
                wrist_normal_in_2d = wrist_normal_tip_pixels[hand_i] - wrist_pixel
                wrist_normal_in_2d /= np.linalg.norm(wrist_normal_in_2d)
                wrist_normal_in_2d *= NORMAL_VIS_LEN_2D / down_sampling_factor
                wrist_and_palm_normal_arrows_2d.append(wrist_normal_in_2d)
            if palm_normal_tip_pixels[hand_i] is not None and palm_pixel is not None:
                palm_normal_in_2d = palm_normal_tip_pixels[hand_i] - palm_pixel
                palm_normal_in_2d /= np.linalg.norm(palm_normal_in_2d)
                palm_normal_in_2d *= NORMAL_VIS_LEN_2D / down_sampling_factor
                wrist_and_palm_normal_arrows_2d.append(palm_normal_in_2d)

        if wrist_and_palm_normal_arrows_2d:
            rr.log(
                f"world/device/{rgb_stream_label}/wrist-and-palm_projection/normals",
                rr.Arrows2D(
                    origins=wrist_and_palm_points_2d,
                    vectors=wrist_and_palm_normal_arrows_2d,
                    colors=[WRIST_PALM_COLOR],
                ),
            )


def log_hand_tracking_results(
    hand_tracking_results: List[mps.hand_tracking.HandTrackingResult],
    device_time_ns: int,
    rgb_camera_calibration: CameraCalibration,
    rgb_stream_label: str,
    down_sampling_factor: int,
) -> None:
    rr.log("world/device/landmarks", rr.Clear.recursive())
    rr.log(
        f"world/device/{rgb_stream_label}/landmarks_projection",
        rr.Clear.recursive(),
    )
    rr.log(
        f"world/device/{rgb_stream_label}/landmarks_projection/link",
        rr.Clear.recursive(),
    )
    rr.log("world/device/wrist-and-palm", rr.Clear.recursive())
    rr.log(
        f"world/device/{rgb_stream_label}/wrist-and-palm_projection",
        rr.Clear.recursive(),
    )
    rr.log(
        f"world/device/{rgb_stream_label}/wrist-and-palm_projection/link",
        rr.Clear.recursive(),
    )
    rr.log(
        f"world/device/{rgb_stream_label}/wrist-and-palm_projection/normals",
        rr.Clear.recursive(),
    )

    if hand_tracking_results is None:
        return

    hand_tracking_result = get_nearest_hand_tracking_result(
        hand_tracking_results, device_time_ns
    )

    if (
        hand_tracking_result is None
        or np.abs(
            hand_tracking_result.tracking_timestamp.total_seconds() * 1e9
            - device_time_ns
        )
        >= WRIST_PALM_TIME_DIFFERENCE_THRESHOLD_NS
    ):
        return

    wrist_points: List[np.array] = []
    palm_points: List[np.array] = []
    wrist_normals: List[np.array] = []
    palm_normals: List[np.array] = []
    if (
        hand_tracking_result.left_hand is not None
        and hand_tracking_result.left_hand.confidence > 0
    ):
        left_hand_tracking_result = hand_tracking_result.left_hand
        wrist_points.append(
            left_hand_tracking_result.landmark_positions_device[
                int(mps.hand_tracking.HandLandmark.WRIST)
            ]
        )
        palm_points.append(
            left_hand_tracking_result.landmark_positions_device[
                int(mps.hand_tracking.HandLandmark.PALM_CENTER)
            ]
        )

        if left_hand_tracking_result.wrist_and_palm_normal_device is not None:
            wrist_normals.append(
                left_hand_tracking_result.wrist_and_palm_normal_device.wrist_normal_device
            )
            palm_normals.append(
                left_hand_tracking_result.wrist_and_palm_normal_device.palm_normal_device
            )

        log_single_hand_landmarks(
            hand_landmarks_in_device=left_hand_tracking_result.landmark_positions_device,
            hand_label="left",
            rgb_camera_calibration=rgb_camera_calibration,
            rgb_stream_label=rgb_stream_label,
            down_sampling_factor=down_sampling_factor,
        )

    if (
        hand_tracking_result.right_hand is not None
        and hand_tracking_result.right_hand.confidence > 0
    ):
        right_hand_tracking_result = hand_tracking_result.right_hand
        wrist_points.append(
            right_hand_tracking_result.landmark_positions_device[
                int(mps.hand_tracking.HandLandmark.WRIST)
            ]
        )
        palm_points.append(
            right_hand_tracking_result.landmark_positions_device[
                int(mps.hand_tracking.HandLandmark.PALM_CENTER)
            ]
        )

        if right_hand_tracking_result.wrist_and_palm_normal_device is not None:
            wrist_normals.append(
                right_hand_tracking_result.wrist_and_palm_normal_device.wrist_normal_device
            )
            palm_normals.append(
                right_hand_tracking_result.wrist_and_palm_normal_device.palm_normal_device
            )

        log_single_hand_landmarks(
            hand_landmarks_in_device=hand_tracking_result.right_hand.landmark_positions_device,
            hand_label="right",
            rgb_camera_calibration=rgb_camera_calibration,
            rgb_stream_label=rgb_stream_label,
            down_sampling_factor=down_sampling_factor,
        )

    log_wrist_and_palm(
        wrist_points=wrist_points,
        palm_points=palm_points,
        wrist_normals=wrist_normals,
        palm_normals=palm_normals,
        rgb_camera_calibration=rgb_camera_calibration,
        rgb_stream_label=rgb_stream_label,
        down_sampling_factor=down_sampling_factor,
    )


def log_mps_to_rerun(
    vrs_path: Optional[str],
    trajectory_files: List[str],
    points_files: List[str],
    eye_gaze_file: Optional[str],
    wrist_and_palm_poses_file: Optional[str],
    hand_tracking_results_file: Optional[str],
    should_rectify_image: bool = False,
    down_sampling_factor: int = 4,
    jpeg_quality: int = 75,
    rrd_output_path: Optional[str] = None,
) -> None:
    if rrd_output_path:
        print(f"Saving .rrd file to {rrd_output_path}")
        rr.save(rrd_output_path)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    if trajectory_files:
        log_device_trajectory(trajectory_files)

    if points_files:
        log_point_clouds(points_files)

    # If we we do not have a vrs file, we are done
    if not vrs_path:
        return

    #
    # Go over RGB timestamps and
    # - Plot camera pose
    # - Plot user eye gaze
    # - Plot user wrist and palm pose
    #

    provider = data_provider.create_vrs_data_provider(vrs_path)
    device_calibration = provider.get_device_calibration()

    # Automatically rotate the image for Gen1 Aria devices
    should_rotate_image: bool = (
        device_calibration.get_device_version() == DeviceVersion.Gen1
    )

    T_device_CPF = (
        device_calibration.get_transform_device_cpf()
    )  # this is always CAD value
    rgb_stream_id = StreamId("214-1")
    rgb_stream_label = provider.get_label_from_stream_id(rgb_stream_id)
    rgb_camera_calibration = device_calibration.get_camera_calib(rgb_stream_label)

    if should_rectify_image:
        rgb_linear_camera_calibration = calibration.get_linear_camera_calibration(
            int(rgb_camera_calibration.get_image_size()[0]),
            int(rgb_camera_calibration.get_image_size()[1]),
            rgb_camera_calibration.get_focal_lengths()[0],
            "pinhole",
            rgb_camera_calibration.get_transform_device_camera(),
        )
        if should_rotate_image:
            rgb_rotated_linear_camera_calibration = (
                calibration.rotate_camera_calib_cw90deg(rgb_linear_camera_calibration)
            )
            camera_calibration = rgb_rotated_linear_camera_calibration
        else:
            camera_calibration = rgb_linear_camera_calibration
    else:  # No rectification
        if should_rotate_image:
            rgb_rotated_camera_calibration = calibration.rotate_camera_calib_cw90deg(
                rgb_camera_calibration
            )
            camera_calibration = rgb_rotated_camera_calibration
        else:
            camera_calibration = rgb_camera_calibration

    def post_process_image(img):
        if should_rectify_image:
            img = calibration.distort_by_calibration(
                img,
                rgb_linear_camera_calibration,
                rgb_camera_calibration,
            )
        if should_rotate_image:
            img = np.rot90(img, k=3)
        return img

    # Load Trajectory, Eye Gaze, and Wrist and Palm Pose data - corresponding to this specific VRS file
    if trajectory_files:
        trajectory_data = mps.read_closed_loop_trajectory(str(trajectory_files[0]))
    else:
        trajectory_data = None
    if trajectory_data and len(trajectory_data) == 0:
        trajectory_data = mps.read_open_loop_trajectory(str(trajectory_files[0]))
    eyegaze_data = mps.read_eyegaze(eye_gaze_file) if eye_gaze_file else None

    # If both wrist and palm poses and hand tracking results are available, use hand tracking
    # results as it's a superset of wrist and palm poses.
    hand_tracking_results = (
        mps.hand_tracking.read_hand_tracking_results(hand_tracking_results_file)
        if hand_tracking_results_file
        else None
    )
    wrist_and_palm_poses = None
    if hand_tracking_results is None:
        wrist_and_palm_poses = (
            mps.hand_tracking.read_wrist_and_palm_poses(wrist_and_palm_poses_file)
            if wrist_and_palm_poses_file
            else None
        )
    else:
        print(
            f"Using hand tracking results provided at {hand_tracking_results_file} "
            f"for visualization. wrist_and_palm_poses_file ({wrist_and_palm_poses_file}) will be ignored."
        )

    # Log Aria Glasses outline
    log_RGB_camera_calibration(
        camera_calibration, rgb_stream_label, down_sampling_factor
    )

    log_Aria_glasses_outline(device_calibration)

    # Configure the loop for data replay
    deliver_option = provider.get_default_deliver_queued_options()
    deliver_option.deactivate_stream_all()
    deliver_option.activate_stream(rgb_stream_id)  # RGB Stream Id
    rgb_frame_count = provider.get_num_data(rgb_stream_id)

    progress_bar = tqdm(total=rgb_frame_count)
    # Iterate over the data and LOG data as we see fit
    for data in provider.deliver_queued_sensor_data(deliver_option):
        device_time_ns = data.get_time_ns(TimeDomain.DEVICE_TIME)
        rr.set_time_nanos("device_time", device_time_ns)
        rr.set_time_sequence("timestamp", device_time_ns)
        progress_bar.update(1)

        log_camera_pose(
            trajectory_data,
            device_time_ns,
            camera_calibration,
            rgb_stream_label,
        )

        log_RGB_image(
            data,
            down_sampling_factor,
            jpeg_quality,
            rgb_stream_label,
            postprocess_image=post_process_image,
        )

        log_eye_gaze(
            eyegaze_data,
            device_time_ns,
            T_device_CPF,
            rgb_stream_label,
            device_calibration,
            camera_calibration,
            down_sampling_factor,
            should_rotate_image,
        )

        if hand_tracking_results is not None:
            log_hand_tracking_results(
                hand_tracking_results,
                device_time_ns,
                camera_calibration,
                rgb_stream_label,
                down_sampling_factor,
            )
        if wrist_and_palm_poses is not None:
            log_hand_tracking(
                wrist_and_palm_poses,
                device_time_ns,
                camera_calibration,
                rgb_stream_label,
                down_sampling_factor,
            )
