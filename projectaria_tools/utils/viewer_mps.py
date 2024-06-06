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

import argparse
import os

from pathlib import Path

from typing import Callable, List, Optional

import numpy as np

import rerun as rr

from projectaria_tools.core import calibration, data_provider, mps
from projectaria_tools.core.calibration import CameraCalibration, DeviceCalibration
from projectaria_tools.core.mps import MpsDataPathsProvider
from projectaria_tools.core.mps.utils import (
    filter_points_from_confidence,
    filter_points_from_count,
    get_gaze_vector_reprojection,
    get_nearest_eye_gaze,
    get_nearest_pose,
    get_nearest_wrist_and_palm_pose,
)
from projectaria_tools.core.sensor_data import SensorData, SensorDataType, TimeDomain
from projectaria_tools.core.sophus import SE3
from projectaria_tools.core.stream_id import StreamId
from projectaria_tools.utils.rerun_helpers import AriaGlassesOutline, ToTransform3D
from tqdm import tqdm


WRIST_PALM_TIME_DIFFERENCE_THRESHOLD_NS: int = 2e8
WRIST_PALM_COLOR: List[int] = [255, 64, 0]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vrs",
        type=str,
        help="path to VRS file",
    )
    # Add options for the MPS Artifacts
    # - They can be specified individually,
    # - Or globally as a 'mps_folder' path
    parser.add_argument(
        "--trajectory",
        nargs="+",
        type=str,
        help="path(s) to MPS closed-loop trajectory files",
    )
    parser.add_argument(
        "--points",
        nargs="+",
        type=str,
        help="path(s) to the MPS global points file",
    )
    parser.add_argument(
        "--eyegaze",
        type=str,
        help="path to the MPS eye gaze file",
    )
    parser.add_argument(
        "--hands",
        type=str,
        help="path to the MPS hand tracking file",
    )
    parser.add_argument(
        "--mps_folder",
        type=str,
        help="path to the MPS folder (will overwrite default value <vrs_file>/mps)",
    )

    # Add options that does not show by default, but still accessible for debugging purpose
    parser.add_argument(
        "--down_sampling_factor", type=int, default=4, help=argparse.SUPPRESS
    )
    parser.add_argument("--jpeg_quality", type=int, default=75, help=argparse.SUPPRESS)

    # If this path is set, we will save the rerun (.rrd) file to the given path
    parser.add_argument(
        "--rrd_output_path", type=str, default="", help=argparse.SUPPRESS
    )

    parser.add_argument(
        "--no_rotate_image_upright",
        action="store_true",
        help="If set, the RGB images are shown in their original orientation, which is rotated 90 degrees ccw from upright.",
    )

    parser.add_argument(
        "--no_rectify_image",
        action="store_true",
        help="If set, the raw fisheye RGB images are shown without being undistorted.",
    )
    return parser.parse_args()


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
        trajectory_data = mps.read_closed_loop_trajectory(trajectory_file)
        device_trajectory = [
            it.transform_world_device.translation()[0] for it in trajectory_data
        ][0::80]
        rr.log(
            (
                "world/device_trajectory"
                if trajectory_list_size == 1
                else f"world/device_trajectory_{i}"
            ),
            rr.LineStrips3D(device_trajectory, radii=0.008),
            timeless=True,
        )
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
        rr.log(
            "world/points" if point_cloud_list_size == 1 else f"world/points_{i}",
            rr.Points3D(point_positions, radii=0.006),
            timeless=True,
        )
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
        timeless=True,
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
        timeless=True,
    )


def log_camera_pose(
    trajectory_data: List[mps.ClosedLoopTrajectoryPose],
    device_time_ns: int,
    rgb_camera_calibration: CameraCalibration,
    rgb_stream_label: str,
) -> None:
    # Camera pose
    if trajectory_data:
        pose_info = get_nearest_pose(trajectory_data, device_time_ns)
        if pose_info:
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
    if wrist_and_palm_poses:
        wrist_and_palm_pose = get_nearest_wrist_and_palm_pose(
            wrist_and_palm_poses, device_time_ns
        )
        wrist_points: List[np.array] = []
        palm_points: List[np.array] = []
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

    if not logged_hand_tracking_3D:
        rr.log("world/device/wrist-and-palm", rr.Clear.recursive())

    if not logged_hand_tracking_2D_points:
        # If no points were found, recursively clear the 2D projections
        rr.log(
            f"world/device/{rgb_stream_label}/wrist-and-palm_projection",
            rr.Clear.recursive(),
        )
    elif not logged_hand_tracking_2D_links:
        # If only the links are missing, clear the links
        rr.log(
            f"world/device/{rgb_stream_label}/wrist-and-palm_projection/link",
            rr.Clear.flat(),
        )


def main():
    args = parse_args()

    #
    # Gather data input
    #
    if args.vrs:
        vrs_folder_path = os.path.dirname(args.vrs)
        # - If MPS data has not been provided we try to find them automatically using default folder hierarchy
        if args.points is None and args.eyegaze is None and args.trajectory is None:
            if args.mps_folder:
                mps_data_paths_provider = MpsDataPathsProvider(args.mps_folder)
            else:
                # Try loading from default mps path (<vrs_file>/mps)
                mps_data_paths_provider = MpsDataPathsProvider(
                    str(Path(vrs_folder_path + "/mps"))
                )
            mps_data_paths = mps_data_paths_provider.get_data_paths()

            if not args.trajectory and os.path.exists(
                mps_data_paths.slam.closed_loop_trajectory
            ):
                args.trajectory = [str(mps_data_paths.slam.closed_loop_trajectory)]

            if not args.points and os.path.exists(mps_data_paths.slam.semidense_points):
                args.points = [str(mps_data_paths.slam.semidense_points)]

            if not args.eyegaze and os.path.exists(
                mps_data_paths.eyegaze.personalized_eyegaze
            ):
                args.eyegaze = mps_data_paths.eyegaze.personalized_eyegaze
            if not args.eyegaze and os.path.exists(
                mps_data_paths.eyegaze.general_eyegaze
            ):
                args.eyegaze = mps_data_paths.eyegaze.general_eyegaze

            if not args.hands and os.path.exists(
                mps_data_paths.hand_tracking.wrist_and_palm_poses
            ):
                args.hands = mps_data_paths.hand_tracking.wrist_and_palm_poses

    mps_data_available = args.trajectory or args.points or args.eyegaze or args.hands

    print(
        f"""
    Trying to load the following list of files:
    - vrs: {args.vrs}
    - trajectory/closed_loop_trajectory: {args.trajectory}
    - trajectory/point_cloud: {args.points}
    - eye_gaze/general_eye_gaze: {args.eyegaze}
    - hand_tracking/wrist_and_palm_poses: {args.hands}
    """
    )

    if mps_data_available is None and args.vrs is None:
        print("Nothing to display.")
        exit(1)

    # Initializing Rerun viewer
    rr.init("MPS Data Viewer", spawn=(not args.rrd_output_path))
    if args.rrd_output_path:
        print(f"Saving .rrd file to {args.rrd_output_path}")
        rr.save(args.rrd_output_path)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, timeless=True)

    if args.trajectory:
        log_device_trajectory(args.trajectory)

    if args.points:
        log_point_clouds(args.points)

    #
    # If we we do not have a vrs file, we are done
    #
    if not args.vrs:
        return

    #
    # Go over RGB timestamps and
    # - Plot camera pose
    # - Plot user eye gaze
    # - Plot user wrist and palm pose
    #

    provider = data_provider.create_vrs_data_provider(args.vrs)
    device_calibration = provider.get_device_calibration()
    T_device_CPF = (
        device_calibration.get_transform_device_cpf()
    )  # this is always CAD value
    rgb_stream_id = StreamId("214-1")
    rgb_stream_label = provider.get_label_from_stream_id(rgb_stream_id)
    rgb_camera_calibration = device_calibration.get_camera_calib(rgb_stream_label)

    if not args.no_rectify_image:
        rgb_linear_camera_calibration = calibration.get_linear_camera_calibration(
            int(rgb_camera_calibration.get_image_size()[0]),
            int(rgb_camera_calibration.get_image_size()[1]),
            rgb_camera_calibration.get_focal_lengths()[0],
            "pinhole",
            rgb_camera_calibration.get_transform_device_camera(),
        )
        if not args.no_rotate_image_upright:
            rgb_rotated_linear_camera_calibration = (
                calibration.rotate_camera_calib_cw90deg(rgb_linear_camera_calibration)
            )
            camera_calibration = rgb_rotated_linear_camera_calibration
        else:
            camera_calibration = rgb_linear_camera_calibration
    else:  # No rectification
        if args.no_rotate_image_upright:
            camera_calibration = rgb_camera_calibration
        else:
            raise NotImplementedError(
                "Showing upright-rotated image without rectification is not currently supported.\n"
                "Please use --no-rotate-image-upright and --no-rectify-image together."
            )

    def post_process_image(img):
        if not args.no_rectify_image:
            img = calibration.distort_by_calibration(
                img,
                rgb_linear_camera_calibration,
                rgb_camera_calibration,
            )
            if not args.no_rotate_image_upright:
                img = np.rot90(img, k=3)
        return img

    # Load Trajectory, Eye Gaze, and Wrist and Palm Pose data - corresponding to this specific VRS file
    trajectory_data = (
        mps.read_closed_loop_trajectory(str(args.trajectory[0]))
        if args.trajectory
        else None
    )
    eyegaze_data = mps.read_eyegaze(args.eyegaze) if args.eyegaze else None
    wrist_and_palm_poses = (
        mps.hand_tracking.read_wrist_and_palm_poses(args.hands) if args.hands else None
    )

    # Log Aria Glasses outline
    log_RGB_camera_calibration(
        rgb_camera_calibration, rgb_stream_label, args.down_sampling_factor
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
            args.down_sampling_factor,
            args.jpeg_quality,
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
            args.down_sampling_factor,
            not args.no_rotate_image_upright,
        )

        log_hand_tracking(
            wrist_and_palm_poses,
            device_time_ns,
            camera_calibration,
            rgb_stream_label,
            args.down_sampling_factor,
        )


if __name__ == "__main__":
    main()
