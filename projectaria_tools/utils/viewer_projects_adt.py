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

from math import tan
from typing import Dict, Set

import numpy as np
import rerun as rr

from projectaria_tools.core.mps.utils import get_gaze_vector_reprojection

from projectaria_tools.core.sophus import SE3

from projectaria_tools.core.stream_id import StreamId
from projectaria_tools.projects.adt import (
    AriaDigitalTwinDataPathsProvider,
    AriaDigitalTwinDataProvider,
    AriaDigitalTwinSkeletonProvider,
    bbox3d_to_line_coordinates,
    DYNAMIC,
    STATIC,
)
from projectaria_tools.utils.rerun_helpers import AriaGlassesOutline, ToTransform3D
from tqdm import tqdm


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sequence_path",
        type=str,
        required=True,
        help="path to the ADT sequence",
    )
    parser.add_argument(
        "--device_number",
        type=int,
        default=0,
        help="Device_number you want to visualize, default is 0",
    )
    # Add options that does not show by default, but still accessible for debugging purpose
    parser.add_argument(
        "--down_sampling_factor", type=int, default=4, help=argparse.SUPPRESS
    )
    parser.add_argument("--jpeg_quality", type=int, default=75, help=argparse.SUPPRESS)
    return parser.parse_args()


def main():
    args = parse_args()

    print("sequence_path: ", args.sequence_path)
    try:
        paths_provider = AriaDigitalTwinDataPathsProvider(args.sequence_path)
        data_paths = paths_provider.get_datapaths_by_device_num(args.device_number)
        gt_provider = AriaDigitalTwinDataProvider(data_paths)
    except Exception as e:
        print("Error: ", str(e))
        exit(-1)

    # We are limiting display to the time span where the Aria glasses are moving
    aria_pose_start_timestamp = gt_provider.get_start_time_ns()
    aria_pose_end_timestamp = gt_provider.get_end_time_ns()
    rgb_stream_id = StreamId("214-1")
    img_timestamps_ns = gt_provider.get_aria_device_capture_timestamps_ns(rgb_stream_id)
    img_timestamps_ns = [
        img_timestamp_ns
        for i, img_timestamp_ns in enumerate(img_timestamps_ns)
        if (
            img_timestamp_ns >= aria_pose_start_timestamp
            and img_timestamp_ns <= aria_pose_end_timestamp
        )
    ]

    # get all available skeletons in a sequence
    skeleton_ids = gt_provider.get_skeleton_ids()
    device_serial_numbers = paths_provider.get_device_serial_numbers()
    for skeleton_id in skeleton_ids:
        skeleton_info = gt_provider.get_instance_info_by_id(skeleton_id)
        is_skeleton_displayed = (
            skeleton_info.associated_device_serial
            == device_serial_numbers[args.device_number]
        )
        print(
            f"skeleton (id: {skeleton_id} name: {skeleton_info.name}) wears {skeleton_info.associated_device_serial} - Wearing glasses: {is_skeleton_displayed}"
        )

    # Initializing Rerun viewer
    rr.init("ADT Sequence Viewer", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, timeless=True)

    #
    # Log timeless information
    #

    # Log RGB camera calibration
    rgb_camera_calibration = gt_provider.get_aria_camera_calibration(rgb_stream_id)
    rr.log(
        "world/device/rgb",
        rr.Pinhole(
            resolution=[
                rgb_camera_calibration.get_image_size()[0] / args.down_sampling_factor,
                rgb_camera_calibration.get_image_size()[1] / args.down_sampling_factor,
            ],
            focal_length=float(
                rgb_camera_calibration.get_focal_lengths()[0]
                / args.down_sampling_factor
            ),
        ),
        timeless=True,
    )
    # Log Aria Glasses outline
    raw_data_provider_ptr = gt_provider.raw_data_provider_ptr()
    device_calibration = raw_data_provider_ptr.get_device_calibration()
    aria_glasses_point_outline = AriaGlassesOutline(device_calibration)
    rr.log(
        "world/device/glasses_outline",
        rr.LineStrips3D([aria_glasses_point_outline]),
        timeless=True,
    )

    # For all selected timestamp (log data we want to see)
    # Store a Pose cache for dynamic object, so we log them only if they changed position
    dynamic_obj_pose_cache: Dict[str, SE3] = {}
    static_obj_ids: Set[int] = set()
    dynamic_obj_moved: Set[str] = set()
    for timestamp_ns in tqdm(img_timestamps_ns):
        rr.set_time_nanos("device_time", timestamp_ns)
        rr.set_time_sequence("timestamp", timestamp_ns)

        # Log RGB image
        image_with_dt = gt_provider.get_aria_image_by_timestamp_ns(
            timestamp_ns, rgb_stream_id
        )

        if args.down_sampling_factor > 1:
            img = image_with_dt.data().to_numpy_array()[
                :: args.down_sampling_factor, :: args.down_sampling_factor
            ]
            # Note: We configure the QUEUE to return only RGB image, so we are sure this image is corresponding to a RGB frame
            rr.log(
                "world/device/rgb",
                rr.Image(img).compress(jpeg_quality=args.jpeg_quality),
            )

        # Log skeleton(s)
        for skeleton_id in skeleton_ids:
            skeleton_with_dt = gt_provider.get_skeleton_by_timestamp_ns(
                timestamp_ns, skeleton_id
            )
            skeleton_info = gt_provider.get_instance_info_by_id(skeleton_id)
            assert skeleton_with_dt.is_valid(), "skeleton is not valid"
            skeleton = skeleton_with_dt.data()
            joints = skeleton.joints

            traces = []
            # draw skeleton
            joint_connections = AriaDigitalTwinSkeletonProvider.get_joint_connections()
            for i in range(0, len(joint_connections)):
                joint_1 = joints[joint_connections[i][0]]
                joint_2 = joints[joint_connections[i][1]]
                traces.append([joint_1, joint_2])
            rr.log(
                f"world/skeletons/{skeleton_info.associated_device_serial}",
                rr.LineStrips3D(traces),
            )

        # Log device position and trajectory
        aria_3d_pose_with_dt = gt_provider.get_aria_3d_pose_by_timestamp_ns(
            timestamp_ns
        )

        if aria_3d_pose_with_dt.is_valid():
            aria_3d_pose = aria_3d_pose_with_dt.data()
            device_to_rgb = gt_provider.get_aria_transform_device_camera(rgb_stream_id)

            rr.log(
                "world/device",
                ToTransform3D(aria_3d_pose.transform_scene_device, False),
            )
            rr.log("world/device/rgb", ToTransform3D(device_to_rgb.inverse(), True))

            # Log device location
            rr.log(
                "world/device_translation",
                rr.Points3D(aria_3d_pose.transform_scene_device.translation()),
            )

            # Log Eye gaze vector in CPF coordinate system
            eye_gaze_with_dt = gt_provider.get_eyegaze_by_timestamp_ns(timestamp_ns)
            transform_device_cpf = (
                gt_provider.raw_data_provider_ptr()
                .get_device_calibration()
                .get_transform_device_cpf()
            )
            T_scene_cpf = aria_3d_pose.transform_scene_device @ transform_device_cpf

            if eye_gaze_with_dt.is_valid():
                eye_gaze = eye_gaze_with_dt.data()
                gaze_ray_in_cpf = (
                    np.array(
                        [tan(eye_gaze.yaw), tan(eye_gaze.pitch), 1.0], dtype=np.float64
                    )
                    * eye_gaze.depth
                )
                gaze_ray_in_camera = T_scene_cpf @ gaze_ray_in_cpf
                rr.log(
                    "world/eye_gaze",
                    rr.LineStrips3D([[T_scene_cpf @ [0, 0, 0], gaze_ray_in_camera]]),
                )

                # Compute eye_gaze vector at depth_m reprojection in the image
                gaze_projection = get_gaze_vector_reprojection(
                    eye_gaze,
                    rgb_camera_calibration.get_label(),
                    device_calibration,
                    rgb_camera_calibration,
                    eye_gaze.depth,
                )
                rr.log(
                    "world/device/rgb/eye-gaze_projection",
                    rr.Points2D(gaze_projection / args.down_sampling_factor, radii=4),
                )

        # Log object Bounding Boxes
        bbox3d_with_dt = gt_provider.get_object_3d_boundingboxes_by_timestamp_ns(
            timestamp_ns
        )
        assert bbox3d_with_dt.is_valid(), "3D bounding box is not available"
        bboxes3d = bbox3d_with_dt.data()
        for obj_id in bboxes3d:
            instance_info = gt_provider.get_instance_info_by_id(obj_id)
            if instance_info.motion_type == DYNAMIC:

                bbox_3d = bboxes3d[obj_id]
                instance_info = gt_provider.get_instance_info_by_id(obj_id)

                display_dynamic_object = True

                # Implement a logic to LOG only effective object motion
                #  - by using a cache of the last know pose
                #  - and measuring if the motion amplitude is larger than a threshold
                if instance_info.name in dynamic_obj_pose_cache.keys():
                    current_pose = bbox_3d.transform_scene_object
                    old_pose = dynamic_obj_pose_cache[instance_info.name]
                    current_pose_R = current_pose.rotation().to_matrix()
                    current_pose_t = current_pose.translation()
                    old_pose_R = old_pose.rotation().to_matrix()
                    old_pose_t = old_pose.translation()

                    norm_delta_R = np.linalg.norm(current_pose_R - old_pose_R, ord=2)
                    norm_delta_t = np.linalg.norm(current_pose_t - old_pose_t, ord=2)

                    if norm_delta_R < 0.02 or norm_delta_t < 0.015:
                        # Object did not move, so we do not log it (the last position will persist)
                        display_dynamic_object = False
                    else:
                        dynamic_obj_moved.add(instance_info.name)

                if display_dynamic_object:  # Not in cache, or object moved enough

                    # We store the last known pose of the object
                    dynamic_obj_pose_cache[
                        instance_info.name
                    ] = bbox_3d.transform_scene_object

                    aabb_coords = bbox3d_to_line_coordinates(bbox_3d.aabb)
                    obb = np.zeros(shape=(len(aabb_coords), 3))
                    for i in range(0, len(aabb_coords)):
                        aabb_pt = aabb_coords[i]
                        aabb_pt_homo = np.append(aabb_pt, [1])
                        obb_pt = (
                            bbox_3d.transform_scene_object.to_matrix() @ aabb_pt_homo
                        )[0:3]
                        obb[i] = obb_pt
                    rr.log(
                        f"world/objects/dynamic/{instance_info.name}",
                        rr.LineStrips3D([obb]),
                    )
            # Static
            elif instance_info.motion_type == STATIC and obj_id not in static_obj_ids:
                static_obj_ids.add(obj_id)
                bbox_3d = bboxes3d[obj_id]
                instance_info = gt_provider.get_instance_info_by_id(obj_id)
                aabb_coords = bbox3d_to_line_coordinates(bbox_3d.aabb)
                obb = np.zeros(shape=(len(aabb_coords), 3))
                for i in range(0, len(aabb_coords)):
                    aabb_pt = aabb_coords[i]
                    aabb_pt_homo = np.append(aabb_pt, [1])
                    obb_pt = (
                        bbox_3d.transform_scene_object.to_matrix() @ aabb_pt_homo
                    )[0:3]
                    obb[i] = obb_pt
                rr.log(
                    f"world/objects/static/{instance_info.name}",
                    rr.LineStrips3D([obb]),
                    timeless=True,
                )
    print(f"Loaded scene: {args.sequence_path}")
    print("Scene characteristics:")
    print(f"\t Aria RGB frames count: {len(img_timestamps_ns)}")
    print(f"\t Skeleton count: {len(gt_provider.get_skeleton_ids())}")
    print(f"\t Static object count: {len(static_obj_ids)}")
    print(f"\t Dynamic object count (tracked): {len(dynamic_obj_pose_cache)}")
    print(
        f"\t Dynamic object count (tracked and moved - estimated): {len(dynamic_obj_moved)}"
    )


if __name__ == "__main__":
    main()
