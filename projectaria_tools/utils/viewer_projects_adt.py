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
from math import tan
from typing import Dict, Final, List, Set

import numpy as np
import rerun as rr
from PIL import Image

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
from projectaria_tools.utils.calibration_utils import (
    rotate_upright_image_and_calibration,
    undistort_image_and_calibration,
)
from projectaria_tools.utils.rerun_helpers import AriaGlassesOutline, ToTransform3D
from tqdm import tqdm

GLB_FILENAME: Final[str] = "3d-asset.glb"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sequence_path",
        type=str,
        required=True,
        help="Path to the ADT sequence",
    )
    parser.add_argument(
        "--object_library_path",
        type=str,
        required=False,
        help="Path to the ADT object library",
    )
    parser.add_argument(
        "--display_objects",
        type=str,
        required=False,
        nargs="*",
        help="Space separated list of object names. Each object's glb model will be loaded from object_library_path and rendered.",
    )
    parser.add_argument(
        "--no_rotate_image_upright",
        action="store_true",
        help="If set, the RGB images are shown in their original orientation, which is rotated 90 degrees ccw from upright.",
    )
    parser.add_argument(
        "--no_undistort_image",
        action="store_true",
        help="If set, the RGB images will not be undistorted.",
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
    return parser.parse_args()


#
def add(self):
    """returns a dict with dynamic object names to display as keys and their instance ids as values"""


def log_glbs(
    object_names: List[str],
    object_library_path: str,
    adt_data_provider: AriaDigitalTwinDataProvider,
) -> Dict[str, int]:
    if object_names is None or len(object_names) == 0:
        return {}

    if not object_library_path:
        raise Exception("object_library_path is required")

    # load instance name to instance info
    instance_name_to_info = {}
    for instance_id in adt_data_provider.get_instance_ids():
        instance_info = adt_data_provider.get_instance_info_by_id(instance_id)
        instance_name_to_info[instance_info.name] = instance_info

    bboxes_3d_initial = adt_data_provider.get_object_3d_boundingboxes_by_timestamp_ns(
        adt_data_provider.get_start_time_ns()
    )
    assert bboxes_3d_initial.is_valid(), "3D bounding box is not available"

    obj_meshes_to_log = []
    for object_name in object_names:
        glb_path = os.path.join(object_library_path, object_name, GLB_FILENAME)
        if not os.path.exists(glb_path):
            raise Exception(f"glb file {glb_path} does not exist")

        # Get first pose. For dynamic objects, we will update it as we iterate through the timestamps.
        instance_info = instance_name_to_info[object_name]
        instance_id = instance_info.id
        assert (
            instance_id in bboxes_3d_initial.data()
        ), f"object {object_name} is not available in ADT sequence"
        bbox_3d = bboxes_3d_initial.data()[instance_id]
        T_scene_object = bbox_3d.transform_scene_object
        is_static = instance_info.motion_type == STATIC
        if is_static:
            entity_path = f"world/objects/static/{object_name}/model"
        else:
            entity_path = f"world/objects/dynamic/{object_name}/model"
        rr.log(
            entity_path,
            rr.Asset3D(path=glb_path),
            static=True,
        )
        rr.log(entity_path, ToTransform3D(T_scene_object))
        if not is_static:
            obj_meshes_to_log.append(instance_id)

    return obj_meshes_to_log


def main():
    args = parse_args()

    print("sequence_path: ", args.sequence_path)
    try:
        paths_provider = AriaDigitalTwinDataPathsProvider(args.sequence_path)
        data_paths = paths_provider.get_datapaths()
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
    for skeleton_id in skeleton_ids:
        skeleton_info = gt_provider.get_instance_info_by_id(skeleton_id)
        print(
            f"skeleton (id: {skeleton_id} name: {skeleton_info.name}) wears {skeleton_info.associated_device_serial}"
        )

    # Initializing Rerun viewer
    rr.init("ADT Sequence Viewer", spawn=(not args.rrd_output_path))
    if args.rrd_output_path:
        print(f"Saving .rrd file to {args.rrd_output_path}")
        rr.save(args.rrd_output_path)

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

    #
    # Log static information
    #

    # Log RGB camera calibration
    rgb_camera_calibration = gt_provider.get_aria_camera_calibration(rgb_stream_id)
    rr.log(
        "world/device/rgb",
        rr.Pinhole(
            resolution=[
                int(
                    rgb_camera_calibration.get_image_size()[0]
                    / args.down_sampling_factor
                ),
                int(
                    rgb_camera_calibration.get_image_size()[1]
                    / args.down_sampling_factor
                ),
            ],
            focal_length=float(
                rgb_camera_calibration.get_focal_lengths()[0]
                / args.down_sampling_factor
            ),
        ),
        static=True,
    )

    # Log Aria Glasses outline
    raw_data_provider_ptr = gt_provider.raw_data_provider_ptr()
    device_calibration = raw_data_provider_ptr.get_device_calibration()
    # Log Aria Glasses outline only if CAD calibration is available. For simulated data, CAD calibration is not available
    if device_calibration.get_transform_device_sensor("camera-slam-left", True):
        aria_glasses_point_outline = AriaGlassesOutline(device_calibration)
        rr.log(
            "world/device/glasses_outline",
            rr.LineStrips3D([aria_glasses_point_outline]),
            static=True,
        )
    else:
        print("CAD calibration not available, not logging Aria Glasses outline")

    # Log GLB files
    obj_meshes_to_log = log_glbs(
        args.display_objects, args.object_library_path, gt_provider
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

        # rescale image
        image_display = Image.fromarray(image_with_dt.data().to_numpy_array())
        new_resolution = (
            rgb_camera_calibration.get_image_size() / args.down_sampling_factor
        )
        new_resolution = new_resolution.astype(int)
        updated_camera_calibration = rgb_camera_calibration.rescale(
            new_resolution, 1.0 / args.down_sampling_factor
        )
        image_display = image_display.resize((new_resolution[0], new_resolution[1]))
        image_display = np.array(image_display)

        # rectify image (unless otherwise specified)
        if not args.no_undistort_image:
            image_display, updated_camera_calibration = undistort_image_and_calibration(
                image_display,
                updated_camera_calibration,
            )

        # rotate image (unless otherwise specified)
        if not args.no_rotate_image_upright:
            image_display, updated_camera_calibration = (
                rotate_upright_image_and_calibration(
                    image_display, updated_camera_calibration
                )
            )

        rr.log(
            "world/device/rgb/image",
            rr.Image(image_display).compress(jpeg_quality=args.jpeg_quality),
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
            T_device_rgb = updated_camera_calibration.get_transform_device_camera()

            rr.log(
                "world/device",
                ToTransform3D(aria_3d_pose.transform_scene_device, False),
            )
            rr.log("world/device/rgb", ToTransform3D(T_device_rgb.inverse(), True))

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
                    updated_camera_calibration,
                    eye_gaze.depth,
                    not args.no_rotate_image_upright,
                )
                rr.log(
                    "world/device/rgb/eye-gaze_projection",
                    rr.Points2D(gaze_projection, radii=4),
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
                    dynamic_obj_pose_cache[instance_info.name] = (
                        bbox_3d.transform_scene_object
                    )

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
                        f"world/objects/dynamic/{instance_info.name}/bbox",
                        rr.LineStrips3D([obb]),
                    )

                if obj_id in obj_meshes_to_log:
                    rr.log(
                        f"world/objects/dynamic/{instance_info.name}/model",
                        ToTransform3D(bbox_3d.transform_scene_object),
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
                    f"world/objects/static/{instance_info.name}/bbox",
                    rr.LineStrips3D([obb]),
                    static=True,
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
