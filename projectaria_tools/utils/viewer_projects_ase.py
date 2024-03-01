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

import numpy as np
import rerun as rr
from PIL import Image
from projectaria_tools.core import calibration
from projectaria_tools.core.sophus import SO3
from projectaria_tools.projects import ase

from projectaria_tools.projects.ase.interpreter import language_to_bboxes
from projectaria_tools.projects.ase.readers import (
    read_language_file,
    read_points_file,
    read_trajectory_file,
)
from projectaria_tools.utils.rerun_helpers import ToTransform3D


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset_path",
        type=str,
        required=True,
        help="path to dataset (root path / scene_id)",
    )
    parser.add_argument(
        "--frame_id",
        type=int,
        help="Display the frame_id of the scene as a 3D camera (if valid, i.e id exists in the dataset)",
    )

    # If this path is set, we will save the rerun (.rrd) file to the given path
    parser.add_argument(
        "--rrd_output_path", type=str, default="", help=argparse.SUPPRESS
    )

    return parser.parse_args()


PLOTTING_COLORS = {
    "wall": [251, 250, 245],
    "door": [247, 197, 159],
    "window": [83, 244, 255],
    "points": [199, 218, 232],
    "trajectory": [249, 42, 130],
}


def main():
    args = parse_args()

    scene_path = os.path.join(args.dataset_path)
    print("Chosen ASE data path: ", scene_path)

    #
    # Load input data
    # - Scene point cloud
    # - Device trajectory
    # - Scene command language
    #
    points_path = os.path.join(scene_path, "semidense_points.csv.gz")
    trajectory_path = os.path.join(scene_path, "trajectory.csv")
    language_path = os.path.join(scene_path, "ase_scene_language.txt")

    points = read_points_file(points_path)
    trajectory = read_trajectory_file(trajectory_path)
    entities = read_language_file(language_path)

    rr.init(
        "ASE Data Inspector",
        recording_id=None,
        spawn=(not args.rrd_output_path),
        default_enabled=True,
        strict=False,
    )
    if args.rrd_output_path:
        print(f"Saving .rrd file to {args.rrd_output_path}")
        rr.save(args.rrd_output_path)

    # Interpret scene commands into 3D Boxes
    entity_boxes = language_to_bboxes(entities)

    #
    # Plot language boxes
    # Using batches (all BBox at once)
    #

    boxes_labels = [
        entity_box["id"]
        if "wall" in entity_box["id"]
        else "walls/" + entity_box["id"]
        if "window" in entity_box["id"]
        else "windows/" + entity_box["id"]
        if "door" in entity_box["id"]
        else "doors/" + entity_box["id"]
        for entity_box in entity_boxes
    ]
    rr.log(
        "world/boxes",
        rr.Boxes3D(
            centers=[entity_box["center"] for entity_box in entity_boxes],
            half_sizes=[entity_box["scale"] * 0.5 for entity_box in entity_boxes],
            rotations=[
                np.roll(SO3.from_matrix(entity_box["rotation"]).to_quat()[0], -1)
                for entity_box in entity_boxes
            ],
            colors=[
                PLOTTING_COLORS[entity_box["class"]] for entity_box in entity_boxes
            ],
            labels=boxes_labels,
        ),
        timeless=True,
    )

    #
    # Plot camera trajectory
    #
    device_positions = np.stack(
        [pose.translation()[0] for pose in trajectory["Ts_world_from_device"]]
    )

    rr.log(
        "world/device_trajectory",
        rr.LineStrips3D([device_positions], colors=[PLOTTING_COLORS["trajectory"]]),
        timeless=True,
    )

    #
    # Plot time based camera trajectory
    #

    # Load camera calibration
    device = ase.get_ase_rgb_calibration()
    T_device_from_camera = device.get_transform_device_camera()
    for frame_id, (timestamp_ns, pose) in enumerate(
        zip(
            trajectory["timestamps"],
            trajectory["Ts_world_from_device"],
        )
    ):
        rr.set_time_nanos("device_time", int(timestamp_ns * 1e3))  # convert to us to ns
        rr.set_time_sequence("frame_id", frame_id)
        T_world_from_device = pose  # SE3.from_matrix(pose)
        T_world_camera = T_world_from_device @ T_device_from_camera
        rr.log("world/device", ToTransform3D(T_world_camera, False))

        rr.log(
            "world/device",
            rr.Pinhole(
                resolution=[device.get_image_size()[0], device.get_image_size()[1]],
                focal_length=float(device.get_focal_lengths()[0]),
            ),
        )

    #
    # Plot scene points
    #
    max_points_to_plot = 500_000
    if len(points) > max_points_to_plot:
        print(
            f"For performance reason we reduce number of points from {len(points)} to {max_points_to_plot}."
        )
        sampled = np.random.choice(len(points), max_points_to_plot, replace=False)
        points = points[sampled]

    rr.log(
        "world/points",
        rr.Points3D(points, colors=PLOTTING_COLORS["points"], radii=0.005),
        timeless=True,
    )

    #
    # Plot image (if 'frame_id' is specified)
    #

    # Check that we can load the specific frame id
    rgb_dir = os.path.join(scene_path, "rgb")
    num_frames = len(list(Path(rgb_dir).glob("*.jpg")))
    if args.frame_id is not None and args.frame_id > num_frames and args.frame_id >= 0:
        print(
            "frame_id is too large, please specify a valid one, no frame will be displayed"
        )
        args.frame_id = None

    if args.frame_id is None:
        exit(1)

    # We have a frame to display
    # - Load the image
    # - Log a camera view of it and stich the image to it

    # Load image
    frame_idx = args.frame_id
    frame_id = str(frame_idx).zfill(7)
    rgb_path = os.path.join(rgb_dir, f"vignette{frame_id}.jpg")
    rgb = Image.open(rgb_path)

    # Retrieve extrinsic and
    T_world_from_device = trajectory["Ts_world_from_device"][frame_idx]
    T_world_camera = T_world_from_device @ T_device_from_camera
    # intrinsic (undistorted camera model), used to show a undistorted RGB image
    pinhole = calibration.get_linear_camera_calibration(
        device.get_image_size()[0],
        device.get_image_size()[1],
        device.get_focal_lengths()[0],
    )

    # Log the camera pose
    rr.log(
        f"world/frame/{frame_idx}", ToTransform3D(T_world_camera, False), timeless=True
    )
    # Log the pinhole camera model
    rr.log(
        f"world/frame/{frame_idx}",
        rr.Pinhole(
            resolution=[pinhole.get_image_size()[0], pinhole.get_image_size()[1]],
            focal_length=float(pinhole.get_focal_lengths()[0]),
        ),
        timeless=True,
    )
    # Log the RGB image (undistorted so we have pixel perfect reprojection in Rerun UI)
    rgb = Image.open(rgb_path)
    undistorted_rgb = calibration.distort_by_calibration(rgb, pinhole, device)
    rr.log(f"world/frame/{frame_idx}", rr.Image(undistorted_rgb), timeless=True)


if __name__ == "__main__":
    main()
