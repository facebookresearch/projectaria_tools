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

"""
A utilities module for Aria Digital Twin dataset tools
"""

import csv
import io
import json
from typing import Dict, List, Optional
from zipfile import ZipFile

import numpy as _np

from projectaria_tools.core.calibration import CameraCalibration as _CameraCalibration
from projectaria_tools.projects.adt import (
    bbox3d_to_line_coordinates as _bbox3d_to_line_coordinates,
)

from scipy.spatial.transform import Rotation as R

# The minimum angular step between two consecutive poses to for continuous symmetry
MIN_SYM_ANGLE_STEP = 0.01

# Vertices of x, y, z axis for visualization.
VERTICES_AXIS = _np.array(
    [
        [0, 0, 0],
        [0.2, 0, 0],
        [0, 0.2, 0],
        [0, 0, 0.2],
    ]
)

# Project an obj AABB into the image coordinate frame of a camera
#
# Inputs:
#  - aabb: numpy.ndarray [6x1: xmin, xmax, ymin, ymax, zmin, zmax]
#  - transform_cam_obj: numpy.ndarray [4x4]
#  - cam_calibration: pyCamCalib
# Output:
# - projected_line_strips: numpy.ndarray in the format of
#   [b1, b2, b3, b4, b1, t1, t2, t3, t4, t1, t2, b2, b3, t3, t4, b4]
def project_3d_bbox_to_image(
    aabb: List[float],
    transform_cam_obj: _np.ndarray,
    cam_calibration: _CameraCalibration,
) -> List[int]:
    # convert to actual coordinates
    aabb_coords = _bbox3d_to_line_coordinates(aabb)
    # convert coordinates to camera frame
    num_points = len(aabb_coords)
    bbox_3d_in_cam = _np.ndarray(shape=(num_points, 3))
    for i in range(0, num_points):
        p_aabb_homo = _np.append(aabb_coords[i], [1])
        p_in_cam_homo = _np.matmul(transform_cam_obj, p_aabb_homo)
        bbox_3d_in_cam[i, :] = p_in_cam_homo[0:3]

    # project into image, break if any points don't project
    projected_line_strips = []
    for i in range(0, num_points):
        pt_in_cam = bbox_3d_in_cam[i, :]
        pt_projected = cam_calibration.project(pt_in_cam)
        if pt_projected is None:
            break
        projected_line_strips.append(pt_projected)
    if len(projected_line_strips) != num_points:
        print("full bounding box did not reproject to image, not drawing")
        return []
    return projected_line_strips


def get_timed_homo_poses(archive: ZipFile, seq_name: str):
    """
    Get the poses (4x4 matrix) from an archive file.
    Args:
        archive: the archive file object.
        seq_name: the name of the sequence to extract poses from.
    Returns:
        A dictionary mapping timestamp to poses of a list of objects indexed
        by prototype name.
    """
    timed_poses = {}
    with archive.open(f"{seq_name}.csv", "r") as fp:
        reader = csv.DictReader(io.TextIOWrapper(fp))
        for row in reader:
            timestamp = int(row["timestamp_ns"])
            prototype = row["prototype"]
            if timestamp not in timed_poses:
                timed_poses[timestamp] = {}
            if prototype not in timed_poses[timestamp]:
                timed_poses[timestamp][prototype] = []
            transform_scene_object = _np.zeros([4, 4])
            transform_scene_object[3, 3] = 1
            transform_scene_object[:3, 3] = _np.array(
                [float(row["t_wo_x"]), float(row["t_wo_y"]), float(row["t_wo_z"])]
            )
            transform_scene_object[:3, :3] = R.from_quat(
                [
                    float(row["q_wo_x"]),
                    float(row["q_wo_y"]),
                    float(row["q_wo_z"]),
                    float(row["q_wo_w"]),
                ]
            ).as_matrix()
            timed_poses[timestamp][prototype].append(transform_scene_object)
    return timed_poses


def get_rotation_matrices(symmetry: Dict) -> List[_np.ndarray]:
    """
    Get rotation matrices given symmetry parameters.

    Every rotation matrices will rotate the object about a symmetric axis and
    perfectly fit the original one.
    Args:
        The symmetry parameters (Dict).
    Returns:
        The rotation matrices.
    """
    out = []
    if symmetry["angle_deg"] == 0:
        angles = _np.arange(0, 2 * _np.pi, MIN_SYM_ANGLE_STEP)
    else:
        angles = _np.arange(0, 2 * _np.pi, symmetry["angle_deg"] * _np.pi / 180)
    for angle in angles:
        rotvec = _np.array(symmetry["axis"]) * angle
        out.append(R.from_rotvec(rotvec).as_matrix())
    return out


def apply_pose(
    pose: Dict, vertices: _np.ndarray, sym_rot: Optional[_np.ndarray] = None
) -> _np.ndarray:
    """
    Apply the pose to the vertices with optional symmetry rotation matrix.
    Args:
        pose: The pose parameters (Dict) containing translation and rotation.
        vertices: The vertices (np.ndarray).
        sym_rot: The symmetry rotation matrix.
    Returns:
        The vertices after applying the pose.
    """
    translation = pose["translation"]
    rotation = pose["rotation"].as_matrix()
    if sym_rot is None:
        return (rotation @ vertices.T).T + translation
    else:
        return (rotation @ sym_rot @ vertices.T).T + translation


def compute_mssd(
    predicted_pose: Dict,
    annotated_pose: Dict,
    symmetries: List[Dict],
    vertices: _np.ndarray,
):
    """
    Compute the Maximum Symmetry-aware Surface Distance (MSSD) between
    predicted and annotated poses.
    Args:
        predicted_pose: The predicted pose parameters (Dict) containing
            translation and rotation.
        annotated_pose: The annotated pose parameters (Dict) containing
            translation and rotation.
        symmetries: The symmetry parameters (List).
        vertices: The vertices (np.ndarray).
    Returns:
        The MSSD between predicted and annotated poses.
    """
    vertices_pred = apply_pose(predicted_pose, vertices)
    vertices_anno = apply_pose(annotated_pose, vertices)
    mssd = _np.linalg.norm(vertices_pred - vertices_anno, 2, axis=1).max()
    for symmetry in symmetries:
        for sym_rot in get_rotation_matrices(symmetry):
            vertices_anno = apply_pose(annotated_pose, vertices, sym_rot)
            error = _np.linalg.norm(vertices_pred - vertices_anno, 2, axis=1).max()
            mssd = min(mssd, error)
    return mssd


def voc_ap(rec, prec):
    """Compute VOC AP given precision and recall.

    Args: rec, prec (list): precision and recall values
    Returns: average precision
    """
    # correct AP calculation
    # first append sentinel values at the end
    mrec = _np.concatenate(([0.0], rec, [1.0]))
    mpre = _np.concatenate(([0.0], prec, [0.0]))

    # compute the precision envelope
    for i in range(mpre.size - 1, 0, -1):
        mpre[i - 1] = _np.maximum(mpre[i - 1], mpre[i])

    # to calculate area under PR curve, look for points
    # where X axis (recall) changes value
    i = _np.where(mrec[1:] != mrec[:-1])[0]

    # and sum (\Delta recall) * prec
    ap = _np.sum((mrec[i + 1] - mrec[i]) * mpre[i + 1])
    return ap


def get_timed_poses(archive, seq_name: str):
    """
    Get the timed poses from the archive file.
    Args:
        archive: The archive file object.
        seq_name: The sequence name.
    Returns:
        The timed poses (Dict).
    """
    timed_poses = {}
    with archive.open(f"{seq_name}.csv", "r") as fp:
        reader = csv.DictReader(io.TextIOWrapper(fp))
        for row in reader:
            timestamp = row["timestamp_ns"]
            prototype = row["prototype"]
            if timestamp not in timed_poses:
                timed_poses[timestamp] = {}
            if prototype not in timed_poses[timestamp]:
                timed_poses[timestamp][prototype] = []
            translation = _np.array(
                [float(row["t_wo_x"]), float(row["t_wo_y"]), float(row["t_wo_z"])]
            )
            rotation = R.from_quat(
                [
                    float(row["q_wo_x"]),
                    float(row["q_wo_y"]),
                    float(row["q_wo_z"]),
                    float(row["q_wo_w"]),
                ]
            )
            timed_poses[timestamp][prototype].append(
                {
                    "translation": translation,
                    "rotation": rotation,
                }
            )
    return timed_poses


def get_vertices(archive):
    """
    Get the vertices from the archive file.
    Args:
        archive: The archive file object.
    Returns:
        The vertices (Dict) for each prototype.
    """
    with archive.open("vertices.json", "r") as fp:
        all_vertices = json.load(io.TextIOWrapper(fp))
    all_vertices = {k: _np.array(v) for k, v in all_vertices.items()}
    return all_vertices


def get_3d_bounding_box(archive):
    """
    Get the 3D bounding boxes from the archive file.
    Args:
        archive: The archive file object.
    Returns:
        The 3D bounding box for each prototype.
    """
    all_bboxes3d = {}
    with archive.open("3d_bounding_box.csv", "r") as fp:
        reader = csv.DictReader(io.TextIOWrapper(fp))
        for row in reader:
            all_bboxes3d[row["prototype"]] = _np.array(
                [
                    [
                        float(row["p_local_obj_xmin[m]"]),
                        float(row["p_local_obj_ymin[m]"]),
                        float(row["p_local_obj_zmin[m]"]),
                    ],
                    [
                        float(row["p_local_obj_xmin[m]"]),
                        float(row["p_local_obj_ymax[m]"]),
                        float(row["p_local_obj_zmin[m]"]),
                    ],
                    [
                        float(row["p_local_obj_xmax[m]"]),
                        float(row["p_local_obj_ymax[m]"]),
                        float(row["p_local_obj_zmin[m]"]),
                    ],
                    [
                        float(row["p_local_obj_xmax[m]"]),
                        float(row["p_local_obj_ymin[m]"]),
                        float(row["p_local_obj_zmin[m]"]),
                    ],
                    [
                        float(row["p_local_obj_xmin[m]"]),
                        float(row["p_local_obj_ymin[m]"]),
                        float(row["p_local_obj_zmax[m]"]),
                    ],
                    [
                        float(row["p_local_obj_xmin[m]"]),
                        float(row["p_local_obj_ymax[m]"]),
                        float(row["p_local_obj_zmax[m]"]),
                    ],
                    [
                        float(row["p_local_obj_xmax[m]"]),
                        float(row["p_local_obj_ymax[m]"]),
                        float(row["p_local_obj_zmax[m]"]),
                    ],
                    [
                        float(row["p_local_obj_xmax[m]"]),
                        float(row["p_local_obj_ymin[m]"]),
                        float(row["p_local_obj_zmax[m]"]),
                    ],
                ]
            )
    return all_bboxes3d


def get_symmetries(archive):
    """
    Get the symmetries from the archive file.
    Args:
        archive: The archive file object.
    Returns:
        The symmetries (Dict) for each prototype.
    """
    with archive.open("symmetries.json", "r") as fp:
        all_symmetries = json.load(io.TextIOWrapper(fp))
    return all_symmetries


def get_diameters(archive):
    """
    Get the diameters from the archive file.
    Args:
        archive: The archive file object.
    Returns:
        The diameters (Dict) for each prototype.
    """
    with archive.open("diameters.json", "r") as fp:
        all_diameters = json.load(io.TextIOWrapper(fp))
    return all_diameters
