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

from collections import Counter

import numpy as np


CLASS_LABELS = {
    "wall": 0,
    "door": 1,
    "window": 2,
}


def z_rotation(angle):
    """
    Helper function to convert angle around z-axis to rotation matrix
    Args:
        angle: Float. Angle around z-axis in radians
    Returns:
        rot_matrix: np.ndarray. 3x3 Rotation matrix
    """
    s = np.sin(angle)
    c = np.cos(angle)
    rot_matrix = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return rot_matrix


def _compute_counts(boxes):
    class_counts = Counter([b["class"] for b in boxes])
    print("Scene contains:")
    for class_name, count in class_counts.items():
        print(f"  {class_name}: {count}")


# This function converts language entities to corresponding bounding boxes
def language_to_bboxes(entities):
    """
    Args:
        entities: List. List of (cmd,params) tuples that contain the entity command and its parameters
    """
    box_definitions = []
    # lookup table
    lookup = {}
    for command, params in entities:
        if command == "make_wall":
            identifier = int(params["id"])
            height = params["height"]
            thickness = params["thickness"]
            # corners
            corner_a = np.array([params["a_x"], params["a_y"], params["a_z"]])
            corner_b = np.array([params["b_x"], params["b_y"], params["b_z"]])
            length = np.linalg.norm(corner_a - corner_b)

            direction = corner_b - corner_a
            angle = np.arctan2(direction[1], direction[0])
            lookup[identifier] = {**params, "angle": angle}

            center = (corner_a + corner_b) * 0.5 + np.array([0, 0, 0.5 * height])
            scale = np.array([length, thickness, height])
            rotation = z_rotation(angle)
        elif command in {"make_door", "make_window"}:
            # skip if pointer is None
            for key in ["wall_id", "wall0_id", "wall1_id"]:
                wall_id = params.get(key, None)
                if wall_id is not None and wall_id >= 0:
                    break
            # skip if wall is None
            if wall_id is None:
                continue
            wall = lookup.get(wall_id, None)
            if wall is None:
                continue
            identifier = int(params["id"])
            angle, thickness = wall["angle"], wall["thickness"]

            center = np.array(
                [
                    params["position_x"],
                    params["position_y"],
                    params["position_z"],
                ]
            )
            rotation = z_rotation(angle)
            scale = np.array([params["width"], thickness, params["height"]])
        else:
            print(f"Entity to box conversion not implemented for: cmd={command}")
            continue

        class_name = command[5:]  # remove "make_"
        class_label = CLASS_LABELS[command[5:]]
        identifier_label = f"{class_name}{identifier}"
        box = {
            "id": identifier_label,
            "cmd": command,
            "class": class_name,
            "label": class_label,
            "center": center,
            "rotation": rotation,
            "scale": scale,
        }
        box_definitions.append(box)
    _compute_counts(box_definitions)
    return box_definitions
