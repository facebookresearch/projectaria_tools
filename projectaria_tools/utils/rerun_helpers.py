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
import rerun as rr
from projectaria_tools.core import calibration
from projectaria_tools.core.calibration import DeviceCalibration, DeviceVersion
from projectaria_tools.core.mps import hand_tracking

from projectaria_tools.core.sophus import SE3


#
# Rerun helper functions
#
def ToTransform3D(
    pose: SE3, from_parent: bool = False, axis_length: float = 1e-2
) -> rr.Transform3D:
    """
    Helper function to convert Sophus SE3D pose to a Rerun Transform3D
    """
    quat_xyzw = np.roll(
        pose.rotation().to_quat()[0], -1
    )  # change from w,x,y,z to x,y,z,w
    transform_3d = rr.Transform3D(
        axis_length=axis_length,
        translation=pose.translation()[0],
        rotation=rr.Quaternion(xyzw=quat_xyzw),
    )

    if from_parent:
        transform_3d.relation = rr.TransformRelation.ChildFromParent

    return transform_3d


def ToBox3D(label: str, size=[1e-3, 1e-3, 1e-3]) -> rr.Boxes3D:
    """
    Helper function to create one 3D box with a given label and size
    """
    return rr.Boxes3D(
        centers=[0, 0, 0],
        sizes=[size],
        fill_mode="solid",
        labels=[label],
    )


def AriaGlassesOutline(
    device_calibration: DeviceCalibration, use_cad_calib: bool = True
) -> [np.ndarray]:
    """
    Return a list of points to be used to draw the outline of the glasses (line strip).
    - Points are returned in Local coordinate system and can be used as the following:
        rr.log(
                "world/device/glasses_outline",
                rr.LineStrips3D([aria_glasses_point_outline]),
        )
    """
    device_version = device_calibration.get_device_version()
    if device_version == DeviceVersion.Gen1:
        left_corner_label = "camera-slam-left"
        right_corner_label = "camera-slam-right"
    elif device_version == DeviceVersion.Gen2:
        left_corner_label = "slam-front-left"
        right_corner_label = "slam-front-right"
    else:
        raise ValueError(
            f"Unsupported device version {calibration.get_name(device_version)}"
        )

    glasses_outline_indexes = [
        "mic5",
        left_corner_label,
        "mic2",
        "mic1",
        "baro0",
        "mic1",
        left_corner_label,
        right_corner_label,
        "mic0",
        "baro0",
        right_corner_label,
        "mic6",
    ]
    points = []
    for index in glasses_outline_indexes:
        if "mic" in index or "mag" in index or "baro" in index:
            get_cad_value = True
        else:
            get_cad_value = use_cad_calib
        T_device_sensor = device_calibration.get_transform_device_sensor(
            index, get_cad_value=get_cad_value
        )
        points.append(T_device_sensor.translation()[0])
    return points


HandLandmark = hand_tracking.HandLandmark


def create_hand_skeleton_segments_from_landmarks(
    all_landmark_locations: List,
    segment_landmark_names: List[HandLandmark],
):
    """
    This is a helper function to create a list of hand skeleton segments from its landmark location, and landmark names
    """
    skeleton_segments = []

    # insert pairs into outline segments
    for i in range(len(segment_landmark_names) - 1):
        start_index = segment_landmark_names[i]
        end_index = segment_landmark_names[i + 1]
        skeleton_segments.append(
            [all_landmark_locations[start_index], all_landmark_locations[end_index]]
        )
    return skeleton_segments


def create_hand_skeleton_from_landmarks(landmark_locations: List):
    """
    This is a helper function to create an outline of hand from its landmark locations (for both 3D and 2D).
    """
    hand_skeleton = []
    # Palm shape
    hand_skeleton.extend(
        create_hand_skeleton_segments_from_landmarks(
            landmark_locations,
            [
                HandLandmark.WRIST,
                HandLandmark.THUMB_INTERMEDIATE,
                HandLandmark.INDEX_PROXIMAL,
                HandLandmark.MIDDLE_PROXIMAL,
                HandLandmark.RING_PROXIMAL,
                HandLandmark.PINKY_PROXIMAL,
                HandLandmark.WRIST,
                HandLandmark.PALM_CENTER,
            ],
        )
    )

    # Thumb line
    hand_skeleton.extend(
        create_hand_skeleton_segments_from_landmarks(
            landmark_locations,
            [
                HandLandmark.WRIST,
                HandLandmark.THUMB_INTERMEDIATE,
                HandLandmark.THUMB_DISTAL,
                HandLandmark.THUMB_FINGERTIP,
            ],
        )
    )

    # Index line
    hand_skeleton.extend(
        create_hand_skeleton_segments_from_landmarks(
            landmark_locations,
            [
                HandLandmark.WRIST,
                HandLandmark.INDEX_PROXIMAL,
                HandLandmark.INDEX_INTERMEDIATE,
                HandLandmark.INDEX_DISTAL,
                HandLandmark.INDEX_FINGERTIP,
            ],
        )
    )

    # Middle line
    hand_skeleton.extend(
        create_hand_skeleton_segments_from_landmarks(
            landmark_locations,
            [
                HandLandmark.WRIST,
                HandLandmark.MIDDLE_PROXIMAL,
                HandLandmark.MIDDLE_INTERMEDIATE,
                HandLandmark.MIDDLE_DISTAL,
                HandLandmark.MIDDLE_FINGERTIP,
            ],
        )
    )

    # Ring line
    hand_skeleton.extend(
        create_hand_skeleton_segments_from_landmarks(
            landmark_locations,
            [
                HandLandmark.WRIST,
                HandLandmark.RING_PROXIMAL,
                HandLandmark.RING_INTERMEDIATE,
                HandLandmark.RING_DISTAL,
                HandLandmark.RING_FINGERTIP,
            ],
        )
    )

    # Pinky line
    hand_skeleton.extend(
        create_hand_skeleton_segments_from_landmarks(
            landmark_locations,
            [
                HandLandmark.WRIST,
                HandLandmark.PINKY_PROXIMAL,
                HandLandmark.PINKY_INTERMEDIATE,
                HandLandmark.PINKY_DISTAL,
                HandLandmark.PINKY_FINGERTIP,
            ],
        )
    )

    # Remove segments that may contain empty pixels
    hand_skeleton = list(
        filter(lambda x: x[0] is not None and x[1] is not None, hand_skeleton)
    )

    return hand_skeleton
