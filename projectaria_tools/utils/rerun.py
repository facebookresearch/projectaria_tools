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

import numpy as np
import rerun as rr
from projectaria_tools.core.calibration import DeviceCalibration

from projectaria_tools.core.sophus import SE3

#
# Rerun helper functions
#


def ToTransform3D(pose: SE3, from_parent: bool = False) -> rr.Transform3D:
    """
    Helper function to convert Sophus SE3D pose to a Rerun Transform3D
    """
    quat_xyzw = np.roll(
        pose.rotation().to_quat()[0], -1
    )  # change from w,x,y,z to x,y,z,w
    return rr.Transform3D(
        translation=pose.translation()[0],
        rotation=rr.Quaternion(xyzw=quat_xyzw),
        from_parent=from_parent,
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
    glasses_outline_indexes = [
        "mic5",
        "camera-slam-left",
        "mic2",
        "mic1",
        "baro0",
        "mic1",
        "camera-slam-left",
        "camera-slam-right",
        "mic0",
        "baro0",
        "camera-slam-right",
        "mic6",
    ]
    points = []
    for index in glasses_outline_indexes:
        T_device_sensor = device_calibration.get_transform_device_sensor(
            index, True if "mic" or "mag" or "baro" in index else use_cad_calib
        )
        points.append(T_device_sensor.translation()[0])
    return points
