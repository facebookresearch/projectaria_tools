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

from typing import Tuple

from projectaria_tools.core.calibration import CameraCalibration
from projectaria_tools.core.data_provider import VrsDataProvider
from projectaria_tools.core.mps import MpsDataProvider
from projectaria_tools.core.sophus import SE3
from projectaria_tools.core.stream_id import StreamId


def OnlineRgbCameraHelper(
    vrs_data_provider: VrsDataProvider,
    mps_data_provider: MpsDataProvider,
    device_time_ns: int,
) -> Tuple[CameraCalibration, SE3]:
    """
    Retrieve the most accurate RGB calibration (online if available, otherwise static)
    """
    #
    # Retrieve the RGB camera calibration
    corrected_rgb_pose = mps_data_provider.get_rgb_corrected_closed_loop_pose(
        device_time_ns
    )
    if corrected_rgb_pose is None:
        # Deal with this case
        exit(1)

    rgb_stream_id = StreamId("214-1")
    rgb_stream_name = vrs_data_provider.get_label_from_stream_id(rgb_stream_id)

    online_camera_calibration = mps_data_provider.get_online_calibration(
        device_time_ns
    ).get_camera_calib(rgb_stream_name)

    # Handle the case if the MPS online calibration output is a OLD (and does not have up to date RGB camera calibration)
    # by testing if the camera resolution is matching the image stream size
    if online_camera_calibration.get_image_size().tolist() != [
        vrs_data_provider.get_configuration(rgb_stream_id)
        .image_configuration()
        .image_width,
        vrs_data_provider.get_configuration(rgb_stream_id)
        .image_configuration()
        .image_height,
    ]:
        print(
            "You are most likely using an old MPS SLAM output, please update it to a more recent version for more accurate results",
            end="",
        )
        # As a backup solution, we will use the static camera calibration
        device_calibration = vrs_data_provider.get_device_calibration()
        online_camera_calibration = device_calibration.get_camera_calib(rgb_stream_name)

    return online_camera_calibration, corrected_rgb_pose
