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
from projectaria_tools.core import calibration


def undistort_image_and_calibration(
    input_image: np.ndarray,
    input_calib: calibration.CameraCalibration,
) -> [np.ndarray, calibration.CameraCalibration]:
    """
    Return the undistorted image and the updated camera calibration.
    """
    input_calib_width = input_calib.get_image_size()[0]
    input_calib_height = input_calib.get_image_size()[1]
    input_calib_focal = input_calib.get_focal_lengths()[0]
    if (
        input_image.shape[0] != input_calib_width
        or input_image.shape[1] != input_calib_height
    ):
        raise ValueError(
            f"Input image shape {input_image.shape} does not match calibration {input_calib.get_image_size()}"
        )

    # Undistort the image
    pinhole = calibration.get_linear_camera_calibration(
        int(input_calib_width),
        int(input_calib_height),
        input_calib_focal,
        "pinhole",
        input_calib.get_transform_device_camera(),
    )
    updated_calib = pinhole
    output_image = calibration.distort_by_calibration(
        input_image, updated_calib, input_calib
    )

    return output_image, updated_calib


def rotate_upright_image_and_calibration(
    input_image: np.ndarray,
    input_calib: calibration.CameraCalibration,
) -> [np.ndarray, calibration.CameraCalibration]:
    """
    Return the rotated upright image and update both intrinsics and extrinsics of the camera calibration
    NOTE: This function only supports pinhole and fisheye624 camera model.
    """
    output_image = np.rot90(input_image, k=3)
    updated_calib = calibration.rotate_camera_calib_cw90deg(input_calib)

    return output_image, updated_calib
