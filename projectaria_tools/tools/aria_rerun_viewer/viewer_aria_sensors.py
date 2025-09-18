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
import sys

import rerun as rr

from projectaria_tools.core import data_provider
from projectaria_tools.core.calibration import DeviceVersion
from projectaria_tools.utils.rerun_helpers import (
    AriaGlassesOutline,
    ToBox3D,
    ToTransform3D,
)


def main():
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--vrs",
            type=str,
            required=True,
            help="path to the VRS file (calibration will be used to display the Sensors locations and orientations)",
        )
        parser.add_argument(
            "--use_cad_calib",
            type=eval,
            choices=[True, False],
            default="True",
            help="Use CAD value, or Factory calibration data (default use CAD calibration)",
        )

        # If this path is set, we will save the rerun (.rrd) file to the given path
        parser.add_argument(
            "--rrd_output_path", type=str, default="", help=argparse.SUPPRESS
        )
        parser.add_argument(
            "--web",
            type=eval,
            choices=[True, False],
            default="False",
            help=argparse.SUPPRESS,
        )

        args = parser.parse_args()
    except SystemExit as e:
        print(f"Error: {e}")
        sys.exit(1)

    ##
    # Retrieve device calibration and plot sensors locations, orientations
    provider = data_provider.create_vrs_data_provider(args.vrs)
    if not provider:
        print("Couldn't create data provider from vrs file")
        exit(1)

    device_calibration = provider.get_device_calibration()

    # Spawn rerun and log things we want to see
    rr.init(
        "Aria Sensors Data Viewer", spawn=(not args.rrd_output_path and not args.web)
    )

    # Run the viewer in the web browser or desktop app
    if args.web:
        rr.serve_web()
    else:
        rr.spawn()

    if args.rrd_output_path:
        print(f"Saving .rrd file to {args.rrd_output_path}")
        rr.save(args.rrd_output_path)
    # Aria coordinate system sets X down, Z in front, Y Left
    device_version = device_calibration.get_device_version()
    if device_version == DeviceVersion.Gen1:
        rr.log("device", rr.ViewCoordinates.RIGHT_HAND_X_DOWN, static=True)
    elif device_version == DeviceVersion.Gen2:
        rr.log("device", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

    cam_labels = device_calibration.get_camera_labels()
    print(f"Log {len(cam_labels)} Cameras")
    for cam in cam_labels:
        camera_calibration = device_calibration.get_camera_calib(cam)
        T_device_sensor = camera_calibration.get_transform_device_camera()
        rr.log(f"device/camera/{cam}", ToTransform3D(T_device_sensor))
        rr.log(
            f"device/camera/{cam}",
            rr.Pinhole(
                resolution=[
                    camera_calibration.get_image_size()[0],
                    camera_calibration.get_image_size()[1],
                ],
                focal_length=float(camera_calibration.get_focal_lengths()[0]),
            ),
        )
    mic_labels = device_calibration.get_microphone_labels()
    print(f"Log {len(mic_labels)} Microphones")
    for mic in mic_labels:  # Note: Only defined in CAD extrinsics
        # Skip Left and Right Speakers because they are not in the CAD
        if mic in ["LSPK", "RSPK"]:
            continue
        T_device_sensor = device_calibration.get_transform_device_sensor(mic, True)
        rr.log(f"device/mic/{mic}", ToTransform3D(T_device_sensor))
        rr.log(
            f"device/mic/{mic}/box",
            ToBox3D(mic, [3e-3, 1.5e-3, 2e-3]),
        )

    imu_labels = device_calibration.get_imu_labels()
    print(f"Log {len(imu_labels)} IMUs")
    for imu in imu_labels:
        T_device_sensor = device_calibration.get_transform_device_sensor(
            imu, args.use_cad_calib
        )
        rr.log(f"device/imu/{imu}", ToTransform3D(T_device_sensor))
        rr.log(
            f"device/imu/{imu}",
            ToBox3D(imu, [5e-3, 4e-3, 3e-3]),
        )

    magnetometer_labels = device_calibration.get_magnetometer_labels()
    print(f"Log {len(magnetometer_labels)} Magnetometer")
    for magnetometer in magnetometer_labels:  # Note: Only defined in CAD calibration
        T_device_sensor = device_calibration.get_transform_device_sensor(
            magnetometer, True
        )
        rr.log(f"device/{magnetometer}", ToTransform3D(T_device_sensor))
        rr.log(
            f"device/{magnetometer}/box",
            ToBox3D(magnetometer, [3e-3, 3e-3, 2e-3]),
        )

    barometer_labels = device_calibration.get_barometer_labels()
    print(f"Log {len(barometer_labels)} Barometer")
    for barometer in barometer_labels:  # Note: Only defined in CAD calibration
        T_device_sensor = device_calibration.get_transform_device_sensor(
            barometer, True
        )
        rr.log(f"device/{barometer}", ToTransform3D(T_device_sensor))
        rr.log(
            f"device/{barometer}",
            ToBox3D(barometer, [3e-3, 3e-3, 2e-3]),
        )

    # Plot CPF (Central Pupil Frame coordinate system)
    T_device_CPF = device_calibration.get_transform_device_cpf()
    rr.log("device/CPF_CentralPupilFrame", ToTransform3D(T_device_CPF))
    rr.log(
        "device/CPF_CentralPupilFrame",
        ToBox3D("CPF_CentralPupilFrame", [5e-3, 5e-3, 5e-3]),
    )

    # Plot Project Aria Glasses outline (as lines)
    aria_glasses_point_outline = AriaGlassesOutline(
        device_calibration, args.use_cad_calib
    )
    rr.log("device/glasses_outline", rr.LineStrips3D([aria_glasses_point_outline]))

    ##
    # Use the ability to log a markdown file to ease how users can understand what they are looking at
    rr.log(
        "Notes",
        rr.TextDocument(
            """
  # Project Aria Glasses - Sensor Viewer

  Here is a visual preview of the Project Aria glasses sensors (location and orientation).

  Cameras sensors:

  - two (Gen1) / four (Gen2) Mono SLAM cameras
  - one [RGB](recording://device/camera/camera-rgb)
  - two Eye Tracking cameras [Eye left](recording://device/camera/camera-et-left), [Eye right](recording://device/camera/camera-et-right)

  Non-visual sensors:

  - microphones
  - two IMUs [Imu-left](recording://device/imu/imu-left), [Imu-right](recording://device/imu/imu-right)
  - [magnetometer](recording://device/mag0)
  - [barometer](recording://device/baro0)
  - GPS, Wi-Fi beacon, Bluetooth beacon.
        """,
            media_type=rr.MediaType.MARKDOWN,
        ),
    )
    if args.web:
        # Keep the server running
        try:
            while True:
                pass
        except KeyboardInterrupt:
            print("Shutting down server...")


#
# Project Aria glasses have five cameras (two Mono Scene, one RGB, and two Eye Tracking cameras) as well as non-visual sensors (two IMUs, magnetometer, barometer, GPS, Wi-Fi beacon, Bluetooth beacon and Microphones).

if __name__ == "__main__":
    main()
