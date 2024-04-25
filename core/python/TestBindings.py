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

import numpy as np

from projectaria_tools.core import calibration, data_provider
from projectaria_tools.core.sensor_data import TimeDomain, TimeQueryOptions
from projectaria_tools.core.stream_id import RecordableTypeId


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vrs",
        dest="vrs_path",
        type=str,
        help="path to vrs file",
    )
    return parser.parse_args()


def print_streamid_label(provider):
    streams = provider.get_all_streams()
    for stream_id in streams:
        label = provider.get_label_from_stream_id(stream_id)
        print(
            f"{stream_id} and {label} and back to streamId {provider.get_stream_id_from_label(label)}"
        )


def print_sample_calibration(provider):
    # get calibration from provider and labels of all sensors in device
    device_calib = provider.get_device_calibration()
    assert device_calib is not None
    sensor_labels = device_calib.get_all_labels()
    print(f"device calibration contains calibrations for {sensor_labels}")

    # get calibration for the rgb camera
    camera_name = "camera-rgb"

    # extrinsics
    transform_device_rgb = device_calib.get_transform_device_sensor(camera_name)
    print(f"Device calibration origin label {device_calib.get_origin_label()}")
    print(f"{camera_name} has extrinsics of \n {transform_device_rgb.to_matrix()}")

    # intrinsics
    rgb_calib = device_calib.get_camera_calib(camera_name)

    # project a pixel in device frame to rgb camera
    point_device = np.array([0, 0, 10])
    point_camera = transform_device_rgb @ point_device

    maybe_pixel = rgb_calib.project(point_camera)
    if maybe_pixel is not None:
        print(
            f"Get pixel {maybe_pixel} within image of size {rgb_calib.get_image_size()}"
        )

    # obtain calibration data for ET sensor
    et_calib = device_calib.get_aria_et_camera_calib()
    print(et_calib[0].get_label())
    print(et_calib[1].get_label())
    print(et_calib[0].get_transform_device_camera())

    # obtain calibration data for IMU
    imu_calib = device_calib.get_imu_calib("imu-left")
    print(imu_calib.get_label())
    print(imu_calib.get_transform_device_imu())

    # obtain a rescaled camera calibration
    rescaled_calib = rgb_calib.rescale(new_resolution=(704, 704), scale=0.5)
    print("testing for camera calibration downscaling (factor = 0.5)")
    print(
        f"resolution for original camera is {rgb_calib.get_image_size()}, intrinsics params are: {rgb_calib.projection_params}, valid_radius is {rgb_calib.get_valid_radius()}"
    )
    print(
        f"resolution for new camera is {rescaled_calib.get_image_size()}, intrinsics params are: {rescaled_calib.projection_params}, valid_radius is {rescaled_calib.get_valid_radius()}"
    )


def print_distort_image(provider):
    camera_name = "camera-slam-left"
    device_calib = provider.get_device_calibration()
    src_calib = device_calib.get_camera_calib(camera_name)
    dst_calib = calibration.get_spherical_camera_calibration(512, 512, 150, camera_name)
    print(f"focalLengths {dst_calib.get_focal_lengths()}")
    print(f"principalPoint {dst_calib.get_principal_point()}")

    sensor_stream_id = provider.get_stream_id_from_label(camera_name)
    image_data = provider.get_image_data_by_index(sensor_stream_id, 0)
    image_array = image_data[0].to_numpy_array()
    rectified_array = calibration.distort_by_calibration(
        image_array, dst_calib, src_calib
    )
    print(image_array)
    print(rectified_array)


def print_sample_sequential_data(provider):
    # get sequential sensor data
    options = (
        provider.get_default_deliver_queued_options()
    )  # default options activates all streams
    options.set_truncate_first_device_time_ns(
        int(1e8)
    )  # 0.1 secs after vrs first timestamp
    options.set_truncate_last_device_time_ns(
        int(1e9)
    )  # 1 sec before vrs last timestamp

    # deactivate all sensors
    options.deactivate_stream_all()
    # activate only a subset of sensors
    slam_stream_ids = options.get_stream_ids(RecordableTypeId.SLAM_CAMERA_DATA)
    imu_stream_ids = options.get_stream_ids(RecordableTypeId.SLAM_IMU_DATA)

    for stream_id in slam_stream_ids:
        options.activate_stream(stream_id)  # activate slam cameras
        options.set_subsample_rate(
            stream_id, 1
        )  # sample every data for each slam camera

    for stream_id in imu_stream_ids:
        options.activate_stream(stream_id)  # activate imus
        options.set_subsample_rate(stream_id, 10)  # sample every 10th data for each imu

    iterator = provider.deliver_queued_sensor_data(options)
    for sensor_data in iterator:
        label = provider.get_label_from_stream_id(sensor_data.stream_id())
        time_domain = TimeDomain.DEVICE_TIME
        sensor_type = sensor_data.sensor_data_type()
        timestamp = sensor_data.get_time_ns(time_domain)
        print(
            f"obtain data from sensor {label} of sensor type {sensor_type} with time {timestamp}"
        )


def print_image_config(config):
    print(f"deviceType {config.device_type}")
    print(f"deviceVersion {config.device_version}")
    print(f"deviceSerial {config.device_serial}")
    print(f"sensorSerial {config.sensor_serial}")
    print(f"nominalRateHz {config.nominal_rate_hz}")
    print(f"imageWidth {config.image_width}")
    print(f"imageHeight {config.image_height}")
    print(f"pixelFormat {config.pixel_format}")


def print_sample_random_access_data(provider):
    # random access data from a specific sensor
    sensor_name = "camera-slam-right"
    sensor_stream_id = provider.get_stream_id_from_label(sensor_name)

    # get sensor data configuration
    config = provider.get_image_configuration(sensor_stream_id)
    print_image_config(config)

    # get all image data by index
    num_data = provider.get_num_data(sensor_stream_id)
    for index in range(0, num_data):
        imageData = provider.get_image_data_by_index(sensor_stream_id, index)
        print(
            f"Getting image from random accessor with timestamp {imageData[1].capture_timestamp_ns}"
        )

    # random access data from a set of timestamps
    time_domain = TimeDomain.DEVICE_TIME  # query data based on device time
    option = (
        TimeQueryOptions.CLOSEST
    )  # get data whose time [in TimeDomain] is Closest to query time
    start_time = provider.get_first_time_ns(sensor_stream_id, time_domain)
    end_time = provider.get_last_time_ns(sensor_stream_id, time_domain)
    for time in range(start_time, end_time, int(1e8)):
        image_data = provider.get_image_data_by_time_ns(
            sensor_stream_id, time, time_domain, option
        )
        print(
            f"query time {time} and get image time {image_data[1].arrival_timestamp_ns} within range {start_time} {end_time}"
        )


if __name__ == "__main__":
    args = parse_args()

    print("")
    print("------------------------------------------------")
    print(f"Creating data provider from {args.vrs_path}")
    provider = data_provider.create_vrs_data_provider(args.vrs_path)
    assert provider is not None
    print("")

    print("------------------------------------------------")
    print("Example: streamId to labels mapping")
    print_streamid_label(provider)
    print("")

    print("------------------------------------------------")
    print("Example: calibration")
    print_sample_calibration(provider)
    print("")

    print("------------------------------------------------")
    print("Example: get sensor data in a sequence")
    print_sample_sequential_data(provider)
    print("")

    print("------------------------------------------------")
    print("Example: random access sensor data")
    print_sample_random_access_data(provider)
    print("------------------------------------------------")

    print("------------------------------------------------")
    print("Example: distort image data")
    print_distort_image(provider)
    print("------------------------------------------------")
