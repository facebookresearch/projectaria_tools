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

    # Print device version
    device_version = device_calib.get_device_version()
    print(f"device version is {calibration.get_name(device_version)}")

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


def print_distort_image(provider, camera_name):
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


def print_color_corrected_image(provider):
    # mothod 1: color correct image data by calling api from device calibration
    sensor_stream_id = provider.get_stream_id_from_label("camera-rgb")
    image_data = provider.get_image_data_by_index(sensor_stream_id, 0)
    image_array = image_data[0].to_numpy_array()
    device_version = provider.get_device_calibration().get_device_version()
    color_corrected_array_1 = calibration.color_correct(image_array, device_version)
    print(color_corrected_array_1)

    # method 2: color correct image data by set color correction in VrsDataProvider
    provider.set_color_correction(True)
    image_data = provider.get_image_data_by_index(sensor_stream_id, 0)
    color_corrected_array_2 = image_data[0].to_numpy_array()
    print(color_corrected_array_2)
    assert color_corrected_array_1.all() == color_corrected_array_2.all()


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
    eyegaze_stream_ids = options.get_stream_ids(RecordableTypeId.GAZE_RECORDABLE_CLASS)
    pose_stream_ids = options.get_stream_ids(RecordableTypeId.POSE_RECORDABLE_CLASS)

    for stream_id in slam_stream_ids:
        options.activate_stream(stream_id)  # activate slam cameras
        options.set_subsample_rate(
            stream_id, 1
        )  # sample every data for each slam camera

    for stream_id in imu_stream_ids:
        options.activate_stream(stream_id)  # activate imus
        options.set_subsample_rate(stream_id, 10)  # sample every 10th data for each imu

    for stream_id in eyegaze_stream_ids:
        options.activate_stream(stream_id)  # activate eyegaze
        options.set_subsample_rate(stream_id, 1)  # sample every 10th data for each imu

    for stream_id in pose_stream_ids:
        options.activate_stream(stream_id)  # activate vio and hand pose
        options.set_subsample_rate(stream_id, 1)  # sample every 10th data for each imu

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


def print_ppg_config(ppg_config):
    # Print PPG Configuration details
    print("PPG Configuration:")
    print(f"Stream ID: {ppg_config.stream_id}")
    print(f"Sensor Model: {ppg_config.sensor_model}")
    print(f"Device ID: {ppg_config.device_id}")
    print(f"Nominal Rate (Hz): {ppg_config.nominal_rate_hz}")
    print(f"Description: {ppg_config.description}")


def print_ppg_data(ppg_data):
    # Print PPG Data details
    print("PPG Data:")
    print(f"Capture Timestamp (ns): {ppg_data.capture_timestamp_ns}")
    print(f"Value: {ppg_data.value}")
    print(f"LED Current (mA): {ppg_data.led_current_ma}")
    print(f"Integration Time (us): {ppg_data.integration_time_us}")


def print_als_config(als_config):
    # Print ALS Configuration details
    print("ALS Configuration:")
    print(f"--- Stream ID: {als_config.stream_id}")
    print(f"--- Device ID: {als_config.device_id}")
    print(f"--- Nominal Rate (Hz): {als_config.nominal_rate_hz}")
    print(f"--- Sensor Model: {als_config.sensor_model}")


def print_als_data(als_data):
    # Print ALS Data details
    print("ALS Data:")
    print(f"--- Capture Timestamp (ns): {als_data.capture_timestamp_ns}")
    print(f"--- Red Channel (normalized) intensity: {als_data.red_channel_normalized}")
    print(
        f"--- Green Channel (normalized) intensity: {als_data.green_channel_normalized}"
    )
    print(
        f"--- Blue Channel (normalized) intensity: {als_data.blue_channel_normalized}"
    )
    print(f"--- UV Channel (normalized): {als_data.uv_channel_normalized}")
    print(f"--- IR Channel (normalized): {als_data.ir_channel_normalized}")
    print(f"--- Clear Channel (normalized): {als_data.clear_channel_normalized}")
    print(f"--- UV Flux (W/m²): {als_data.uv_flux_watt_per_square_meter}")
    print(f"--- IR Flux (W/m²): {als_data.ir_flux_watt_per_square_meter}")
    print(f"--- Clear Flux (W/m²): {als_data.clear_flux_watt_per_square_meter}")
    print(f"--- Red Gain: {als_data.gain_red}")
    print(f"--- Green Gain: {als_data.gain_green}")
    print(f"--- Blue Gain: {als_data.gain_blue}")
    print(f"--- UV Gain: {als_data.gain_uv}")
    print(f"--- IR Gain: {als_data.gain_ir}")
    print(f"--- Clear Gain: {als_data.gain_clear}")
    print(f"--- Exposure Time (us): {als_data.exposure_time_us}")
    print(f"--- CCT: {als_data.cct}")
    print(f"--- Lux: {als_data.lux}")


def print_temperature_config(temperature_config):
    # Print Temperature Configuration details
    print("Temperature Configuration:")
    print(f"--- Stream ID: {temperature_config.stream_id}")
    print(f"--- Sensor Model: {temperature_config.sensor_model}")
    print(f"--- Device ID: {temperature_config.device_id}")
    print(f"--- Nominal Rate (Hz): {temperature_config.nominal_rate_hz}")


def print_temperature_data(temperature_data):
    # Print Temperature Data details
    print("Temperature Data:")
    print(f"--- Capture Timestamp (ns): {temperature_data.capture_timestamp_ns}")
    print(f"--- Temperature (°C): {temperature_data.temperature_celsius}")
    print(f"--- Sensor Name: {temperature_data.sensor_name}")


def print_sample_random_access_image_data(provider, camera_name):
    sensor_stream_id = provider.get_stream_id_from_label(camera_name)

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
        pixel_format = image_data[0].get_pixel_format()
        print(
            f"query time {time} and get image time {image_data[1].arrival_timestamp_ns} within range {start_time} {end_time} \n"
            f"image pixel format is {pixel_format}"
        )


def print_sample_random_access_ppg_data(provider):
    ppg_stream_id = provider.get_stream_id_from_label("ppg")

    # get sensor data configuration
    ppg_config = provider.get_ppg_configuration(ppg_stream_id)
    print_ppg_config(ppg_config)

    # get all image data by index
    num_data = provider.get_num_data(ppg_stream_id)
    for index in range(0, num_data):
        ppg_data = provider.get_ppg_data_by_index(ppg_stream_id, index)
        print(
            f"Getting ppg data from random accessor with timestamp {ppg_data.capture_timestamp_ns}"
        )

    # random access data from a set of timestamps
    time_domain = TimeDomain.DEVICE_TIME  # query data based on device time
    option = (
        TimeQueryOptions.CLOSEST
    )  # get data whose time [in TimeDomain] is Closest to query time
    start_time = provider.get_first_time_ns(ppg_stream_id, time_domain)
    end_time = provider.get_last_time_ns(ppg_stream_id, time_domain)
    for time in range(start_time, end_time, int(1e8)):
        ppg_data = provider.get_ppg_data_by_time_ns(
            ppg_stream_id, time, time_domain, option
        )
        print_ppg_data(ppg_data)


def print_sample_random_access_als_data(provider):
    als_stream_id = provider.get_stream_id_from_label("als")

    # get sensor data configuration
    als_config = provider.get_als_configuration(als_stream_id)
    print_als_config(als_config)

    # get all ALS data by index
    num_data = provider.get_num_data(als_stream_id)
    for index in range(0, min(4, num_data - 1)):
        als_data = provider.get_als_data_by_index(als_stream_id, index)
        print(
            f"Getting ALS data from random accessor with timestamp {als_data.capture_timestamp_ns}"
        )
        print_als_data(als_data)


def print_sample_random_access_temperature_data(provider):
    temperature_stream_id = provider.get_stream_id_from_label("temperature")

    # get sensor data configuration
    temperature_config = provider.get_temperature_configuration(temperature_stream_id)
    print_temperature_config(temperature_config)

    # get all Temperature data by index
    num_data = provider.get_num_data(temperature_stream_id)
    for index in range(0, min(4, num_data - 1)):
        temperature_data = provider.get_temperature_data_by_index(
            temperature_stream_id, index
        )
        print(
            f"Getting Temperature data from random accessor with timestamp {temperature_data.capture_timestamp_ns}"
        )
        print_temperature_data(temperature_data)


def print_sample_vio_data(provider):
    sensor_stream_id = provider.get_stream_id_from_label("vio")

    # get sensor data configuration
    # TODO: enable this once we have vio config supported
    # config = provider.get_vio_configuration(sensor_stream_id)
    # print_image_config(config)

    # get vio data by index
    num_data = provider.get_num_data(sensor_stream_id)
    for index in range(0, num_data, 50):
        vioData = provider.get_vio_data_by_index(sensor_stream_id, index)
        print(
            f"Getting vio data from random accessor with timestamp {vioData.capture_timestamp_ns}, frame id is {vioData.frame_id}\n"
            f"linear velocity is {vioData.linear_velocity_in_odometry}, \n"
            f"Device position is {vioData.transform_odometry_bodyimu.translation()}, T_bodyImu_device is {vioData.transform_bodyimu_device.to_matrix()}"
        )

    # random access data from a set of timestamps
    time_domain = TimeDomain.DEVICE_TIME  # query data based on device time
    option = (
        TimeQueryOptions.CLOSEST
    )  # get data whose time [in TimeDomain] is Closest to query time
    start_time = provider.get_first_time_ns(sensor_stream_id, time_domain)
    end_time = provider.get_last_time_ns(sensor_stream_id, time_domain)
    for time in range(start_time, end_time, int(1e8)):
        vioData = provider.get_vio_data_by_time_ns(
            sensor_stream_id, time, time_domain, option
        )
        print(
            f"Getting vio data from random accessor with timestamp {vioData.capture_timestamp_ns}, frame id is {vioData.frame_id}\n"
            f"linear velocity is {vioData.linear_velocity_in_odometry}, \n"
            f"Device position is {vioData.transform_odometry_bodyimu.translation()}, T_bodyImu_device is {vioData.transform_bodyimu_device.to_matrix()}"
        )


def print_mp_configuration_data(provider):
    # get mp configuration data
    device_version = provider.get_device_calibration().get_device_version()
    if device_version != calibration.DeviceVersion.Gen2:
        print(
            "MP configuration is only supported for Gen2 VRS, current version:",
            calibration.get_name(device_version),
        )
        return
    eye_gaze_configuration = provider.get_eye_gaze_configuration(
        provider.get_stream_id_from_label("eyegaze")
    )
    print(
        f"eyeGazeConfiguration stream_id: {eye_gaze_configuration.stream_id}, "
        f"nominal_rate_hz: {eye_gaze_configuration.nominal_rate_hz}, "
        f"user_calibrated: {eye_gaze_configuration.user_calibrated}, "
        f"user_calibration_error: {eye_gaze_configuration.user_calibration_error}"
    )
    hand_pose_configuration = provider.get_hand_pose_configuration(
        provider.get_stream_id_from_label("handtracking")
    )
    print(
        f"handPoseConfiguration stream_id: {hand_pose_configuration.stream_id}, "
        f"nominal_rate_hz: {hand_pose_configuration.nominal_rate_hz}, "
        f"is_wrist_palm_only: {hand_pose_configuration.is_wrist_palm_only}, "
        f"user_profile: {hand_pose_configuration.user_profile}"
    )
    vio_configuration = provider.get_vio_configuration(
        provider.get_stream_id_from_label("vio")
    )
    print(
        f"vioConfiguration stream_id: {vio_configuration.stream_id}, "
        f"nominal_rate_hz: {vio_configuration.nominal_rate_hz}, "
        f"message_version: {vio_configuration.message_version}"
    )
    vioHighFreqConfiguration = provider.get_vio_high_freq_configuration(
        provider.get_stream_id_from_label("vio_high_frequency")
    )
    print(
        f"vioHighFreqConfiguration stream_id: {vioHighFreqConfiguration.stream_id}, "
        f"nominal_rate_hz: {vioHighFreqConfiguration.nominal_rate_hz}, "
        f"message_version: {vioHighFreqConfiguration.message_version}"
    )


if __name__ == "__main__":
    args = parse_args()

    print("")
    print("------------------------------------------------")
    print(f"Creating data provider from {args.vrs_path}")
    provider = data_provider.create_vrs_data_provider(args.vrs_path)
    assert provider is not None
    print("")

    # Obtain an example camera label
    device_version = provider.get_device_calibration().get_device_version()
    print(f"device version is {calibration.get_name(device_version)}")

    # random access data from a specific sensor
    if device_version == calibration.DeviceVersion.Gen1:
        camera_name = "camera-slam-right"
    elif device_version == calibration.DeviceVersion.Gen2:
        camera_name = "slam-front-left"
    else:
        raise Exception(
            f"Unsupported device version: {calibration.get_name(device_version)}"
        )

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
    print("Example: random access image sensor data")
    print_sample_random_access_image_data(provider, camera_name)
    print("------------------------------------------------")

    print("------------------------------------------------")
    print("Example: distort image data")
    print_distort_image(provider, camera_name)
    print("------------------------------------------------")

    print("------------------------------------------------")
    print("Example: get color correction image data")
    print_color_corrected_image(provider)
    print("------------------------------------------------")

    if device_version == calibration.DeviceVersion.Gen2:
        # Gen2-specific data type checks
        print("------------------------------------------------")
        print("Example: random access vio data")
        print_sample_vio_data(provider)
        print("------------------------------------------------")

        print("------------------------------------------------")
        print("Example: random access ppg data")
        print_sample_random_access_ppg_data(provider)
        print("------------------------------------------------")

        print("------------------------------------------------")
        print("Example: random access als data")
        print_sample_random_access_als_data(provider)
        print("------------------------------------------------")

        print("------------------------------------------------")
        print("Example: random access temperature data")
        print_sample_random_access_temperature_data(provider)
        print("------------------------------------------------")

        print("------------------------------------------------")
        print("Example: mp configuration data")
        print_mp_configuration_data(provider)
        print("------------------------------------------------")
