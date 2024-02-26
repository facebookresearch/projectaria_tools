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

import os
import unittest

import numpy as np

from projectaria_tools.core import calibration, data_provider
from projectaria_tools.core.sensor_data import (
    SensorDataType,
    TimeDomain,
    TimeQueryOptions,
)

vrs_filepath = os.path.join(
    os.getenv("TEST_FOLDER"), "aria_unit_test_sequence_calib.vrs"
)

timecode_vrs_filepath = os.path.join(
    os.getenv("TEST_FOLDER"), "aria_unit_test_timecode_sequence_calib.vrs"
)


class CalibrationTests(unittest.TestCase):
    def test_imu_calibration_getter(self) -> None:
        provider = data_provider.create_vrs_data_provider(vrs_filepath)

        device_calib = provider.get_device_calibration()
        assert device_calib is not None

        imu_labels = ["imu-left", "imu-right"]

        raw_accel_data = np.array([1.0, 0.2, 3.1])
        raw_gyro_data = np.array([11.7, 1.2, 10.1])

        for label in imu_labels:
            imu_calib = device_calib.get_imu_calib(label)
            assert imu_calib is not None

            accel_model = imu_calib.get_accel_model()
            gyro_model = imu_calib.get_gyro_model()

            accel_rect = accel_model.get_rectification()
            accel_bias = accel_model.get_bias()

            gyro_rect = gyro_model.get_rectification()
            gyro_bias = gyro_model.get_bias()

            rectified_accel = imu_calib.raw_to_rectified_accel(raw_accel_data)
            rectified_accel_compare = np.linalg.inv(accel_rect) @ (
                raw_accel_data - accel_bias
            )
            np.testing.assert_array_almost_equal(
                rectified_accel, rectified_accel_compare
            )

            rectified_gyro = imu_calib.raw_to_rectified_gyro(raw_gyro_data)
            rectified_gyro_compare = np.linalg.inv(gyro_rect) @ (
                raw_gyro_data - gyro_bias
            )
            np.testing.assert_array_almost_equal(rectified_gyro, rectified_gyro_compare)

    def test_calibration_label(self) -> None:
        provider = data_provider.create_vrs_data_provider(vrs_filepath)

        device_calib = provider.get_device_calibration()
        assert device_calib is not None

        sensor_labels = device_calib.get_all_labels()
        assert sensor_labels is not None
        origin_label = device_calib.get_origin_label()
        assert any(label == origin_label for label in sensor_labels)
        for label in sensor_labels:
            calib = device_calib.get_sensor_calib(label)
            assert calib is not None

            calib_type = calib.sensor_calibration_type()
            if calib_type == calibration.SensorCalibrationType.CAMERA_CALIBRATION:
                assert calib.camera_calibration().get_label() == label
            elif calib_type == calibration.SensorCalibrationType.IMU_CALIBRATION:
                assert calib.imu_calibration().get_label() == label
            elif (
                calib_type == calibration.SensorCalibrationType.MAGNETOMETER_CALIBRATION
            ):
                assert calib.magnetometer_calibration().get_label() == label
            elif calib_type == calibration.SensorCalibrationType.BAROMETER_CALIBRATION:
                assert calib.barometer_calibration().get_label() == label
            elif calib_type == calibration.SensorCalibrationType.MICROPHONE_CALIBRATION:
                assert calib.microphone_calibration().get_label() == label
            elif calib_type == calibration.SensorCalibrationType.ARIA_ET_CALIBRATION:
                assert calib.aria_et_calibration().get_label() == label
            elif calib_type == calibration.SensorCalibrationType.ARIA_MIC_CALIBRATION:
                assert calib.aria_mic_calibration().get_label() == label

    def test_random_accessor_index(self) -> None:
        provider = data_provider.create_vrs_data_provider(vrs_filepath)

        streams = provider.get_all_streams()
        for stream_id in streams:
            num_data = provider.get_num_data(stream_id)
            assert num_data > 0
            for i in range(0, num_data):
                data = provider.get_sensor_data_by_index(stream_id, i)
                assert data.sensor_data_type() != SensorDataType.NOT_VALID
                assert data.stream_id() == stream_id

    def test_random_accessor_timestamp(self) -> None:
        provider = data_provider.create_vrs_data_provider(vrs_filepath)

        streams = provider.get_all_streams()
        for stream_id in streams:
            for time_domain in (
                TimeDomain.RECORD_TIME,
                TimeDomain.DEVICE_TIME,
                TimeDomain.HOST_TIME,
            ):
                first_time = provider.get_first_time_ns(stream_id, time_domain)
                last_time = provider.get_last_time_ns(stream_id, time_domain)
                assert first_time <= last_time
                for time in range(first_time, last_time, int(1e8)):
                    time_before = provider.get_sensor_data_by_time_ns(
                        stream_id, time, time_domain, TimeQueryOptions.BEFORE
                    ).get_time_ns(time_domain)
                    time_closest = provider.get_sensor_data_by_time_ns(
                        stream_id, time, time_domain, TimeQueryOptions.CLOSEST
                    ).get_time_ns(time_domain)
                    time_after = provider.get_sensor_data_by_time_ns(
                        stream_id, time, time_domain, TimeQueryOptions.AFTER
                    ).get_time_ns(time_domain)
                    assert time_before <= time
                    assert time_after >= time
                    delta = abs(time_closest - time)
                    assert delta <= (time_after - time) and delta <= (
                        time - time_before
                    )

    def test_random_accessor_timecode(self) -> None:
        provider = data_provider.create_vrs_data_provider(timecode_vrs_filepath)

        streams = provider.get_all_streams()
        for stream_id in streams:
            assert provider.supports_time_domain(stream_id, TimeDomain.TIME_CODE)

            first_time = provider.get_first_time_ns(stream_id, TimeDomain.TIME_CODE)
            last_time = provider.get_last_time_ns(stream_id, TimeDomain.TIME_CODE)
            first_device_time = provider.convert_from_timecode_to_device_time_ns(
                first_time
            )
            last_device_time = provider.convert_from_timecode_to_device_time_ns(
                last_time
            )

            assert first_time <= last_time
            assert first_device_time <= last_device_time
            for time in range(first_time, last_time, int(1e7)):
                time_before = provider.get_sensor_data_by_time_ns(
                    stream_id, time, TimeDomain.TIME_CODE, TimeQueryOptions.BEFORE
                ).get_time_ns(TimeDomain.TIME_CODE)
                time_closest = provider.get_sensor_data_by_time_ns(
                    stream_id, time, TimeDomain.TIME_CODE, TimeQueryOptions.CLOSEST
                ).get_time_ns(TimeDomain.TIME_CODE)
                time_after = provider.get_sensor_data_by_time_ns(
                    stream_id, time, TimeDomain.TIME_CODE, TimeQueryOptions.AFTER
                ).get_time_ns(TimeDomain.TIME_CODE)
                assert time_before <= time
                assert time_after >= time
                delta = abs(time_closest - time)
                assert delta <= (time_after - time) and delta <= (time - time_before)

    def test_camera_calibration_rotation(self) -> None:
        provider = data_provider.create_vrs_data_provider(timecode_vrs_filepath)
        sensor_name = "camera-rgb"
        # input: retrieve image calibration
        src_calib = provider.get_device_calibration().get_camera_calib(sensor_name)

        image_size = [300, 400]
        dst_calib = calibration.get_linear_camera_calibration(
            image_size[0],
            image_size[1],
            150,
            sensor_name,
            src_calib.get_transform_device_camera(),
        )
        # Get rotated image calibration
        dst_calib_cw90 = calibration.rotate_camera_calib_cw90deg(dst_calib)

        test_pixel = [10.4, 23.1]
        ray_dst = (
            dst_calib.get_transform_device_camera()
            @ dst_calib.unproject_no_checks(test_pixel)
        )
        ray_dst_cw90 = (
            dst_calib_cw90.get_transform_device_camera()
            @ dst_calib_cw90.unproject_no_checks(
                [image_size[1] - test_pixel[1] - 1, test_pixel[0]]
            )
        )
        np.testing.assert_array_almost_equal(ray_dst, ray_dst_cw90)
