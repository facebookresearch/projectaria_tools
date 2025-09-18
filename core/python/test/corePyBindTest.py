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
import pickle
import unittest

import numpy as np

from projectaria_tools.core import calibration, data_provider
from projectaria_tools.core.sensor_data import (
    SensorDataType,
    TimeDomain,
    TimeQueryOptions,
    TimeSyncMode,
)
from projectaria_tools.core.stream_id import StreamId

vrs_filepath_list = [
    os.path.join(os.getenv("TEST_FOLDER"), "aria_unit_test_sequence_calib.vrs"),
    os.path.join(os.getenv("TEST_FOLDER_GEN2"), "aria_gen2_unit_test_sequence.vrs"),
]

timecode_vrs_filepath = os.path.join(
    os.getenv("TEST_FOLDER"), "aria_unit_test_timecode_sequence_calib.vrs"
)


def compare_camera_calib(calib1, calib2):
    assert calib1.get_label() == calib2.get_label()
    assert calib1.get_serial_number() == calib2.get_serial_number()
    assert np.allclose(
        calib1.get_transform_device_camera().to_matrix(),
        calib2.get_transform_device_camera().to_matrix(),
    )
    assert np.array_equal(calib1.get_image_size(), calib2.get_image_size())
    assert np.isclose(calib1.get_max_solid_angle(), calib2.get_max_solid_angle())

    if (calib1.get_valid_radius() is None) != (calib2.get_valid_radius() is None):
        raise AssertionError(
            "calib1.get_valid_radius() is {} != calib2.get_valid_radius() is {}",
            calib1.get_valid_radius(),
            calib2.get_valid_radius(),
        )
    if calib1.get_valid_radius() is not None:
        assert np.isclose(calib1.get_valid_radius(), calib2.get_valid_radius())

    assert np.isclose(
        calib1.get_time_offset_sec_device_camera(),
        calib2.get_time_offset_sec_device_camera(),
    )

    if (calib1.get_readout_time_sec() is None) != (
        calib2.get_readout_time_sec() is None
    ):
        raise AssertionError(
            "calib1.get_readout_time_sec() is {} != calib2.get_readout_time_sec() is {}",
            calib1.get_readout_time_sec(),
            calib2.get_readout_time_sec(),
        )
    if calib1.get_readout_time_sec() is not None:
        assert np.isclose(calib1.get_readout_time_sec(), calib2.get_readout_time_sec())

    assert calib1.get_model_name() == calib2.get_model_name()
    assert np.allclose(calib1.get_projection_params(), calib2.get_projection_params())


def compare_linear_rectification_model(model1, model2):
    assert np.allclose(model1.get_rectification(), model2.get_rectification())
    assert np.allclose(model1.get_bias(), model2.get_bias())


def compare_imu_calib(calib1, caib2):
    assert calib1.get_label() == caib2.get_label()
    assert np.allclose(
        calib1.get_transform_device_imu().to_matrix(),
        caib2.get_transform_device_imu().to_matrix(),
    )
    compare_linear_rectification_model(
        calib1.get_accel_model(), caib2.get_accel_model()
    )
    compare_linear_rectification_model(calib1.get_gyro_model(), caib2.get_gyro_model())


def compare_mag_calib(calib1, calib2):
    assert calib1.get_label() == calib2.get_label()
    compare_linear_rectification_model(calib1.get_model(), calib2.get_model())


def compare_baro_calib(calib1, calib2):
    assert calib1.get_label() == calib2.get_label()
    assert np.isclose(calib1.get_slope(), calib2.get_slope())
    assert np.isclose(calib1.get_offset_pa(), calib2.get_offset_pa())


def compare_microphone_calib(calib1, calib2):
    assert calib1.get_label() == calib2.get_label()
    assert np.isclose(
        calib1.get_d_sensitivity_1k_dbv(), calib2.get_d_sensitivity_1k_dbv()
    )


def compare_sensor_calib(calib1, calib2):
    assert calib1.sensor_calibration_type() == calib2.sensor_calibration_type()
    if (
        calib1.sensor_calibration_type()
        == calibration.SensorCalibrationType.CAMERA_CALIBRATION
    ):
        compare_camera_calib(calib1.camera_calibration(), calib2.camera_calibration())
    elif (
        calib1.sensor_calibration_type()
        == calibration.SensorCalibrationType.IMU_CALIBRATION
    ):
        compare_imu_calib(calib1.imu_calibration(), calib2.imu_calibration())
    elif (
        calib1.sensor_calibration_type()
        == calibration.SensorCalibrationType.MAGNETOMETER_CALIBRATION
    ):
        compare_mag_calib(
            calib1.magnetometer_calibration(), calib2.magnetometer_calibration()
        )
    elif (
        calib1.sensor_calibration_type()
        == calibration.SensorCalibrationType.BAROMETER_CALIBRATION
    ):
        compare_baro_calib(
            calib1.barometer_calibration(), calib2.barometer_calibration()
        )
    elif (
        calib1.sensor_calibration_type()
        == calibration.SensorCalibrationType.MICROPHONE_CALIBRATION
    ):
        compare_microphone_calib(
            calib1.microphone_calibration(), calib2.microphone_calibration()
        )
    else:
        raise ValueError(
            f"Unsupported sensor calibration type: {calib1.sensor_calibration_type()}"
        )


class PickleTests(unittest.TestCase):
    def test_device_version(self) -> None:
        for device_version in [
            calibration.DeviceVersion.Gen1,
            calibration.DeviceVersion.Gen2,
            calibration.DeviceVersion.NotValid,
        ]:
            pickled = pickle.dumps(device_version)
            unpickled = pickle.loads(pickled)
            assert unpickled == device_version

    def test_camera_calibration(self) -> None:
        for vrs_filepath in vrs_filepath_list:
            provider = data_provider.create_vrs_data_provider(vrs_filepath)
            calib = provider.get_device_calibration()
            for cam_label in calib.get_camera_labels():
                cam_calib = calib.get_camera_calib(cam_label)
                pickled = pickle.dumps(cam_calib)
                unpickled = pickle.loads(pickled)
                compare_camera_calib(cam_calib, unpickled)

    def test_imu_calibration(self) -> None:
        for vrs_filepath in vrs_filepath_list:
            provider = data_provider.create_vrs_data_provider(vrs_filepath)
            calib = provider.get_device_calibration()
            for imu_label in calib.get_imu_labels():
                imu_calib = calib.get_imu_calib(imu_label)
                pickled = pickle.dumps(imu_calib)
                unpickled = pickle.loads(pickled)
                compare_imu_calib(imu_calib, unpickled)

    def test_mag_calibration(self) -> None:
        for vrs_filepath in vrs_filepath_list:
            provider = data_provider.create_vrs_data_provider(vrs_filepath)
            calib = provider.get_device_calibration()
            for mag_label in calib.get_magnetometer_labels():
                mag_calib = calib.get_magnetometer_calib(mag_label)
                pickled = pickle.dumps(mag_calib)
                unpickled = pickle.loads(pickled)
                compare_mag_calib(mag_calib, unpickled)

    def test_baro_calibration(self) -> None:
        for vrs_filepath in vrs_filepath_list:
            provider = data_provider.create_vrs_data_provider(vrs_filepath)
            calib = provider.get_device_calibration()
            for baro_label in calib.get_barometer_labels():
                baro_calib = calib.get_barometer_calib(baro_label)
                pickled = pickle.dumps(baro_calib)
                unpickled = pickle.loads(pickled)
                compare_baro_calib(baro_calib, unpickled)

    def test_microphone_calibration(self) -> None:
        for vrs_filepath in vrs_filepath_list:
            provider = data_provider.create_vrs_data_provider(vrs_filepath)
            calib = provider.get_device_calibration()
            for mic_label in calib.get_microphone_labels():
                mic_calib = calib.get_microphone_calib(mic_label)
                pickled = pickle.dumps(mic_calib)
                unpickled = pickle.loads(pickled)
                compare_microphone_calib(mic_calib, unpickled)

    def test_sensor_calibration(self) -> None:
        for vrs_filepath in vrs_filepath_list:
            provider = data_provider.create_vrs_data_provider(vrs_filepath)
            calib = provider.get_device_calibration()
            for label in calib.get_all_labels():
                sensor_calib = calib.get_sensor_calib(label)
                if sensor_calib is None:
                    continue
                pickled = pickle.dumps(sensor_calib)
                unpickled = pickle.loads(pickled)
                compare_sensor_calib(sensor_calib, unpickled)

    def test_device_calibration(self) -> None:
        for vrs_filepath in vrs_filepath_list:
            provider = data_provider.create_vrs_data_provider(vrs_filepath)
            calib = provider.get_device_calibration()
            pickled = pickle.dumps(calib)
            unpicked = pickle.loads(pickled)
            # compare device info
            assert calib.get_device_version() == unpicked.get_device_version()
            assert calib.get_device_subtype() == unpicked.get_device_subtype()
            assert calib.get_origin_label() == unpicked.get_origin_label()
            for label in calib.get_all_labels():
                # compare CAD extrinsics
                cad_extrinsics = calib.get_transform_device_sensor(label, True)
                if cad_extrinsics is None:
                    continue
                cad_extrinsics_unpicked = unpicked.get_transform_device_sensor(
                    label, True
                )
                assert np.allclose(
                    cad_extrinsics.to_matrix(), cad_extrinsics_unpicked.to_matrix()
                )

                # compare factory calbration
                sensor_calib = calib.get_sensor_calib(label)
                sensor_calib_unpickled = calib.get_sensor_calib(label)
                if sensor_calib is None:
                    continue
                compare_sensor_calib(sensor_calib, sensor_calib_unpickled)


class CalibrationTests(unittest.TestCase):
    def _check_cam_rotation_by_pixel_unprojection(self, test_pixel, cam_calib) -> bool:
        """
        A helper function to check the rotation of camera calibration unprojects same pixels to same rays
        """
        # rotate camera
        cam_calib_cw90 = calibration.rotate_camera_calib_cw90deg(cam_calib)
        original_image_size = cam_calib.get_image_size()

        # Ray in device frame
        original_ray = (
            cam_calib.get_transform_device_camera()
            @ cam_calib.unproject_no_checks(test_pixel)
        )
        # Pixels are effectively rotated too!
        cw_90_ray = (
            cam_calib_cw90.get_transform_device_camera()
            @ cam_calib_cw90.unproject_no_checks(
                [original_image_size[1] - test_pixel[1] - 1, test_pixel[0]]
            )
        )

        # ray needs to be normalized for comparison
        original_ray /= original_ray[2]
        cw_90_ray /= cw_90_ray[2]

        return np.allclose(original_ray, cw_90_ray, rtol=1e-5)

    def test_imu_calibration_getter(self) -> None:
        for vrs_filepath in vrs_filepath_list:
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
                np.testing.assert_array_almost_equal(
                    rectified_gyro, rectified_gyro_compare
                )

    def test_calibration_label(self) -> None:
        for vrs_filepath in vrs_filepath_list:
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
                    calib_type
                    == calibration.SensorCalibrationType.MAGNETOMETER_CALIBRATION
                ):
                    assert calib.magnetometer_calibration().get_label() == label
                elif (
                    calib_type
                    == calibration.SensorCalibrationType.BAROMETER_CALIBRATION
                ):
                    assert calib.barometer_calibration().get_label() == label
                elif (
                    calib_type
                    == calibration.SensorCalibrationType.MICROPHONE_CALIBRATION
                ):
                    assert calib.microphone_calibration().get_label() == label
                elif (
                    calib_type == calibration.SensorCalibrationType.ARIA_ET_CALIBRATION
                ):
                    assert calib.aria_et_calibration().get_label() == label
                elif (
                    calib_type == calibration.SensorCalibrationType.ARIA_MIC_CALIBRATION
                ):
                    assert calib.aria_mic_calibration().get_label() == label

    def test_random_accessor_index(self) -> None:
        for vrs_filepath in vrs_filepath_list:
            provider = data_provider.create_vrs_data_provider(vrs_filepath)

            streams = provider.get_all_streams()
            # Skip a few streams for Gen2, because the unit test sequence is known not to contain any data for certain streams
            streams_to_skip = [
                StreamId("281-1"),  # GPS
                StreamId("282-1"),  # Wifi
                StreamId("283-1"),  # Bluetooth
                StreamId("246-1"),  # Temperature
            ]
            for stream_id in streams:
                if (
                    vrs_filepath == vrs_filepath_list[1]
                    and stream_id in streams_to_skip
                ):
                    continue

                num_data = provider.get_num_data(stream_id)
                assert num_data > 0
                for i in range(0, num_data):
                    data = provider.get_sensor_data_by_index(stream_id, i)
                    assert data.sensor_data_type() != SensorDataType.NOT_VALID
                    assert data.stream_id() == stream_id

    def test_random_accessor_timestamp(self) -> None:
        for vrs_filepath in vrs_filepath_list:
            provider = data_provider.create_vrs_data_provider(vrs_filepath)

            streams = provider.get_all_streams()
            for stream_id in streams:
                for time_domain in (
                    TimeDomain.RECORD_TIME,
                    TimeDomain.DEVICE_TIME,
                    TimeDomain.HOST_TIME,
                ):
                    if not provider.supports_time_domain(stream_id, time_domain):
                        continue
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
            first_device_time_compare = (
                provider.convert_from_synctime_to_device_time_ns(
                    first_time, TimeSyncMode.TIME_CODE
                )
            )
            last_device_time = provider.convert_from_timecode_to_device_time_ns(
                last_time
            )
            last_device_time_compare = provider.convert_from_synctime_to_device_time_ns(
                last_time, TimeSyncMode.TIME_CODE
            )
            assert first_device_time == first_device_time_compare
            assert last_device_time == last_device_time_compare

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

        # 1. test for linear camera
        image_size = [300, 400]
        test_pixel = [10.4, 23.1]
        linear_calib = calibration.get_linear_camera_calibration(
            image_size[0],
            image_size[1],
            150,
            sensor_name,
            src_calib.get_transform_device_camera(),
        )
        self.assertTrue(
            self._check_cam_rotation_by_pixel_unprojection(
                test_pixel=test_pixel, cam_calib=linear_calib
            )
        )

        # 2. Test for Fisheye624 camera
        self.assertTrue(
            self._check_cam_rotation_by_pixel_unprojection(
                test_pixel=test_pixel, cam_calib=src_calib
            )
        )

    def test_vrs_file_tags(self) -> None:
        provider = data_provider.create_vrs_data_provider(vrs_filepath_list[0])
        file_tags = provider.get_file_tags()
        assert len(file_tags) == 25


class DataProviderTests(unittest.TestCase):
    def test_vrs_file_metadata_gen1(self) -> None:
        provider = data_provider.create_vrs_data_provider(vrs_filepath_list[0])
        file_metadata = provider.get_metadata()
        assert file_metadata.device_serial == "1WM093701M1276"
        assert file_metadata.recording_profile == "profile9"
        assert file_metadata.shared_session_id == ""
        assert file_metadata.filename == "d3c61c3a-18ec-460e-a35c-cec9579494ca.vrs"
        assert (
            file_metadata.time_sync_mode
            == data_provider.MetadataTimeSyncMode.NotEnabled
        )
        assert (
            provider.get_time_sync_mode()
            == data_provider.MetadataTimeSyncMode.NotEnabled
        )
        assert file_metadata.device_id == "35ec1d5b-689d-4531-a0c9-1c8ff42d2e89"
        assert file_metadata.start_time_epoch_sec == 1649265055

    def test_vrs_file_metadata_gen2(self) -> None:
        provider = data_provider.create_vrs_data_provider(vrs_filepath_list[1])
        file_metadata = provider.get_metadata()
        assert file_metadata.device_serial == "1M0YCB3G7Y0055"
        assert file_metadata.recording_profile == "profile4"
        assert file_metadata.shared_session_id == ""
        assert file_metadata.filename == ""
        assert (
            file_metadata.time_sync_mode
            == data_provider.MetadataTimeSyncMode.NotEnabled
        )
        assert (
            provider.get_time_sync_mode()
            == data_provider.MetadataTimeSyncMode.NotEnabled
        )
        assert file_metadata.device_id == "f6e94724-e773-437e-a2fb-eb3e108bc54d"
        assert file_metadata.start_time_epoch_sec == 1749702390

    def test_device_version_gen1(self) -> None:
        provider = data_provider.create_vrs_data_provider(vrs_filepath_list[0])
        device_version = provider.get_device_version()
        assert device_version == calibration.DeviceVersion.Gen1

    def test_device_version_gen2(self) -> None:
        provider = data_provider.create_vrs_data_provider(vrs_filepath_list[1])
        device_version = provider.get_device_version()
        assert device_version == calibration.DeviceVersion.Gen2
