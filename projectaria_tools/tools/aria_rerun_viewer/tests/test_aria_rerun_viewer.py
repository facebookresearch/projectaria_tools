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

# pyre-strict

"""Unit tests for aria_rerun_viewer CLI/driver module."""

import argparse
import sys
import unittest
from threading import Event
from unittest.mock import MagicMock, patch

from projectaria_tools.core.calibration import DeviceVersion
from projectaria_tools.core.sensor_data import SensorDataType, TimeDomain, TimeSyncMode
from projectaria_tools.tools.aria_rerun_viewer.aria_rerun_viewer import (
    ALL_STREAM_LABELS_GEN1,
    ALL_STREAM_LABELS_GEN2,
    get_deliver_option,
    log_vrs_to_rerun,
    main,
    MAX_IMU_BATCH_SIZE,
    parse_args,
    parse_subsample_rates,
    plot_interpolated_hand_pose_in_image,
    plot_queued_sensor_data,
)

_MODULE = "projectaria_tools.tools.aria_rerun_viewer.aria_rerun_viewer"


class ParseSubsampleRatesTest(unittest.TestCase):
    """Tests for `parse_subsample_rates()`."""

    def test_parses_single_valid_pair(self) -> None:
        # Arrange / Act
        result = parse_subsample_rates(["camera-rgb=2"], ["camera-rgb"])

        # Assert
        self.assertEqual(result, {"camera-rgb": 2})

    def test_parses_multiple_valid_pairs(self) -> None:
        # Arrange / Act
        result = parse_subsample_rates(
            ["camera-rgb=2", "imu-left=5", "mic=10"],
            ["camera-rgb", "imu-left", "mic"],
        )

        # Assert
        self.assertEqual(result, {"camera-rgb": 2, "imu-left": 5, "mic": 10})

    def test_empty_args_returns_empty_dict(self) -> None:
        # Arrange / Act
        result = parse_subsample_rates([], ["camera-rgb"])

        # Assert
        self.assertEqual(result, {})

    def test_missing_equals_raises_argument_type_error(self) -> None:
        # Arrange / Act / Assert
        with self.assertRaises(argparse.ArgumentTypeError) as ctx:
            parse_subsample_rates(["camera-rgb-no-equals"], ["camera-rgb"])
        self.assertIn("Invalid format", str(ctx.exception))
        self.assertIn("camera-rgb-no-equals", str(ctx.exception))

    def test_too_many_equals_raises_argument_type_error(self) -> None:
        # `foo=2=3` cannot be unpacked into (label, rate).
        with self.assertRaises(argparse.ArgumentTypeError):
            parse_subsample_rates(["camera-rgb=2=3"], ["camera-rgb"])

    def test_stream_not_in_enabled_streams_raises_error(self) -> None:
        # The inner ValueError with the label-not-enabled message is caught and
        # re-raised as ArgumentTypeError.
        with self.assertRaises(argparse.ArgumentTypeError):
            parse_subsample_rates(["camera-rgb=2"], ["imu-left"])

    def test_non_integer_rate_raises_error(self) -> None:
        with self.assertRaises(argparse.ArgumentTypeError):
            parse_subsample_rates(["camera-rgb=abc"], ["camera-rgb"])


class ParseArgsTest(unittest.TestCase):
    """Tests for `parse_args()`."""

    def test_requires_vrs_argument(self) -> None:
        with patch.object(sys, "argv", ["aria_rerun_viewer"]):
            with self.assertRaises(SystemExit):
                parse_args()

    def test_minimal_args_populates_defaults(self) -> None:
        # Arrange
        with patch.object(sys, "argv", ["aria_rerun_viewer", "--vrs", "/tmp/foo.vrs"]):
            # Act
            args = parse_args()

        # Assert
        self.assertEqual(args.vrs, "/tmp/foo.vrs")
        self.assertIsNone(args.skip_begin_sec)
        self.assertIsNone(args.skip_end_sec)
        self.assertIsNone(args.enabled_streams)
        self.assertIsNone(args.subsample_rates)
        self.assertEqual(args.rrd_output_path, "")

    def test_full_args_are_parsed(self) -> None:
        # Arrange
        argv = [
            "aria_rerun_viewer",
            "--vrs",
            "/tmp/foo.vrs",
            "--skip-begin-sec",
            "1.5",
            "--skip-end-sec",
            "2.0",
            "--enabled-streams",
            "camera-rgb",
            "imu-left",
            "--subsample-rates",
            "camera-rgb=2",
            "vio_high_frequency=10",
            "--rrd-output-path",
            "/tmp/out.rrd",
        ]
        with patch.object(sys, "argv", argv):
            # Act
            args = parse_args()

        # Assert
        self.assertEqual(args.vrs, "/tmp/foo.vrs")
        self.assertAlmostEqual(args.skip_begin_sec, 1.5)
        self.assertAlmostEqual(args.skip_end_sec, 2.0)
        self.assertEqual(args.enabled_streams, ["camera-rgb", "imu-left"])
        self.assertEqual(
            args.subsample_rates, ["camera-rgb=2", "vio_high_frequency=10"]
        )
        self.assertEqual(args.rrd_output_path, "/tmp/out.rrd")

    def test_rejects_invalid_enabled_stream_choice(self) -> None:
        argv = [
            "aria_rerun_viewer",
            "--vrs",
            "/tmp/foo.vrs",
            "--enabled-streams",
            "not-a-real-stream",
        ]
        with patch.object(sys, "argv", argv):
            with self.assertRaises(SystemExit):
                parse_args()


class GetDeliverOptionTest(unittest.TestCase):
    """Tests for `get_deliver_option()`."""

    def setUp(self) -> None:
        # Stream label -> mock StreamId. Named MagicMocks keep identity stable
        # across lookups so `in` checks against get_stream_ids work.
        self.stream_id_map = {
            "camera-rgb": MagicMock(name="rgb_id"),
            "imu-left": MagicMock(name="imu_left_id"),
            "vio_high_frequency": MagicMock(name="vio_hf_id"),
        }
        self.deliver_options = MagicMock()
        self.deliver_options.get_stream_ids.return_value = list(
            self.stream_id_map.values()
        )
        self.vrs_data_provider = MagicMock()
        self.vrs_data_provider.get_default_deliver_queued_options.return_value = (
            self.deliver_options
        )
        self.vrs_data_provider.get_stream_id_from_label.side_effect = (
            lambda label: self.stream_id_map.get(label)
        )
        self.viewer_config = MagicMock()
        self.viewer_config.vio_high_freq_subsample_rate = 80

    def test_default_vio_hf_subsample_rate_injected(self) -> None:
        # Arrange
        subsample_rates: dict = {}

        # Act
        get_deliver_option(
            vrs_data_provider=self.vrs_data_provider,
            enabled_stream_labels=["vio_high_frequency"],
            subsample_rates=subsample_rates,
            viewer_config=self.viewer_config,
        )

        # Assert: default was injected into caller-supplied dict.
        self.assertEqual(subsample_rates["vio_high_frequency"], 80)
        self.deliver_options.set_subsample_rate.assert_any_call(
            self.stream_id_map["vio_high_frequency"], 80
        )

    def test_user_provided_vio_hf_rate_is_preserved(self) -> None:
        # Arrange
        subsample_rates = {"vio_high_frequency": 5}

        # Act
        get_deliver_option(
            vrs_data_provider=self.vrs_data_provider,
            enabled_stream_labels=["vio_high_frequency"],
            subsample_rates=subsample_rates,
            viewer_config=self.viewer_config,
        )

        # Assert
        self.assertEqual(subsample_rates["vio_high_frequency"], 5)
        self.deliver_options.set_subsample_rate.assert_any_call(
            self.stream_id_map["vio_high_frequency"], 5
        )

    def test_activates_only_enabled_and_available_streams(self) -> None:
        # Enable a mix of available and unavailable labels.
        self.vrs_data_provider.get_stream_id_from_label.side_effect = lambda label: {
            **self.stream_id_map,
            "not-in-vrs": None,
        }.get(label)

        get_deliver_option(
            vrs_data_provider=self.vrs_data_provider,
            enabled_stream_labels=["camera-rgb", "not-in-vrs"],
            subsample_rates={},
            viewer_config=self.viewer_config,
        )

        activated = [
            call.args[0] for call in self.deliver_options.activate_stream.call_args_list
        ]
        self.assertIn(self.stream_id_map["camera-rgb"], activated)
        self.assertNotIn(self.stream_id_map["imu-left"], activated)

    def test_skips_stream_when_id_not_in_available_ids(self) -> None:
        # get_stream_id_from_label returns a mock id, but get_stream_ids
        # does NOT contain it, so activation must be skipped.
        rogue_stream = MagicMock(name="rogue_id")
        self.vrs_data_provider.get_stream_id_from_label.side_effect = (
            lambda label: rogue_stream if label == "camera-rgb" else None
        )
        self.deliver_options.get_stream_ids.return_value = [MagicMock(name="unrelated")]

        get_deliver_option(
            vrs_data_provider=self.vrs_data_provider,
            enabled_stream_labels=["camera-rgb"],
            subsample_rates={},
            viewer_config=self.viewer_config,
        )

        self.deliver_options.activate_stream.assert_not_called()

    def test_sets_subsample_rate_only_for_enabled_stream(self) -> None:
        subsample_rates = {"camera-rgb": 3, "imu-left": 7}

        get_deliver_option(
            vrs_data_provider=self.vrs_data_provider,
            enabled_stream_labels=["camera-rgb"],  # imu-left NOT enabled
            subsample_rates=subsample_rates,
            viewer_config=self.viewer_config,
        )

        set_calls = [
            (c.args[0], c.args[1])
            for c in self.deliver_options.set_subsample_rate.call_args_list
        ]
        self.assertIn((self.stream_id_map["camera-rgb"], 3), set_calls)
        self.assertNotIn((self.stream_id_map["imu-left"], 7), set_calls)

    def test_skip_begin_and_end_convert_seconds_to_ns(self) -> None:
        get_deliver_option(
            vrs_data_provider=self.vrs_data_provider,
            enabled_stream_labels=[],
            subsample_rates={},
            viewer_config=self.viewer_config,
            skip_begin_sec=1.5,
            skip_end_sec=2.0,
        )

        self.deliver_options.set_truncate_first_device_time_ns.assert_called_once_with(
            int(1.5 * 1e9)
        )
        self.deliver_options.set_truncate_last_device_time_ns.assert_called_once_with(
            int(2.0 * 1e9)
        )

    def test_skip_none_does_not_call_truncate(self) -> None:
        get_deliver_option(
            vrs_data_provider=self.vrs_data_provider,
            enabled_stream_labels=[],
            subsample_rates={},
            viewer_config=self.viewer_config,
        )

        self.deliver_options.set_truncate_first_device_time_ns.assert_not_called()
        self.deliver_options.set_truncate_last_device_time_ns.assert_not_called()


class PlotInterpolatedHandPoseInImageTest(unittest.TestCase):
    """Tests for `plot_interpolated_hand_pose_in_image()`."""

    def test_plots_hand_pose_when_available(self) -> None:
        # Arrange
        hand_pose = MagicMock(name="hand_pose")
        vrs_data_provider = MagicMock()
        vrs_data_provider.get_interpolated_hand_pose_data.return_value = hand_pose
        aria_data_viewer = MagicMock()
        stream_id = MagicMock(name="hand_pose_stream")

        # Act
        plot_interpolated_hand_pose_in_image(
            vrs_data_provider,
            aria_data_viewer,
            timestamp_ns=100,
            camera_label="camera-rgb",
            hand_pose_stream_id=stream_id,
        )

        # Assert
        vrs_data_provider.get_interpolated_hand_pose_data.assert_called_once_with(
            stream_id=stream_id, timestamp_ns=100
        )
        aria_data_viewer.plot_hand_pose_data_2d.assert_called_once_with(
            hand_pose_data=hand_pose, camera_label="camera-rgb"
        )

    def test_does_not_plot_when_hand_pose_is_none(self) -> None:
        # Arrange
        vrs_data_provider = MagicMock()
        vrs_data_provider.get_interpolated_hand_pose_data.return_value = None
        aria_data_viewer = MagicMock()

        # Act
        plot_interpolated_hand_pose_in_image(
            vrs_data_provider,
            aria_data_viewer,
            timestamp_ns=100,
            camera_label="camera-rgb",
            hand_pose_stream_id=MagicMock(),
        )

        # Assert
        aria_data_viewer.plot_hand_pose_data_2d.assert_not_called()


class PlotQueuedSensorDataTest(unittest.TestCase):
    """Tests for `plot_queued_sensor_data()`."""

    def _make_event(self, sensor_type: SensorDataType) -> MagicMock:
        """Create a single queued sensor data mock event of the given type."""
        data = MagicMock()
        data.sensor_data_type.return_value = sensor_type
        data.get_time_ns.return_value = 1000
        return data

    def setUp(self) -> None:
        self.vrs_data_provider = MagicMock()
        # Default: no hand tracking stream, no UTC support.
        self.vrs_data_provider.get_stream_id_from_label.return_value = None
        self.vrs_data_provider.supports_time_domain.return_value = False
        # Two IMUs, matching the module's expectation for batched IMU data.
        (
            self.vrs_data_provider.get_device_calibration.return_value
        ).get_imu_labels.return_value = ["imu-left", "imu-right"]
        self.deliver_options = MagicMock()
        self.aria_data_viewer = MagicMock()

    def _run(self, events: list) -> None:
        self.vrs_data_provider.deliver_queued_sensor_data.return_value = events
        plot_queued_sensor_data(
            self.vrs_data_provider, self.deliver_options, self.aria_data_viewer
        )

    def test_stop_event_terminates_before_dispatch(self) -> None:
        # Arrange: several events but stop_event is already set.
        events = [self._make_event(SensorDataType.GPS) for _ in range(5)]
        self.vrs_data_provider.deliver_queued_sensor_data.return_value = events
        stop_event = Event()
        stop_event.set()

        # Act
        plot_queued_sensor_data(
            self.vrs_data_provider,
            self.deliver_options,
            self.aria_data_viewer,
            stop_event=stop_event,
        )

        # Assert
        self.aria_data_viewer.plot_gps.assert_not_called()

    def test_simple_sensor_types_dispatch_to_matching_plotter(self) -> None:
        """
        Verify each simple sensor data type is routed to the correct plotter
        method, and the sensor's payload is forwarded verbatim.
        """
        # Each entry: (sensor_data_type, data_getter_attr, viewer_plot_method_attr).
        # Simple = (a) single-arg payload, (b) no extra label/time context, and
        # (c) no batching. EMG/AUDIO/IMU/IMAGE are covered by dedicated tests
        # because they involve extra context.
        cases = [
            (SensorDataType.GPS, "gps_data", "plot_gps"),
            (SensorDataType.MAGNETOMETER, "magnetometer_data", "plot_magnetometer"),
            (SensorDataType.BAROMETER, "barometer_data", "plot_barometer"),
            (SensorDataType.EYE_GAZE, "eye_gaze_data", "plot_eye_gaze_data"),
            (
                SensorDataType.VIO_HIGH_FREQ,
                "vio_high_freq_data",
                "plot_vio_high_freq_data",
            ),
            (SensorDataType.VIO, "vio_data", "plot_vio_data"),
            (SensorDataType.HAND_POSE, "hand_pose_data", "plot_hand_pose_data_3d"),
        ]
        for sensor_type, data_attr, plot_attr in cases:
            with self.subTest(sensor_type=sensor_type):
                self.aria_data_viewer.reset_mock()
                payload = MagicMock(name=f"{data_attr}_payload")
                event = self._make_event(sensor_type)
                getattr(event, data_attr).return_value = payload

                self._run([event])

                getattr(self.aria_data_viewer, plot_attr).assert_called_once_with(
                    payload
                )

    def test_batches_imu_up_to_max_batch_size(self) -> None:
        # Exactly MAX_IMU_BATCH_SIZE IMU events flush once.
        self.vrs_data_provider.get_label_from_stream_id.return_value = "imu-left"
        events = [
            self._make_event(SensorDataType.IMU) for _ in range(MAX_IMU_BATCH_SIZE)
        ]

        self._run(events)

        self.aria_data_viewer.plot_imu_batch_vectorized.assert_called_once()
        args, _ = self.aria_data_viewer.plot_imu_batch_vectorized.call_args
        batched_data, label = args
        self.assertEqual(len(batched_data), MAX_IMU_BATCH_SIZE)
        self.assertEqual(label, "imu-left")

    def test_imu_batches_flush_multiple_times(self) -> None:
        # 2.5x batch size should flush exactly twice (partial third stays in buffer).
        self.vrs_data_provider.get_label_from_stream_id.return_value = "imu-left"
        n_events = MAX_IMU_BATCH_SIZE * 2 + MAX_IMU_BATCH_SIZE // 2
        events = [self._make_event(SensorDataType.IMU) for _ in range(n_events)]

        self._run(events)

        self.assertEqual(self.aria_data_viewer.plot_imu_batch_vectorized.call_count, 2)

    def test_imu_batches_are_kept_per_label(self) -> None:
        # Alternating labels means neither per-label bucket reaches batch size.
        labels = ["imu-left", "imu-right"]
        self.vrs_data_provider.get_label_from_stream_id.side_effect = [
            labels[i % 2] for i in range(MAX_IMU_BATCH_SIZE)
        ]
        events = [
            self._make_event(SensorDataType.IMU) for _ in range(MAX_IMU_BATCH_SIZE)
        ]

        self._run(events)

        self.aria_data_viewer.plot_imu_batch_vectorized.assert_not_called()

    def test_image_plots_frame_with_capture_timestamp(self) -> None:
        self.vrs_data_provider.get_label_from_stream_id.return_value = "slam-front-left"
        self.aria_data_viewer.sensor_labels.rgb_and_slam_labels = [
            "camera-rgb",
            "slam-front-left",
        ]
        frame = MagicMock(name="frame")
        image_data = MagicMock()
        image_data.to_numpy_array.return_value = frame
        record = MagicMock()
        record.capture_timestamp_ns = 2000
        event = self._make_event(SensorDataType.IMAGE)
        event.image_data_and_record.return_value = (image_data, record)

        self._run([event])

        self.aria_data_viewer.plot_image.assert_called_once_with(
            frame, "slam-front-left", 2000
        )

    def test_image_utc_timestamp_only_for_rgb_when_supported(self) -> None:
        self.vrs_data_provider.get_label_from_stream_id.return_value = "camera-rgb"
        self.vrs_data_provider.supports_time_domain.return_value = True
        self.vrs_data_provider.convert_from_device_time_to_synctime_ns.return_value = (
            5000
        )
        self.aria_data_viewer.sensor_labels.rgb_and_slam_labels = ["camera-rgb"]

        image_data = MagicMock()
        image_data.to_numpy_array.return_value = MagicMock()
        record = MagicMock()
        event = self._make_event(SensorDataType.IMAGE)
        event.image_data_and_record.return_value = (image_data, record)

        self._run([event])

        # UTC conversion path is exercised end-to-end.
        self.vrs_data_provider.convert_from_device_time_to_synctime_ns.assert_called_with(
            1000, TimeSyncMode.UTC
        )
        self.aria_data_viewer.plot_utc_timestamp.assert_called_once_with(
            5000, "camera-rgb", 1000
        )
        # Confirm the domain-support check uses TimeDomain.UTC.
        supports_args = self.vrs_data_provider.supports_time_domain.call_args.args
        self.assertEqual(supports_args[1], TimeDomain.UTC)

    def test_image_hand_pose_plotted_when_stream_and_camera_match(self) -> None:
        # Gen2-style config with hand pose stream present.
        hand_pose_stream_id = MagicMock(name="hp_stream")
        self.vrs_data_provider.get_stream_id_from_label.side_effect = (
            lambda label: hand_pose_stream_id if label == "handtracking" else None
        )
        self.vrs_data_provider.get_label_from_stream_id.return_value = "camera-rgb"
        self.aria_data_viewer.sensor_labels.rgb_and_slam_labels = ["camera-rgb"]

        interpolated_hp = MagicMock(name="interp_hp")
        self.vrs_data_provider.get_interpolated_hand_pose_data.return_value = (
            interpolated_hp
        )

        image_data = MagicMock()
        image_data.to_numpy_array.return_value = MagicMock()
        event = self._make_event(SensorDataType.IMAGE)
        event.image_data_and_record.return_value = (image_data, MagicMock())

        self._run([event])

        self.aria_data_viewer.plot_hand_pose_data_2d.assert_called_once_with(
            hand_pose_data=interpolated_hp, camera_label="camera-rgb"
        )

    def test_plots_emg_forwards_label_and_time(self) -> None:
        emg = MagicMock(name="emg")
        self.vrs_data_provider.get_label_from_stream_id.return_value = "emg"
        event = self._make_event(SensorDataType.EMG)
        event.emg_data.return_value = emg

        self._run([event])

        self.aria_data_viewer.plot_emg.assert_called_once_with(emg, "emg", 1000)

    def test_plots_audio_forwards_channel_count_from_mic_config(self) -> None:
        audio_record = MagicMock(name="audio_and_record")
        self.vrs_data_provider.get_label_from_stream_id.return_value = "mic"
        (self.vrs_data_provider.get_audio_configuration.return_value).num_channels = 7
        event = self._make_event(SensorDataType.AUDIO)
        event.audio_data_and_record.return_value = audio_record

        self._run([event])

        self.aria_data_viewer.plot_audio.assert_called_once_with(audio_record, 7)


class LogVrsToRerunTest(unittest.TestCase):
    """
    Tests for `log_vrs_to_rerun()`.

    External dependencies (VRS provider, Rerun viewer, plot loop) are mocked,
    but the internal helpers `get_deliver_option` and `parse_subsample_rates`
    execute for real so we verify orchestration end-to-end rather than merely
    checking argument forwarding.
    """

    def _setup_provider(
        self,
        device_version: DeviceVersion,
        stream_labels: list = None,
    ) -> MagicMock:
        """
        Build a mock VRS provider whose helpers cooperate with the real
        `get_deliver_option` implementation. Each label in `stream_labels`
        gets a distinct mock StreamId that flows through both
        `get_stream_id_from_label` and `deliver_options.get_stream_ids()`.
        """
        stream_labels = stream_labels or []
        self.stream_id_map = {
            label: MagicMock(name=f"stream_id_{label}") for label in stream_labels
        }

        provider = MagicMock()
        (
            provider.get_device_calibration.return_value
        ).get_device_version.return_value = device_version
        provider.get_stream_id_from_label.side_effect = (
            lambda label: self.stream_id_map.get(label)
        )

        self.deliver_options = MagicMock()
        self.deliver_options.get_stream_ids.return_value = list(
            self.stream_id_map.values()
        )
        provider.get_default_deliver_queued_options.return_value = self.deliver_options
        return provider

    def setUp(self) -> None:
        # Patch only external dependencies. `get_deliver_option` and
        # `parse_subsample_rates` are intentionally NOT patched so they run
        # for real against the mock provider.
        patchers = [
            patch(f"{_MODULE}.create_vrs_data_provider"),
            patch(f"{_MODULE}.AriaDataViewer"),
            patch(f"{_MODULE}.plot_queued_sensor_data"),
        ]
        self.mock_create_provider = patchers[0].start()
        self.mock_viewer_cls = patchers[1].start()
        self.mock_plot = patchers[2].start()
        for p in patchers:
            self.addCleanup(p.stop)

    def test_returns_early_when_provider_is_falsy(self) -> None:
        # Falsy provider short-circuits before any device access.
        self.mock_create_provider.return_value = None

        log_vrs_to_rerun("/nonexistent.vrs")

        self.mock_create_provider.assert_called_once_with("/nonexistent.vrs")
        self.mock_viewer_cls.assert_not_called()
        self.mock_plot.assert_not_called()

    def test_unsupported_device_version_raises(self) -> None:
        provider = self._setup_provider(DeviceVersion.NotValid)
        self.mock_create_provider.return_value = provider

        with self.assertRaises(ValueError) as ctx:
            log_vrs_to_rerun("/some.vrs")
        self.assertIn("Unsupported", str(ctx.exception))

    def test_gen1_activates_gen1_streams_end_to_end(self) -> None:
        # All Gen1 labels are available in this recording.
        provider = self._setup_provider(
            DeviceVersion.Gen1, stream_labels=list(ALL_STREAM_LABELS_GEN1)
        )
        self.mock_create_provider.return_value = provider

        log_vrs_to_rerun("/some.vrs")

        activated = [
            c.args[0] for c in self.deliver_options.activate_stream.call_args_list
        ]
        for label in ALL_STREAM_LABELS_GEN1:
            self.assertIn(self.stream_id_map[label], activated)

    def test_gen2_activates_gen2_streams_end_to_end(self) -> None:
        provider = self._setup_provider(
            DeviceVersion.Gen2, stream_labels=list(ALL_STREAM_LABELS_GEN2)
        )
        self.mock_create_provider.return_value = provider

        log_vrs_to_rerun("/some.vrs")

        activated = [
            c.args[0] for c in self.deliver_options.activate_stream.call_args_list
        ]
        for label in ALL_STREAM_LABELS_GEN2:
            self.assertIn(self.stream_id_map[label], activated)

    def test_default_vio_hf_subsample_rate_is_applied(self) -> None:
        from projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter import (
            AriaDataViewerConfig,
        )

        provider = self._setup_provider(
            DeviceVersion.Gen2, stream_labels=["vio_high_frequency"]
        )
        self.mock_create_provider.return_value = provider

        log_vrs_to_rerun("/some.vrs")

        # No user-provided subsample rate for vio_high_frequency, so the
        # default from AriaDataViewerConfig must be applied automatically.
        self.deliver_options.set_subsample_rate.assert_any_call(
            self.stream_id_map["vio_high_frequency"],
            AriaDataViewerConfig.vio_high_freq_subsample_rate,
        )

    def test_emg_enabled_only_when_emg_stream_present(self) -> None:
        # Provider WITH emg
        with_emg = self._setup_provider(DeviceVersion.Gen2, stream_labels=["emg"])
        self.mock_create_provider.return_value = with_emg
        log_vrs_to_rerun("/some.vrs")
        with_emg_kwargs = self.mock_viewer_cls.call_args.kwargs
        self.assertTrue(with_emg_kwargs["config"].enable_emg)

        # Provider WITHOUT emg
        self.mock_viewer_cls.reset_mock()
        no_emg = self._setup_provider(DeviceVersion.Gen2, stream_labels=["camera-rgb"])
        self.mock_create_provider.return_value = no_emg
        log_vrs_to_rerun("/some.vrs")
        no_emg_kwargs = self.mock_viewer_cls.call_args.kwargs
        self.assertFalse(no_emg_kwargs["config"].enable_emg)

    def test_user_subsample_rates_are_parsed_and_applied(self) -> None:
        # Real parse_subsample_rates + real get_deliver_option must translate
        # the CLI string `camera-rgb=2` into a set_subsample_rate call with 2.
        provider = self._setup_provider(
            DeviceVersion.Gen2, stream_labels=["camera-rgb"]
        )
        self.mock_create_provider.return_value = provider

        log_vrs_to_rerun("/some.vrs", subsample_rates=["camera-rgb=2"])

        self.deliver_options.set_subsample_rate.assert_any_call(
            self.stream_id_map["camera-rgb"], 2
        )

    def test_malformed_subsample_rates_raise_argument_error(self) -> None:
        # A malformed CLI string must surface as ArgumentTypeError even when
        # log_vrs_to_rerun is called programmatically.
        provider = self._setup_provider(
            DeviceVersion.Gen2, stream_labels=["camera-rgb"]
        )
        self.mock_create_provider.return_value = provider

        with self.assertRaises(argparse.ArgumentTypeError):
            log_vrs_to_rerun("/some.vrs", subsample_rates=["not-valid"])


class MainTest(unittest.TestCase):
    """Tests for `main()` CLI entry point."""

    @patch(f"{_MODULE}.log_vrs_to_rerun")
    @patch(f"{_MODULE}.parse_args")
    def test_main_forwards_parsed_args(
        self, mock_parse: MagicMock, mock_log: MagicMock
    ) -> None:
        mock_parse.return_value = argparse.Namespace(
            vrs="/foo.vrs",
            rrd_output_path="/out.rrd",
            subsample_rates=["camera-rgb=2"],
            enabled_streams=["camera-rgb"],
            skip_begin_sec=0.5,
            skip_end_sec=None,
        )

        main()

        mock_log.assert_called_once_with(
            vrs="/foo.vrs",
            rrd_output_path="/out.rrd",
            subsample_rates=["camera-rgb=2"],
            enabled_streams=["camera-rgb"],
            skip_begin_sec=0.5,
            skip_end_sec=None,
        )


if __name__ == "__main__":
    unittest.main()
