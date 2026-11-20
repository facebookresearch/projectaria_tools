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

"""Unit tests for `log_vrs_to_rerun(recording=...)` against a real `rr.RecordingStream`."""

import os
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import rerun as rr
from projectaria_tools.core.calibration import DeviceVersion
from projectaria_tools.core.sensor_data import SensorDataType
from projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter import AriaDataViewer
from projectaria_tools.tools.aria_rerun_viewer.aria_rerun_viewer import log_vrs_to_rerun


class LogVrsToSuppliedRecordingTest(unittest.TestCase):
    """`log_vrs_to_rerun(recording=...)` against a real `rr.RecordingStream`."""

    MODULE = "projectaria_tools.tools.aria_rerun_viewer.aria_rerun_viewer"
    PLOTTER_MODULE = "projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter"

    @staticmethod
    def _entity_paths(
        recording: rr.RecordingStream, memory: rr.MemoryRecording
    ) -> set[str]:
        """Entity paths logged to `recording`, read back from its in-memory sink."""
        recording.flush()
        with tempfile.TemporaryDirectory() as tmp_dir:
            rrd_path = os.path.join(tmp_dir, "logged.rrd")
            with open(rrd_path, "wb") as rrd_file:
                rrd_file.write(memory.drain_as_bytes())
            return LogVrsToSuppliedRecordingTest._rrd_entity_paths(rrd_path)

    @staticmethod
    def _rrd_entity_paths(rrd_path: str) -> set[str]:
        """Entity paths in the `.rrd` file at `rrd_path`."""
        chunk_store = rr.experimental.RrdReader(rrd_path).stream().collect()
        return set(chunk_store.schema().entity_paths())

    def _provider_with_one_gps_sample(self) -> MagicMock:
        provider = MagicMock()
        calibration = provider.get_device_calibration.return_value
        calibration.get_device_version.return_value = DeviceVersion.Gen1
        calibration.get_imu_labels.return_value = []
        calibration.get_camera_calib.return_value.get_image_size.return_value = [
            100,
            100,
        ]
        provider.get_stream_id_from_label.return_value = None
        event = MagicMock()
        event.sensor_data_type.return_value = SensorDataType.GPS
        event.get_time_ns.return_value = 1000
        event.gps_data.return_value = SimpleNamespace(
            latitude=47.6,
            longitude=-122.3,
            capture_timestamp_ns=1_000_000_000,
            provider="GPS",
        )
        provider.deliver_queued_sensor_data.return_value = [event]
        return provider

    def test_supplied_recording_receives_logs(self) -> None:
        recording = rr.RecordingStream("supplied_recording_test")
        memory = recording.memory_recording()
        on_first_data = MagicMock()

        with (
            patch(f"{self.MODULE}.create_vrs_data_provider") as mock_create_provider,
            patch(f"{self.PLOTTER_MODULE}.rr.init") as mock_init,
            patch(f"{self.PLOTTER_MODULE}._spawn_rerun_viewer") as mock_spawn,
            patch.object(AriaDataViewer, "plot_device_extrinsics"),
        ):
            mock_create_provider.return_value = self._provider_with_one_gps_sample()

            log_vrs_to_rerun(
                "/some.vrs", recording=recording, on_first_data=on_first_data
            )

        self.assertIn(
            "/gps",
            self._entity_paths(recording, memory),
            "the GPS sample reaches the supplied recording",
        )
        on_first_data.assert_called_once_with()
        mock_init.assert_not_called()
        mock_spawn.assert_not_called()

    def test_recording_already_entered_by_caller_is_not_entered_again(self) -> None:
        recording = rr.RecordingStream("nested_recording_test")
        memory = recording.memory_recording()

        with (
            patch(f"{self.MODULE}.create_vrs_data_provider") as mock_create_provider,
            patch.object(AriaDataViewer, "plot_device_extrinsics"),
        ):
            mock_create_provider.return_value = self._provider_with_one_gps_sample()

            # Exiting the outer `with` used to raise "Token ... has already been
            # used once" after the inner enter overwrote the context token.
            with recording:
                log_vrs_to_rerun("/some.vrs", recording=recording)
                self.assertEqual(
                    rr.get_thread_local_data_recording().get_recording_id(),
                    recording.get_recording_id(),
                    "the caller's recording is still the thread-local one",
                )

        self.assertIn(
            "/gps",
            self._entity_paths(recording, memory),
            "the GPS sample reaches the caller's recording",
        )
        self.assertIsNone(
            rr.get_thread_local_data_recording(),
            "the caller's exit leaves no thread-local recording",
        )

    def test_corrupt_vrs_error_survives_a_caller_entered_recording(self) -> None:
        recording = rr.RecordingStream("nested_corrupt_test")

        with patch(f"{self.MODULE}.create_vrs_data_provider", return_value=None):
            with self.assertRaisesRegex(ValueError, "/corrupt.vrs"):
                with recording:
                    log_vrs_to_rerun("/corrupt.vrs", recording=recording)

    def test_different_active_recording_is_replaced_for_the_call(self) -> None:
        outer = rr.RecordingStream("outer_recording_test")
        supplied = rr.RecordingStream("supplied_recording_test")
        memory = supplied.memory_recording()

        with (
            patch(f"{self.MODULE}.create_vrs_data_provider") as mock_create_provider,
            patch.object(AriaDataViewer, "plot_device_extrinsics"),
        ):
            mock_create_provider.return_value = self._provider_with_one_gps_sample()

            with outer:
                log_vrs_to_rerun("/some.vrs", recording=supplied)
                self.assertEqual(
                    rr.get_thread_local_data_recording().get_recording_id(),
                    outer.get_recording_id(),
                    "the outer recording is restored after the call",
                )

        self.assertIn(
            "/gps",
            self._entity_paths(supplied, memory),
            "the GPS sample reaches the supplied recording, not the outer one",
        )

    def test_caller_entered_recording_used_from_a_worker_thread(self) -> None:
        recording = rr.RecordingStream("cross_thread_recording_test")
        memory = recording.memory_recording()
        errors: list[Exception] = []

        def load_on_worker() -> None:
            try:
                log_vrs_to_rerun("/some.vrs", recording=recording)
            except Exception as e:
                errors.append(e)

        with (
            patch(f"{self.MODULE}.create_vrs_data_provider") as mock_create_provider,
            patch.object(AriaDataViewer, "plot_device_extrinsics"),
        ):
            mock_create_provider.return_value = self._provider_with_one_gps_sample()
            with recording:
                worker = threading.Thread(target=load_on_worker)
                worker.start()
                worker.join()
                self.assertEqual(
                    rr.get_thread_local_data_recording().get_recording_id(),
                    recording.get_recording_id(),
                    "the caller's thread still has its recording",
                )

        self.assertEqual(errors, [], "the worker's load succeeds")
        self.assertIsNone(
            rr.get_thread_local_data_recording(),
            "the caller's exit succeeds and leaves no thread-local recording",
        )
        self.assertIn(
            "/gps",
            self._entity_paths(recording, memory),
            "the worker's data reaches the caller's recording",
        )

    def test_caller_file_sink_stays_open_across_loads(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            rrd_path = os.path.join(tmp_dir, "out.rrd")
            recording = rr.RecordingStream("file_sink_test")
            recording.save(rrd_path)

            with (
                patch(
                    f"{self.MODULE}.create_vrs_data_provider"
                ) as mock_create_provider,
                patch.object(AriaDataViewer, "plot_device_extrinsics"),
            ):
                mock_create_provider.side_effect = (
                    lambda *_, **__: self._provider_with_one_gps_sample()
                )
                log_vrs_to_rerun("/first.vrs", recording=recording)
                log_vrs_to_rerun("/second.vrs", recording=recording)
            rr.log("caller/marker", rr.TextLog("done"), recording=recording)
            recording.flush()
            recording.disconnect()

            entity_paths = self._rrd_entity_paths(rrd_path)

        self.assertIn("/gps", entity_paths, "the loads reach the caller's file")
        self.assertIn(
            "/caller/marker",
            entity_paths,
            "the caller's file sink is still open after both loads",
        )
