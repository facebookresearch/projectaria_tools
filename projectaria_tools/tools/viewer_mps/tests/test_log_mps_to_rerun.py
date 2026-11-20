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

"""Unit tests for `log_mps_to_rerun`: sinks, supplied recordings, stop and skips."""

import argparse
import contextlib
import io
import os
import tempfile
import unittest
from threading import Event
from types import SimpleNamespace
from typing import Final, List
from unittest.mock import MagicMock, patch

import rerun as rr
from projectaria_tools.tools.viewer_mps.rerun_viewer_mps import log_mps_to_rerun
from projectaria_tools.tools.viewer_mps.viewer_mps import main


class LogMpsToRerunTest(unittest.TestCase):
    """`log_mps_to_rerun` against a real `rr.RecordingStream`, native readers patched."""

    MODULE: Final[str] = "projectaria_tools.tools.viewer_mps.viewer_mps"
    MPS_MODULE: Final[str] = "projectaria_tools.tools.viewer_mps.rerun_viewer_mps"

    @classmethod
    def _entity_paths(
        cls, recording: rr.RecordingStream, memory: rr.MemoryRecording
    ) -> set[str]:
        """Entity paths logged to `recording`, read back from its in-memory sink."""
        recording.flush()
        with tempfile.TemporaryDirectory() as tmp_dir:
            rrd_path = os.path.join(tmp_dir, "logged.rrd")
            with open(rrd_path, "wb") as rrd_file:
                rrd_file.write(memory.drain_as_bytes())
            return cls._rrd_entity_paths(rrd_path)

    @staticmethod
    def _rrd_entity_paths(rrd_path: str) -> set[str]:
        """Entity paths in the `.rrd` file at `rrd_path`."""
        chunk_store = rr.experimental.RrdReader(rrd_path).stream().collect()
        return set(chunk_store.schema().entity_paths())

    @staticmethod
    def _closed_loop_samples() -> List[SimpleNamespace]:
        return [
            SimpleNamespace(
                transform_world_device=SimpleNamespace(
                    translation=lambda x=float(i): [[x, 0.0, 0.0]]
                )
            )
            for i in range(3)
        ]

    @staticmethod
    def _points() -> List[SimpleNamespace]:
        return [SimpleNamespace(position_world=[float(i), 0.0, 0.0]) for i in range(3)]

    def setUp(self) -> None:
        self._tmp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp_dir.cleanup)
        self.trajectory_file = self._touch("closed_loop_trajectory.csv")
        self.second_trajectory_file = self._touch("closed_loop_trajectory_2.csv")
        self.points_file = self._touch("semidense_points.csv.gz")

        patchers = {
            "read_closed_loop": patch(
                f"{self.MPS_MODULE}.mps.read_closed_loop_trajectory",
                side_effect=lambda _path: self._closed_loop_samples(),
            ),
            "read_open_loop": patch(f"{self.MPS_MODULE}.mps.read_open_loop_trajectory"),
            "read_points": patch(
                f"{self.MPS_MODULE}.mps.read_global_point_cloud",
                side_effect=lambda _path: self._points(),
            ),
            "filter_confidence": patch(
                f"{self.MPS_MODULE}.filter_points_from_confidence",
                side_effect=lambda points: points,
            ),
            "filter_count": patch(
                f"{self.MPS_MODULE}.filter_points_from_count",
                side_effect=lambda points, _count: points,
            ),
        }
        self.mocks = {}
        for name, patcher in patchers.items():
            self.mocks[name] = patcher.start()
            self.addCleanup(patcher.stop)

    def _touch(self, name: str) -> str:
        path = os.path.join(self._tmp_dir.name, name)
        with open(path, "w") as f:
            f.write("data")
        return path

    def _log(self, **overrides: object) -> None:
        kwargs = {
            "vrs_path": None,
            "trajectory_files": [],
            "points_files": [],
            "eye_gaze_file": None,
            "wrist_and_palm_poses_file": None,
            "hand_tracking_results_file": None,
        }
        kwargs.update(overrides)
        log_mps_to_rerun(**kwargs)

    def test_default_path_unchanged_for_cli(self) -> None:
        """
        Arrange: `rr` patched in the module under test.
        Act: Log a trajectory and points with `rrd_output_path` and no recording.
        Assert: The CLI path saves to the file, never looks up a thread-local recording,
            and logs the world, trajectory and points in that order.
        """
        with (
            patch(f"{self.MPS_MODULE}.rr") as mock_rr,
            patch(
                "projectaria_tools.utils.rerun_helpers.rr.set_thread_local_data_recording"
            ) as set_thread_local,
        ):
            self._log(
                trajectory_files=[self.trajectory_file],
                points_files=[self.points_file],
                rrd_output_path="/tmp/out.rrd",
            )

        mock_rr.save.assert_called_once_with("/tmp/out.rrd")
        set_thread_local.assert_not_called()
        logged = [call.args[0] for call in mock_rr.log.call_args_list]
        self.assertEqual(
            logged,
            ["world", "world/device_trajectory", "world/points"],
            "the CLI logs the world, trajectory and points in order",
        )

    def test_cli_main_passes_no_new_arguments(self) -> None:
        """
        Arrange: CLI arguments, with `log_mps_to_rerun` patched.
        Act: Run `main`.
        Assert: `main` passes none of `recording`, `on_first_data` or `stop_event`.
        """
        args = argparse.Namespace(
            vrs="/tmp/foo.vrs",
            trajectory=["/tmp/closed_loop_trajectory.csv"],
            points=None,
            eyegaze=None,
            hands=None,
            hands_all=None,
            mps_folder=None,
            down_sampling_factor=4,
            jpeg_quality=75,
            rrd_output_path="/tmp/out.rrd",
            no_rectify_image=False,
            web=False,
        )
        with (
            patch(f"{self.MODULE}.parse_args", return_value=args),
            patch(f"{self.MODULE}.rr"),
            patch(f"{self.MODULE}.log_mps_to_rerun") as mock_log,
        ):
            main()

        for name in ("recording", "on_first_data", "stop_event"):
            self.assertNotIn(
                name, mock_log.call_args.kwargs, "main does not pass the new argument"
            )

    def test_supplied_recording_receives_world_and_trajectory(self) -> None:
        """
        Arrange: A real recording with a memory sink, and `rr.save` patched.
        Act: Log a trajectory and points into it, also passing `rrd_output_path`.
        Assert: The world, trajectory and points reach the recording, and the `.rrd`
            path is ignored.
        """
        recording = rr.RecordingStream("supplied_mps_recording_test")
        memory = recording.memory_recording()

        with patch(f"{self.MPS_MODULE}.rr.save") as mock_save:
            self._log(
                trajectory_files=[self.trajectory_file],
                points_files=[self.points_file],
                rrd_output_path="/tmp/ignored.rrd",
                recording=recording,
            )

        paths = self._entity_paths(recording, memory)
        self.assertIn("/world", paths, "the supplied recording receives the world")
        self.assertIn(
            "/world/device_trajectory",
            paths,
            "the supplied recording receives the trajectory",
        )
        self.assertIn(
            "/world/points", paths, "the supplied recording receives the points"
        )
        mock_save.assert_not_called()

    def test_recording_already_entered_by_caller_is_not_entered_again(self) -> None:
        """
        Arrange: A real recording that the caller has entered.
        Act: Log a trajectory into it inside the caller's `with`. Exiting that `with`
            used to raise "Token ... has already been used once" after an inner enter
            overwrote the context token.
        Assert: The recording stays thread-local during the call, the trajectory reaches
            it, and the caller's exit leaves no thread-local recording.
        """
        recording = rr.RecordingStream("nested_mps_recording_test")
        memory = recording.memory_recording()

        with recording:
            self._log(trajectory_files=[self.trajectory_file], recording=recording)
            self.assertEqual(
                rr.get_thread_local_data_recording().get_recording_id(),
                recording.get_recording_id(),
                "the caller's recording stays thread-local during the call",
            )

        self.assertIn(
            "/world/device_trajectory",
            self._entity_paths(recording, memory),
            "the trajectory reaches the caller's recording",
        )
        self.assertIsNone(
            rr.get_thread_local_data_recording(),
            "the caller's exit leaves no thread-local recording",
        )

    def test_different_active_recording_is_replaced_for_the_call(self) -> None:
        """
        Arrange: An outer recording that is active, and a supplied recording with a
            memory sink.
        Act: Log a trajectory into the supplied recording inside the outer `with`.
        Assert: The outer recording is thread-local again after the call, and the
            trajectory reached the supplied recording.
        """
        outer = rr.RecordingStream("outer_mps_recording_test")
        supplied = rr.RecordingStream("supplied_mps_recording_test")
        memory = supplied.memory_recording()

        with outer:
            self._log(trajectory_files=[self.trajectory_file], recording=supplied)
            self.assertEqual(
                rr.get_thread_local_data_recording().get_recording_id(),
                outer.get_recording_id(),
                "the outer recording is restored after the call",
            )

        self.assertIn(
            "/world/device_trajectory",
            self._entity_paths(supplied, memory),
            "the trajectory reaches the supplied recording, not the outer one",
        )

    def test_stop_before_point_cloud_skips_read(self) -> None:
        """
        Arrange: A stop event that is already set.
        Act: Log a points file.
        Assert: The point cloud is not read.
        """
        stop_event = Event()
        stop_event.set()

        self._log(points_files=[self.points_file], stop_event=stop_event)

        self.mocks["read_points"].assert_not_called()

    def test_stop_before_trajectory_skips_read(self) -> None:
        """
        Arrange: A stop event that is already set.
        Act: Log a trajectory file.
        Assert: The trajectory is not read.
        """
        stop_event = Event()
        stop_event.set()

        self._log(trajectory_files=[self.trajectory_file], stop_event=stop_event)

        self.mocks["read_closed_loop"].assert_not_called()

    def test_stop_between_trajectories(self) -> None:
        """
        Arrange: Two trajectories and a points file, with `on_first_data` setting the
            stop event.
        Act: Log them; the stop request arrives as the first trajectory is logged.
        Assert: Only the first trajectory is read and logged, and the points are not
            read.
        """
        recording = rr.RecordingStream("stop_between_trajectories_test")
        memory = recording.memory_recording()
        stop_event = Event()

        self._log(
            trajectory_files=[self.trajectory_file, self.second_trajectory_file],
            points_files=[self.points_file],
            recording=recording,
            stop_event=stop_event,
            on_first_data=stop_event.set,
        )

        self.mocks["read_closed_loop"].assert_called_once_with(self.trajectory_file)
        self.mocks["read_points"].assert_not_called()
        paths = self._entity_paths(recording, memory)
        self.assertIn(
            "/world/device_trajectory_0", paths, "the first trajectory is logged"
        )
        self.assertNotIn(
            "/world/device_trajectory_1",
            paths,
            "the second trajectory is not logged after the stop",
        )

    def test_stop_after_trajectory_read_skips_log(self) -> None:
        """
        Arrange: A closed-loop reader that returns samples and sets the stop event, with
            `rr.LineStrips3D` patched.
        Act: Log the trajectory.
        Assert: No sample is converted, no line strip is built, the trajectory is not
            logged and `on_first_data` is not called.
        """
        recording = rr.RecordingStream("stop_after_trajectory_read_test")
        memory = recording.memory_recording()
        stop_event = Event()
        translation = MagicMock(return_value=[[0.0, 0.0, 0.0]])
        samples = [
            SimpleNamespace(
                transform_world_device=SimpleNamespace(translation=translation)
            )
        ]
        self.mocks["read_closed_loop"].side_effect = lambda _path: (
            stop_event.set() or samples
        )
        on_first_data = MagicMock()

        with patch(f"{self.MPS_MODULE}.rr.LineStrips3D") as line_strips:
            self._log(
                trajectory_files=[self.trajectory_file],
                recording=recording,
                stop_event=stop_event,
                on_first_data=on_first_data,
            )

        translation.assert_not_called()
        line_strips.assert_not_called()
        self.assertNotIn(
            "/world/device_trajectory",
            self._entity_paths(recording, memory),
            "a stopped load does not log the trajectory",
        )
        on_first_data.assert_not_called()

    def test_stop_after_open_loop_read_skips_log(self) -> None:
        """
        Arrange: An empty closed-loop read, then an open-loop reader that returns
            samples and sets the stop event, with `rr.LineStrips3D` patched.
        Act: Log the trajectory.
        Assert: The open-loop file is read, no sample is converted, no line strip is
            built, the trajectory is not logged and `on_first_data` is not called.
        """
        recording = rr.RecordingStream("stop_after_open_loop_read_test")
        memory = recording.memory_recording()
        stop_event = Event()
        translation = MagicMock(return_value=[[0.0, 0.0, 0.0]])
        samples = [
            SimpleNamespace(
                transform_odometry_device=SimpleNamespace(translation=translation)
            )
        ]
        self.mocks["read_closed_loop"].side_effect = lambda _path: []
        self.mocks["read_open_loop"].side_effect = lambda _path: (
            stop_event.set() or samples
        )
        on_first_data = MagicMock()

        with patch(f"{self.MPS_MODULE}.rr.LineStrips3D") as line_strips:
            self._log(
                trajectory_files=[self.trajectory_file],
                recording=recording,
                stop_event=stop_event,
                on_first_data=on_first_data,
            )

        self.mocks["read_open_loop"].assert_called_once_with(self.trajectory_file)
        translation.assert_not_called()
        line_strips.assert_not_called()
        self.assertNotIn(
            "/world/device_trajectory",
            self._entity_paths(recording, memory),
            "a stopped load does not log the trajectory",
        )
        on_first_data.assert_not_called()

    def test_open_loop_trajectory_is_used_when_closed_loop_is_empty(self) -> None:
        """
        Arrange: A closed-loop reader that finds no samples and an open-loop reader that
            finds one.
        Act: Log the trajectory.
        Assert: The trajectory is logged from the open-loop samples.
        """
        recording = rr.RecordingStream("open_loop_trajectory_test")
        memory = recording.memory_recording()
        self.mocks["read_closed_loop"].side_effect = lambda _path: []
        self.mocks["read_open_loop"].side_effect = lambda _path: [
            SimpleNamespace(
                transform_odometry_device=SimpleNamespace(
                    translation=lambda: [[1.0, 2.0, 3.0]]
                )
            )
        ]

        self._log(trajectory_files=[self.trajectory_file], recording=recording)

        self.assertIn(
            "/world/device_trajectory",
            self._entity_paths(recording, memory),
            "the open-loop trajectory is logged",
        )

    def test_stop_after_point_cloud_read_skips_filter_and_log(self) -> None:
        """
        Arrange: A point cloud reader that sets the stop event.
        Act: Log the points file.
        Assert: The points are not filtered.
        """
        stop_event = Event()
        self.mocks["read_points"].side_effect = lambda _path: (
            stop_event.set() or self._points()
        )

        self._log(points_files=[self.points_file], stop_event=stop_event)

        self.mocks["filter_confidence"].assert_not_called()

    def test_stop_between_filter_and_log_skips_log(self) -> None:
        """
        Arrange: A count filter that sets the stop event.
        Act: Log the points file.
        Assert: The points are not logged.
        """
        recording = rr.RecordingStream("stop_between_filter_and_log_test")
        memory = recording.memory_recording()
        stop_event = Event()
        self.mocks["filter_count"].side_effect = lambda points, _count: (
            stop_event.set() or points
        )

        self._log(
            points_files=[self.points_file],
            recording=recording,
            stop_event=stop_event,
        )

        self.assertNotIn(
            "/world/points",
            self._entity_paths(recording, memory),
            "a stopped load does not log the points",
        )

    def test_first_data_fires_for_group_only_input(self) -> None:
        """
        Arrange: An `on_first_data` callback, and no VRS, so there is no RGB loop
            to carry the callback.
        Act: Log a points file only.
        Assert: The callback fires once.
        """
        on_first_data = MagicMock()

        self._log(points_files=[self.points_file], on_first_data=on_first_data)

        on_first_data.assert_called_once_with()

    def test_first_data_fires_once_across_trajectories_and_points(self) -> None:
        """
        Arrange: An `on_first_data` callback.
        Act: Log two trajectories and a points file.
        Assert: The callback fires once.
        """
        on_first_data = MagicMock()

        self._log(
            trajectory_files=[self.trajectory_file, self.second_trajectory_file],
            points_files=[self.points_file],
            on_first_data=on_first_data,
        )

        on_first_data.assert_called_once_with()

    def test_first_data_exception_propagates(self) -> None:
        """
        Arrange: An `on_first_data` callback that raises.
        Act: Log a points file.
        Assert: The callback's `RuntimeError` propagates.
        """
        on_first_data = MagicMock(side_effect=RuntimeError("callback failed"))

        with self.assertRaisesRegex(RuntimeError, "callback failed"):
            self._log(points_files=[self.points_file], on_first_data=on_first_data)

    def test_missing_points_file_is_skipped_with_warning(self) -> None:
        """
        Arrange: A missing points file listed before an existing one.
        Act: Log both, capturing stdout.
        Assert: A warning names the missing file, only the existing file is read, and
            the first-data callback fires once.
        """
        missing = os.path.join(self._tmp_dir.name, "missing_points.csv.gz")
        on_first_data = MagicMock()
        stdout = io.StringIO()

        with contextlib.redirect_stdout(stdout):
            self._log(
                points_files=[missing, self.points_file], on_first_data=on_first_data
            )

        self.assertIn(missing, stdout.getvalue(), "the warning names the missing file")
        self.assertIn("WARNING", stdout.getvalue(), "the skip is reported as a warning")
        self.mocks["read_points"].assert_called_once_with(self.points_file)
        on_first_data.assert_called_once_with()

    def test_missing_trajectory_file_is_skipped_with_warning(self) -> None:
        """
        Arrange: A missing trajectory file listed before an existing one, and a points
            file.
        Act: Log them, capturing stdout.
        Assert: A warning names the missing file, only the existing trajectory is read,
            and the second trajectory and the points are logged.
        """
        recording = rr.RecordingStream("missing_trajectory_test")
        memory = recording.memory_recording()
        missing = os.path.join(self._tmp_dir.name, "missing_trajectory.csv")
        stdout = io.StringIO()

        with contextlib.redirect_stdout(stdout):
            self._log(
                trajectory_files=[missing, self.trajectory_file],
                points_files=[self.points_file],
                recording=recording,
            )

        self.assertIn(missing, stdout.getvalue(), "the warning names the missing file")
        self.mocks["read_closed_loop"].assert_called_once_with(self.trajectory_file)
        paths = self._entity_paths(recording, memory)
        self.assertNotIn(
            "/world/device_trajectory_0", paths, "the missing trajectory is not logged"
        )
        self.assertIn(
            "/world/device_trajectory_1", paths, "the existing trajectory is logged"
        )
        self.assertIn("/world/points", paths, "the points are still logged")

    def test_empty_points_file_is_skipped_with_warning(self) -> None:
        """
        Arrange: A points reader that returns an empty cloud for the first file, as the
            native reader does for a malformed file.
        Act: Log both files, capturing stdout.
        Assert: A warning names the empty file, only the second cloud is logged, and the
            first-data callback fires once.
        """
        recording = rr.RecordingStream("empty_points_test")
        memory = recording.memory_recording()
        empty = self._touch("empty_points.csv.gz")
        self.mocks["read_points"].side_effect = lambda path: (
            [] if path == empty else self._points()
        )
        on_first_data = MagicMock()
        stdout = io.StringIO()

        with contextlib.redirect_stdout(stdout):
            self._log(
                points_files=[empty, self.points_file],
                recording=recording,
                on_first_data=on_first_data,
            )

        self.assertIn(empty, stdout.getvalue(), "the warning names the empty file")
        paths = self._entity_paths(recording, memory)
        self.assertNotIn("/world/points_0", paths, "the empty cloud is not logged")
        self.assertIn("/world/points_1", paths, "the second cloud is logged")
        on_first_data.assert_called_once_with()

    def test_empty_trajectory_file_is_skipped_with_warning(self) -> None:
        """
        Arrange: Readers that find no samples in the first trajectory file.
        Act: Log both files, capturing stdout.
        Assert: A warning names the empty file, and only the second trajectory is
            logged.
        """
        recording = rr.RecordingStream("empty_trajectory_test")
        memory = recording.memory_recording()
        empty = self._touch("empty_trajectory.csv")
        self.mocks["read_closed_loop"].side_effect = lambda path: (
            [] if path == empty else self._closed_loop_samples()
        )
        self.mocks["read_open_loop"].side_effect = lambda _path: []
        stdout = io.StringIO()

        with contextlib.redirect_stdout(stdout):
            self._log(
                trajectory_files=[empty, self.trajectory_file], recording=recording
            )

        self.assertIn(empty, stdout.getvalue(), "the warning names the empty file")
        paths = self._entity_paths(recording, memory)
        self.assertNotIn(
            "/world/device_trajectory_0", paths, "the empty trajectory is not logged"
        )
        self.assertIn(
            "/world/device_trajectory_1", paths, "the second trajectory is logged"
        )

    def test_first_data_does_not_fire_when_every_file_is_skipped(self) -> None:
        """
        Arrange: Only a missing points file.
        Act: Log it.
        Assert: The first-data callback does not fire.
        """
        missing = os.path.join(self._tmp_dir.name, "missing_points.csv.gz")
        on_first_data = MagicMock()

        self._log(points_files=[missing], on_first_data=on_first_data)

        on_first_data.assert_not_called()

    def test_reader_exception_is_wrapped_with_the_path(self) -> None:
        """
        Arrange: A point cloud reader that raises.
        Act: Log the points file.
        Assert: A `ValueError` names the path and the reader's message.
        """
        self.mocks["read_points"].side_effect = RuntimeError("bad header")

        with self.assertRaisesRegex(ValueError, f"{self.points_file}: bad header"):
            self._log(points_files=[self.points_file])

    def test_trajectory_reader_exception_is_wrapped_with_the_path(self) -> None:
        """
        Arrange: A trajectory reader that raises.
        Act: Log the trajectory file.
        Assert: A `ValueError` names the path and is chained from the reader's error.
        """
        self.mocks["read_closed_loop"].side_effect = RuntimeError("bad header")

        with self.assertRaises(ValueError) as ctx:
            self._log(trajectory_files=[self.trajectory_file])
        self.assertIn(
            self.trajectory_file,
            str(ctx.exception),
            "the error names the trajectory file",
        )
        self.assertIsInstance(
            ctx.exception.__cause__,
            RuntimeError,
            "the error is chained from the reader's error",
        )

    def test_reader_error_survives_a_caller_entered_recording(self) -> None:
        """
        Arrange: A point cloud reader that raises, and a recording the caller has
            entered.
        Act: Log the points file inside the caller's `with`.
        Assert: The `ValueError` is not replaced by a token `RuntimeError`.
        """
        recording = rr.RecordingStream("nested_reader_error_test")
        self.mocks["read_points"].side_effect = RuntimeError("bad header")

        with self.assertRaisesRegex(ValueError, self.points_file):
            with recording:
                self._log(points_files=[self.points_file], recording=recording)

    def test_stop_before_vrs_skips_opening_it(self) -> None:
        """
        Arrange: A points file and a VRS, with `on_first_data` setting the stop event.
        Act: Log them; the stop request arrives as the point cloud is logged.
        Assert: The VRS is never opened.
        """
        stop_event = Event()

        with patch(f"{self.MPS_MODULE}.data_provider") as mock_data_provider:
            self._log(
                vrs_path="/some.vrs",
                points_files=[self.points_file],
                stop_event=stop_event,
                on_first_data=stop_event.set,
            )

        mock_data_provider.create_vrs_data_provider.assert_not_called()

    def test_caller_file_sink_stays_open_across_loads(self) -> None:
        """
        Arrange: A real recording that saves to a caller-owned `.rrd` file.
        Act: Load a trajectory into it twice, then log a caller marker and close it.
        Assert: Both the trajectory and the caller's marker are in the file, so the
            loads did not finalize the caller's sink.
        """
        rrd_path = os.path.join(self._tmp_dir.name, "out.rrd")
        recording = rr.RecordingStream("mps_file_sink_test")
        recording.save(rrd_path)

        self._log(trajectory_files=[self.trajectory_file], recording=recording)
        self._log(trajectory_files=[self.trajectory_file], recording=recording)
        rr.log("caller/marker", rr.TextLog("done"), recording=recording)
        recording.flush()
        recording.disconnect()

        paths = self._rrd_entity_paths(rrd_path)
        self.assertIn(
            "/world/device_trajectory", paths, "the loads reach the caller's file"
        )
        self.assertIn(
            "/caller/marker",
            paths,
            "the caller's file sink is still open after both loads",
        )
