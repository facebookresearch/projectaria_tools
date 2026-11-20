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

"""Unit tests for the RGB loop of `log_mps_to_rerun`."""

import tempfile
import unittest
from threading import Event
from types import SimpleNamespace
from typing import Final, List
from unittest.mock import MagicMock, patch

import rerun as rr
from projectaria_tools.core.calibration import DeviceVersion
from projectaria_tools.tools.viewer_mps.rerun_viewer_mps import log_mps_to_rerun


class LogMpsRgbLoopTest(unittest.TestCase):
    """`log_mps_to_rerun` with a VRS and no MPS files, so only the RGB loop logs."""

    MPS_MODULE: Final[str] = "projectaria_tools.tools.viewer_mps.rerun_viewer_mps"

    def _frame(self, time_ns: int) -> MagicMock:
        frame = MagicMock()
        frame.get_time_ns.return_value = time_ns
        return frame

    def _run(
        self,
        frames: List[MagicMock],
        image_side_effect: object = None,
        trajectory_files: object = (),
        points_files: object = (),
        closed_loop_side_effect: object = None,
        open_loop_samples: object = (),
        eye_gaze_file: object = None,
        **kwargs: object,
    ) -> MagicMock:
        provider = MagicMock()
        provider.get_device_calibration.return_value.get_device_version.return_value = (
            DeviceVersion.Gen2
        )
        provider.deliver_queued_sensor_data.return_value = frames
        with (
            patch(f"{self.MPS_MODULE}.data_provider") as mock_data_provider,
            patch(f"{self.MPS_MODULE}.StreamId"),
            patch(f"{self.MPS_MODULE}.log_RGB_camera_calibration"),
            patch(f"{self.MPS_MODULE}.log_Aria_glasses_outline"),
            patch(f"{self.MPS_MODULE}.log_camera_pose") as self.mock_log_camera_pose,
            patch(
                f"{self.MPS_MODULE}.log_RGB_image", side_effect=image_side_effect
            ) as mock_log_image,
            patch(f"{self.MPS_MODULE}.log_eye_gaze"),
            patch(f"{self.MPS_MODULE}.tqdm"),
            patch(
                f"{self.MPS_MODULE}.mps.read_closed_loop_trajectory",
                return_value=[],
                side_effect=closed_loop_side_effect,
            ),
            patch(
                f"{self.MPS_MODULE}.mps.read_open_loop_trajectory",
                return_value=list(open_loop_samples),
            ),
        ):
            mock_data_provider.create_vrs_data_provider.return_value = provider
            log_mps_to_rerun(
                vrs_path="/some.vrs",
                trajectory_files=list(trajectory_files),
                points_files=list(points_files),
                eye_gaze_file=eye_gaze_file,
                wrist_and_palm_poses_file=None,
                hand_tracking_results_file=None,
                recording=rr.RecordingStream("mps_rgb_loop_test"),
                **kwargs,
            )
        return mock_log_image

    def test_first_data_fires_once_after_first_rgb_frame(self) -> None:
        """
        Arrange: Two RGB frames, with the image logger and the callback recording their
            order.
        Act: Run the RGB loop.
        Assert: The callback fires once, right after the first image is logged.
        """
        events: List[str] = []
        on_first_data = MagicMock(side_effect=lambda: events.append("callback"))

        self._run(
            [self._frame(1000), self._frame(2000)],
            image_side_effect=lambda *_a, **_k: events.append("image"),
            on_first_data=on_first_data,
        )

        on_first_data.assert_called_once_with()
        self.assertEqual(
            events[:2],
            ["image", "callback"],
            "the callback fires right after the first image",
        )

    def test_missing_group_files_do_not_stop_the_rgb_loop(self) -> None:
        """
        Arrange: A group member whose SLAM outputs were never produced.
        Act: Run the RGB loop with the missing trajectory and points files.
        Assert: The frame is logged and the first-data callback fires once.
        """
        on_first_data = MagicMock()

        mock_log_image = self._run(
            [self._frame(1000)],
            trajectory_files=["/nonexistent/closed_loop_trajectory.csv"],
            points_files=["/nonexistent/semidense_points.csv.gz"],
            on_first_data=on_first_data,
        )

        mock_log_image.assert_called_once()
        on_first_data.assert_called_once_with()

    def test_vrs_phase_trajectory_reader_exception_is_wrapped_with_the_path(
        self,
    ) -> None:
        """
        Arrange: A trajectory file in which the group phase finds no samples, and whose
            re-read in the VRS phase raises.
        Act: Run the RGB loop.
        Assert: A `ValueError` names the path and is chained from the reader's error.
        """
        with tempfile.NamedTemporaryFile(suffix=".csv") as trajectory:
            with self.assertRaises(ValueError) as ctx:
                self._run(
                    [self._frame(1000)],
                    trajectory_files=[trajectory.name],
                    closed_loop_side_effect=[[], RuntimeError("bad header")],
                )
        self.assertIn(
            trajectory.name, str(ctx.exception), "the error names the trajectory file"
        )
        self.assertIsInstance(
            ctx.exception.__cause__,
            RuntimeError,
            "the error is chained from the reader's error",
        )

    def test_vrs_phase_falls_back_to_open_loop_trajectory(self) -> None:
        """
        Arrange: A closed-loop reader that finds no samples and an open-loop reader that
            finds one.
        Act: Run the RGB loop.
        Assert: The camera poses come from the open-loop samples.
        """
        open_loop = [
            SimpleNamespace(
                transform_odometry_device=SimpleNamespace(
                    translation=lambda: [[0.0, 0.0, 0.0]]
                )
            )
        ]
        with tempfile.NamedTemporaryFile(suffix=".csv") as trajectory:
            self._run(
                [self._frame(1000)],
                trajectory_files=[trajectory.name],
                open_loop_samples=open_loop,
            )

        self.assertEqual(
            self.mock_log_camera_pose.call_args.args[0],
            open_loop,
            "the camera poses come from the open-loop samples",
        )

    def test_vrs_phase_eye_gaze_reader_exception_is_wrapped_with_the_path(
        self,
    ) -> None:
        """
        Arrange: An existing eye gaze file whose reader raises.
        Act: Run the RGB loop with that file.
        Assert: ValueError naming the file, like the trajectory and point reads.
        """
        with tempfile.NamedTemporaryFile(suffix=".csv") as eye_gaze_file:
            with (
                patch(
                    f"{self.MPS_MODULE}.mps.read_eyegaze",
                    side_effect=RuntimeError("bad header"),
                ),
                self.assertRaisesRegex(ValueError, eye_gaze_file.name),
            ):
                self._run([self._frame(1000)], eye_gaze_file=eye_gaze_file.name)

    def test_stop_event_ends_the_rgb_loop(self) -> None:
        """
        Arrange: Two RGB frames, and a stop event that the first logged image sets.
        Act: Run the RGB loop.
        Assert: Exactly one image is logged: the loop checks the stop event before
            each frame.
        """
        stop_event = Event()

        mock_log_image = self._run(
            [self._frame(1000), self._frame(2000)],
            image_side_effect=lambda *args, **kwargs: stop_event.set(),
            stop_event=stop_event,
        )

        self.assertEqual(
            mock_log_image.call_count, 1, "the loop stops after the first frame"
        )


if __name__ == "__main__":
    unittest.main()
