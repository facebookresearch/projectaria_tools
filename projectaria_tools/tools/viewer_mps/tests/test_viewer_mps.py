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

"""Unit tests for the viewer_mps CLI entry point."""

import argparse
import os
import sys
import unittest
from unittest.mock import MagicMock, patch

from projectaria_tools.tools.viewer_mps.viewer_mps import main, parse_args

_MODULE = "projectaria_tools.tools.viewer_mps.viewer_mps"

# Stand-in for whatever the build wrapper puts in RERUN_PATH. `rr` is mocked, so
# this is only ever compared for equality -- it is never resolved on disk.
_FAKE_RERUN_PATH = "/nonexistent/rerun-cli"


class _StopAfterSinkSelection(Exception):
    """Sentinel raised from the mocked logger to abort `main()` early.

    `main()` busy-waits forever after logging when `--web` is set, so the web
    test cannot let `main()` run to completion.
    """


class ParseArgsTest(unittest.TestCase):
    """Tests for `parse_args()`."""

    def test_defaults_to_no_rrd_output(self) -> None:
        # Arrange
        with patch.object(sys, "argv", ["viewer_mps", "--vrs", "/tmp/foo.vrs"]):
            # Act
            args = parse_args()

        # Assert
        self.assertEqual(args.rrd_output_path, "")
        self.assertFalse(args.web)

    def test_parses_rrd_output_path(self) -> None:
        # Arrange
        argv = ["viewer_mps", "--vrs", "/tmp/foo.vrs", "--rrd_output_path", "/o.rrd"]
        with patch.object(sys, "argv", argv):
            # Act
            args = parse_args()

        # Assert
        self.assertEqual(args.rrd_output_path, "/o.rrd")
        self.assertFalse(args.web)

    def test_rrd_output_path_is_documented(self) -> None:
        # The flag used to be argparse.SUPPRESS, which hid it from `--help`.
        with patch.object(sys, "argv", ["viewer_mps", "--vrs", "/tmp/foo.vrs"]):
            args = parse_args()
        self.assertEqual(args.rrd_output_path, "")

        with patch.object(sys, "argv", ["viewer_mps", "--help"]):
            with patch.object(sys, "stdout", MagicMock()) as mock_stdout:
                with self.assertRaises(SystemExit):
                    parse_args()

        help_text = "".join(
            call.args[0] for call in mock_stdout.write.call_args_list if call.args
        )
        self.assertIn("--rrd_output_path", help_text)

    def test_rrd_output_path_and_web_are_mutually_exclusive(self) -> None:
        # A run feeds exactly one rerun sink, so asking for both must fail loudly
        # rather than silently picking one.
        argv = [
            "viewer_mps",
            "--vrs",
            "/tmp/foo.vrs",
            "--rrd_output_path",
            "/o.rrd",
            "--web",
        ]
        with patch.object(sys, "argv", argv):
            with patch.object(sys, "stderr", MagicMock()):
                with self.assertRaises(SystemExit):
                    parse_args()


class MainSinkSelectionTest(unittest.TestCase):
    """Tests for which rerun sink `main()` installs."""

    def _make_args(self, **overrides: object) -> argparse.Namespace:
        # `trajectory` is populated so main() skips MpsDataPathsProvider lookup.
        defaults = {
            "vrs": "/tmp/foo.vrs",
            "trajectory": ["/tmp/closed_loop_trajectory.csv"],
            "points": None,
            "eyegaze": None,
            "hands": None,
            "hands_all": None,
            "mps_folder": None,
            "down_sampling_factor": 4,
            "jpeg_quality": 75,
            "rrd_output_path": "",
            "no_rectify_image": False,
            "web": False,
        }
        defaults.update(overrides)
        return argparse.Namespace(**defaults)

    def test_rrd_output_path_does_not_open_a_viewer(self) -> None:
        # Arrange
        args = self._make_args(rrd_output_path="/tmp/out.rrd")

        with (
            patch(f"{_MODULE}.parse_args", return_value=args),
            patch(f"{_MODULE}.rr") as mock_rr,
            patch(f"{_MODULE}.log_mps_to_rerun") as mock_log,
        ):
            # Act
            main()

        # Assert
        mock_rr.spawn.assert_not_called()
        mock_rr.serve_web.assert_not_called()
        self.assertEqual(
            mock_log.call_args.kwargs["rrd_output_path"],
            "/tmp/out.rrd",
        )

    def test_default_spawns_desktop_viewer_with_rerun_path(self) -> None:
        # Arrange
        args = self._make_args()

        with (
            patch.dict(os.environ, {"RERUN_PATH": _FAKE_RERUN_PATH}),
            patch(f"{_MODULE}.parse_args", return_value=args),
            patch(f"{_MODULE}.rr") as mock_rr,
            patch(f"{_MODULE}.log_mps_to_rerun"),
        ):
            # Act
            main()

        # Assert
        mock_rr.spawn.assert_called_once_with(executable_path=_FAKE_RERUN_PATH)
        mock_rr.serve_web.assert_not_called()

    def test_web_serves_and_does_not_spawn(self) -> None:
        # Arrange
        args = self._make_args(web=True)

        with (
            patch(f"{_MODULE}.parse_args", return_value=args),
            patch(f"{_MODULE}.rr") as mock_rr,
            patch(f"{_MODULE}.log_mps_to_rerun", side_effect=_StopAfterSinkSelection),
        ):
            # Act / Assert
            with self.assertRaises(_StopAfterSinkSelection):
                main()

        mock_rr.serve_web.assert_called_once()
        mock_rr.spawn.assert_not_called()

    def test_no_vrs_and_no_mps_data_exits(self) -> None:
        # Arrange
        args = self._make_args(vrs=None, trajectory=None)

        with (
            patch(f"{_MODULE}.parse_args", return_value=args),
            patch(f"{_MODULE}.rr") as mock_rr,
            patch(f"{_MODULE}.log_mps_to_rerun") as mock_log,
        ):
            # Act / Assert
            with self.assertRaises(SystemExit):
                main()

        mock_rr.init.assert_not_called()
        mock_log.assert_not_called()


if __name__ == "__main__":
    unittest.main()
