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

"""Unit tests for the Rerun sink that `AriaDataViewer` selects on construction."""

import unittest
from unittest.mock import MagicMock, patch

from projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter import (
    AriaDataViewer,
    AriaDataViewerConfig,
)


class RerunSinkSelectionTest(unittest.TestCase):
    """`AriaDataViewer.__init__` sink selection and `manage_rerun_sinks`."""

    PLOTTER_MODULE = "projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter"

    def setUp(self) -> None:
        self.mocks: dict[str, MagicMock] = {}
        for name, target in {
            "init": f"{self.PLOTTER_MODULE}.rr.init",
            "save": f"{self.PLOTTER_MODULE}.rr.save",
            "connect_grpc": f"{self.PLOTTER_MODULE}.rr.connect_grpc",
            "log_file_from_path": f"{self.PLOTTER_MODULE}.rr.log_file_from_path",
            "rr_spawn": f"{self.PLOTTER_MODULE}.rr.spawn",
            "spawn": f"{self.PLOTTER_MODULE}._spawn_rerun_viewer",
        }.items():
            patcher = patch(target)
            self.mocks[name] = patcher.start()
            self.addCleanup(patcher.stop)

    def _assert_only_called(self, *names: str) -> None:
        for name, mock in self.mocks.items():
            if name in names:
                mock.assert_called()
            else:
                mock.assert_not_called()

    def test_default_config_still_inits_and_spawns(self) -> None:
        AriaDataViewer()

        self.mocks["init"].assert_called_once_with("AriaDataViewer")
        self.mocks["spawn"].assert_called_once_with(
            AriaDataViewerConfig.rerun_memory_limit
        )
        self._assert_only_called("init", "spawn")

    def test_rrd_output_path_still_saves(self) -> None:
        AriaDataViewer(rrd_output_path="/tmp/out.rrd")

        self.mocks["init"].assert_called_once_with("AriaDataViewer", spawn=False)
        self.mocks["save"].assert_called_once_with("/tmp/out.rrd")
        self._assert_only_called("init", "save")

    def test_connect_to_existing_viewer_still_connects(self) -> None:
        config = AriaDataViewerConfig()
        config.connect_to_existing_viewer = True
        config.blueprint_path = "/tmp/custom.rbl"

        AriaDataViewer(config=config)

        self.mocks["init"].assert_called_once_with("AriaDataViewer")
        self.mocks["connect_grpc"].assert_called_once_with(None)
        self.mocks["log_file_from_path"].assert_called_once_with("/tmp/custom.rbl")
        self._assert_only_called("init", "connect_grpc", "log_file_from_path")

    def test_connect_branch_uses_grpc_url(self) -> None:
        config = AriaDataViewerConfig()
        config.connect_to_existing_viewer = True
        config.grpc_url = "rerun+http://127.0.0.1:9877/proxy"

        AriaDataViewer(config=config)

        self.mocks["init"].assert_called_once_with("AriaDataViewer")
        self.mocks["connect_grpc"].assert_called_once_with(
            "rerun+http://127.0.0.1:9877/proxy"
        )
        self._assert_only_called("init", "connect_grpc")

    def test_connect_branch_without_url_unchanged(self) -> None:
        config = AriaDataViewerConfig()
        config.connect_to_existing_viewer = True

        AriaDataViewer(config=config)

        self.mocks["connect_grpc"].assert_called_once_with(None)
        self._assert_only_called("init", "connect_grpc")

    def test_connect_branch_passes_an_empty_url_through(self) -> None:
        config = AriaDataViewerConfig()
        config.connect_to_existing_viewer = True
        config.grpc_url = ""

        AriaDataViewer(config=config)

        self.mocks["connect_grpc"].assert_called_once_with("")
        self._assert_only_called("init", "connect_grpc")

    def test_connect_branch_with_url_loads_custom_blueprint(self) -> None:
        config = AriaDataViewerConfig()
        config.connect_to_existing_viewer = True
        config.grpc_url = "rerun+http://127.0.0.1:9877/proxy"
        config.blueprint_path = "/tmp/custom.rbl"

        AriaDataViewer(config=config)

        self.mocks["connect_grpc"].assert_called_once_with(
            "rerun+http://127.0.0.1:9877/proxy"
        )
        self.mocks["log_file_from_path"].assert_called_once_with("/tmp/custom.rbl")
        self._assert_only_called("init", "connect_grpc", "log_file_from_path")

    def test_default_branch_unchanged(self) -> None:
        config = AriaDataViewerConfig()
        config.grpc_url = "rerun+http://127.0.0.1:9877/proxy"

        AriaDataViewer(config=config)

        self.mocks["spawn"].assert_called_once_with(config.rerun_memory_limit)
        self._assert_only_called("init", "spawn")

    def test_blueprint_path_still_spawns_with_blueprint(self) -> None:
        config = AriaDataViewerConfig()
        config.blueprint_path = "/tmp/custom.rbl"

        AriaDataViewer(config=config)

        self.mocks["init"].assert_called_once_with("AriaDataViewer")
        self.mocks["spawn"].assert_called_once_with(config.rerun_memory_limit)
        self.mocks["log_file_from_path"].assert_called_once_with("/tmp/custom.rbl")
        self._assert_only_called("init", "spawn", "log_file_from_path")

    def test_unmanaged_sinks_makes_no_init_save_connect_spawn(self) -> None:
        cases = {
            "default": {},
            "rrd_output_path": {"rrd_output_path": "/tmp/out.rrd"},
            "connect_to_existing_viewer": {"connect_to_existing_viewer": True},
            "grpc_url": {
                "connect_to_existing_viewer": True,
                "grpc_url": "rerun+http://127.0.0.1:9877/proxy",
            },
        }
        for name, settings in cases.items():
            with self.subTest(case=name):
                config = AriaDataViewerConfig()
                config.manage_rerun_sinks = False
                rrd_output_path = settings.get("rrd_output_path", "")
                config.connect_to_existing_viewer = settings.get(
                    "connect_to_existing_viewer", False
                )
                config.grpc_url = settings.get("grpc_url")
                config.blueprint_path = settings.get("blueprint_path")

                AriaDataViewer(config=config, rrd_output_path=rrd_output_path)

                self._assert_only_called()

    def test_unmanaged_sinks_load_the_blueprint_into_the_active_recording(
        self,
    ) -> None:
        config = AriaDataViewerConfig()
        config.manage_rerun_sinks = False
        config.blueprint_path = "/tmp/custom.rbl"

        AriaDataViewer(config=config)

        self.mocks["log_file_from_path"].assert_called_once_with("/tmp/custom.rbl")
        self._assert_only_called("log_file_from_path")
