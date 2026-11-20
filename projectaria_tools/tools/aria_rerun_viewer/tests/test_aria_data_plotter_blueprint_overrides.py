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

"""Unit tests for `blueprint_panel_overrides` in the blueprints `AriaDataViewer` sends."""

import unittest
from unittest.mock import MagicMock, patch

import rerun.blueprint as rrb
from projectaria_tools.core.calibration import DeviceVersion
from projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter import (
    AriaDataViewer,
    AriaDataViewerConfig,
)


class BlueprintPanelOverridesTest(unittest.TestCase):
    """`blueprint_panel_overrides` in the generated Gen1/Gen2 blueprints."""

    PLOTTER_MODULE = "projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter"

    @staticmethod
    def _make_device_calibration(device_version: DeviceVersion) -> MagicMock:
        calibration = MagicMock()
        calibration.get_device_version.return_value = device_version
        calibration.get_camera_calib.return_value.get_image_size.return_value = [
            100,
            100,
        ]
        return calibration

    def setUp(self) -> None:
        patchers = [
            patch(f"{self.PLOTTER_MODULE}.rr.send_blueprint"),
            patch.object(AriaDataViewer, "plot_device_extrinsics"),
        ]
        self.mock_send_blueprint = patchers[0].start()
        patchers[1].start()
        for p in patchers:
            self.addCleanup(p.stop)

    def _viewer(
        self,
        overrides: list[rrb.TimePanel | rrb.SelectionPanel] | None,
        blueprint_path: str | None = None,
    ) -> AriaDataViewer:
        config = AriaDataViewerConfig()
        config.manage_rerun_sinks = False
        config.blueprint_panel_overrides = overrides
        config.blueprint_path = blueprint_path
        return AriaDataViewer(config=config)

    def _sent_blueprints(self) -> list[rrb.Blueprint]:
        return [c.args[0] for c in self.mock_send_blueprint.call_args_list]

    def test_default_blueprint_collapses_panels(self) -> None:
        for device_version in (DeviceVersion.Gen1, DeviceVersion.Gen2):
            with self.subTest(device_version=device_version):
                self.mock_send_blueprint.reset_mock()
                viewer = self._viewer(overrides=None)

                viewer.set_device_calibration(
                    self._make_device_calibration(device_version)
                )

                (blueprint,) = self._sent_blueprints()
                self.assertTrue(
                    blueprint.collapse_panels, "the default blueprint collapses panels"
                )
                self.assertFalse(
                    hasattr(blueprint, "time_panel"), "no time panel is overridden"
                )

    def test_panel_overrides_survive_calibration_resend(self) -> None:
        time_panel = rrb.TimePanel(state="expanded")
        selection_panel = rrb.SelectionPanel(state="hidden")
        for device_version in (DeviceVersion.Gen1, DeviceVersion.Gen2):
            with self.subTest(device_version=device_version):
                self.mock_send_blueprint.reset_mock()
                viewer = self._viewer(overrides=[time_panel, selection_panel])

                # First calibration builds the blueprint; an optional panel
                # toggle then rebuilds it. Both sends must keep the overrides.
                viewer.set_device_calibration(
                    self._make_device_calibration(device_version)
                )
                viewer.set_optional_panel_visible("gps", True)

                blueprints = self._sent_blueprints()
                self.assertEqual(
                    len(blueprints), 2, "calibration and the panel toggle each send"
                )
                for blueprint in blueprints:
                    self.assertFalse(
                        blueprint.collapse_panels, "overrides expand the panels"
                    )
                    self.assertIs(
                        blueprint.time_panel, time_panel, "the time panel is kept"
                    )
                    self.assertIs(
                        blueprint.selection_panel,
                        selection_panel,
                        "the selection panel is kept",
                    )

    def test_panel_overrides_apply_when_calibration_is_given_at_init(self) -> None:
        time_panel = rrb.TimePanel(state="expanded")
        config = AriaDataViewerConfig()
        config.manage_rerun_sinks = False
        config.blueprint_panel_overrides = [time_panel]

        AriaDataViewer(
            config=config,
            device_calibration=self._make_device_calibration(DeviceVersion.Gen2),
        )

        (blueprint,) = self._sent_blueprints()
        self.assertFalse(blueprint.collapse_panels, "overrides expand the panels")
        self.assertIs(
            blueprint.time_panel, time_panel, "the overridden time panel is sent"
        )

    def test_empty_overrides_expand_default_panels(self) -> None:
        viewer = self._viewer(overrides=[])

        viewer.set_device_calibration(self._make_device_calibration(DeviceVersion.Gen1))

        (blueprint,) = self._sent_blueprints()
        self.assertFalse(
            blueprint.collapse_panels, "empty overrides expand the default panels"
        )
        self.assertFalse(
            hasattr(blueprint, "time_panel"), "no time panel is overridden"
        )

    def test_custom_blueprint_path_wins_over_overrides(self) -> None:
        viewer = self._viewer(
            overrides=[rrb.TimePanel(state="expanded")],
            blueprint_path="/tmp/custom.rbl",
        )

        viewer.set_device_calibration(self._make_device_calibration(DeviceVersion.Gen2))

        self.mock_send_blueprint.assert_not_called()
