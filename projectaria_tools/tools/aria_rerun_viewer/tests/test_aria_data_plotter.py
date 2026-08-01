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

import unittest
from unittest.mock import MagicMock, patch

import numpy as np
from projectaria_tools.core.sensor_data import (
    NeuralBandAccelSample,
    NeuralBandBatch,
    NeuralBandEmgSample,
    NeuralBandGyroSample,
)
from projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter import (
    AriaDataViewer,
    NEURAL_BAND_ACCEL_LABEL,
    NEURAL_BAND_EMG_LABEL,
    NEURAL_BAND_EMG_VOLTS_LABEL,
    NEURAL_BAND_GYRO_LABEL,
)

_PLOTTER_MODULE = "projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter"


def _make_emg_sample(
    timestamp_ns: int, channel_values: list[int]
) -> NeuralBandEmgSample:
    sample = NeuralBandEmgSample()
    sample.capture_timestamp_ns = timestamp_ns
    sample.channel_values = channel_values
    return sample


def _make_accel_sample(timestamp_ns: int, xyz: list[float]) -> NeuralBandAccelSample:
    sample = NeuralBandAccelSample()
    sample.capture_timestamp_ns = timestamp_ns
    sample.accel_msec2 = xyz
    return sample


def _make_gyro_sample(timestamp_ns: int, xyz: list[float]) -> NeuralBandGyroSample:
    sample = NeuralBandGyroSample()
    sample.capture_timestamp_ns = timestamp_ns
    sample.gyro_radsec = xyz
    return sample


def _make_neural_band_batch(
    emg: list[NeuralBandEmgSample] | None = None,
    accel: list[NeuralBandAccelSample] | None = None,
    gyro: list[NeuralBandGyroSample] | None = None,
    emg_channel_count: int = 0,
) -> NeuralBandBatch:
    batch = NeuralBandBatch()
    batch.emg = emg if emg is not None else []
    batch.accel = accel if accel is not None else []
    batch.gyro = gyro if gyro is not None else []
    batch.emg_channel_count = emg_channel_count
    return batch


class PlotNeuralBandBatchTest(unittest.TestCase):
    def setUp(self) -> None:
        # Bypass __init__ — the neural-band methods only read instance fields
        # via getattr defaults, so nothing needs seeding here.
        self.viewer = AriaDataViewer.__new__(AriaDataViewer)

        patchers = [
            patch(f"{_PLOTTER_MODULE}.rr.send_columns"),
            patch(f"{_PLOTTER_MODULE}.rr.TimeColumn"),
            patch(f"{_PLOTTER_MODULE}.rr.Scalars"),
        ]
        self.mock_send_columns = patchers[0].start()
        self.mock_time_column = patchers[1].start()
        self.mock_scalars = patchers[2].start()
        for p in patchers:
            self.addCleanup(p.stop)

    def test_emg_multi_sample_uses_per_channel_entity_paths(self) -> None:
        emg = [
            _make_emg_sample(1_000_000_000, [10, 20]),
            _make_emg_sample(2_000_000_000, [11, 21]),
            _make_emg_sample(3_000_000_000, [12, 22]),
        ]
        batch = _make_neural_band_batch(emg=emg, emg_channel_count=2)

        self.viewer._plot_neural_band_emg(batch)

        entity_paths = [c.args[0] for c in self.mock_send_columns.call_args_list]
        self.assertEqual(
            entity_paths,
            [
                f"{NEURAL_BAND_EMG_LABEL}/channels/channel_0",
                f"{NEURAL_BAND_EMG_LABEL}/channels/channel_1",
            ],
        )
        self.assertEqual(self.mock_time_column.call_count, 2)
        for tc_call in self.mock_time_column.call_args_list:
            self.assertEqual(tc_call.args[0], "device_time")
            np.testing.assert_array_almost_equal(
                tc_call.kwargs["timestamp"], [1.0, 2.0, 3.0]
            )
        # Raw ADC counts are published as-is per channel; no baseline subtraction.
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[0].kwargs["scalars"],
            [10.0, 11.0, 12.0],
        )
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[1].kwargs["scalars"],
            [20.0, 21.0, 22.0],
        )

    def test_accel_multi_sample_uses_per_axis_entity_paths(self) -> None:
        accel = [
            _make_accel_sample(1_000_000_000, [1.0, 2.0, 3.0]),
            _make_accel_sample(2_500_000_000, [4.0, 5.0, 6.0]),
        ]
        batch = _make_neural_band_batch(accel=accel)

        self.viewer._plot_neural_band_accel(batch)

        entity_paths = [c.args[0] for c in self.mock_send_columns.call_args_list]
        self.assertEqual(
            entity_paths,
            [
                f"{NEURAL_BAND_ACCEL_LABEL}/x[m-sec2]",
                f"{NEURAL_BAND_ACCEL_LABEL}/y[m-sec2]",
                f"{NEURAL_BAND_ACCEL_LABEL}/z[m-sec2]",
            ],
        )
        self.assertEqual(self.mock_time_column.call_count, 3)
        for tc_call in self.mock_time_column.call_args_list:
            self.assertEqual(tc_call.args[0], "device_time")
            np.testing.assert_array_almost_equal(
                tc_call.kwargs["timestamp"], [1.0, 2.5]
            )
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[0].kwargs["scalars"], [1.0, 4.0]
        )
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[1].kwargs["scalars"], [2.0, 5.0]
        )
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[2].kwargs["scalars"], [3.0, 6.0]
        )

    def test_gyro_multi_sample_uses_per_axis_entity_paths(self) -> None:
        gyro = [
            _make_gyro_sample(500_000_000, [0.1, 0.2, 0.3]),
            _make_gyro_sample(1_500_000_000, [0.4, 0.5, 0.6]),
        ]
        batch = _make_neural_band_batch(gyro=gyro)

        self.viewer._plot_neural_band_gyro(batch)

        entity_paths = [c.args[0] for c in self.mock_send_columns.call_args_list]
        self.assertEqual(
            entity_paths,
            [
                f"{NEURAL_BAND_GYRO_LABEL}/x[rad-sec]",
                f"{NEURAL_BAND_GYRO_LABEL}/y[rad-sec]",
                f"{NEURAL_BAND_GYRO_LABEL}/z[rad-sec]",
            ],
        )
        self.assertEqual(self.mock_time_column.call_count, 3)
        for tc_call in self.mock_time_column.call_args_list:
            self.assertEqual(tc_call.args[0], "device_time")
            np.testing.assert_array_almost_equal(
                tc_call.kwargs["timestamp"], [0.5, 1.5]
            )
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[0].kwargs["scalars"], [0.1, 0.4]
        )
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[1].kwargs["scalars"], [0.2, 0.5]
        )
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[2].kwargs["scalars"], [0.3, 0.6]
        )

    def test_none_batch_is_skipped_without_crashing(self) -> None:
        self.viewer.plot_neural_band_batch(None)

        self.mock_send_columns.assert_not_called()

    def test_empty_batch_calls_no_send_columns(self) -> None:
        batch = _make_neural_band_batch()

        self.viewer.plot_neural_band_batch(batch)

        self.mock_send_columns.assert_not_called()

    def test_emg_channel_count_zero_is_skipped(self) -> None:
        emg = [_make_emg_sample(1_000_000_000, [1, 2])]
        batch = _make_neural_band_batch(emg=emg, emg_channel_count=0)

        self.viewer._plot_neural_band_emg(batch)

        self.mock_send_columns.assert_not_called()

    def test_partial_batch_only_populated_streams_are_plotted(self) -> None:
        emg = [_make_emg_sample(1_000_000_000, [5])]
        batch = _make_neural_band_batch(emg=emg, emg_channel_count=1)

        self.viewer.plot_neural_band_batch(batch)

        self.assertEqual(self.mock_send_columns.call_count, 1)
        self.assertEqual(
            self.mock_send_columns.call_args_list[0].args[0],
            f"{NEURAL_BAND_EMG_LABEL}/channels/channel_0",
        )

    def test_emg_ragged_samples_are_filtered_out(self) -> None:
        emg = [
            _make_emg_sample(1_000_000_000, [10, 20]),
            _make_emg_sample(2_000_000_000, []),
            _make_emg_sample(3_000_000_000, [12, 22]),
        ]
        batch = _make_neural_band_batch(emg=emg, emg_channel_count=2)

        self.viewer._plot_neural_band_emg(batch)

        for tc_call in self.mock_time_column.call_args_list:
            np.testing.assert_array_almost_equal(
                tc_call.kwargs["timestamp"], [1.0, 3.0]
            )
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[0].kwargs["scalars"],
            [10.0, 12.0],
        )
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[1].kwargs["scalars"],
            [20.0, 22.0],
        )

    def test_emg_all_ragged_produces_no_output(self) -> None:
        emg = [_make_emg_sample(1_000_000_000, [1])]
        batch = _make_neural_band_batch(emg=emg, emg_channel_count=2)

        self.viewer._plot_neural_band_emg(batch)

        self.mock_send_columns.assert_not_called()

    def test_full_mixed_batch_invokes_all_three_helpers(self) -> None:
        emg = [
            _make_emg_sample(1_000_000_000, [1, 2]),
            _make_emg_sample(2_000_000_000, [3, 4]),
        ]
        accel = [_make_accel_sample(1_000_000_000, [1.0, 2.0, 3.0])]
        gyro = [
            _make_gyro_sample(1_000_000_000, [0.1, 0.2, 0.3]),
            _make_gyro_sample(2_000_000_000, [0.4, 0.5, 0.6]),
            _make_gyro_sample(3_000_000_000, [0.7, 0.8, 0.9]),
        ]
        batch = _make_neural_band_batch(
            emg=emg, accel=accel, gyro=gyro, emg_channel_count=2
        )

        self.viewer.plot_neural_band_batch(batch)

        # No calibration set → volts helper silently skips.
        entity_paths = [c.args[0] for c in self.mock_send_columns.call_args_list]
        self.assertEqual(len(entity_paths), 8)
        emg_paths = [
            p for p in entity_paths if p.startswith(NEURAL_BAND_EMG_LABEL + "/")
        ]
        volts_paths = [
            p for p in entity_paths if p.startswith(NEURAL_BAND_EMG_VOLTS_LABEL)
        ]
        accel_paths = [p for p in entity_paths if p.startswith(NEURAL_BAND_ACCEL_LABEL)]
        gyro_paths = [p for p in entity_paths if p.startswith(NEURAL_BAND_GYRO_LABEL)]
        self.assertEqual(len(emg_paths), 2)
        self.assertEqual(len(volts_paths), 0)
        self.assertEqual(len(accel_paths), 3)
        self.assertEqual(len(gyro_paths), 3)

    def test_volts_no_calibration_skips_send(self) -> None:
        emg = [
            _make_emg_sample(1_000_000_000, [10, 20]),
            _make_emg_sample(2_000_000_000, [11, 21]),
        ]
        batch = _make_neural_band_batch(emg=emg, emg_channel_count=2)

        self.viewer._plot_neural_band_emg_volts(batch)

        self.mock_send_columns.assert_not_called()

    def test_volts_uses_calibration_and_per_channel_paths(self) -> None:
        emg = [
            _make_emg_sample(1_000_000_000, [10, 20]),
            _make_emg_sample(2_000_000_000, [11, 21]),
        ]
        batch = _make_neural_band_batch(emg=emg, emg_channel_count=2)
        fake_calib = MagicMock()
        # Fake calibration: multiply every ADC count by a fixed scale.
        fake_calib.adc_to_volts.side_effect = lambda cv: [float(v) * 0.001 for v in cv]
        self.viewer.set_neural_band_emg_calibration(fake_calib)

        with patch(f"{_PLOTTER_MODULE}.rr.log"):
            self.viewer._plot_neural_band_emg_volts(batch)

        entity_paths = [c.args[0] for c in self.mock_send_columns.call_args_list]
        self.assertEqual(
            entity_paths,
            [
                f"{NEURAL_BAND_EMG_VOLTS_LABEL}/channels/channel_0",
                f"{NEURAL_BAND_EMG_VOLTS_LABEL}/channels/channel_1",
            ],
        )
        # Single vectorized call for the whole batch: 2 samples * 2 channels
        # arrive flattened.
        self.assertEqual(fake_calib.adc_to_volts.call_count, 1)
        self.assertEqual(fake_calib.adc_to_volts.call_args.args[0], [10, 20, 11, 21])
        # Reshape back into per-channel columns.
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[0].kwargs["scalars"],
            [0.010, 0.011],  # channel 0
        )
        np.testing.assert_array_almost_equal(
            self.mock_scalars.columns.call_args_list[1].kwargs["scalars"],
            [0.020, 0.021],  # channel 1
        )

    def test_volts_channel_count_zero_is_skipped(self) -> None:
        emg = [_make_emg_sample(1_000_000_000, [10, 20])]
        batch = _make_neural_band_batch(emg=emg, emg_channel_count=0)
        self.viewer.set_neural_band_emg_calibration(MagicMock())

        self.viewer._plot_neural_band_emg_volts(batch)

        self.mock_send_columns.assert_not_called()

    def test_volts_empty_emg_is_skipped(self) -> None:
        batch = _make_neural_band_batch(emg=[], emg_channel_count=2)
        self.viewer.set_neural_band_emg_calibration(MagicMock())

        self.viewer._plot_neural_band_emg_volts(batch)

        self.mock_send_columns.assert_not_called()

    def test_volts_all_ragged_produces_no_output(self) -> None:
        emg = [
            _make_emg_sample(1_000_000_000, [1])
        ]  # channel_values shorter than count
        batch = _make_neural_band_batch(emg=emg, emg_channel_count=2)
        fake_calib = MagicMock()
        self.viewer.set_neural_band_emg_calibration(fake_calib)

        self.viewer._plot_neural_band_emg_volts(batch)

        self.mock_send_columns.assert_not_called()
        fake_calib.adc_to_volts.assert_not_called()

    def test_set_neural_band_emg_calibration_replaces_and_clears(self) -> None:
        fake_calib = MagicMock()
        self.viewer.set_neural_band_emg_calibration(fake_calib)
        self.assertIs(self.viewer._neural_band_emg_calibration, fake_calib)

        self.viewer.set_neural_band_emg_calibration(None)
        self.assertIsNone(self.viewer._neural_band_emg_calibration)
