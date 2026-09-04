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
    TrackingQuality,
    VioStatus,
)
from projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter import (
    AriaDataViewer,
    AriaDataViewerConfig,
    FadingTrajectory,
    NEURAL_BAND_ACCEL_LABEL,
    NEURAL_BAND_EMG_LABEL,
    NEURAL_BAND_EMG_VOLTS_LABEL,
    NEURAL_BAND_GYRO_LABEL,
    TRAJECTORY_FADE_MIN_ALPHA,
)

_PLOTTER_MODULE = "projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter"


def _make_emg_sample(
    timestamp_ns: int | None, channel_values: list[int]
) -> NeuralBandEmgSample:
    sample = NeuralBandEmgSample()
    sample.device_timestamp_ns = timestamp_ns
    sample.channel_values = channel_values
    return sample


def _make_accel_sample(
    timestamp_ns: int | None, xyz: list[float]
) -> NeuralBandAccelSample:
    sample = NeuralBandAccelSample()
    sample.device_timestamp_ns = timestamp_ns
    sample.accel_msec2 = xyz
    return sample


def _make_gyro_sample(
    timestamp_ns: int | None, xyz: list[float]
) -> NeuralBandGyroSample:
    sample = NeuralBandGyroSample()
    sample.device_timestamp_ns = timestamp_ns
    sample.gyro_radsec = xyz
    return sample


def _make_neural_band_batch(
    emg: list[NeuralBandEmgSample] | None = None,
    accel: list[NeuralBandAccelSample] | None = None,
    gyro: list[NeuralBandGyroSample] | None = None,
    emg_channel_count: int = 0,
    arrival_timestamp_ns: int = 9_000_000_000,
) -> NeuralBandBatch:
    batch = NeuralBandBatch()
    batch.arrival_timestamp_ns = arrival_timestamp_ns
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
            patch(f"{_PLOTTER_MODULE}.rr.log"),
        ]
        self.mock_send_columns = patchers[0].start()
        self.mock_time_column = patchers[1].start()
        self.mock_scalars = patchers[2].start()
        self.mock_log = patchers[3].start()
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

    def test_emg_without_device_timestamps_is_skipped(self) -> None:
        emg = [_make_emg_sample(None, [10, 20]), _make_emg_sample(None, [11, 21])]
        batch = _make_neural_band_batch(emg=emg, emg_channel_count=2)

        self.viewer._plot_neural_band_emg(batch)

        self.mock_send_columns.assert_not_called()
        # Styling is static and would leave empty named series behind.
        self.mock_log.assert_not_called()

    def test_accel_without_device_timestamps_is_skipped(self) -> None:
        accel = [_make_accel_sample(None, [1.0, 2.0, 3.0])]
        batch = _make_neural_band_batch(accel=accel)

        self.viewer._plot_neural_band_accel(batch)

        self.mock_send_columns.assert_not_called()

    def test_gyro_without_device_timestamps_is_skipped(self) -> None:
        gyro = [_make_gyro_sample(None, [0.1, 0.2, 0.3])]
        batch = _make_neural_band_batch(gyro=gyro)

        self.viewer._plot_neural_band_gyro(batch)

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


class _FakePose:
    """The bare SE3 surface `plot_vio_*` touches: `@` then `translation()`."""

    def __init__(self, position) -> None:
        self._position: np.ndarray = np.asarray([position], dtype=np.float64)

    def __matmul__(self, other: "_FakePose") -> "_FakePose":
        return self

    def translation(self) -> np.ndarray:
        return self._position


def _make_vio_data(timestamp_ns: int, position: list[float]) -> MagicMock:
    vio_data = MagicMock()
    vio_data.status = VioStatus.VALID
    vio_data.pose_quality = TrackingQuality.GOOD
    vio_data.capture_timestamp_ns = timestamp_ns
    vio_data.transform_odometry_bodyimu = _FakePose(position)
    vio_data.transform_bodyimu_device = _FakePose(position)
    vio_data.gravity_in_odometry = np.array([0.0, 0.0, -9.81])
    return vio_data


def _make_vio_high_freq_data(timestamp_sec: float, position: list[float]) -> MagicMock:
    data = MagicMock()
    data.tracking_timestamp.total_seconds.return_value = timestamp_sec
    data.transform_odometry_device = _FakePose(position)
    return data


class FadingTrajectoryTest(unittest.TestCase):
    """The trail's own bookkeeping, with no Rerun in the picture."""

    _COLOR = [173, 216, 255]

    def _trail(self, window_sec: float, count: int, step_sec: float = 10.0):
        trail = FadingTrajectory(window_sec)
        for i in range(count):
            trail.append(i * step_sec, [float(i), 0.0, 0.0])
        return trail

    def test_an_unbounded_window_keeps_everything_in_one_opaque_strip(self) -> None:
        trail = self._trail(0.0, 50)
        strips, colors = trail.strips_and_colors(self._COLOR)
        self.assertEqual(len(trail), 50)
        self.assertEqual(len(strips), 1)
        self.assertEqual(len(strips[0]), 50)
        np.testing.assert_array_equal(colors, [self._COLOR + [255]])

    def test_a_negative_window_is_read_as_unbounded(self) -> None:
        # Not as "drop everything": a nonsense window must not blank the view.
        trail = self._trail(-1.0, 50)
        self.assertEqual(len(trail), 50)
        self.assertEqual(len(trail.strips_and_colors(self._COLOR)[0]), 1)

    def test_positions_older_than_the_window_are_dropped(self) -> None:
        # 20 minutes of track at 0.1 Hz against a 10 minute window.
        trail = self._trail(600.0, 121)
        self.assertEqual(len(trail), 61)
        strips, _ = trail.strips_and_colors(self._COLOR)
        oldest_kept = strips[0][0]
        # x == index, so the surviving head is the sample exactly one window old.
        self.assertEqual(float(oldest_kept[0]), 60.0)

    def test_two_positions_always_survive_so_the_line_never_vanishes(self) -> None:
        # A window shorter than the sample interval would otherwise trim the
        # trail down to a single invisible vertex.
        trail = self._trail(1.0, 20, step_sec=10.0)
        self.assertEqual(len(trail), 2)
        strips, colors = trail.strips_and_colors(self._COLOR)
        self.assertEqual(len(strips), 1)
        self.assertEqual(len(colors), 1)

    def test_a_lone_position_draws_nothing(self) -> None:
        strips, colors = self._trail(600.0, 1).strips_and_colors(self._COLOR)
        self.assertEqual(list(strips), [])
        self.assertEqual(len(colors), 0)

    def test_an_empty_trail_draws_nothing(self) -> None:
        strips, colors = FadingTrajectory(600.0).strips_and_colors(self._COLOR)
        self.assertEqual(list(strips), [])
        self.assertEqual(len(colors), 0)

    def test_alpha_ramps_from_faint_at_the_tail_to_opaque_at_the_head(self) -> None:
        strips, colors = self._trail(600.0, 61).strips_and_colors(self._COLOR)
        self.assertEqual(len(colors), len(strips))
        alphas = [color[3] for color in colors]
        self.assertEqual(alphas, sorted(alphas))
        self.assertLess(alphas[0], TRAJECTORY_FADE_MIN_ALPHA + 10)
        self.assertGreater(alphas[-1], 240)
        np.testing.assert_array_equal(colors[:, :3], [self._COLOR] * len(colors))

    def test_consecutive_strips_share_a_vertex_so_the_trail_has_no_gaps(self) -> None:
        strips, _ = self._trail(600.0, 61).strips_and_colors(self._COLOR)
        self.assertGreater(len(strips), 1)
        for previous, following in zip(strips[:-1], strips[1:]):
            np.testing.assert_array_equal(previous[-1], following[0])

    def test_a_young_trail_is_not_mostly_thrown_away(self) -> None:
        # The chunks have to be equal length to go over as one array, so the
        # remainder is dropped from the far end. Sized wrong that remainder is
        # a fifth of a trail that has only just started.
        for count in range(2, 200):
            with self.subTest(count=count):
                # 0.1 s apart, so the window itself never trims and the only
                # thing that can shorten the trail is the chunk remainder.
                trail = self._trail(600.0, count, step_sec=0.1)
                strips, _ = trail.strips_and_colors(self._COLOR)
                drawn = strips.shape[0] * (strips.shape[1] - 1) + 1
                self.assertGreaterEqual(drawn, count - max(1, (count - 1) // 24))
                # Whatever is dropped comes off the old end; the trail must
                # still reach the newest position.
                self.assertEqual(float(strips[-1][-1][0]), float(count - 1))

    def test_a_trail_shorter_than_the_segment_count_yields_drawable_strips(
        self,
    ) -> None:
        # `linspace` repeats indices on a short trail; collapsing them is what
        # keeps single-vertex (invisible) strips out of the output.
        strips, colors = self._trail(600.0, 3).strips_and_colors(self._COLOR)
        self.assertEqual(len(strips), len(colors))
        for strip in strips:
            self.assertGreaterEqual(len(strip), 2)

    def test_a_windowed_trail_settles_into_a_capacity_it_never_exceeds(self) -> None:
        # The point of the whole exercise: 100 minutes of 10 Hz VIO must cost
        # the same as the first ten, or an 8 hour session degrades as it runs.
        trail = FadingTrajectory(600.0)
        for i in range(60_000):
            trail.append(i * 0.1, [float(i), 0.0, 0.0])
        self.assertEqual(len(trail), 6001)
        self.assertLessEqual(trail._positions.shape[0], 4 * trail._INITIAL_CAPACITY)

    def test_compaction_preserves_the_trail_it_moves(self) -> None:
        # `_make_room` copies live samples to the front of the buffer; an
        # off-by-one there would silently shift or duplicate the track.
        trail = FadingTrajectory(600.0)
        for i in range(20_000):
            trail.append(i * 0.1, [float(i), 2.0 * i, 0.0])
        strips, _ = trail.strips_and_colors(self._COLOR)
        track = np.concatenate([strip[:-1] for strip in strips] + [strips[-1][-1:]])
        self.assertEqual(len(track), len(trail))
        expected_x = np.arange(20_000 - 6001, 20_000, dtype=np.float32)
        np.testing.assert_array_equal(track[:, 0], expected_x)
        np.testing.assert_array_equal(track[:, 1], 2.0 * expected_x)

    def test_an_unbounded_trail_is_still_free_to_grow(self) -> None:
        trail = self._trail(0.0, 10_000, step_sec=0.1)
        self.assertEqual(len(trail), 10_000)
        strips, _ = trail.strips_and_colors(self._COLOR)
        self.assertEqual(len(strips[0]), 10_000)

    def test_time_running_backwards_stays_within_the_alpha_range(self) -> None:
        # A reattached stream can restate its clock; age is measured against
        # the newest sample, so this must clamp rather than overflow.
        trail = FadingTrajectory(600.0)
        trail.append(100.0, [0.0, 0.0, 0.0])
        trail.append(50.0, [1.0, 0.0, 0.0])
        _, colors = trail.strips_and_colors(self._COLOR)
        for color in colors:
            self.assertGreaterEqual(color[3], TRAJECTORY_FADE_MIN_ALPHA)
            self.assertLessEqual(color[3], 255)


class VioTrajectoryWindowTest(unittest.TestCase):
    """`plot_vio_*` honouring `vio_trajectory_window_sec`."""

    def setUp(self) -> None:
        patchers = [
            patch(f"{_PLOTTER_MODULE}.rr"),
            patch(f"{_PLOTTER_MODULE}.ToTransform3D"),
            patch(f"{_PLOTTER_MODULE}.AriaGlassesOutline"),
        ]
        self.mock_rr = patchers[0].start()
        for p in patchers[1:]:
            p.start()
        for p in patchers:
            self.addCleanup(p.stop)

    def _viewer(self, window_sec: float) -> AriaDataViewer:
        viewer = AriaDataViewer.__new__(AriaDataViewer)
        viewer.config = AriaDataViewerConfig()
        viewer.config.vio_trajectory_window_sec = window_sec
        viewer.vio_trajectory = FadingTrajectory(window_sec)
        viewer.vio_high_freq_trajectory = FadingTrajectory(window_sec)
        viewer._low_rate_vio_trajectory_cleared = False
        viewer.device_calibration = MagicMock()
        viewer.sensor_labels = MagicMock(
            vio_label="vio", vio_high_freq_label="vio_high_frequency"
        )
        return viewer

    def _last_strips(self):
        return self.mock_rr.LineStrips3D.call_args_list[-1].args[0]

    def test_vio_drops_track_older_than_the_window(self) -> None:
        viewer = self._viewer(600.0)
        for i in range(121):
            viewer.plot_vio_data(_make_vio_data(i * 10_000_000_000, [float(i), 0, 0]))
        self.assertEqual(len(viewer.vio_trajectory), 61)
        self.assertEqual(float(self._last_strips()[0][0][0]), 60.0)

    def test_vio_high_freq_drops_track_older_than_the_window(self) -> None:
        viewer = self._viewer(600.0)
        for i in range(121):
            viewer.plot_vio_high_freq_data(
                _make_vio_high_freq_data(i * 10.0, [float(i), 0, 0])
            )
        self.assertEqual(len(viewer.vio_high_freq_trajectory), 61)
        self.assertEqual(float(self._last_strips()[0][0][0]), 60.0)

    def test_a_zero_window_keeps_the_whole_session(self) -> None:
        # The recording viewer's default: a bounded file wants its whole track.
        viewer = self._viewer(0.0)
        for i in range(121):
            viewer.plot_vio_data(_make_vio_data(i * 10_000_000_000, [float(i), 0, 0]))
        self.assertEqual(len(viewer.vio_trajectory), 121)

    def test_an_invalid_pose_is_not_added_to_the_trail(self) -> None:
        viewer = self._viewer(600.0)
        vio_data = _make_vio_data(0, [0.0, 0.0, 0.0])
        vio_data.status = VioStatus.INVALID
        viewer.plot_vio_data(vio_data)
        self.assertEqual(len(viewer.vio_trajectory), 0)

    def test_the_default_config_keeps_the_whole_session(self) -> None:
        self.assertEqual(AriaDataViewerConfig().vio_trajectory_window_sec, 0.0)


class OnlyOneVioTrailIsDrawnTest(unittest.TestCase):
    """`vio` and `vio_high_frequency` are the same path at different rates.

    Drawn together they are two same-radius tubes in the same place, which
    z-fight into a line that alternates between their two colours.
    """

    def setUp(self) -> None:
        patchers = [
            patch(f"{_PLOTTER_MODULE}.rr"),
            patch(f"{_PLOTTER_MODULE}.ToTransform3D"),
            patch(f"{_PLOTTER_MODULE}.AriaGlassesOutline"),
        ]
        self.mock_rr = patchers[0].start()
        for p in patchers[1:]:
            p.start()
        for p in patchers:
            self.addCleanup(p.stop)

        self.viewer = AriaDataViewer.__new__(AriaDataViewer)
        self.viewer.config = AriaDataViewerConfig()
        self.viewer.vio_trajectory = FadingTrajectory(600.0)
        self.viewer.vio_high_freq_trajectory = FadingTrajectory(600.0)
        self.viewer._low_rate_vio_trajectory_cleared = False
        self.viewer.device_calibration = MagicMock()
        self.viewer.sensor_labels = MagicMock(
            vio_label="vio", vio_high_freq_label="vio_high_frequency"
        )

    def _payloads(self, entity: str) -> list:
        """What was logged to `entity`, in order. `rr` is a mock, so each entry
        is the archetype mock's return value and is identity-comparable."""
        return [
            call.args[1]
            for call in self.mock_rr.log.call_args_list
            if len(call.args) > 1 and call.args[0] == entity
        ]

    def test_high_frequency_suppresses_the_low_rate_trail(self) -> None:
        for i in range(5):
            self.viewer.plot_vio_high_freq_data(
                _make_vio_high_freq_data(i * 0.1, [float(i), 0, 0])
            )
            self.viewer.plot_vio_data(_make_vio_data(i * 100_000_000, [float(i), 0, 0]))
        self.assertEqual(len(self.viewer.vio_trajectory), 0)
        # `world/vio` is touched once, to clear it -- never with geometry.
        self.assertEqual(self._payloads("world/vio"), [self.mock_rr.Clear.return_value])
        self.assertEqual(
            self._payloads("world/vio_high_frequency"),
            [self.mock_rr.LineStrips3D.return_value] * 5,
        )

    def test_the_low_rate_trail_is_the_fallback_when_nothing_else_arrives(
        self,
    ) -> None:
        # A recording, or a `--stream-labels` selection, carrying only `vio`
        # must still show a trajectory.
        for i in range(5):
            self.viewer.plot_vio_data(_make_vio_data(i * 100_000_000, [float(i), 0, 0]))
        self.assertEqual(len(self.viewer.vio_trajectory), 5)
        self.assertEqual(
            self._payloads("world/vio"), [self.mock_rr.LineStrips3D.return_value] * 5
        )
        self.mock_rr.Clear.assert_not_called()

    def test_a_late_first_high_frequency_sample_clears_the_stub_once(self) -> None:
        # The low-rate stream can win the race by a few samples; whatever it
        # drew has to go, and the clear must not then fire on every sample.
        for i in range(3):
            self.viewer.plot_vio_data(_make_vio_data(i * 100_000_000, [float(i), 0, 0]))
        for i in range(3, 8):
            self.viewer.plot_vio_high_freq_data(
                _make_vio_high_freq_data(i * 0.1, [float(i), 0, 0])
            )
            self.viewer.plot_vio_data(_make_vio_data(i * 100_000_000, [float(i), 0, 0]))
        self.assertEqual(self.mock_rr.Clear.call_count, 1)
        # The stub stopped growing the moment the high-frequency trail started.
        self.assertEqual(len(self.viewer.vio_trajectory), 3)
