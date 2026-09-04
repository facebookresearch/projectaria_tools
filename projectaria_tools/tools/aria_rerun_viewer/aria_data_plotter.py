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

import colorsys
import os
from dataclasses import dataclass
from functools import partial
from typing import Final, List, Optional, Sequence, Tuple, Union

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from projectaria_tools.core.calibration import DeviceCalibration, DeviceVersion
from projectaria_tools.core.mps import get_unit_vector_from_yaw_pitch
from projectaria_tools.core.sensor_data import TrackingQuality, VioStatus
from projectaria_tools.utils.rerun_helpers import (
    AriaGlassesOutline,
    create_hand_skeleton_from_landmarks,
    ToBox3D,
    ToTransform3D,
)


def warn_once(func, message):
    """
    Issue a warning only once per function/method.
    Works for plain functions and bound methods.
    """
    target = getattr(func, "__func__", func)  # bound method -> underlying function
    if not getattr(target, "_warned", False):
        print(f"[aria_data_plotter][WARNING]: {message}")
        setattr(target, "_warned", True)


def _spawn_rerun_viewer(memory_limit: str) -> None:
    # RERUN_PATH is set by the :aria_rerun_viewer oxx_command_alias to the
    # buck-built rerun-cli; without it rerun-sdk searches PATH and fails.
    rr.spawn(memory_limit=memory_limit, executable_path=os.environ.get("RERUN_PATH"))


NEURAL_BAND_EMG_LABEL: Final = "neural-band-emg"
NEURAL_BAND_EMG_VOLTS_LABEL: Final = "neural-band-emg-volts"
NEURAL_BAND_ACCEL_LABEL: Final = "neural-band-accel"
NEURAL_BAND_GYRO_LABEL: Final = "neural-band-gyro"


# A trail is emitted as this many chunks so its colour can ramp along its
# length: Rerun colours a line strip as a whole, never per vertex, so a
# gradient has to be built out of pieces. Set for legibility alone -- the
# chunks go over as one strided array (see `strips_and_colors`), so the cost of
# a log does not move with this number.
TRAJECTORY_FADE_SEGMENT_COUNT: Final = 24

# Alpha the trail has reached once it is a full window old -- the instant
# before it is dropped. Non-zero so the far end still reads as "where I came
# from", low enough that its removal is not a visible pop.
TRAJECTORY_FADE_MIN_ALPHA: Final = 25


class FadingTrajectory:
    """A trail of positions that forgets its own tail.

    A live session runs for hours, so a trajectory that is only ever appended
    to grows without bound -- in memory, and in the cost of the whole-strip
    re-log that every new sample pays. Positions older than ``window_sec`` are
    dropped, and what survives is handed back as a run of strips whose alpha
    ramps from ``TRAJECTORY_FADE_MIN_ALPHA`` at the far end to opaque at the
    newest sample, so the tail dissolves instead of being cut.

    Age is measured against the newest sample rather than wall-clock: the same
    plotter replays recorded VRS, where "now" is wherever playback has reached.
    """

    # Samples live in a flat buffer that doubles when it fills. 4096 holds ten
    # minutes of 10 Hz VIO outright, so a steady-state session never
    # reallocates and never rebuilds an array per sample.
    _INITIAL_CAPACITY: Final = 4096

    def __init__(
        self,
        window_sec: float = 0.0,
        segment_count: int = TRAJECTORY_FADE_SEGMENT_COUNT,
    ) -> None:
        # <= 0 keeps everything, which is what replaying a recording of bounded
        # length wants.
        self.window_sec: float = max(0.0, float(window_sec))
        self._segment_count: int = max(1, int(segment_count))
        self._timestamps_sec: np.ndarray = np.empty(
            self._INITIAL_CAPACITY, dtype=np.float64
        )
        # float32 because that is what Rerun stores positions as; keeping the
        # buffer in the wire dtype saves a full-trail conversion per log.
        self._positions: np.ndarray = np.empty(
            (self._INITIAL_CAPACITY, 3), dtype=np.float32
        )
        # Live samples are `[_start, _end)`. Expiry only advances `_start`, so
        # dropping the tail costs nothing; the dead prefix is reclaimed in bulk
        # by `_make_room`.
        self._start: int = 0
        self._end: int = 0

    def __len__(self) -> int:
        return self._end - self._start

    def append(self, timestamp_sec: float, position: Sequence[float]) -> None:
        """Add one position, then drop whatever has aged out of the window."""
        if self._end == len(self._timestamps_sec):
            self._make_room()
        self._timestamps_sec[self._end] = timestamp_sec
        self._positions[self._end] = position
        self._end += 1
        if self.window_sec <= 0.0:
            return
        cutoff_sec = self._timestamps_sec[self._end - 1] - self.window_sec
        # Two positions are the least that still draws a line, so the trail
        # never shrinks to a single invisible vertex.
        while (
            self._end - self._start > 2
            and self._timestamps_sec[self._start] < cutoff_sec
        ):
            self._start += 1

    def _make_room(self):
        """Slide the live samples to the front, growing only if they fill it.

        Amortises the cost of expiry: without this, dropping the oldest sample
        would memmove the whole trail on every append. A windowed trail settles
        into a capacity it never exceeds and stops reallocating for good.
        """
        live = self._end - self._start
        capacity = len(self._timestamps_sec)
        # `.copy()` because compaction in place reads and writes one buffer.
        timestamps_sec = self._timestamps_sec[self._start : self._end].copy()
        positions = self._positions[self._start : self._end].copy()
        if live * 2 > capacity:
            capacity *= 2
            self._timestamps_sec = np.empty(capacity, dtype=np.float64)
            self._positions = np.empty((capacity, 3), dtype=np.float32)
        self._timestamps_sec[:live] = timestamps_sec
        self._positions[:live] = positions
        self._start = 0
        self._end = live

    def strips_and_colors(
        self, rgb: Sequence[int]
    ) -> Tuple[Union[np.ndarray, List[np.ndarray]], np.ndarray]:
        """The trail as (strips, per-strip RGBA), ready for `rr.LineStrips3D`.

        Nothing here copies the trail: the strips are strided views onto the
        live buffer. They stay valid until the next ``append`` compacts, which
        is long after `rr.log` has taken its own copy -- but it does mean they
        are for logging now, not for keeping.
        """
        count = self._end - self._start
        if count < 2:
            return [], np.empty((0, 4), dtype=np.uint8)
        points = self._positions[self._start : self._end]
        base_color = np.asarray(rgb, dtype=np.uint8)[:3]
        if self.window_sec <= 0.0:
            return [points], np.array([[*base_color, 255]], dtype=np.uint8)

        # Equal-length chunks, so the fade goes to Rerun as ONE
        # (segments, chunk + 1, 3) array rather than a list of that many
        # strips. Rerun charges ~5 us per strip for the list form -- 130 us a
        # sample at 24 strips -- while the array form costs the same 30 us as
        # one undivided strip. That is what makes the fade free rather than a
        # 4x tax on every VIO sample.
        # Size the chunk first, then fit whole chunks into the trail. Sizing it
        # the other way round -- fix the count, divide -- strands a remainder of
        # up to `segment_count` positions, which on a trail that has only just
        # started is most of it.
        span = count - 1
        chunk = max(1, span // self._segment_count)
        segments = span // chunk
        # The chunks tile backwards from the newest position, so the trail
        # always reaches the current pose and the remainder -- always under one
        # chunk, of the oldest and faintest track -- falls off the far end.
        offset = span - segments * chunk
        head = points[offset:]
        # Consecutive chunks overlap by their shared boundary vertex -- without
        # it the trail is drawn with a gap at every colour step. Overlapping is
        # also why this is a strided view and not a reshape.
        strips = np.lib.stride_tricks.as_strided(
            head,
            shape=(segments, chunk + 1, 3),
            strides=(chunk * head.strides[0], head.strides[0], head.strides[1]),
        )

        # Same tiling over the timestamps: one edge per chunk boundary.
        edges_sec = self._timestamps_sec[self._start + offset : self._end : chunk]
        # Alpha is taken at each chunk's midpoint, so a chunk is not coloured by
        # whichever of its two ends happens to be sampled.
        midpoints_sec = 0.5 * (edges_sec[:-1] + edges_sec[1:])
        freshness = np.clip(
            1.0 - (edges_sec[-1] - midpoints_sec) / self.window_sec, 0.0, 1.0
        )
        # Built as one array rather than a list comprehension: per-chunk Python
        # rounding costs more than everything else in this method put together.
        colors = np.empty((segments, 4), dtype=np.uint8)
        colors[:, :3] = base_color
        colors[:, 3] = np.rint(
            TRAJECTORY_FADE_MIN_ALPHA + (255 - TRAJECTORY_FADE_MIN_ALPHA) * freshness
        )
        return strips, colors


@dataclass
class SensorLabels:
    """
    Container for all sensor labels based on device version.
    """

    # Camera labels
    rgb_label: str
    slam_labels: List[str]
    eye_tracking_camera_labels: List[str]

    # Non-visual sensor labels
    imu_labels: List[str]
    magnetometer_label: str
    barometer_labels: List[str]
    microphone_label: str
    gps_label: str

    # Contact mic label (Gen2 only)
    contact_microphone_label: Optional[str] = None

    # Machine perception data labels (Gen2 only)
    eye_gaze_label: Optional[str] = None
    hand_tracking_label: Optional[str] = None
    vio_label: Optional[str] = None
    vio_high_freq_label: Optional[str] = None

    @classmethod
    def from_device_version(cls, device_version: DeviceVersion):
        """Create SensorLabels instance based on device version."""
        common_labels = {
            "rgb_label": "camera-rgb",
            "imu_labels": ["imu-left", "imu-right"],
            "magnetometer_label": "mag0",
            "barometer_labels": ["baro_P", "baro_T"],
            "microphone_label": "mic",
            "gps_label": "gps",
        }

        # Gen2-only machine perception data labels
        gen2_mp_labels = {
            "eye_gaze_label": "eye_gaze",
            "hand_tracking_label": "hand_tracking",
            "vio_label": "vio",
            "vio_high_freq_label": "vio_high_frequency",
        }

        if device_version == DeviceVersion.Gen1:
            return cls(
                slam_labels=["camera-slam-left", "camera-slam-right"],
                eye_tracking_camera_labels=[
                    "camera-et"
                ],  # gen1 only has one eye tracking camera streaming, containing view from both eyes
                **common_labels,
            )
        elif device_version == DeviceVersion.Gen2:
            return cls(
                slam_labels=[
                    "slam-front-left",
                    "slam-front-right",
                    "slam-side-left",
                    "slam-side-right",
                ],
                eye_tracking_camera_labels=["camera-et-left", "camera-et-right"],
                contact_microphone_label="contact_mic",
                **common_labels,
                **gen2_mp_labels,
            )
        else:
            raise ValueError(f"Unsupported device version: {device_version}")

    @property
    def rgb_and_slam_labels(self) -> List[str]:
        """Get combined RGB and SLAM camera labels."""
        assert self.rgb_label and self.slam_labels, (
            "RGB and SLAM labels must not be empty"
        )
        return [self.rgb_label] + self.slam_labels


@dataclass
class AriaDataViewerConfig:
    # ============================================
    # Performance and Processing Settings
    # ============================================

    # For VIO high frequency, rendering at the original freq (800Hz) causes GUI to be slow.
    # Hence we set the target frequency to 80Hz by default (down-sample by 10)
    # TODO: Set by target sample rate rather than sub-sampling factor
    # TODO: consider how to speed this up without sub-sampling.

    vio_high_freq_subsample_rate = 80

    # Audio subsample rate
    audio_subsample_rate = 40

    # JPEG compression quality for image logging.
    # Set this value between 1 (lowest quality, smallest file size) and 100 (highest quality, largest file size).
    # Used by rr.Image(...).compress(jpeg_quality=jpeg_quality) to control the trade-off between image quality and storage/bandwidth.
    jpeg_quality = 50

    enable_gps = False

    # Enable only when a Neural Band stream is present; else the panel wastes vertical space.
    enable_neural_band_batch = False

    enable_crop_visualization = False

    # rerun memory limit (default parameter is 75% of available memory)
    rerun_memory_limit = "75%"

    # Path to a custom Rerun blueprint (.rbl) file. When set, the viewer will
    # launch Rerun with this blueprint and skip the auto-generated layout.
    blueprint_path: Optional[str] = None

    # How much VIO trajectory history stays on screen, in seconds. Older track
    # is dropped, and what is left fades out towards that age so the tail
    # dissolves rather than being cut. 0 keeps the whole session, which is
    # right for replaying a recording of bounded length; a live session has no
    # bound, so the streaming viewer sets a window instead.
    vio_trajectory_window_sec: float = 0.0

    # Whether to show latency plot in the blueprint (only relevant for streaming)
    show_latency: bool = False

    # Whether to connect to an existing Rerun viewer via gRPC instead of spawning
    # a new viewer process. Used for frozen Electron builds where the viewer is
    # already started by the parent process via start_frozen_rerun().
    connect_to_existing_viewer: bool = False


class AriaDataViewer:
    """
    A visualizer class for Aria data based on ReRun
    """

    # ============================================
    # 2D Visualization Settings, not expected to change
    # plot_size: Size in pixels for rendering 2D elements (lines and points) on camera images
    #           - For RGB cameras: Used directly as specified
    #           - For SLAM cameras: Automatically scaled by slam_to_rgb_plot_scale to maintain proportional appearance
    # ============================================
    PLOT_COLORS_AND_SIZES_2D: Final = {
        "eye_gaze_point": {"color": [255, 64, 255], "plot_size": 18},
        "fixation_gaze_point": {"color": [0, 255, 128], "plot_size": 14},
        "left_hand_markers": {"color": [255, 64, 0], "plot_size": 10},
        "left_hand_lines": {"color": [0, 255, 0], "plot_size": 2},
        "right_hand_markers": {"color": [255, 255, 0], "plot_size": 10},
        "right_hand_lines": {"color": [0, 255, 0], "plot_size": 2},
        "overlay_text": {"color": [255, 255, 0], "plot_size": 1},
        "gps_app": {"color": [255, 0, 0], "plot_size": 4.0},
        "gps_sensor": {"color": [0, 255, 0], "plot_size": 4.0},
    }

    # ============================================
    # 3D Visualization Settings, not expected to change
    # ============================================
    PLOT_COLORS_AND_SIZES_3D: Final = {
        "glass_outline": {
            "color": [200, 200, 200],
            "glasses_outline_radius": 5e-4,
        },
        "vio_high_freq": {
            "color": [248, 254, 180],
            "trajectory_radius": 1.5e-3,
        },
        "vio": {
            "color": [173, 216, 255],
            "trajectory_radius": 1.5e-3,
            "device_axis_length": 0.05,
        },
        "vio_gravity": {
            "color": [101, 67, 33],
        },
        "handtracking": {
            "left_hand_markers_color": [255, 64, 0],
            "left_hand_lines_color": [0, 255, 0],
            "right_hand_markers_color": [255, 255, 0],
            "right_hand_lines_color": [0, 255, 0],
            "landmarks_radius_3d": 5e-3,
            "skeleton_radius_3d": 3e-3,
        },
        "eyegaze": {
            "combined_gaze_origin_size": [2e-2, 2e-2, 2e-2],
            "spatial_gaze_point_size": [1e-2, 1e-2, 1e-2],
        },
    }

    PLOT_SIZES_DEVICE_EXTRINSICS_BOXES: Final = {
        "microphone": [3e-3, 1.5e-3, 2e-3],
        "imu": [5e-3, 4e-3, 3e-3],
        "magnetometer": [3e-3, 3e-3, 2e-3],
        "barometer": [3e-3, 3e-3, 2e-3],
        "cpf": [5e-3, 5e-3, 5e-3],
    }

    def __init__(
        self,
        config: AriaDataViewerConfig = None,
        device_calibration: DeviceCalibration = None,
        rrd_output_path: str = "",
    ):
        """
        Initialization - supports both streaming and recording modes
        """

        self.config = config if config is not None else AriaDataViewerConfig()
        # Both VIO trails are bounded and faded by
        # `config.vio_trajectory_window_sec`, but only one of them is ever
        # drawn -- see `_plot_low_rate_vio_trajectory`.
        self.vio_high_freq_trajectory = FadingTrajectory(
            self.config.vio_trajectory_window_sec
        )
        self.vio_trajectory = FadingTrajectory(self.config.vio_trajectory_window_sec)
        self._low_rate_vio_trajectory_cleared = False
        self._neural_band_emg_styling_logged = False
        self._neural_band_emg_volts_styling_logged = False
        self._neural_band_emg_calibration = None
        # Scale ratio to convert plot sizes from RGB camera space to SLAM camera space (based on camera resolution ratio)

        if rrd_output_path:
            rr.init("AriaDataViewer", spawn=False)
            rr.save(rrd_output_path)
        elif self.config.connect_to_existing_viewer:
            rr.init("AriaDataViewer")
            rr.connect_grpc()
            # If a blueprint path is configured, load it after connecting
            if self.config.blueprint_path:
                rr.log_file_from_path(self.config.blueprint_path)
        elif self.config.blueprint_path:
            self._spawn_rerun_with_blueprint(
                self.config.blueprint_path, self.config.rerun_memory_limit
            )
        else:
            rr.init("AriaDataViewer")
            _spawn_rerun_viewer(self.config.rerun_memory_limit)

        if device_calibration is not None:
            self.device_calibration = device_calibration
            self.sensor_labels = SensorLabels.from_device_version(
                device_calibration.get_device_version()
            )
            self.update_rerun_blueprint()
            self.set_slam_to_rgb_plotting_ratio()
        else:
            self.device_calibration = None
            self.sensor_labels = None
            self.slam_to_rgb_plot_scale = (
                1  # will be reset after device_calibration is set
            )
            print(
                "Warning: device_calibration is None. Cannot create rerun blueprint during initialization."
            )

    def set_viewer_config(self, config: AriaDataViewerConfig):
        self.config = config

    def set_slam_to_rgb_plotting_ratio(self):
        """Calculate and set the scale ratio for converting plot sizes from RGB camera space to SLAM camera space.

        This ratio is based on the height ratio between SLAM and RGB camera resolutions.
        Used to ensure plot elements (markers, gaze points, etc.) appear proportionally
        sized across different camera views with different resolutions.
        """
        if self.device_calibration is None:
            warn_once(
                self.set_slam_to_rgb_plotting_ratio,
                "device_calibration is None. Cannot set slam_to_rgb_plotting_ratio.",
            )
            return
        rgb_image_height = self.device_calibration.get_camera_calib(
            self.sensor_labels.rgb_label
        ).get_image_size()[1]
        slam_image_height = self.device_calibration.get_camera_calib(
            self.sensor_labels.slam_labels[0]
        ).get_image_size()[1]
        if rgb_image_height == 0 or slam_image_height == 0:
            raise RuntimeError(
                "RGB or SLAM image height is 0. Cannot set slam_to_rgb_plotting_ratio."
            )

        self.slam_to_rgb_plot_scale = slam_image_height / rgb_image_height

    def set_device_calibration(self, device_calibration):
        is_first_calibration = self.device_calibration is None
        self.device_calibration = device_calibration
        self.sensor_labels = SensorLabels.from_device_version(
            device_calibration.get_device_version()
        )
        self.set_slam_to_rgb_plotting_ratio()
        if is_first_calibration:
            self.update_rerun_blueprint()
        self.plot_device_extrinsics()

    OPTIONAL_PANEL_CONFIG_FLAGS = {
        "gps": "enable_gps",
        "neural_band_batch": "enable_neural_band_batch",
        "crop_visualization": "enable_crop_visualization",
    }

    def set_optional_panel_visible(self, panel: str, visible: bool) -> None:
        """Show or hide an optional panel, rebuilding the blueprint if it changed.

        Idempotent: repeated calls with the same value are no-ops. When a custom
        blueprint is in use or the first calibration has not yet arrived, the
        config flag is still updated so it takes effect on the next blueprint
        build, but no rebuild is triggered here.

        panel: one of the keys in `OPTIONAL_PANEL_CONFIG_FLAGS`
            (`"gps"`, `"neural_band_batch"`, `"crop_visualization"`).
        """
        if panel not in self.OPTIONAL_PANEL_CONFIG_FLAGS:
            raise ValueError(
                f"Unknown optional panel {panel!r}; expected one of "
                f"{sorted(self.OPTIONAL_PANEL_CONFIG_FLAGS)}"
            )
        flag = self.OPTIONAL_PANEL_CONFIG_FLAGS[panel]
        if getattr(self.config, flag) == visible:
            return
        setattr(self.config, flag, visible)
        if self.config.blueprint_path or self.device_calibration is None:
            return
        self.update_rerun_blueprint()

    def _get_plot_color(self, plot_label):
        """Helper function to get the color for 2D plots."""
        if plot_label in self.PLOT_COLORS_AND_SIZES_2D:
            return self.PLOT_COLORS_AND_SIZES_2D[plot_label]["color"]
        else:
            raise ValueError(f"Unknown plot label: {plot_label}")

    def _get_plot_size(self, plot_label, camera_label=None):
        """Helper function to get the plot size for 2D plots."""
        if plot_label in self.PLOT_COLORS_AND_SIZES_2D:
            plot_settings = self.PLOT_COLORS_AND_SIZES_2D[plot_label]
            # Return appropriate plot size based on camera type
            if camera_label is None or camera_label == self.sensor_labels.rgb_label:
                return plot_settings["plot_size"]
            else:
                return plot_settings["plot_size"] * self.slam_to_rgb_plot_scale
        else:
            raise ValueError(f"Unknown plot label: {plot_label}")

    def _create_gen1_rerun_blueprint(self):
        # Create device extrinsics 3D view
        blueprint_device_extrinsics_view = rrb.Spatial3DView(
            origin="device",
            name="Device Extrinsics",
        )

        # Create RGB camera 2D view
        blueprint_rgb_view = rrb.Spatial2DView(
            name=self.sensor_labels.rgb_label,
            origin=self.sensor_labels.rgb_label,
        )

        # Create 2D grid view with SLAM cameras, eye tracking cameras, and GPS
        contents = [
            rrb.Spatial2DView(name=label, origin=label)
            for label in self.sensor_labels.slam_labels
            + self.sensor_labels.eye_tracking_camera_labels
        ]
        if self.config.enable_gps:
            contents = contents + [  # GPS map view
                rrb.MapView(
                    name=self.sensor_labels.gps_label,
                    origin=self.sensor_labels.gps_label,
                    zoom=rrb.archetypes.MapZoom(16.0),  # initial zoom level
                    background=rrb.MapProvider.OpenStreetMap,
                )
            ]
        blueprint_2d_view = rrb.Grid(contents=contents)

        # Create all 1D views (IMU, audio, and tabbed views)
        # IMU 1D view
        imu_1d_view = rrb.Horizontal(
            contents=[
                rrb.TimeSeriesView(origin=label)
                for label in self.sensor_labels.imu_labels
            ]
        )

        # Audio 1D view
        audio_1d_view = rrb.TimeSeriesView(origin=self.sensor_labels.microphone_label)

        # Tabbed 1D view for magnetometer and barometer
        tabbed_1d_view = rrb.Tabs(
            contents=[
                rrb.TimeSeriesView(origin=label)
                for label in self.sensor_labels.barometer_labels
                + [self.sensor_labels.magnetometer_label]
            ],
        )

        blueprint_1d_view = rrb.Vertical(
            imu_1d_view,
            audio_1d_view,
            tabbed_1d_view,
        )

        # Gen1-specific 3D view layout (simple vertical layout)
        blueprint_3d_view = rrb.Vertical(
            blueprint_rgb_view, blueprint_device_extrinsics_view
        )

        # Create final horizontal blueprint layout.
        # hide left&right, with compact timeline on bottom
        return rrb.Blueprint(
            rrb.Horizontal(
                blueprint_3d_view,
                blueprint_2d_view,
                blueprint_1d_view,
            ),
            collapse_panels=True,
        )

    def _create_gen2_rerun_blueprint(self):
        # Get a template blueprint from Gen1
        template_blueprint_container = (
            self._create_gen1_rerun_blueprint().root_container
        )
        _3d_view_container = template_blueprint_container.contents[0]
        _2d_view_container = template_blueprint_container.contents[1]
        _1d_view_container = template_blueprint_container.contents[2]

        # Update the 3D view
        rgb_tab_contents = [_3d_view_container.contents[0]]  # RGB view
        if self.config.enable_crop_visualization:
            rgb_tab_contents.extend(
                [
                    rrb.Spatial2DView(
                        name="Cropped POV",
                        origin="camera-cropped-pov",
                    ),
                    rrb.Spatial2DView(
                        name="Fixation Crop",
                        origin="camera-fixation-crop",
                    ),
                ]
            )
        updated_3d_view_container = rrb.Vertical(
            rrb.Tabs(contents=rgb_tab_contents),
            rrb.Tabs(
                contents=[
                    rrb.Spatial3DView(
                        origin="world",
                        name="3D Scene",
                        line_grid=rrb.archetypes.LineGrid3D(
                            visible=False,
                        ),
                    ),  # Gen2-added world 3D view
                    _3d_view_container.contents[1],  # Device extrinsics
                ]
            ),
        )

        # Update the 1D view to add contact mic
        contact_mic_1d_view = rrb.TimeSeriesView(
            origin=self.sensor_labels.contact_microphone_label
        )

        neural_band_views = (
            [
                rrb.Tabs(
                    contents=[
                        rrb.TimeSeriesView(
                            name="Neural Band EMG", origin=NEURAL_BAND_EMG_LABEL
                        ),
                        rrb.TimeSeriesView(
                            name="Neural Band EMG (V)",
                            origin=NEURAL_BAND_EMG_VOLTS_LABEL,
                        ),
                        rrb.TimeSeriesView(
                            name="Neural Band Accel", origin=NEURAL_BAND_ACCEL_LABEL
                        ),
                        rrb.TimeSeriesView(
                            name="Neural Band Gyro", origin=NEURAL_BAND_GYRO_LABEL
                        ),
                    ],
                )
            ]
            if self.config.enable_neural_band_batch
            else []
        )

        # Create latency view if enabled (for streaming use case)
        latency_views = []
        if self.config.show_latency:
            latency_views = [
                rrb.TimeSeriesView(
                    name="Latency",
                    origin="latency",
                )
            ]

        updated_1d_view_container = rrb.Vertical(
            _1d_view_container.contents[0],  # IMU plots
            _1d_view_container.contents[1],  # mic
            contact_mic_1d_view,  # contact mic
            *neural_band_views,
            _1d_view_container.contents[2],  # Tabbed baro + mag
            *latency_views,  # latency (optional, for streaming)
        )

        # Create final horizontal blueprint layout.
        # hide left&right, with compact timeline on bottom
        return rrb.Blueprint(
            rrb.Horizontal(
                updated_3d_view_container, _2d_view_container, updated_1d_view_container
            ),
            collapse_panels=True,
        )

    def _spawn_rerun_with_blueprint(self, blueprint_path: str, memory_limit: str):
        """Launch the Rerun viewer with a custom .rbl blueprint file.

        Uses the normal rr.spawn() to start the viewer, then loads the
        blueprint file via rr.log_file_from_path() which can send any
        Rerun recording (including .rbl blueprints) to the viewer.
        """
        rr.init("AriaDataViewer")
        _spawn_rerun_viewer(memory_limit)
        rr.log_file_from_path(blueprint_path)

    def update_rerun_blueprint(self):
        if self.config.blueprint_path:
            return
        if self.device_calibration is None:
            print("Warning: device_calibration is None. Cannot create rerun blueprint.")
            return
        blueprint = None

        # Call the appropriate function based on Aria version
        if self.device_calibration.get_device_version() == DeviceVersion.Gen1:
            blueprint = self._create_gen1_rerun_blueprint()
        elif self.device_calibration.get_device_version() == DeviceVersion.Gen2:
            blueprint = self._create_gen2_rerun_blueprint()
        else:
            print(
                f"Warning: Unsupported device version: {self.device_calibration.get_device_version()}, cannot create rerun blueprint."
            )
            return
        rr.send_blueprint(blueprint, make_active=True)

    def plot_device_extrinsics(self):
        if self.device_calibration is None:
            warn_once(
                self.plot_device_extrinsics,
                "device_calibration is None. Cannot plot device extrinsics.",
            )
            return

        device_version = self.device_calibration.get_device_version()
        if device_version == DeviceVersion.Gen1:
            rr.log("device", rr.ViewCoordinates.RIGHT_HAND_X_DOWN, static=True)
        elif device_version == DeviceVersion.Gen2:
            rr.log("device", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

        # A helper to log components with timeless = True
        log_static = partial(rr.log, static=True)

        cam_labels = self.device_calibration.get_camera_labels()
        for cam in cam_labels:
            camera_calibration = self.device_calibration.get_camera_calib(cam)
            T_device_sensor = camera_calibration.get_transform_device_camera()
            log_static(f"device/camera/{cam}", ToTransform3D(T_device_sensor))
            log_static(
                f"device/camera/{cam}",
                rr.Pinhole(
                    resolution=[
                        camera_calibration.get_image_size()[0],
                        camera_calibration.get_image_size()[1],
                    ],
                    focal_length=float(camera_calibration.get_focal_lengths()[0]),
                ),
            )
        mic_labels = self.device_calibration.get_microphone_labels()
        for mic in mic_labels:  # Note: Only defined in CAD extrinsics
            # Skip Left and Right Speakers because they are not in the CAD
            if mic in ["LSPK", "RSPK"]:
                continue
            T_device_sensor = self.device_calibration.get_transform_device_sensor(
                mic, get_cad_value=True
            )
            log_static(f"device/mic/{mic}", ToTransform3D(T_device_sensor))
            log_static(
                f"device/mic/{mic}/box",
                ToBox3D(mic, self.PLOT_SIZES_DEVICE_EXTRINSICS_BOXES["microphone"]),
            )

        imu_labels = self.device_calibration.get_imu_labels()
        for imu in imu_labels:
            T_device_sensor = self.device_calibration.get_transform_device_sensor(
                imu, get_cad_value=False
            )
            log_static(f"device/imu/{imu}", ToTransform3D(T_device_sensor))
            log_static(
                f"device/imu/{imu}",
                ToBox3D(imu, self.PLOT_SIZES_DEVICE_EXTRINSICS_BOXES["imu"]),
            )

        magnetometer_labels = self.device_calibration.get_magnetometer_labels()
        for (
            magnetometer
        ) in magnetometer_labels:  # Note: Only defined in CAD calibration
            T_device_sensor = self.device_calibration.get_transform_device_sensor(
                magnetometer, get_cad_value=True
            )
            log_static(f"device/{magnetometer}", ToTransform3D(T_device_sensor))
            log_static(
                f"device/{magnetometer}/box",
                ToBox3D(
                    magnetometer,
                    self.PLOT_SIZES_DEVICE_EXTRINSICS_BOXES["magnetometer"],
                ),
            )

        barometer_labels = self.device_calibration.get_barometer_labels()
        for barometer in barometer_labels:  # Note: Only defined in CAD calibration
            T_device_sensor = self.device_calibration.get_transform_device_sensor(
                barometer, True
            )
            log_static(f"device/{barometer}", ToTransform3D(T_device_sensor))
            log_static(
                f"device/{barometer}",
                ToBox3D(
                    barometer, self.PLOT_SIZES_DEVICE_EXTRINSICS_BOXES["barometer"]
                ),
            )

        # Plot CPF (Central Pupil Frame coordinate system)
        T_device_CPF = self.device_calibration.get_transform_device_cpf()
        log_static("device/CPF_CentralPupilFrame", ToTransform3D(T_device_CPF))
        log_static(
            "device/CPF_CentralPupilFrame",
            ToBox3D(
                "CPF_CentralPupilFrame", self.PLOT_SIZES_DEVICE_EXTRINSICS_BOXES["cpf"]
            ),
        )

        # Plot Project Aria Glasses outline (as lines)
        aria_glasses_point_outline = AriaGlassesOutline(
            self.device_calibration, use_cad_calib=False
        )
        log_static(
            "device/glasses_outline", rr.LineStrips3D([aria_glasses_point_outline])
        )

        ##
        # Use the ability to log a markdown file to ease how users can understand what they are looking at
        log_static(
            "Notes",
            rr.TextDocument(
                """
    # Project Aria Glasses - Sensor Viewer

    Here is a visual preview of the Project Aria glasses sensors (location and orientation).

    Cameras sensors:

    - two (Gen1) / four (Gen2) Mono SLAM cameras
    - one [RGB](recording://device/camera/camera-rgb)
    - two Eye Tracking cameras [Eye left](recording://device/camera/camera-et-left), [Eye right](recording://device/camera/camera-et-right)

    Non-visual sensors:

    - microphones
    - two IMUs [Imu-left](recording://device/imu/imu-left), [Imu-right](recording://device/imu/imu-right)
    - [magnetometer](recording://device/mag0)
    - [barometer](recording://device/baro0)
    - GPS, Wi-Fi beacon, Bluetooth beacon.
            """,
                media_type=rr.MediaType.MARKDOWN,
            ),
        )

    def _check_and_rotate_image_for_gen1(self, frame, label):
        if (
            self.device_calibration is not None
            and self.device_calibration.get_device_version() == DeviceVersion.Gen1
            and label != self.sensor_labels.eye_tracking_camera_labels[0]
        ):
            # rotate the image for Gen1 device, except for the eye tracking camera
            # k:Number of 90-degree counterclockwise rotations to apply to the image
            return np.rot90(frame, k=3)
        return frame

    def plot_image(self, frame, label, device_timestamp_ns):
        """Plot image data."""
        if frame is None:
            warn_once(
                self.plot_image,
                f"Frame is None for label {label}",
            )
            return

        if not isinstance(frame, np.ndarray):
            warn_once(
                self.plot_image,
                f"Frame is not a numpy array for label {label}",
            )
            return
        rr.set_time("device_time", timestamp=device_timestamp_ns * 1e-9)
        frame = self._check_and_rotate_image_for_gen1(frame, label)
        rr.log(
            label,
            rr.Image(frame).compress(self.config.jpeg_quality),
        )

    def plot_imu_batch_vectorized(self, imu_data_list, label):
        """Plot multiple IMU sensor data points with maximum efficiency using bulk logging."""
        if not imu_data_list:
            return

        # Filter valid data and extract arrays efficiently
        valid_data = [
            d
            for d in imu_data_list
            if d is not None and hasattr(d, "accel_msec2") and hasattr(d, "gyro_radsec")
        ]

        if not valid_data:
            return

        # Extract timestamps and data as numpy arrays for bulk operations
        timestamps = np.array(
            [d.capture_timestamp_ns for d in valid_data], dtype=np.int64
        )
        accel_data = np.array([d.accel_msec2 for d in valid_data])  # Shape: (N, 3)
        gyro_data = np.array([d.gyro_radsec for d in valid_data])  # Shape: (N, 3)

        # Use bulk logging with send_columns - same pattern as audio logging
        # Log accelerometer data
        rr.send_columns(
            f"{label}/accl/x[m-sec2]",
            indexes=[rr.TimeColumn("device_time", timestamp=timestamps * 1e-9)],
            columns=rr.Scalars.columns(scalars=accel_data[:, 0]),
        )
        rr.send_columns(
            f"{label}/accl/y[m-sec2]",
            indexes=[rr.TimeColumn("device_time", timestamp=timestamps * 1e-9)],
            columns=rr.Scalars.columns(scalars=accel_data[:, 1]),
        )
        rr.send_columns(
            f"{label}/accl/z[m-sec2]",
            indexes=[rr.TimeColumn("device_time", timestamp=timestamps * 1e-9)],
            columns=rr.Scalars.columns(scalars=accel_data[:, 2]),
        )

        # Log gyroscope data
        rr.send_columns(
            f"{label}/gyro/x[rad-sec]",
            indexes=[rr.TimeColumn("device_time", timestamp=timestamps * 1e-9)],
            columns=rr.Scalars.columns(scalars=gyro_data[:, 0]),
        )
        rr.send_columns(
            f"{label}/gyro/y[rad-sec]",
            indexes=[rr.TimeColumn("device_time", timestamp=timestamps * 1e-9)],
            columns=rr.Scalars.columns(scalars=gyro_data[:, 1]),
        )
        rr.send_columns(
            f"{label}/gyro/z[rad-sec]",
            indexes=[rr.TimeColumn("device_time", timestamp=timestamps * 1e-9)],
            columns=rr.Scalars.columns(scalars=gyro_data[:, 2]),
        )

    def plot_imu(self, imu_data, label):
        """Plot IMU sensor data."""
        if imu_data is None:
            warn_once(
                self.plot_imu,
                f"IMU data is None for label {label}",
            )
            return

        if not hasattr(imu_data, "accel_msec2") or not hasattr(imu_data, "gyro_radsec"):
            warn_once(
                self.plot_imu,
                f"IMU data missing required attributes for label {label}",
            )
            return
        rr.set_time("device_time", timestamp=imu_data.capture_timestamp_ns * 1e-9)
        rr.log(f"{label}/accl/x[m-sec2]", rr.Scalars(imu_data.accel_msec2[0]))
        rr.log(f"{label}/accl/y[m-sec2]", rr.Scalars(imu_data.accel_msec2[1]))
        rr.log(f"{label}/accl/z[m-sec2]", rr.Scalars(imu_data.accel_msec2[2]))
        rr.log(f"{label}/gyro/x[rad-sec]", rr.Scalars(imu_data.gyro_radsec[0]))
        rr.log(f"{label}/gyro/y[rad-sec]", rr.Scalars(imu_data.gyro_radsec[1]))
        rr.log(f"{label}/gyro/z[rad-sec]", rr.Scalars(imu_data.gyro_radsec[2]))

    def plot_magnetometer(self, magnetometer_data):
        """Plot magnetometer sensor data."""
        if magnetometer_data is None:
            warn_once(
                self.plot_magnetometer,
                "Magnetometer data is None",
            )
            return

        if not hasattr(magnetometer_data, "mag_tesla"):
            warn_once(
                self.plot_magnetometer,
                "Magnetometer data missing mag_tesla attribute",
            )
            return

        rr.set_time(
            "device_time", timestamp=magnetometer_data.capture_timestamp_ns * 1e-9
        )
        # Convert magnetometer reading from tesla (SI unit) to microtesla (µT): 1 tesla = 1e6 microtesla
        rr.log(
            f"{self.sensor_labels.magnetometer_label}/x[µT]",
            rr.Scalars(magnetometer_data.mag_tesla[0] * 1e6),
        )
        rr.log(
            f"{self.sensor_labels.magnetometer_label}/y[µT]",
            rr.Scalars(magnetometer_data.mag_tesla[1] * 1e6),
        )
        rr.log(
            f"{self.sensor_labels.magnetometer_label}/z[µT]",
            rr.Scalars(magnetometer_data.mag_tesla[2] * 1e6),
        )

    def plot_barometer(self, barometer_data):
        """Plot barometer sensor data."""
        if barometer_data is None:
            warn_once(
                self.plot_barometer,
                "Barometer data is None",
            )
            return

        if not hasattr(barometer_data, "pressure") or not hasattr(
            barometer_data, "temperature"
        ):
            warn_once(
                self.plot_barometer,
                "Barometer data missing required attributes",
            )
            return
        rr.set_time("device_time", timestamp=barometer_data.capture_timestamp_ns * 1e-9)
        # Convert pressure from pascal (SI unit) to kilopascal (kPa): 1 pascal = 1e-3 kilopascal
        rr.log(
            f"{self.sensor_labels.barometer_labels[0]}/pressure[kPa]",
            rr.Scalars(barometer_data.pressure * 1e-3),
        )  # Pascals converter to kPascals
        rr.log(
            f"{self.sensor_labels.barometer_labels[1]}/temperature[Celsius]",
            rr.Scalars(barometer_data.temperature),
        )  # Degree Celsius

    def set_neural_band_emg_calibration(self, calibration):
        self._neural_band_emg_calibration = calibration

    def _neural_band_times_sec(self, samples, panel):
        """Device-timeline seconds for a batch's samples, or None if unmapped.

        No fallback: the wristband clock the samples also carry is a different
        timebase, seconds away from the device one, so plotting on it would put
        the trace somewhere it was not recorded.

        `panel` is the plotting method asking. Sub-streams are mapped
        independently, so keying the warning on it lets each panel report its own
        skip instead of the first one silencing the rest.
        """
        device_times_ns = [s.device_timestamp_ns for s in samples]
        if any(t is None for t in device_times_ns):
            name = panel.__name__.removeprefix("_plot_neural_band_")
            warn_once(
                panel,
                f"Neural Band {name} samples carry no device timestamp; "
                "that panel stays empty",
            )
            return None
        return np.fromiter(
            (t * 1e-9 for t in device_times_ns),
            dtype=np.float64,
            count=len(device_times_ns),
        )

    def plot_neural_band_batch(self, batch):
        if batch is None:
            warn_once(self.plot_neural_band_batch, "NeuralBandBatch is None")
            return

        self._plot_neural_band_emg(batch)
        self._plot_neural_band_emg_volts(batch)
        self._plot_neural_band_accel(batch)
        self._plot_neural_band_gyro(batch)

    def _ensure_neural_band_emg_styling(self, channel_count):
        """Log a distinct color + legend name per EMG channel, once."""
        if getattr(self, "_neural_band_emg_styling_logged", False):
            return
        for channel in range(channel_count):
            hue = channel / max(channel_count, 1)
            r, g, b = colorsys.hsv_to_rgb(hue, 0.85, 0.9)
            rr.log(
                f"{NEURAL_BAND_EMG_LABEL}/channels/channel_{channel}",
                rr.SeriesLines(
                    colors=[[int(r * 255), int(g * 255), int(b * 255)]],
                    names=[f"channel_{channel}"],
                ),
                static=True,
            )
        self._neural_band_emg_styling_logged = True

    def _plot_neural_band_emg(self, batch):
        if not batch.emg:
            return
        channel_count = batch.emg_channel_count
        if channel_count <= 0:
            warn_once(
                self._plot_neural_band_emg,
                "NeuralBandBatch.emg_channel_count is not populated; skipping EMG plot",
            )
            return

        valid = [s for s in batch.emg if len(s.channel_values) == channel_count]
        if not valid:
            return

        timestamps_sec = self._neural_band_times_sec(valid, self._plot_neural_band_emg)
        if timestamps_sec is None:
            return
        self._ensure_neural_band_emg_styling(channel_count)
        values = np.array([s.channel_values for s in valid], dtype=np.float64)

        for channel in range(channel_count):
            rr.send_columns(
                f"{NEURAL_BAND_EMG_LABEL}/channels/channel_{channel}",
                indexes=[rr.TimeColumn("device_time", timestamp=timestamps_sec)],
                columns=rr.Scalars.columns(scalars=values[:, channel]),
            )

    def _ensure_neural_band_emg_volts_styling(self, channel_count):
        """Log a distinct color + legend name per EMG-volts channel, once."""
        if getattr(self, "_neural_band_emg_volts_styling_logged", False):
            return
        for channel in range(channel_count):
            hue = channel / max(channel_count, 1)
            r, g, b = colorsys.hsv_to_rgb(hue, 0.85, 0.9)
            rr.log(
                f"{NEURAL_BAND_EMG_VOLTS_LABEL}/channels/channel_{channel}",
                rr.SeriesLines(
                    colors=[[int(r * 255), int(g * 255), int(b * 255)]],
                    names=[f"channel_{channel}"],
                ),
                static=True,
            )
        self._neural_band_emg_volts_styling_logged = True

    def _plot_neural_band_emg_volts(self, batch):
        calibration = getattr(self, "_neural_band_emg_calibration", None)
        if calibration is None or not batch.emg:
            return
        channel_count = batch.emg_channel_count
        if channel_count <= 0:
            return
        valid = [s for s in batch.emg if len(s.channel_values) == channel_count]
        if not valid:
            return

        timestamps_sec = self._neural_band_times_sec(
            valid, self._plot_neural_band_emg_volts
        )
        if timestamps_sec is None:
            return
        self._ensure_neural_band_emg_volts_styling(channel_count)
        # One vectorized adc_to_volts call for the whole batch, then reshape.
        flat_adcs = [v for s in valid for v in s.channel_values]
        volts = np.asarray(
            calibration.adc_to_volts(flat_adcs), dtype=np.float64
        ).reshape(len(valid), channel_count)
        for channel in range(channel_count):
            rr.send_columns(
                f"{NEURAL_BAND_EMG_VOLTS_LABEL}/channels/channel_{channel}",
                indexes=[rr.TimeColumn("device_time", timestamp=timestamps_sec)],
                columns=rr.Scalars.columns(scalars=volts[:, channel]),
            )

    def _plot_neural_band_accel(self, batch):
        if not batch.accel:
            return
        timestamps_sec = self._neural_band_times_sec(
            batch.accel, self._plot_neural_band_accel
        )
        if timestamps_sec is None:
            return
        values = np.array([s.accel_msec2 for s in batch.accel])
        for i, axis in enumerate(("x", "y", "z")):
            rr.send_columns(
                f"{NEURAL_BAND_ACCEL_LABEL}/{axis}[m-sec2]",
                indexes=[rr.TimeColumn("device_time", timestamp=timestamps_sec)],
                columns=rr.Scalars.columns(scalars=values[:, i]),
            )

    def _plot_neural_band_gyro(self, batch):
        if not batch.gyro:
            return
        timestamps_sec = self._neural_band_times_sec(
            batch.gyro, self._plot_neural_band_gyro
        )
        if timestamps_sec is None:
            return
        values = np.array([s.gyro_radsec for s in batch.gyro])
        for i, axis in enumerate(("x", "y", "z")):
            rr.send_columns(
                f"{NEURAL_BAND_GYRO_LABEL}/{axis}[rad-sec]",
                indexes=[rr.TimeColumn("device_time", timestamp=timestamps_sec)],
                columns=rr.Scalars.columns(scalars=values[:, i]),
            )

    def _plot_audio_from_selected_channels(
        self,
        audio_data_and_record,
        total_num_audio_channels,
        selected_channel_indices,
        selected_channel_labels,
        rerun_plotter_label,
    ):
        """Implementation: plot audio sensor data from selected channels."""
        audio_data = audio_data_and_record[0].data
        audio_data_timestamp = audio_data_and_record[1].capture_timestamps_ns

        if audio_data is None or audio_data_timestamp is None:
            warn_once(
                self._plot_audio_from_selected_channels,
                "Audio data or timestamp is None",
            )
            return

        # Reshape audio data into [num_channels, num_samples]
        all_audio_vectors = (
            np.array_split(audio_data, total_num_audio_channels)
            / np.finfo(np.float32).max
        )

        # Filter to only selected channels
        sampled_vectors = [
            all_audio_vectors[i]
            for i in selected_channel_indices
            if i < len(all_audio_vectors)
        ]

        for c in range(0, len(sampled_vectors)):
            rr.send_columns(
                f"{rerun_plotter_label}/{selected_channel_labels[c]}",
                indexes=[
                    rr.TimeColumn(
                        "device_time",
                        timestamp=np.array(audio_data_timestamp)[
                            :: self.config.audio_subsample_rate
                        ]
                        * 1e-9,
                    )
                ],
                columns=rr.Scalars.columns(
                    scalars=sampled_vectors[c][:: self.config.audio_subsample_rate]
                ),
            )

    def plot_audio(self, audio_data_and_record, num_audio_channels):
        """
        Plot audio sensor data, where non-contact mic is plotted in one plot,
        and contact mic is plotted in a separate plot.
        """
        # -------
        # Sanity checks on data size
        # -------
        if audio_data_and_record is None or len(audio_data_and_record) < 2:
            warn_once(
                self.plot_audio,
                "Audio data or record is None or incomplete",
            )
            return

        if num_audio_channels <= 0:
            warn_once(
                self.plot_audio,
                "Invalid number of audio channels",
            )
            return

        # -------
        # Plot for non-contact mics. Note that device calibration only contains non-contact mics
        # -------
        non_contact_mic_labels = self.device_calibration.get_microphone_labels()
        non_contact_mic_indices = list(range(len(non_contact_mic_labels)))
        self._plot_audio_from_selected_channels(
            audio_data_and_record=audio_data_and_record,
            total_num_audio_channels=num_audio_channels,
            selected_channel_indices=non_contact_mic_indices,
            selected_channel_labels=non_contact_mic_labels,
            rerun_plotter_label=self.sensor_labels.microphone_label,
        )

        # --------
        # Gen2 only: Plot for contact mic , which is the last audio channel
        # --------
        if self.device_calibration.get_device_version() == DeviceVersion.Gen2:
            contact_mic_index = num_audio_channels - 1
            contact_mic_label = self.sensor_labels.contact_microphone_label
            self._plot_audio_from_selected_channels(
                audio_data_and_record=audio_data_and_record,
                total_num_audio_channels=num_audio_channels,
                selected_channel_indices=[contact_mic_index],
                selected_channel_labels=[contact_mic_label],
                rerun_plotter_label=contact_mic_label,
            )

    def plot_gps(self, gps_data):
        """Plot GPS data."""
        if gps_data is None:
            warn_once(self.plot_gps, "GPS data is None")
            return

        if (
            not hasattr(gps_data, "latitude")
            or not hasattr(gps_data, "longitude")
            or gps_data.latitude is None
            or gps_data.longitude is None
            or (isinstance(gps_data.latitude, float) and np.isnan(gps_data.latitude))
            or (isinstance(gps_data.longitude, float) and np.isnan(gps_data.longitude))
        ):
            warn_once(
                self.plot_gps,
                "GPS data missing latitude or longitude attributes or contains NaN",
            )
            return

        rr.set_time("device_time", timestamp=gps_data.capture_timestamp_ns * 1e-9)
        # gps_data.provider is a string that can be "APP" or "GPS", indicating data source.
        gps_settings = self.PLOT_COLORS_AND_SIZES_2D[
            "gps_app"
            if (gps_data.provider == "APP" or gps_data.provider == "app")
            else "gps_sensor"
        ]

        rr.log(
            self.sensor_labels.gps_label,
            rr.GeoPoints(
                lat_lon=[gps_data.latitude, gps_data.longitude],
                radii=rr.Radius.ui_points(gps_settings["plot_size"]),
                colors=gps_settings["color"],
            ),
        )

    def plot_eye_gaze_data(self, eyegaze_data):
        """
        Plotter function to plot eye gaze data onto the images
        """
        if self.device_calibration is None:
            warn_once(
                self.plot_eye_gaze_data,
                "device_calibration is None. Cannot plot eye gaze data.",
            )
            return
        rr.set_time(
            "device_time", timestamp=eyegaze_data.tracking_timestamp.total_seconds()
        )
        # Clear the canvas (only if eye_gaze_label exists for this device version)
        if self.sensor_labels.eye_gaze_label:
            rr.log(
                f"world/device/{self.sensor_labels.eye_gaze_label}",
                rr.Clear.recursive(),
            )

            for label in self.sensor_labels.rgb_and_slam_labels:
                rr.log(
                    f"{label}/{self.sensor_labels.eye_gaze_label}", rr.Clear.recursive()
                )

        # get eye gaze data
        if not (
            eyegaze_data.spatial_gaze_point_valid and eyegaze_data.combined_gaze_valid
        ):
            return

        ######### Plot spatial eye gaze point in Camera View #############
        spatial_gaze_point_in_cpf = eyegaze_data.spatial_gaze_point_in_cpf
        T_device_cpf = self.device_calibration.get_transform_device_cpf()
        spatial_gaze_point_in_device = T_device_cpf @ spatial_gaze_point_in_cpf

        # for each camera, project the eye gaze point onto the image
        for camera_label in self.sensor_labels.rgb_and_slam_labels:
            # get calibration for the camera
            camera_calib = self.device_calibration.get_camera_calib(camera_label)
            spatial_gaze_point_in_camera = (
                camera_calib.get_transform_device_camera().inverse()
                @ spatial_gaze_point_in_device
            )

            # project the eye gaze point onto the image
            maybe_pixel = camera_calib.project(spatial_gaze_point_in_camera)

            if maybe_pixel is not None:
                rr.log(
                    f"{camera_label}/{self.sensor_labels.eye_gaze_label}/gaze_point",
                    rr.Points2D(
                        positions=[maybe_pixel],
                        colors=[self._get_plot_color("eye_gaze_point")],
                        radii=self._get_plot_size(
                            plot_label="eye_gaze_point", camera_label=camera_label
                        ),
                    ),
                )

        ######### Plot eye gaze directions in 3D View #############
        combined_gaze_origin_in_device = (
            T_device_cpf @ eyegaze_data.combined_gaze_origin_in_cpf
        )
        combined_gaze_direction_in_cpf = get_unit_vector_from_yaw_pitch(
            eyegaze_data.yaw, eyegaze_data.pitch
        )
        combined_gaze_direction_in_device = (
            T_device_cpf.rotation() @ combined_gaze_direction_in_cpf
        ) * eyegaze_data.depth
        spatial_gaze_point_in_device = (
            T_device_cpf @ eyegaze_data.spatial_gaze_point_in_cpf
        )
        rr.log(
            f"world/device/{self.sensor_labels.eye_gaze_label}/combined_gaze_direction",
            rr.Arrows3D(
                origins=[combined_gaze_origin_in_device],
                vectors=[combined_gaze_direction_in_device],
            ),
        )
        rr.log(
            f"world/device/{self.sensor_labels.eye_gaze_label}/combined_eyegaze_origin",
            rr.Boxes3D(
                centers=[combined_gaze_origin_in_device],
                sizes=self.PLOT_COLORS_AND_SIZES_3D["eyegaze"][
                    "combined_gaze_origin_size"
                ],
                fill_mode="solid",
            ),
        )
        rr.log(
            f"world/device/{self.sensor_labels.eye_gaze_label}/spatial_gaze_point",
            rr.Boxes3D(
                centers=[spatial_gaze_point_in_device],
                sizes=self.PLOT_COLORS_AND_SIZES_3D["eyegaze"][
                    "spatial_gaze_point_size"
                ],
                fill_mode="solid",
            ),
        )

    def plot_fixation_crop_data(self, fixation_crop_data) -> None:
        """Plot cropped POV image with gaze point overlay."""
        image_data = fixation_crop_data.image_data
        image_array = image_data[0].to_numpy_array()
        if image_array is None:
            return

        device_timestamp_ns = image_data[1].capture_timestamp_ns
        camera_label = "camera-fixation-crop"

        rr.set_time("device_time", timestamp=device_timestamp_ns * 1e-9)

        frame = np.array(image_array)
        rr.log(
            camera_label,
            rr.Image(frame).compress(self.config.jpeg_quality),
        )

        gaze_point = fixation_crop_data.gaze_point_in_crop
        if gaze_point is not None:
            plot_label = "fixation_gaze_point"

            rr.log(
                f"{camera_label}/fixation/gaze_point",
                rr.Points2D(
                    positions=[[gaze_point[0], gaze_point[1]]],
                    colors=[self._get_plot_color(plot_label)],
                    radii=self._get_plot_size(plot_label),
                ),
            )
        else:
            rr.log(f"{camera_label}/fixation", rr.Clear.recursive())

    def plot_cropped_pov_image_data(self, image_data, image_record) -> None:
        """Plot cropped POV image for non-fixation crop sources (e.g. HandObject/HOI)."""
        image_array = image_data.to_numpy_array()
        if image_array is None:
            return

        device_timestamp_ns = image_record.capture_timestamp_ns
        camera_label = "camera-cropped-pov"

        rr.set_time("device_time", timestamp=device_timestamp_ns * 1e-9)

        frame = np.array(image_array)
        rr.log(
            camera_label,
            rr.Image(frame).compress(self.config.jpeg_quality),
        )

    def _plot_single_hand_3d(
        self, hand_joints_in_device: List[np.array], hand_label: str
    ):
        """
        Plot single hand data in 3D view
        """
        hand_skeleton_3d = create_hand_skeleton_from_landmarks(hand_joints_in_device)
        rr.log(
            f"world/device/{self.sensor_labels.hand_tracking_label}/{hand_label}/landmarks",
            rr.Points3D(
                positions=hand_joints_in_device,
                colors=[
                    self.PLOT_COLORS_AND_SIZES_3D["handtracking"][
                        f"{hand_label}_hand_markers_color"
                    ]
                ],
                radii=self.PLOT_COLORS_AND_SIZES_3D["handtracking"][
                    "landmarks_radius_3d"
                ],
            ),
        )
        rr.log(
            f"world/device/{self.sensor_labels.hand_tracking_label}/{hand_label}/hand_skeleton",
            rr.LineStrips3D(
                hand_skeleton_3d,
                colors=[
                    self.PLOT_COLORS_AND_SIZES_3D["handtracking"][
                        f"{hand_label}_hand_lines_color"
                    ]
                ],
                radii=self.PLOT_COLORS_AND_SIZES_3D["handtracking"][
                    "skeleton_radius_3d"
                ],
            ),
        )

    def _plot_single_hand_2d(
        self, hand_joints_in_device: List[np.array], hand_label: str, camera_label: str
    ):
        """
        Plot single hand data in 2D camera view
        """
        # get calibration for the camera
        camera_calib = self.device_calibration.get_camera_calib(camera_label)

        # project into camera frame, and also create line segments
        hand_joints_in_camera = []
        for pt_in_device in hand_joints_in_device:
            pt_in_camera = (
                camera_calib.get_transform_device_camera().inverse() @ pt_in_device
            )
            pixel = camera_calib.project(pt_in_camera)
            hand_joints_in_camera.append(pixel)

        # Create hand skeleton in 2D image space
        hand_skeleton = create_hand_skeleton_from_landmarks(hand_joints_in_camera)

        # Remove "None" markers from hand joints in camera. This is intentionally done AFTER the hand skeleton creation
        hand_joints_in_camera = list(
            filter(lambda x: x is not None, hand_joints_in_camera)
        )

        rr.log(
            f"{camera_label}/{self.sensor_labels.hand_tracking_label}/{hand_label}/landmarks",
            rr.Points2D(
                positions=hand_joints_in_camera,
                colors=self._get_plot_color(f"{hand_label}_hand_markers"),
                radii=self._get_plot_size(
                    plot_label=f"{hand_label}_hand_markers",
                    camera_label=camera_label,
                ),
            ),
        )
        rr.log(
            f"{camera_label}/{self.sensor_labels.hand_tracking_label}/{hand_label}/skeleton",
            rr.LineStrips2D(
                hand_skeleton,
                colors=[self._get_plot_color(f"{hand_label}_hand_lines")],
                radii=self._get_plot_size(
                    plot_label=f"{hand_label}_hand_lines",
                    camera_label=camera_label,
                ),
            ),
        )

    def _plot_single_hand(self, hand_joints_in_device: List[np.array], hand_label: str):
        """
        Plot single hand data in 3D and 2D camera views
        """
        # plot in 3D
        self._plot_single_hand_3d(hand_joints_in_device, hand_label)

        # plot in 2D
        for camera_label in self.sensor_labels.rgb_and_slam_labels:
            self._plot_single_hand_2d(hand_joints_in_device, hand_label, camera_label)

    def plot_hand_pose_data(
        self,
        hand_pose_data,
    ):
        """
        Plot hand pose data in both 2D and 3D views
        """
        self.plot_hand_pose_data_3d(hand_pose_data=hand_pose_data)

        for camera_label in self.sensor_labels.rgb_and_slam_labels:
            self.plot_hand_pose_data_2d(
                hand_pose_data=hand_pose_data, camera_label=camera_label
            )

    def plot_hand_pose_data_3d(self, hand_pose_data):
        """
        Plot hand pose data in 3D world view
        """
        rr.set_time(
            "device_time", timestamp=hand_pose_data.tracking_timestamp.total_seconds()
        )

        # Clear the canvas (only if hand_tracking_label exists for this device version)
        if self.sensor_labels.hand_tracking_label:
            rr.log(
                f"world/device/{self.sensor_labels.hand_tracking_label}",
                rr.Clear.recursive(),
            )

        # Plot both hands
        if hand_pose_data.left_hand is not None:
            self._plot_single_hand_3d(
                hand_joints_in_device=hand_pose_data.left_hand.landmark_positions_device,
                hand_label="left",
            )
        if hand_pose_data.right_hand is not None:
            self._plot_single_hand_3d(
                hand_joints_in_device=hand_pose_data.right_hand.landmark_positions_device,
                hand_label="right",
            )

    def clear_hand_pose_data_2d(self, camera_label: str):
        if self.sensor_labels.hand_tracking_label:
            # Clear the canvas first
            rr.log(
                f"{camera_label}/{self.sensor_labels.hand_tracking_label}",
                rr.Clear.recursive(),
            )

    def plot_hand_pose_data_2d(self, hand_pose_data, camera_label: str):
        """
        Plot hand pose data in 2D camera view
        """
        # calibration is needed to project hand pose data into camera view
        if self.device_calibration is None:
            warn_once(
                self.plot_hand_pose_data,
                "device_calibration is None. Cannot plot hand pose data.",
            )
            return

        if self.sensor_labels.hand_tracking_label:
            rr.set_time(
                "device_time",
                timestamp=hand_pose_data.tracking_timestamp.total_seconds(),
            )

            # Clear the canvas first
            self.clear_hand_pose_data_2d(camera_label=camera_label)

            # Plot both hands
            if hand_pose_data.left_hand is not None:
                self._plot_single_hand_2d(
                    hand_joints_in_device=hand_pose_data.left_hand.landmark_positions_device,
                    hand_label="left",
                    camera_label=camera_label,
                )
            if hand_pose_data.right_hand is not None:
                self._plot_single_hand_2d(
                    hand_joints_in_device=hand_pose_data.right_hand.landmark_positions_device,
                    hand_label="right",
                    camera_label=camera_label,
                )

    def plot_vio_high_freq_data(self, vio_high_freq_data):
        """Plot VIO high frequency data"""
        if self.device_calibration is None:
            warn_once(
                self.plot_vio_high_freq_data,
                "device_calibration is None. Cannot plot VIO high frequency data.",
            )
            return
        rr.set_time(
            "device_time",
            timestamp=vio_high_freq_data.tracking_timestamp.total_seconds(),
        )
        # Set and plot Aria Device for the current timestamp
        T_World_Device = vio_high_freq_data.transform_odometry_device

        # Plot VIO high-freq trajectory that is still inside the fade window
        # TODO: Optimize VIO high-freq trajectory plotting.
        self.vio_high_freq_trajectory.append(
            vio_high_freq_data.tracking_timestamp.total_seconds(),
            T_World_Device.translation()[0],
        )
        strips, colors = self.vio_high_freq_trajectory.strips_and_colors(
            self.PLOT_COLORS_AND_SIZES_3D["vio_high_freq"]["color"]
        )
        rr.log(
            f"world/{self.sensor_labels.vio_high_freq_label}",
            rr.LineStrips3D(
                strips,
                colors=colors,
                radii=self.PLOT_COLORS_AND_SIZES_3D["vio_high_freq"][
                    "trajectory_radius"
                ],
            ),
            static=False,
        )

    def plot_vio_data(self, vio_data):
        # Only plot VIO data if status is valid, and pose quality is good
        if (
            vio_data.status != VioStatus.VALID
            or vio_data.pose_quality != TrackingQuality.GOOD
        ):
            return
        rr.set_time("device_time", timestamp=vio_data.capture_timestamp_ns * 1e-9)
        # Set and plot Aria Device for the current timestamp
        T_World_Device = (
            vio_data.transform_odometry_bodyimu @ vio_data.transform_bodyimu_device
        )
        rr.log(
            "world/device",
            ToTransform3D(
                T_World_Device,
            ),
        )
        rr.log(
            "world/device",
            rr.TransformAxes3D(
                axis_length=self.PLOT_COLORS_AND_SIZES_3D["vio"]["device_axis_length"]
            ),
        )

        # Plot Aria glass outline
        aria_glasses_point_outline = AriaGlassesOutline(
            self.device_calibration, use_cad_calib=True
        )
        rr.log(
            "world/device/glasses_outline",
            rr.LineStrips3D(
                aria_glasses_point_outline,
                colors=[self.PLOT_COLORS_AND_SIZES_3D["glass_outline"]["color"]],
                radii=self.PLOT_COLORS_AND_SIZES_3D["glass_outline"][
                    "glasses_outline_radius"
                ],
            ),
        )

        # Plot gravity direction vector
        rr.log(
            "world/vio_gravity",
            rr.Arrows3D(
                origins=[T_World_Device.translation()[0]],
                vectors=[
                    vio_data.gravity_in_odometry * 1e-2
                ],  # length converted from 9.8 meter -> 10 cm
                colors=[self.PLOT_COLORS_AND_SIZES_3D["vio_gravity"]["color"]],
                radii=1.5e-3,
            ),
            static=False,
        )

        self._plot_low_rate_vio_trajectory(
            vio_data.capture_timestamp_ns * 1e-9, T_World_Device.translation()[0]
        )

    def _plot_low_rate_vio_trajectory(self, timestamp_sec, position) -> None:
        """Draw the `vio` trail, but only when it is the only trail there is.

        `vio` and `vio_high_frequency` are the same odometry-to-device path,
        differing in rate. Drawn together they are two tubes of identical
        radius in identical positions, which z-fight into a dashed line that
        alternates between the two trails' colours -- so the high-frequency
        one, which carries the finer motion, wins whenever it is present.

        It cannot simply be dropped: `vio` and `vio_high_frequency` are
        separately selectable streams, and a recording (or a `--stream-labels`
        selection) that has only the low-rate one would then show no trajectory
        at all. Hence the fallback, resolved from the data rather than declared
        up front.
        """
        if len(self.vio_high_freq_trajectory) > 0:
            # A stream can deliver a few `vio` samples before the first
            # high-frequency one; clear whatever they already drew, once.
            if not self._low_rate_vio_trajectory_cleared:
                self._low_rate_vio_trajectory_cleared = True
                rr.log(
                    f"world/{self.sensor_labels.vio_label}",
                    rr.Clear(recursive=False),
                )
            return

        self.vio_trajectory.append(timestamp_sec, position)
        strips, colors = self.vio_trajectory.strips_and_colors(
            self.PLOT_COLORS_AND_SIZES_3D["vio"]["color"]
        )
        rr.log(
            f"world/{self.sensor_labels.vio_label}",
            rr.LineStrips3D(
                strips,
                colors=colors,
                radii=self.PLOT_COLORS_AND_SIZES_3D["vio"]["trajectory_radius"],
            ),
            static=False,
        )

    def plot_utc_timestamp(
        self, utc_timestamp_ns, camera_label: str, device_timestamp_ns
    ):
        rr.set_time("device_time", timestamp=device_timestamp_ns * 1e-9)
        rr.log(
            f"{camera_label}/utc_timestamp",
            rr.Points2D(
                positions=[500.0, 5.0],
                colors=self._get_plot_color("overlay_text"),
                radii=0,
                labels=[
                    f"UTC timestamp is {utc_timestamp_ns / 1000000000} s"
                ],  # Convert nanoseconds to seconds
            ),
        )
