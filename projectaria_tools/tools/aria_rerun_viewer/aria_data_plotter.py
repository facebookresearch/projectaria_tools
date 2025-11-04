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

from dataclasses import dataclass
from functools import partial
from typing import Final, List, Optional

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
    contact_microphone_label: str
    gps_label: str

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
        assert (
            self.rgb_label and self.slam_labels
        ), "RGB and SLAM labels must not be empty"
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
        if rrd_output_path:
            rr.init("AriaDataViewer", spawn=False)
            rr.save(rrd_output_path)
        else:
            rr.init("AriaDataViewer", spawn=True)

        self.config = config if config is not None else AriaDataViewerConfig()
        # A variable to cache full VIO high frequency trajectory
        self.vio_high_freq_traj_cached_full = []
        # A variable to cache full VIO trajectory
        self.vio_traj_cached_full = []
        # Scale ratio to convert plot sizes from RGB camera space to SLAM camera space (based on camera resolution ratio)

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
        if self.device_calibration is not None:
            print(
                "Warning: device_calibration is already set. Setting device_calibration again."
            )
        self.device_calibration = device_calibration
        self.sensor_labels = SensorLabels.from_device_version(
            device_calibration.get_device_version()
        )
        self.set_slam_to_rgb_plotting_ratio()
        self.update_rerun_blueprint()
        self.plot_device_extrinsics()

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
        updated_3d_view_container = rrb.Vertical(
            _3d_view_container.contents[0],  # RGB view
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
        updated_1d_view_container = rrb.Vertical(
            _1d_view_container.contents[0],  # IMU plots
            _1d_view_container.contents[1],  # mic
            contact_mic_1d_view,  # contact mic
            _1d_view_container.contents[2],  # Tabbed baro + mag
        )

        # Create final horizontal blueprint layout.
        # hide left&right, with compact timeline on bottom
        return rrb.Blueprint(
            rrb.Horizontal(
                updated_3d_view_container, _2d_view_container, updated_1d_view_container
            ),
            collapse_panels=True,
        )

    def update_rerun_blueprint(self):
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
        rr.set_time_nanos("device_time", device_timestamp_ns)
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
            indexes=[rr.TimeNanosColumn("device_time", timestamps)],
            columns=[rr.components.ScalarBatch(accel_data[:, 0])],
        )
        rr.send_columns(
            f"{label}/accl/y[m-sec2]",
            indexes=[rr.TimeNanosColumn("device_time", timestamps)],
            columns=[rr.components.ScalarBatch(accel_data[:, 1])],
        )
        rr.send_columns(
            f"{label}/accl/z[m-sec2]",
            indexes=[rr.TimeNanosColumn("device_time", timestamps)],
            columns=[rr.components.ScalarBatch(accel_data[:, 2])],
        )

        # Log gyroscope data
        rr.send_columns(
            f"{label}/gyro/x[rad-sec]",
            indexes=[rr.TimeNanosColumn("device_time", timestamps)],
            columns=[rr.components.ScalarBatch(gyro_data[:, 0])],
        )
        rr.send_columns(
            f"{label}/gyro/y[rad-sec]",
            indexes=[rr.TimeNanosColumn("device_time", timestamps)],
            columns=[rr.components.ScalarBatch(gyro_data[:, 1])],
        )
        rr.send_columns(
            f"{label}/gyro/z[rad-sec]",
            indexes=[rr.TimeNanosColumn("device_time", timestamps)],
            columns=[rr.components.ScalarBatch(gyro_data[:, 2])],
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
        rr.set_time_nanos("device_time", imu_data.capture_timestamp_ns)
        rr.log(f"{label}/accl/x[m-sec2]", rr.Scalar(imu_data.accel_msec2[0]))
        rr.log(f"{label}/accl/y[m-sec2]", rr.Scalar(imu_data.accel_msec2[1]))
        rr.log(f"{label}/accl/z[m-sec2]", rr.Scalar(imu_data.accel_msec2[2]))
        rr.log(f"{label}/gyro/x[rad-sec]", rr.Scalar(imu_data.gyro_radsec[0]))
        rr.log(f"{label}/gyro/y[rad-sec]", rr.Scalar(imu_data.gyro_radsec[1]))
        rr.log(f"{label}/gyro/z[rad-sec]", rr.Scalar(imu_data.gyro_radsec[2]))

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

        rr.set_time_nanos("device_time", magnetometer_data.capture_timestamp_ns)
        # Convert magnetometer reading from tesla (SI unit) to microtesla (µT): 1 tesla = 1e6 microtesla
        rr.log(
            f"{self.sensor_labels.magnetometer_label}/x[µT]",
            rr.Scalar(magnetometer_data.mag_tesla[0] * 1e6),
        )
        rr.log(
            f"{self.sensor_labels.magnetometer_label}/y[µT]",
            rr.Scalar(magnetometer_data.mag_tesla[1] * 1e6),
        )
        rr.log(
            f"{self.sensor_labels.magnetometer_label}/z[µT]",
            rr.Scalar(magnetometer_data.mag_tesla[2] * 1e6),
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
        rr.set_time_nanos("device_time", barometer_data.capture_timestamp_ns)
        # Convert pressure from pascal (SI unit) to kilopascal (kPa): 1 pascal = 1e-3 kilopascal
        rr.log(
            f"{self.sensor_labels.barometer_labels[0]}/pressure[kPa]",
            rr.Scalar(barometer_data.pressure * 1e-3),
        )  # Pascals converter to kPascals
        rr.log(
            f"{self.sensor_labels.barometer_labels[1]}/temperature[Celsius]",
            rr.Scalar(barometer_data.temperature),
        )  # Degree Celsius

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
                    rr.TimeNanosColumn(
                        "device_time",
                        audio_data_timestamp[:: self.config.audio_subsample_rate],
                    )
                ],
                columns=[
                    rr.components.ScalarBatch(
                        sampled_vectors[c][:: self.config.audio_subsample_rate]
                    )
                ],
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

        rr.set_time_nanos("device_time", gps_data.capture_timestamp_ns)
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
        rr.set_time_nanos(
            "device_time", int(eyegaze_data.tracking_timestamp.total_seconds() * 1e9)
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
        rr.set_time_nanos(
            "device_time", int(hand_pose_data.tracking_timestamp.total_seconds() * 1e9)
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
            rr.set_time_nanos(
                "device_time",
                int(hand_pose_data.tracking_timestamp.total_seconds() * 1e9),
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
        rr.set_time_nanos(
            "device_time",
            int(vio_high_freq_data.tracking_timestamp.total_seconds() * 1e9),
        )
        # Set and plot Aria Device for the current timestamp
        T_World_Device = vio_high_freq_data.transform_odometry_device

        # Plot VIO high-freq trajectory that are cached so far
        # TODO: Optimize VIO high-freq trajectory plotting.
        self.vio_high_freq_traj_cached_full.append(T_World_Device.translation()[0])
        rr.log(
            f"world/{self.sensor_labels.vio_high_freq_label}",
            rr.LineStrips3D(
                self.vio_high_freq_traj_cached_full,
                colors=[self.PLOT_COLORS_AND_SIZES_3D["vio_high_freq"]["color"]],
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
        rr.set_time_nanos("device_time", vio_data.capture_timestamp_ns)
        # Set and plot Aria Device for the current timestamp
        T_World_Device = (
            vio_data.transform_odometry_bodyimu @ vio_data.transform_bodyimu_device
        )
        rr.log(
            "world/device",
            ToTransform3D(
                T_World_Device,
                axis_length=self.PLOT_COLORS_AND_SIZES_3D["vio"]["device_axis_length"],
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

        # Plot VIO trajectory that are cached so far
        self.vio_traj_cached_full.append(T_World_Device.translation()[0])
        rr.log(
            f"world/{self.sensor_labels.vio_label}",
            rr.LineStrips3D(
                self.vio_traj_cached_full,
                colors=[self.PLOT_COLORS_AND_SIZES_3D["vio"]["color"]],
                radii=self.PLOT_COLORS_AND_SIZES_3D["vio"]["trajectory_radius"],
            ),
            static=False,
        )

    def plot_utc_timestamp(
        self, utc_timestamp_ns, camera_label: str, device_timestamp_ns
    ):
        rr.set_time_nanos("device_time", device_timestamp_ns)
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
