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

import argparse

from typing import List

# Import different modules for internal and external
try:
    from projectaria_tools_internal import data_provider
except ImportError:
    from projectaria_tools.core import data_provider

import rerun as rr
from projectaria_tools.core.calibration import DeviceVersion
from projectaria_tools.core.sensor_data import SensorDataType, TimeDomain, TimeSyncMode
from projectaria_tools.tools.aria_rerun_viewer.aria_data_plotter import (
    AriaDataViewer,
    AriaDataViewerConfig,
)
from tqdm import tqdm

ALL_STREAM_LABELS_GEN2 = [
    "camera-rgb",
    "slam-front-left",
    "slam-front-right",
    "slam-side-left",
    "slam-side-right",
    "camera-et-left",
    "camera-et-right",
    "imu-left",
    "imu-right",
    "mic",
    "baro0",
    "mag0",
    "gps",
    "handtracking",
    "eyegaze",
    "vio",
    "vio_high_frequency",
]
ALL_STREAM_LABELS_GEN1 = [
    "camera-slam-left",
    "camera-slam-right",
    "camera-et",
    "camera-rgb",
    "imu-left",
    "imu-right",
    "mic",
    "baro0",
    "mag0",
    "gps",
]


def parse_subsample_rates(subsampling_args: List[str], enabled_streams: List[str]):
    """
    This is a helper function to parse CLI input of subsample rates for each stream

    """
    subsample_rates = {}
    for substring in subsampling_args:
        # Parse each substring, which should look like "camera-rgb=2"
        try:
            stream_label, rate = substring.split("=")
            if stream_label not in enabled_streams:
                raise ValueError(
                    f"Invalid stream label for setting subsample rate: {stream_label}. You need to enable this stream by setting --enabled-streams."
                )
            subsample_rates[stream_label] = int(rate)
        except ValueError as e:
            raise argparse.ArgumentTypeError(
                f"Invalid format for subsampling rate: {substring}. Expected format: stream=rate"
            ) from e
    return subsample_rates


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vrs",
        type=str,
        required=True,
        help="path to a VRS file",
    )
    parser.add_argument(
        "--skip-begin-sec",
        type=float,
        required=False,
        help="seconds to skip at the beginning of VRS file",
    )
    parser.add_argument(
        "--skip-end-sec",
        type=float,
        required=False,
        help="seconds to skip at the end of VRS file",
    )
    parser.add_argument(
        "--enabled-streams",
        nargs="*",
        choices=ALL_STREAM_LABELS_GEN2 + ALL_STREAM_LABELS_GEN1,
        help="Enable specific viewers by their labels, e.g. `camera-rgb eyegaze`, Use space-separated pairs. (default: all)",
    )
    parser.add_argument(
        "--subsample-rates",
        nargs="+",
        help="Specify subsampling rates for streams in the format `stream=rate`, e.g. `camera-rgb=2`,"
        "Use space-separated pairs. (default: vio_high_frequency=10 to bring down rate from 800Hz to 80Hz)",
    )
    parser.add_argument(
        "--rrd-output-path",
        type=str,
        default="",
        help="path to save .rrd file (if not provided, will spawn viewer window)",
    )
    return parser.parse_args()


def get_deliver_option(
    vrs_data_provider,
    enabled_stream_labels: List[str] = None,
    subsample_rates: dict = None,
    viewer_config: AriaDataViewerConfig = None,
    skip_begin_sec: float = None,
    skip_end_sec: float = None,
):
    """
    Get configured deliver options for VRS data provider.

    Args:
        vrs_data_provider: VRS data provider instance
        enabled_stream_labels: List of stream labels to enable (defaults to ALL_STREAM_LABELS)
        subsample_rates: Dictionary of stream labels to subsample rates
        viewer_config: AriaDataViewerConfig instance (used to get DEFAULT_VIO_HIGH_FREQ_SUBSAMPLE_RATE)
        skip_begin_sec: Seconds to skip at the beginning of VRS file
        skip_end_sec: Seconds to skip at the end of VRS file

    Returns:
        Configured deliver options object
    """
    # Use defaults if not provided
    if subsample_rates is None:
        subsample_rates = {}

    # For VIO High Frequency stream, by default set a lower sub_sampling rate from 800Hz -> 10Hz
    if "vio_high_frequency" not in subsample_rates:
        subsample_rates["vio_high_frequency"] = (
            viewer_config.vio_high_freq_subsample_rate
        )

    # Get default deliver options and deactivate all streams
    deliver_options = vrs_data_provider.get_default_deliver_queued_options()
    deliver_options.deactivate_stream_all()

    # Activate selected streams and set subsample rates
    available_stream_ids = deliver_options.get_stream_ids()
    for stream_label in enabled_stream_labels:
        # For each enabled stream label, first find if it is within VRS
        maybe_stream_id = vrs_data_provider.get_stream_id_from_label(stream_label)

        # Then turn it on
        if maybe_stream_id is not None and maybe_stream_id in available_stream_ids:
            deliver_options.activate_stream(maybe_stream_id)
            print(
                f"[AriaDataPlotter]: Enabled visualization of stream {maybe_stream_id}: {stream_label}"
            )

            # Set subsample rates
            if stream_label in subsample_rates:
                deliver_options.set_subsample_rate(
                    maybe_stream_id, subsample_rates[stream_label]
                )
                print(
                    f"[AriaDataPlotter]: Setting subsample rate of stream {stream_label}: {subsample_rates[stream_label]}"
                )

    # Set crop begin and end times if specified
    if skip_begin_sec is not None:
        deliver_options.set_truncate_first_device_time_ns(int(skip_begin_sec * 1e9))
    if skip_end_sec is not None:
        deliver_options.set_truncate_last_device_time_ns(int(skip_end_sec * 1e9))

    return deliver_options


def plot_interpolated_hand_pose_in_image(
    vrs_data_provider, aria_data_viewer, timestamp_ns, camera_label, hand_pose_stream_id
):
    """
    This is a helper function to plot interpolated hand tracking in image.
    Interpolation is needed here because RGB, SLAM images are not synchronized with hand tracking data.
    """
    interpolated_hand_pose = vrs_data_provider.get_interpolated_hand_pose_data(
        stream_id=hand_pose_stream_id, timestamp_ns=timestamp_ns
    )

    if interpolated_hand_pose is not None:
        aria_data_viewer.plot_hand_pose_data_2d(
            hand_pose_data=interpolated_hand_pose, camera_label=camera_label
        )


def plot_queued_sensor_data(vrs_data_provider, deliver_options, aria_data_viewer):
    """
    Plot queued sensor data from VRS file
    """
    hand_pose_stream_id = vrs_data_provider.get_stream_id_from_label("handtracking")

    # Loop over queued sensor data
    for data in tqdm(vrs_data_provider.deliver_queued_sensor_data(deliver_options)):
        device_time_ns = data.get_time_ns(TimeDomain.DEVICE_TIME)

        # Current stream's stream_id and label
        stream_id = data.stream_id()
        label = vrs_data_provider.get_label_from_stream_id(data.stream_id())

        if data.sensor_data_type() == SensorDataType.IMAGE:
            frame = data.image_data_and_record()[0].to_numpy_array()
            aria_data_viewer.plot_image(
                frame, label, data.image_data_and_record()[1].capture_timestamp_ns
            )

            # For Gen2 recordings, plot handtracking data for slam / rgb cameras
            if (hand_pose_stream_id is not None) and (
                label in aria_data_viewer.sensor_labels.rgb_and_slam_labels
            ):
                plot_interpolated_hand_pose_in_image(
                    vrs_data_provider,
                    aria_data_viewer,
                    device_time_ns,
                    label,
                    hand_pose_stream_id,
                )

            # For Gen2 recordings, print UTC timestamp in RGB camera view
            if label == "camera-rgb" and vrs_data_provider.supports_time_domain(
                stream_id, TimeDomain.UTC
            ):
                aria_data_viewer.plot_utc_timestamp(
                    vrs_data_provider.convert_from_device_time_to_synctime_ns(
                        device_time_ns, TimeSyncMode.UTC
                    ),
                    label,
                    device_time_ns,
                )

        elif data.sensor_data_type() == SensorDataType.GPS:
            aria_data_viewer.plot_gps(data.gps_data())
        elif data.sensor_data_type() == SensorDataType.IMU:
            aria_data_viewer.plot_imu(data.imu_data(), label)
        elif data.sensor_data_type() == SensorDataType.MAGNETOMETER:
            aria_data_viewer.plot_magnetometer(data.magnetometer_data())
        elif data.sensor_data_type() == SensorDataType.BAROMETER:
            aria_data_viewer.plot_barometer(data.barometer_data())
        elif data.sensor_data_type() == SensorDataType.AUDIO:
            aria_data_viewer.plot_audio(
                data.audio_data_and_record(),
                vrs_data_provider.get_audio_configuration(
                    vrs_data_provider.get_stream_id_from_label("mic")
                ).num_channels,
            )
        elif data.sensor_data_type() == SensorDataType.EYE_GAZE:
            aria_data_viewer.plot_eye_gaze_data(data.eye_gaze_data())
        elif data.sensor_data_type() == SensorDataType.VIO_HIGH_FREQ:
            aria_data_viewer.plot_vio_high_freq_data(data.vio_high_freq_data())
        elif data.sensor_data_type() == SensorDataType.VIO:
            aria_data_viewer.plot_vio_data(data.vio_data())
        elif data.sensor_data_type() == SensorDataType.HAND_POSE:
            aria_data_viewer.plot_hand_pose_data_3d(data.hand_pose_data())


def main():
    args = parse_args()
    # Step 1: Create VRS data provider
    vrs_data_provider = data_provider.create_vrs_data_provider(args.vrs)
    if not vrs_data_provider:
        print(f"Failed to open {args.vrs}")
        return

    # Step 2: Extract device_calibration from vrs_data_provider
    device_calibration = vrs_data_provider.get_device_calibration()
    device_version = device_calibration.get_device_version()
    if device_version == DeviceVersion.Gen1:
        all_stream_labels = ALL_STREAM_LABELS_GEN1
    elif device_version == DeviceVersion.Gen2:
        all_stream_labels = ALL_STREAM_LABELS_GEN2
    else:
        raise ValueError(f" Unsupported Aria device version: {device_version}")

    # Step 3: Create config
    viewer_config = AriaDataViewerConfig()

    # Step 4: Get configured deliver options
    parsed_subsample_rates = (
        parse_subsample_rates(args.subsample_rates, all_stream_labels)
        if args.subsample_rates
        else {}
    )
    deliver_options = get_deliver_option(
        vrs_data_provider=vrs_data_provider,
        enabled_stream_labels=args.enabled_streams or all_stream_labels,
        subsample_rates=parsed_subsample_rates,
        viewer_config=viewer_config,
        skip_begin_sec=args.skip_begin_sec,
        skip_end_sec=args.skip_end_sec,
    )

    # Step 6: Initialize AriaDataViewer
    aria_data_viewer = AriaDataViewer(
        config=viewer_config,
        device_calibration=device_calibration,
        rrd_output_path=args.rrd_output_path,
    )
    aria_data_viewer.plot_device_extrinsics()

    # Step 6: Plot queued sensor data
    plot_queued_sensor_data(vrs_data_provider, deliver_options, aria_data_viewer)


if __name__ == "__main__":
    main()
