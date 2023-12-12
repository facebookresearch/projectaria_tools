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

from projectaria_tools.core import vrs_health_check as vhc


def update_settings(args, settings):
    if args.max_imu_skip_us:
        settings.max_imu_skip_us = args.max_imu_skip_us
    if args.max_frame_drop_us:
        settings.max_frame_drop_us = args.max_frame_drop_us
    if args.physical_accel_threshold:
        settings.physical_accel_threshold = args.physical_accel_threshold
    if args.max_non_physical_accel:
        settings.max_non_physical_accel = args.max_non_physical_accel
    if args.max_allowed_rotation_accel_rad_per_s2:
        settings.max_allowed_rotation_accel_rad_per_s2 = (
            args.max_allowed_rotation_accel_rad_per_s2
        )
    if args.default_imu_period_us:
        settings.default_imu_period_us = args.default_imu_period_us
    if args.min_imu_score:
        settings.min_imu_score = args.min_imu_score
    if args.min_baro_score:
        settings.min_baro_score = args.min_baro_score
    if args.min_temp:
        settings.min_temp = args.min_temp
    if args.max_temp:
        settings.max_temp = args.max_temp
    if args.min_camera_score:
        settings.min_camera_score = args.min_camera_score
    if args.min_camera_gain:
        settings.min_camera_gain = args.min_camera_gain
    if args.max_camera_gain:
        settings.max_camera_gain = args.max_camera_gain
    if args.min_camera_exposure_ms:
        settings.min_camera_exposure_ms = args.min_camera_exposure_ms
    if args.max_camera_exposure_ms:
        settings.max_camera_exposure_ms = args.max_camera_exposure_ms
    if args.min_time_domain_mapping_score:
        settings.min_time_domain_mapping_score = args.min_time_domain_mapping_score
    if args.min_audio_score:
        settings.min_audio_score = args.min_audio_score
    if args.min_alignment_score:
        settings.min_alignment_score = args.min_alignment_score
    if args.min_gps_accuracy:
        settings.min_gps_accuracy = args.min_gps_accuracy
    if args.default_gps_rate_hz:
        settings.default_gps_rate_hz = args.default_gps_rate_hz

    settings.ignore_gps = args.ignore_gps
    settings.ignore_audio = args.ignore_audio
    settings.ignore_bluetooth = args.ignore_bluetooth
    settings.is_interactive = False


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--path",
        type=str,
        help="Path to vrs file.",
    )
    parser.add_argument(
        "--json-out",
        default="",
        type=str,
        help="Export stats as JSON to console or file.",
    )
    parser.add_argument(
        "--dropped-frames-out",
        default="",
        type=str,
        help="Export detailed stats on dropped frames to file.",
    )
    parser.add_argument(
        "--print-stats",
        action="store_true",
        default=False,
        help="Print stats at the end.",
    )
    parser.add_argument(
        "--disable-logging",
        action="store_true",
        default=False,
        help="Disable logging.",
    )
    # add arguments for settings
    parser.add_argument(
        "--max-imu-skip-us",
        type=float,
        help="Max allowed gap in the IMU stream.",
    )
    parser.add_argument(
        "--max-frame-drop-us",
        type=int,
        help="Max sequential frame drop duration in microseconds.",
    )
    parser.add_argument(
        "--physical-accel-threshold",
        type=float,
        help="Max change in acceleration between consecutive IMU measurements.",
    )
    parser.add_argument(
        "--max-non-physical-accel",
        type=float,
        help="Max proportion [0->1) of non-physical IMU acceleration measurements allowed.",
    )

    parser.add_argument(
        "--max-allowed-rotation-accel-rad-per-s2",
        type=float,
        help="Maximum allowed rotation acceleration in rad per second square.",
    )

    parser.add_argument(
        "--default-imu-period-us",
        type=float,
        help="IMU period to use if not set in the stream.",
    )

    parser.add_argument(
        "--min-imu-score",
        type=float,
        help="Score is 100 * (1 - (dropped + bad) / expected).",
    )

    parser.add_argument(
        "--min-baro-score",
        type=float,
        help="Score is 100 * (1 - (dropped + bad) / expected).",
    )
    parser.add_argument(
        "--min-temp",
        type=float,
        help="Minimum temperature in celsius",
    )
    parser.add_argument(
        "--max-temp",
        type=float,
        help="Maximum temperature in celsius.",
    )
    parser.add_argument(
        "--min-camera-score",
        type=float,
        help="Score is 100 * (1 - (dropped + bad) / expected).",
    )
    parser.add_argument(
        "--min-camera-gain",
        type=float,
        help="Minimum camera gain",
    )
    parser.add_argument(
        "--max-camera-gain",
        type=float,
        help="Maximum camera gain.",
    )
    parser.add_argument(
        "--min-camera-exposure-ms",
        type=float,
        help="Minimum camera exposure in micro seconds.",
    )
    parser.add_argument(
        "--max-camera-exposure-ms",
        type=float,
        help="Maximum camera exposure in micro seconds.",
    )
    parser.add_argument(
        "--min-time-domain-mapping-score",
        type=float,
        help="Score is 100 * (1 - (dropped + bad) / expected).",
    )
    parser.add_argument(
        "--min-audio-score",
        type=float,
        help="Score is 100 * (1 - (dropped + bad) / expected).",
    )
    parser.add_argument(
        "--min-alignment-score",
        type=float,
        help="Score is 100 * (1 - (dropped + bad) / expected).",
    )
    parser.add_argument(
        "--ignore-gps",
        action="store_true",
        default=False,
        help="Ignore GPS signal.",
    )
    parser.add_argument(
        "--min-gps-accuracy",
        type=float,
        help="Minimum required GPS accuracy in meters.",
    )
    parser.add_argument(
        "--default-gps-rate-hz",
        type=float,
        help="Default GPS sample rate in Hz.",
    )
    parser.add_argument(
        "--ignore-audio",
        action="store_true",
        default=False,
        help="Ignore audio signal.",
    )
    parser.add_argument(
        "--ignore-bluetooth",
        action="store_true",
        default=False,
        help="Ignore bluetooth signal.",
    )

    args = parser.parse_args()
    settings = vhc.Settings()
    update_settings(args, settings)
    return settings, args


def main():
    settings, args = parse_args()
    vhc.run(
        args.path,
        args.json_out,
        settings,
        args.dropped_frames_out,
        args.print_stats,
        args.disable_logging,
    )


if __name__ == "__main__":
    main()
