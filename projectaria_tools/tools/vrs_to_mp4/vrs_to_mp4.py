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

from projectaria_tools.tools.vrs_to_mp4.vrs_to_mp4_utils import convert_vrs_to_mp4


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vrs",
        type=str,
        required=True,
        help="path to the VRS file to be converted to a video",
    )
    parser.add_argument(
        "--stream_id",
        type=str,
        required=False,
        default="214-1",
        choices=["214-1", "1201-1", "1201-2", "1201-3", "1201-4"],
        help="stream id to convert to a video, default 214-1 (rgb stream)",
    )
    parser.add_argument(
        "--output_video",
        type=str,
        required=True,
        help="path to the VIDEO file you want to create",
    )
    parser.add_argument(
        "--log_folder",
        type=str,
        required=False,
        help="Folder to store the log: mp4_to_vrs_time_map.csv, audio_log.json, audio.wav",
    )
    parser.add_argument(
        "--downsample",
        type=int,
        required=False,
        default=1,
        help="Downsampling factor on VRS images (Must be >=1)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    convert_vrs_to_mp4(
        args.vrs,
        args.output_video,
        stream_id=args.stream_id,
        log_folder=args.log_folder,
        down_sample_factor=args.downsample,
    )


if __name__ == "__main__":
    main()
