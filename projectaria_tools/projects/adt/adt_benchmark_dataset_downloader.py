#!/usr/bin/env python3
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

from projectaria_tools.projects.adt.aria_digital_twin_downloader import (
    AriaDigitalTwinDataGroup,
    AriaDigitalTwinDatasetDownloader,
    AriaDigitalTwinDataType,
)


def parse_args():
    #####################################################################
    # :cdn_file: input file listing the CDN urls, downloaded from ADT website
    # :output_folder: output folder for storing the downloaded dataset locally
    # :metadata_only: download metadata. If set, the other arguments about
    #    sequences will be ignored
    # :example_only: download only example. If set, the other arguments about sequence
    #    names will be ignored
    # :data_types: list of data types for downloading. 0: main data; 1: segmentation;
    #    0: main data, including Aria data, 2D and 3D bounding boxes, instance information and
    #       skeleton ground-truth. Around 3~6G per sequence \n
    #    1: segmentation images. Around 2~4GB per sequence
    #    2: depth images. Around 4~8GB per sequence
    #    3: synthetic images. Around 2~4GB per sequence
    #    4: MPS Eyegaze. < 1MB per sequence
    #    5: MPS SLAM trajectories. < 100MB per sequence
    #    6: MPS SLAM semidense points and observations. 500MB~1GB per sequence
    #    7: MPS SLAM online calibration. < 50MB per sequence
    # :sequence_names: a list of ADT sequence names (separated by space) to be downloaded,
    #    If not set, all sequences will be downloaed
    #####################################################################
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        "-c",
        "--cdn_file",
        dest="cdn_file",
        type=str,
        required=True,
        default=None,
        help="input file listing the CDN urls, downloaded from ADT website",
    )

    parser.add_argument(
        "-o",
        "--output_folder",
        dest="output_folder",
        type=str,
        required=True,
        help="output folder for storing the downloaded dataset locally",
    )

    parser.add_argument(
        "-m",
        "--metadata_only",
        dest="metadata_only",
        action="store_true",
        required=False,
        default=False,
        help="""
        download metadata. If set, the  --example_only, --data_types, --sequence_names will be ignored
        """,
    )

    parser.add_argument(
        "-w",
        "--overwrite",
        dest="overwrite",
        action="store_true",
        required=False,
        default=False,
        help="""
        If set, redownload and overwrite sequences which have been downloaded before
        """,
    )

    parser.add_argument(
        "-e",
        "--example_only",
        dest="example_only",
        action="store_true",
        required=False,
        help="download only example. If set, --sequence_names will be ignored",
    )

    parser.add_argument(
        "-d",
        "--data_types",
        dest="data_types",
        nargs="+",
        required=False,
        default=[],
        help="""
        List (space separated) of data types to download for each sequence.
        0: main data, including Aria data, 2D and 3D bounding boxes,
           instance information and skeleton ground-truth. Around 3~6G per sequence
        1: segmentation images. Around 2~4GB per sequence
        2: depth images. Around 4~8GB per sequence
        3: synthetic images. Around 2~4GB per sequence
        4: MPS Eyegaze. < 1MB per sequence
        5: MPS SLAM trajectories. < 100MB per sequence
        6: MPS SLAM semidense points and observations. 500MB~1GB per sequence
        7: MPS SLAM online calibration. < 50MB per sequence
        """,
    )

    parser.add_argument(
        "-l",
        "--sequence_names",
        dest="sequence_names",
        nargs="+",
        required=False,
        help="a list of ADT sequence names (separated by space) to be downloaded. If not set, all sequences will be downloaded",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    data_types = []
    for dt_str in args.data_types:
        data_types.append(AriaDigitalTwinDataType(int(dt_str)))
    if not args.metadata_only and not data_types:
        print("-d(, --data_types) must be specified for sequence or example")
        exit(1)

    # add slam summary if any slam data is requested
    for data_type in data_types:
        if (
            data_type == AriaDigitalTwinDataType.MPS_SLAM_CALIBRATION
            or data_type == AriaDigitalTwinDataType.MPS_SLAM_TRAJECTORIES
            or data_type == AriaDigitalTwinDataType.MPS_SLAM_POINTS
        ):
            data_types.append(
                AriaDigitalTwinDataType(AriaDigitalTwinDataType.MPS_SLAM_SUMMARY)
            )
            break

    data_category = None
    if args.metadata_only:
        data_category = "metadata"
    elif args.example_only:
        data_category = "examples"
        args.sequence_names = None
    else:
        # data category for benchmark sequences is "dataset"
        # data category for challenge sequences is "phase1/phase2/phase3/phase4"
        data_category = "dataset"
        if args.sequence_names is None:
            download_all = (
                input(
                    f"""
                    -l(, --sequence_names) is not specified.
                    You are downloading the whole dataset based on the --data_types you specified {[d.value for d in data_types]}
                    0: Total main data: ~700GB
                    1: Total segmentation images: ~750GB
                    2: Total depth images: ~1.5TB
                    3: Total synthetic images: ~500GB
                    4: Total MPS Eyegaze: ~150MB
                    5: Total MPS SLAM trajectories: ~15GB
                    6: Total MPS SLAM semidense points and observations: ~140GB
                    7: Total MPS SLAM online calibration: ~5GB
                    Do you want to download all 222 sequences? [y/N]
                    """
                ).lower()
                == "y"
            )
            if not download_all:
                print("Please rerun the command with -l(, --sequence_names) specified.")
                exit(1)

    downloader = AriaDigitalTwinDatasetDownloader(
        cdn_file=args.cdn_file,
        data_group=AriaDigitalTwinDataGroup.BENCHMARK,
        data_category=data_category,
        data_types=data_types,
        sequences=args.sequence_names,
        overwrite=args.overwrite,
    )

    downloader.download_data(output_folder=args.output_folder)


if __name__ == "__main__":
    main()
