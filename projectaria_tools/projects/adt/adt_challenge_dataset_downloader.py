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
    # :phase: value 1~4, corresponding to the 4 phases of challenge sequences. Default is 1
    # :sequence_names: a list of ADT sequence names (separated by space) to be downloaded,
    #    If not set, all sequences of the input phase will be downloaded
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
        help="""
        download metadata. If set, --example_only, --phase, --sequence_names will be ignored
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
        If set, download again and overwrite sequences which have been downloaded before
        """,
    )

    parser.add_argument(
        "-e",
        "--example_only",
        dest="example_only",
        action="store_true",
        required=False,
        help="download only example. If set, --phase, --sequence_names will be ignored",
    )

    parser.add_argument(
        "-p",
        "--phase",
        dest="phase",
        type=int,
        choices=range(1, 5),
        default=1,
        required=False,
        help="value 1~4, corresponding to the 4 phases of challenge sequences. Default is 1",
    )

    parser.add_argument(
        "-l",
        "--sequence_names",
        dest="sequence_names",
        nargs="+",
        required=False,
        help="""
        a list of ADT sequence names (separated by space) to be downloaded.
        If not set, all sequences of the input phase will be downloaded
        """,
    )

    return parser.parse_args()


def main():
    args = parse_args()

    data_category = None
    if args.metadata_only:
        data_category = "metadata"
    elif args.example_only:
        data_category = "examples"
        args.sequence_names = None
    else:
        # data category for benchmark sequences is "dataset"
        # data category for challenge sequences is "phase1/phase2/phase3/phase4"
        data_category = f"phase{args.phase}"
        if args.sequence_names is None:
            download_all = (
                input(
                    """
                    -l(, --sequence_names) is not specified. Do you want to download all sequences? [y/N]
                    phase1 (4 sequences): ~8GB
                    phase2 (76 sequences): ~150GB
                    phase3 (8 sequences): ~16GB
                    phase4 (80 sequences): ~160GB
                    """
                ).lower()
                == "y"
            )
            if not download_all:
                print("Please rerun the command with -l(, --sequence_names) specified.")
                exit(1)

    # challenge sequence has only main_data, i.e., no segmentations/depth_images/synthetic
    data_types = [AriaDigitalTwinDataType.MAIN_DATA]
    downloader = AriaDigitalTwinDatasetDownloader(
        cdn_file=args.cdn_file,
        data_group=AriaDigitalTwinDataGroup.CHALLENGE,
        data_category=data_category,
        data_types=data_types,
        sequences=args.sequence_names,
        overwrite=args.overwrite,
    )

    downloader.download_data(output_folder=args.output_folder)


if __name__ == "__main__":
    main()
