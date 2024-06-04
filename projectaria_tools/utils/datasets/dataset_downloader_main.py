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

from projectaria_tools.utils.datasets.dataset_downloader import DatasetDownloader
from projectaria_tools.utils.datasets.dataset_downloader_utils import (
    load_data_groups_from_cdn,
    load_sequences_list_from_cdn,
)


def parse_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        "-c",
        "--cdn_file",
        dest="cdn_file",
        type=str,
        required=True,
        default=None,
        help="input file listing the CDN urls",
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
        "-d",
        "--data_types",
        dest="data_types",
        nargs="+",
        required=False,
        default=[],
        help="""
        List (space separated) of data types to download for each sequence.
        To get a list of all data types available for this dataset, leave this blank and it will list all options.
        If only one type exists, it will download it automatically
        """,
    )

    parser.add_argument(
        "-l",
        "--sequence_names",
        dest="sequence_names",
        nargs="+",
        required=False,
        help="a list of sequence names (separated by space) to be downloaded. If not set, all sequences will be downloaded",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    all_data_groups = load_data_groups_from_cdn(args.cdn_file)
    all_data_groups_list = list(all_data_groups.keys())

    # remove slam summary since this gets downloaded automatically if any slam data is requested
    all_data_groups_list.remove("mps_slam_summary")

    if len(args.data_types) == 0 and len(all_data_groups_list) > 1:
        print("-d(, --data_types) must be specified")
        print("Available data types include:")
        for i in range(len(all_data_groups_list)):
            print(f"{i}: {all_data_groups_list[i]}")
        exit(1)
    elif len(all_data_groups_list) == 1:
        args.data_types = [0]  # Download the only existing data type available

    data_types = []
    for input_data_type in args.data_types:
        data_types.append(all_data_groups_list[int(input_data_type)])

    if args.sequence_names is None:
        download_all = (
            input(
                """
                -l(, --sequence_names) is not specified.
                Do you want to download all sequences? [y/n]
                """
            ).lower()
            == "y"
        )
        if not download_all:
            print("Please rerun the command with -l(, --sequence_names) specified.")
            print(
                "Here is the list of available sequences:\n",
                load_sequences_list_from_cdn(args.cdn_file),
            )
            exit(1)
    else:
        print(f"Downloading the following sequences: {args.sequence_names}")

    downloader = DatasetDownloader(
        cdn_file=args.cdn_file,
        data_types=data_types,
        sequences=args.sequence_names,
        overwrite=args.overwrite,
    )
    downloader.download_data(output_folder=args.output_folder)


if __name__ == "__main__":
    main()
