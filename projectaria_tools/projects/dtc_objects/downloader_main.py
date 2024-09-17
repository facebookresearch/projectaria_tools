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

from projectaria_tools.projects.dtc_objects.downloader_lib import (
    DigitalTwinCatalogObjectsDownloader,
)


def parse_args() -> argparse.Namespace:
    """
    Parse command line arguments.

    Returns:
        [argparse.Namespace] Parsed arguments
    """
    #####################################################################
    # :cdn_file: input file listing the CDN urls, downloaded from Project Aria website
    # :output_folder: output folder for storing the downloaded dataset locally
    # :objects: a list of DTC object names (separated by space) to be downloaded.
    #    If not set, all objects of the release will be downloaded
    # :file_keys: a list of DTC file keys (separated by space) to be downloaded.
    #    Options are 3d-asset_glb, license, metadata. If neither the file keys nor
    #    file key prefixes are set, all file keys of the object will be downloaded
    # :file_key_prefixes: a list of DTC file keys prefixes (separated by space) to be downloaded.
    #    Passing "previews_" downloads only the previews data. If neither the file keys
    #    nor file key prefixes are set, all file keys of the object will be downloaded
    #####################################################################

    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        "-c",
        "--cdn_file",
        dest="cdn_file",
        type=str,
        required=True,
        default=None,
        help="Input file listing the CDN urls, downloaded from Project Aria website",
    )

    parser.add_argument(
        "-o",
        "--output_folder",
        dest="output_folder",
        type=str,
        required=True,
        help="Output folder for storing the downloaded dataset locally",
    )

    parser.add_argument(
        "-l",
        "--objects",
        dest="objects",
        nargs="+",
        required=False,
        help="""
        A list of DTC object names (separated by space) to be downloaded.
        If not set, all objects of the release will be downloaded
        """,
    )

    parser.add_argument(
        "-r",
        "--releases",
        dest="releases",
        nargs="+",
        required=False,
        help="""
        A list of DTC release names (separated by space) to be downloaded.
        If not set, all of the releases will be downloaded
        """,
    )

    parser.add_argument(
        "-k",
        "--file_keys",
        dest="file_keys",
        nargs="+",
        required=False,
        help="""
        A list of DTC file keys (separated by space) to be downloaded.
        Options are 3d-asset_glb, license, metadata. If neither the file keys nor
        file key prefixes are set, all file keys of the object will be downloaded
        """,
    )

    parser.add_argument(
        "-x",
        "--file_key_prefixes",
        dest="file_key_prefixes",
        nargs="+",
        required=False,
        help="""
        A list of DTC file keys prefixes (separated by space) to be downloaded.
        Passing "previews_" downloads only the previews data. If neither the file keys
        nor file key prefixes are set, all file keys of the object will be downloaded
        """,
    )

    return parser.parse_args()


def main():
    """
    Main function to download DTC objects.
    """
    args = parse_args()

    downloader = DigitalTwinCatalogObjectsDownloader(
        cdn_file=args.cdn_file,
        output_folder=args.output_folder,
        objects=args.objects,
        file_keys=args.file_keys,
        file_key_prefixes=args.file_key_prefixes,
        releases=args.releases,
    )

    downloader.download_data()


if __name__ == "__main__":
    main()
