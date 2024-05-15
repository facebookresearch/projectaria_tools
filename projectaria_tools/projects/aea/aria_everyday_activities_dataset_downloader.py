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
import hashlib
import json
import os
import shutil
import tempfile
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from zipfile import is_zipfile, ZipFile

import requests

from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
from tqdm import tqdm

CHUCK_SIZE_BYTE = 8192
STATUS_CODE_DEFAULT = 404

DOWNLOAD_STATUS_FILE = ".download_status.json"

DATA_TYPE_TO_SAVE_PATH = {
    "mps_eyegaze": os.path.join("mps", "eye_gaze"),
    "mps_slam_trajectories": os.path.join("mps", "slam"),
    "mps_slam_points": os.path.join("mps", "slam"),
    "mps_slam_calibration": os.path.join("mps", "slam"),
    "mps_slam_summary": os.path.join("mps", "slam"),
}


class AriaEverydayActivitiesDataType(Enum):
    MAIN_DATA = 0
    MPS_EYEGAZE = 1
    MPS_SLAM_TRAJECTORIES = 2
    MPS_SLAM_POINTS = 3
    MPS_SLAM_CALIBRATION = 4
    MPS_SLAM_SUMMARY = 5

    def __str__(self):
        return self.name.lower()


def calculate_file_sha1(file_path: str) -> str:
    sha1 = hashlib.sha1()
    with open(file_path, "rb") as file:
        chunk = file.read(4096)  # Read the file in 4KB chunks
        while chunk:
            sha1.update(chunk)
            chunk = file.read(4096)

    sha1_sum = sha1.hexdigest()
    return sha1_sum


class AriaEverydayActivitiesDownloadStatusManager:
    def __init__(self):
        self.status = {}
        self.status[str(AriaEverydayActivitiesDataType.MAIN_DATA)] = False
        self.status[str(AriaEverydayActivitiesDataType.MPS_EYEGAZE)] = False
        self.status[str(AriaEverydayActivitiesDataType.MPS_SLAM_TRAJECTORIES)] = False
        self.status[str(AriaEverydayActivitiesDataType.MPS_SLAM_POINTS)] = False
        self.status[str(AriaEverydayActivitiesDataType.MPS_SLAM_CALIBRATION)] = False
        self.status[str(AriaEverydayActivitiesDataType.MPS_SLAM_SUMMARY)] = False

    # Read status file from json and assign to status
    def from_json(self, json_path: str) -> None:
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
                if str(AriaEverydayActivitiesDataType.MAIN_DATA) not in data:
                    return
                if str(AriaEverydayActivitiesDataType.MPS_EYEGAZE) not in data:
                    return
                if (
                    str(AriaEverydayActivitiesDataType.MPS_SLAM_TRAJECTORIES)
                    not in data
                ):
                    return
                if str(AriaEverydayActivitiesDataType.MPS_SLAM_POINTS) not in data:
                    return
                if str(AriaEverydayActivitiesDataType.MPS_SLAM_CALIBRATION) not in data:
                    return
                if str(AriaEverydayActivitiesDataType.MPS_SLAM_SUMMARY) not in data:
                    return
                self.status = data
        except Exception as e:
            print(f"[warning]: cannot read status json. {e}")

    def to_json(self, json_path: str):
        try:
            json_object = json.dumps(self.status, indent=4)
            with open(json_path, "w") as outfile:
                outfile.write(json_object)
        except Exception as e:
            print(f"[warning]: can not write status json. {e}")

    def set_download_status(
        self, data_type: AriaEverydayActivitiesDataType, value: bool
    ):
        if data_type == AriaEverydayActivitiesDataType.MAIN_DATA:
            self.status[str(AriaEverydayActivitiesDataType.MAIN_DATA)] = value
        if data_type == AriaEverydayActivitiesDataType.MPS_EYEGAZE:
            self.status[str(AriaEverydayActivitiesDataType.MPS_EYEGAZE)] = value
        if data_type == AriaEverydayActivitiesDataType.MPS_SLAM_TRAJECTORIES:
            self.status[str(AriaEverydayActivitiesDataType.MPS_SLAM_TRAJECTORIES)] = (
                value
            )
        if data_type == AriaEverydayActivitiesDataType.MPS_SLAM_POINTS:
            self.status[str(AriaEverydayActivitiesDataType.MPS_SLAM_POINTS)] = value
        if data_type == AriaEverydayActivitiesDataType.MPS_SLAM_CALIBRATION:
            self.status[str(AriaEverydayActivitiesDataType.MPS_SLAM_CALIBRATION)] = (
                value
            )
        if data_type == AriaEverydayActivitiesDataType.MPS_SLAM_SUMMARY:
            self.status[str(AriaEverydayActivitiesDataType.MPS_SLAM_SUMMARY)] = value

    def get_download_status(self, data_type: AriaEverydayActivitiesDataType) -> bool:
        return self.status[str(data_type)]


class AriaEverydayActivitiesDatasetDownloader:
    #####################################################################
    # By default, each sequence has 1 data type:
    #   - main_data: containing metadata, vrs, and speech data
    # data types: main_data, plus MPS types
    # Data category: there are 3 data categories that can be downloaded:
    #   - metadata: contains all the metadata of the dataset
    #   - examples: an example sequence
    #   - dataset: all data available in AEA release
    #####################################################################

    __DATA_CATEGORY_METADATA = "metadata"
    __DATA_CATEGORY_EXAMPLE = "example"

    __KEY_METADATA: str = "aria_everyday_activities_metadata"
    __KEY_MAIN_DATA: str = "main_data"
    __KEY_MPS_EYEGAZE: str = "mps_eyegaze"
    __KEY_MPS_SLAM_TRAJECTORIES: str = "mps_slam_trajectories"
    __KEY_MPS_SLAM_POINTS: str = "mps_slam_points"
    __KEY_MPS_SLAM_CALIBRATION: str = "mps_slam_calibration"
    __KEY_MPS_SLAM_SUMMARY: str = "mps_slam_summary"

    __EXAMPLE_SEQUENCE: str = "loc1_script1_seq1_rec1"

    __DATA_TYPE_MAP: Dict[AriaEverydayActivitiesDataType, List[str]] = {
        AriaEverydayActivitiesDataType.MAIN_DATA: [__KEY_MAIN_DATA],
        AriaEverydayActivitiesDataType.MPS_EYEGAZE: [__KEY_MPS_EYEGAZE],
        AriaEverydayActivitiesDataType.MPS_SLAM_TRAJECTORIES: [
            __KEY_MPS_SLAM_TRAJECTORIES
        ],
        AriaEverydayActivitiesDataType.MPS_SLAM_POINTS: [__KEY_MPS_SLAM_POINTS],
        AriaEverydayActivitiesDataType.MPS_SLAM_CALIBRATION: [
            __KEY_MPS_SLAM_CALIBRATION
        ],
        AriaEverydayActivitiesDataType.MPS_SLAM_SUMMARY: [__KEY_MPS_SLAM_SUMMARY],
    }

    __KEY_URL: str = "download_url"

    def __init__(
        self,
        cdn_file: str,
        data_category: str,
        data_types: List[AriaEverydayActivitiesDataType],
        sequences: Optional[List[str]] = None,
        overwrite: bool = False,
    ):
        if cdn_file:
            with open(cdn_file, "r") as file_obj:
                self.metadata = json.load(file_obj)

        self.data_category = data_category

        if self.data_category == self.__DATA_CATEGORY_METADATA:
            # only download metadata. ignore data types and sequences
            return

        self.data_types = data_types
        self.sequences = sequences
        if data_category == self.__DATA_CATEGORY_EXAMPLE:
            self.sequences = [self.__EXAMPLE_SEQUENCE]
        elif sequences is None:
            self.sequences = self.__get_all_sequences()
        self.overwrite = overwrite

    def download_data(self, output_folder: str):
        if not os.path.exists(output_folder):
            print(f"Creating local output folder {output_folder}")
            os.makedirs(output_folder)

        # download metadata and update the dt list
        if self.data_category == self.__DATA_CATEGORY_METADATA:
            is_success, status_code = self.__download_data_from_url(
                file_entry=self.__KEY_METADATA,
                data_type=self.__KEY_MAIN_DATA,
                output_folder=output_folder,
            )
            if is_success:
                print(f"{self.__KEY_METADATA} is successfully downloaded")
            else:
                print(
                    f"{self.__KEY_METADATA} is not downloaded with error code {status_code}"
                )
            return

        responses = {}
        # download examples or sequences
        if self.sequences is not None and self.data_types:
            for sequence in self.sequences:
                responses[sequence] = self.__download_sequence(
                    sequence=sequence,
                    output_folder=output_folder,
                )
            num_success_sequences = sum(
                1 for sequence in responses if responses[sequence]
            )
            print(
                f"{num_success_sequences} of {len(self.sequences)} sequences are successfully downloaded"
            )

    def __download_sequence(
        self,
        sequence: str,
        output_folder: str,
    ) -> bool:
        output_folder_seq = os.path.join(output_folder, sequence)
        print(f"downloading sequence {sequence} to {output_folder_seq}")

        if sequence not in self.metadata:
            print(
                f"AEA sequence {sequence} is not available in the selected data group"
            )
            return False

        # check last download status
        status_manager = AriaEverydayActivitiesDownloadStatusManager()
        status_json = os.path.join(output_folder_seq, DOWNLOAD_STATUS_FILE)
        if os.path.exists(status_json):
            status_manager.from_json(status_json)

        final_success = True
        for data_type in self.data_types:
            # skip the data which has already been downloaded if overwrite is not set
            if not self.overwrite and status_manager.get_download_status(data_type):
                print(f"skip downloading datatype {data_type} since already downloaded")
                continue
            data_type_download_is_success = True
            for data_type_sub_key in self.__DATA_TYPE_MAP[data_type]:
                is_success, status_code = self.__download_data_from_url(
                    file_entry=sequence,
                    data_type=data_type_sub_key,
                    output_folder=output_folder_seq,
                )
                if not is_success:
                    data_type_download_is_success = False
                    final_success = False
                    print(
                        f"{data_type_sub_key} is not downloaded with error code {status_code}"
                    )
            # log download status for each data_type
            status_manager.set_download_status(data_type, data_type_download_is_success)
        # write the download status to json
        status_manager.to_json(status_json)
        return final_success

    def __download_data_from_url(
        self,
        file_entry: str,
        data_type: str,
        output_folder: str,
    ) -> Tuple[bool, int]:
        status_code = STATUS_CODE_DEFAULT
        is_success = True
        file_metadata = self.metadata[file_entry]

        if data_type not in file_metadata:
            print(f"Data type {data_type} is not available for {file_entry}")
            return is_success, status_code

        network_link = file_metadata[data_type][self.__KEY_URL]
        if not network_link:
            print(f"Url for {file_entry} {data_type} is not available")
            return is_success, status_code

        try:
            with tempfile.TemporaryDirectory() as local_tmp_path:
                download_filename = file_metadata[data_type]["filename"]
                download_file_path = os.path.join(local_tmp_path, download_filename)
                sha1sum = file_metadata[data_type]["sha1sum"]
                session = requests.Session()
                # allow retry on errors of too many requests (429), internal server error (500),
                # bad gateway (502), service unavailable (503), or gateway timeout (504)

                retries = Retry(
                    total=5,
                    backoff_factor=2,  # final try is 2*(2**5) = 64 seconds
                    status_forcelist=[429, 500, 502, 503, 504],
                )

                session.mount("https://", HTTPAdapter(max_retries=retries))

                with session.get(network_link, stream=True, timeout=10) as r:
                    # r.raise_for_status()
                    total_size_in_bytes = int(r.headers.get("content-length", 0))

                    _, _, free_disk_in_bytes = shutil.disk_usage("/")
                    if free_disk_in_bytes < total_size_in_bytes:
                        raise Exception(
                            f"Insufficient disk space for {file_entry} {data_type}. Required \
                            {total_size_in_bytes} bytes, available {free_disk_in_bytes} bytes"
                        )
                    with open(download_file_path, "wb") as f:
                        print(f"Serializing to {download_file_path}")
                        progress_bar = tqdm(
                            total=total_size_in_bytes, unit="iB", unit_scale=True
                        )
                        for chunk in r.iter_content(chunk_size=CHUCK_SIZE_BYTE):
                            # serialize chunk of 8KB
                            progress_bar.update(len(chunk))
                            f.write(chunk)
                        progress_bar.close()
                    status_code = r.status_code
                    # raise exception if download fails and skip the unzip part below
                    r.raise_for_status()

                calculated_checksum = calculate_file_sha1(download_file_path)
                if sha1sum != calculated_checksum:
                    raise Exception(
                        f"different checksum value, validation failed. Calculated checksum: {calculated_checksum}, expected checksum: {sha1sum}"
                    )

                if is_zipfile(download_file_path):
                    # unzip and reorganize
                    with ZipFile(download_file_path) as zip_ref:
                        unzipped_top_dir_name = file_entry
                        unzipped_dir = os.path.join(
                            local_tmp_path, unzipped_top_dir_name
                        )
                        if not os.path.exists(unzipped_dir):
                            os.makedirs(unzipped_dir)

                        # MOVE MPS FILES
                        if data_type in DATA_TYPE_TO_SAVE_PATH.keys():
                            zip_ref.extractall(output_folder)
                            for local_path in zip_ref.namelist():
                                abs_path = os.path.join(output_folder, local_path)
                                if os.path.isfile(abs_path):
                                    move_filename = os.path.basename(abs_path)
                                    move_path = os.path.join(
                                        Path(abs_path).parent,
                                        DATA_TYPE_TO_SAVE_PATH[data_type],
                                    )
                                    os.makedirs(move_path, exist_ok=True)
                                    shutil.move(
                                        abs_path, os.path.join(move_path, move_filename)
                                    )
                        else:
                            zip_ref.extractall(output_folder)
                else:
                    shutil.copyfile(
                        download_file_path,
                        os.path.join(output_folder, download_filename),
                    )
        except Exception as e:
            # Handle request exceptions (e.g., ConnectionError, TooManyRedirects, etc.)
            print(f"An error occurred: {e}. Status code: {status_code}")
            is_success = False

        return is_success, status_code

    def __get_all_sequences(self) -> List[str]:
        sequences_list = list(self.metadata.keys())
        if self.__KEY_METADATA in sequences_list:
            sequences_list.remove(self.__KEY_METADATA)
        return sequences_list


def parse_args():
    #####################################################################
    # :cdn_file: input file listing the CDN urls, downloaded from AEA website
    # :output_folder: output folder for storing the downloaded dataset locally
    # :metadata_only: download metadata. If set, the other arguments about
    #    sequences will be ignored
    # :example_only: download only example. If set, the other arguments about sequence
    #    names will be ignored
    # :data_types: list of data types for downloading.
    #    0: main data, including sequence metadata, VRS, and speech data, < 5GB per sequence
    #    1: MPS Eyegaze. < 1MB per sequence
    #    2: MPS SLAM trajectories. < 100MB per sequence
    #    3: MPS SLAM semidense points and observations. 500MB~1GB per sequence
    #    4: MPS SLAM online calibration. < 50MB per sequence
    # :sequence_names: a list of AEA sequence names (separated by space) to be downloaded,
    #    If not set, all sequences will be downloaded
    #####################################################################
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        "-c",
        "--cdn_file",
        dest="cdn_file",
        type=str,
        required=True,
        default=None,
        help="input file listing the CDN urls, downloaded from AEA website",
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
        0: main data, including sequence metadata, VRS, and speech data, < 5GB per sequence
        1: MPS Eyegaze. < 1MB per sequence
        2: MPS SLAM trajectories. < 100MB per sequence
        3: MPS SLAM semidense points and observations. 500MB~1GB per sequence
        4: MPS SLAM online calibration. < 50MB per sequence
        """,
    )

    parser.add_argument(
        "-l",
        "--sequence_names",
        dest="sequence_names",
        nargs="+",
        required=False,
        help="a list of AEA sequence names (separated by space) to be downloaded. If not set, all sequences will be downloaded",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    data_types = []
    for dt_str in args.data_types:
        data_types.append(AriaEverydayActivitiesDataType(int(dt_str)))
    if not args.metadata_only and not data_types:
        print("-d(, --data_types) must be specified for sequence or example")
        exit(1)

    # add slam summary if any slam data is requested
    for data_type in data_types:
        if (
            data_type == AriaEverydayActivitiesDataType.MPS_SLAM_CALIBRATION
            or data_type == AriaEverydayActivitiesDataType.MPS_SLAM_TRAJECTORIES
            or data_type == AriaEverydayActivitiesDataType.MPS_SLAM_POINTS
        ):
            data_types.append(
                AriaEverydayActivitiesDataType(
                    AriaEverydayActivitiesDataType.MPS_SLAM_SUMMARY
                )
            )
            break

    data_category = None
    if args.metadata_only:
        data_category = "metadata"
    elif args.example_only:
        data_category = "example"
        args.sequence_names = None
    else:
        data_category = "dataset"
        if args.sequence_names is None:
            download_all = (
                input(
                    f"""
                    -l(, --sequence_names) is not specified.
                    You are downloading the whole dataset based on the --data_types you specified {[d.value for d in data_types]}
                    Do you want to download all sequences? [y/N]
                    """
                ).lower()
                == "y"
            )
            if not download_all:
                print("Please rerun the command with -l(, --sequence_names) specified.")
                exit(1)

    downloader = AriaEverydayActivitiesDatasetDownloader(
        cdn_file=args.cdn_file,
        data_category=data_category,
        data_types=data_types,
        sequences=args.sequence_names,
        overwrite=args.overwrite,
    )

    downloader.download_data(output_folder=args.output_folder)


if __name__ == "__main__":
    main()
