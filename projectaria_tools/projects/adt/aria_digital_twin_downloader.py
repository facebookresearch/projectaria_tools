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

from projectaria_tools.projects.adt import is_dataset_corrupt
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


class AriaDigitalTwinDataGroup(Enum):
    BENCHMARK = 0
    CHALLENGE = 1

    def __str__(self):
        return self.name.lower()


class AriaDigitalTwinDataType(Enum):
    MAIN_DATA = 0
    SEGMENTATIONS = 1
    DEPTH_IMAGES = 2
    SYNTHETIC = 3
    MPS_EYEGAZE = 4
    MPS_SLAM_TRAJECTORIES = 5
    MPS_SLAM_POINTS = 6
    MPS_SLAM_CALIBRATION = 7
    MPS_SLAM_SUMMARY = 8

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


class AriaDigitalTwinDownloadStatusManager:
    def __init__(self):
        self.status = {}
        self.status[str(AriaDigitalTwinDataType.MAIN_DATA)] = False
        self.status[str(AriaDigitalTwinDataType.SEGMENTATIONS)] = False
        self.status[str(AriaDigitalTwinDataType.DEPTH_IMAGES)] = False
        self.status[str(AriaDigitalTwinDataType.SYNTHETIC)] = False
        self.status[str(AriaDigitalTwinDataType.MPS_EYEGAZE)] = False
        self.status[str(AriaDigitalTwinDataType.MPS_SLAM_TRAJECTORIES)] = False
        self.status[str(AriaDigitalTwinDataType.MPS_SLAM_POINTS)] = False
        self.status[str(AriaDigitalTwinDataType.MPS_SLAM_CALIBRATION)] = False
        self.status[str(AriaDigitalTwinDataType.MPS_SLAM_SUMMARY)] = False

    def from_json(self, json_path: str):
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
                if str(AriaDigitalTwinDataType.MAIN_DATA) not in data:
                    return
                if str(AriaDigitalTwinDataType.SEGMENTATIONS) not in data:
                    return
                if str(AriaDigitalTwinDataType.DEPTH_IMAGES) not in data:
                    return
                if str(AriaDigitalTwinDataType.SYNTHETIC) not in data:
                    return
                if str(AriaDigitalTwinDataType.MPS_EYEGAZE) not in data:
                    return
                if str(AriaDigitalTwinDataType.MPS_SLAM_TRAJECTORIES) not in data:
                    return
                if str(AriaDigitalTwinDataType.MPS_SLAM_POINTS) not in data:
                    return
                if str(AriaDigitalTwinDataType.MPS_SLAM_CALIBRATION) not in data:
                    return
                if str(AriaDigitalTwinDataType.MPS_SLAM_SUMMARY) not in data:
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

    def set_download_status(self, data_type: AriaDigitalTwinDataType, value: bool):
        if data_type == AriaDigitalTwinDataType.MAIN_DATA:
            self.status[str(AriaDigitalTwinDataType.MAIN_DATA)] = value
        if data_type == AriaDigitalTwinDataType.SEGMENTATIONS:
            self.status[str(AriaDigitalTwinDataType.SEGMENTATIONS)] = value
        if data_type == AriaDigitalTwinDataType.DEPTH_IMAGES:
            self.status[str(AriaDigitalTwinDataType.DEPTH_IMAGES)] = value
        if data_type == AriaDigitalTwinDataType.SYNTHETIC:
            self.status[str(AriaDigitalTwinDataType.SYNTHETIC)] = value
        if data_type == AriaDigitalTwinDataType.MPS_EYEGAZE:
            self.status[str(AriaDigitalTwinDataType.MPS_EYEGAZE)] = value
        if data_type == AriaDigitalTwinDataType.MPS_SLAM_TRAJECTORIES:
            self.status[str(AriaDigitalTwinDataType.MPS_SLAM_TRAJECTORIES)] = value
        if data_type == AriaDigitalTwinDataType.MPS_SLAM_POINTS:
            self.status[str(AriaDigitalTwinDataType.MPS_SLAM_POINTS)] = value
        if data_type == AriaDigitalTwinDataType.MPS_SLAM_CALIBRATION:
            self.status[str(AriaDigitalTwinDataType.MPS_SLAM_CALIBRATION)] = value
        if data_type == AriaDigitalTwinDataType.MPS_SLAM_SUMMARY:
            self.status[str(AriaDigitalTwinDataType.MPS_SLAM_SUMMARY)] = value

    def get_download_status(self, data_type: AriaDigitalTwinDataType) -> bool:
        return self.status[str(data_type)]


class AriaDigitalTwinDatasetDownloader:
    #####################################################################
    # Data category for each Aria digital twin dataset
    # By default, each sequence has 3 categories:
    #   - main_data: containing data of object/device poses and 3d bounding boxes
    #   - segmentations: 2d segmentation masks of objects
    #   - depth_images: depth images of objects in 2d view of each camera
    #   - synthetic: synthetic video (i.e., digital replica) of the recording
    # For some sequences which enabled bodysuit tracking, there are 2:
    #   - segmentations_with_skeleton: 2d segmentation masks of objects and skeleton
    #   - depth_images_with_skeleton: depth images of objects and skeleton in 2d view
    #     of each camera
    # data group: benchmark vs challenge
    # data types: main_data, segmentations, segmentations_with_skeleton, depth_images,
    #    depth_images_with_skeleton, synthetic, plus MPS types
    # data category: metadata, examples, dataset (for benchmark),
    #    phase1/phase2/phase3/phase4 (for challenge)
    #####################################################################

    __KEY_METADATA: str = "metadata"
    __KEY_MAIN_DATA: str = "main_data"
    __KEY_SEGMENTATIONS: str = "segmentations"
    __KEY_DEPTH_IMAGES: str = "depth_images"
    __KEY_MPS_EYEGAZE: str = "mps_eyegaze"
    __KEY_MPS_SLAM_TRAJECTORIES: str = "mps_slam_trajectories"
    __KEY_MPS_SLAM_POINTS: str = "mps_slam_points"
    __KEY_MPS_SLAM_CALIBRATION: str = "mps_slam_calibration"
    __KEY_MPS_SLAM_SUMMARY: str = "mps_slam_summary"
    __KEY_SEGMENTATIONS_WITH_SKELETON: str = "segmentations_with_skeleton"
    __KEY_DEPTH_IMAGES_WITH_SKELETON: str = "depth_images_with_skeleton"
    __KEY_SYNTHETIC: str = "synthetic"
    __KEY_BENCHMARK: str = "benchmark"
    __KEY_CHALLENGE: str = "challenge"

    __DATA_GROUP_MAP: Dict[AriaDigitalTwinDataGroup, str] = {
        AriaDigitalTwinDataGroup.BENCHMARK: __KEY_BENCHMARK,
        AriaDigitalTwinDataGroup.CHALLENGE: __KEY_CHALLENGE,
    }

    __DATA_TYPE_MAP: Dict[AriaDigitalTwinDataType, List[str]] = {
        AriaDigitalTwinDataType.MAIN_DATA: [__KEY_MAIN_DATA],
        AriaDigitalTwinDataType.SEGMENTATIONS: [
            __KEY_SEGMENTATIONS,
            __KEY_SEGMENTATIONS_WITH_SKELETON,
        ],
        AriaDigitalTwinDataType.DEPTH_IMAGES: [
            __KEY_DEPTH_IMAGES,
            __KEY_DEPTH_IMAGES_WITH_SKELETON,
        ],
        AriaDigitalTwinDataType.SYNTHETIC: [__KEY_SYNTHETIC],
        AriaDigitalTwinDataType.MPS_EYEGAZE: [__KEY_MPS_EYEGAZE],
        AriaDigitalTwinDataType.MPS_SLAM_TRAJECTORIES: [__KEY_MPS_SLAM_TRAJECTORIES],
        AriaDigitalTwinDataType.MPS_SLAM_POINTS: [__KEY_MPS_SLAM_POINTS],
        AriaDigitalTwinDataType.MPS_SLAM_CALIBRATION: [__KEY_MPS_SLAM_CALIBRATION],
        AriaDigitalTwinDataType.MPS_SLAM_SUMMARY: [__KEY_MPS_SLAM_SUMMARY],
    }

    __KEY_URL: str = "download_url"

    def __init__(
        self,
        cdn_file: str,
        data_group: AriaDigitalTwinDataGroup,
        data_category: str,
        data_types: List[AriaDigitalTwinDataType],
        sequences: Optional[List[str]] = None,
        overwrite: bool = False,
    ):
        if cdn_file:
            with open(cdn_file, "r") as file_obj:
                self.metadata = json.load(file_obj)
        self.data_group = data_group

        self.data_category = data_category

        if self.data_category == self.__KEY_METADATA:
            # only download metadata. ingore data types and sequences
            return

        self.data_types = data_types
        self.sequences = sequences
        if sequences is None:
            self.sequences = self.__get_sequences_of_group(data_group)
        else:
            for sequence in sequences:
                if is_dataset_corrupt(sequence):
                    raise ValueError(
                        f"Sequence {sequence} has been removed from downloads list due to corrupt data"
                    )
        self.overwrite = overwrite

    def download_data(self, output_folder: str):
        if not os.path.exists(output_folder):
            print(f"Creating local output folder {output_folder}")
            os.makedirs(output_folder)

        # download metadata and update the dt list
        if self.data_category == self.__KEY_METADATA:
            data_group_key = self.__DATA_GROUP_MAP[self.data_group]
            for file_entry in self.metadata[data_group_key][self.__KEY_METADATA]:
                is_success, status_code = self.__download_data_from_url(
                    file_entry=file_entry,
                    data_type=self.__KEY_MAIN_DATA,
                    output_folder=output_folder,
                )
                if is_success:
                    print(f"{file_entry} is successfully downloaded")
                else:
                    print(
                        f"{file_entry} is not downloaded with error code {status_code}"
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

        data_group_key = self.__DATA_GROUP_MAP[self.data_group]
        if sequence not in self.metadata[data_group_key][self.data_category]:
            print(
                f"ADT sequence {sequence} is not available in the selected data group"
            )
            return False

        # check last download status
        status_manager = AriaDigitalTwinDownloadStatusManager()
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
        data_group_key = self.__DATA_GROUP_MAP[self.data_group]
        file_metadata = self.metadata[data_group_key][self.data_category][file_entry]

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

    def __get_sequences_of_group(self, group: AriaDigitalTwinDataGroup) -> List[str]:
        data_group_key = self.__DATA_GROUP_MAP[self.data_group]
        return list(self.metadata[data_group_key][self.data_category].keys())
