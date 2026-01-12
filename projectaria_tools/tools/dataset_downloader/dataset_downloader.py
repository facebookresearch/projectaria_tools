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
from pathlib import Path
from typing import List, Optional, Tuple
from zipfile import is_zipfile, ZipFile

import requests
from projectaria_tools.tools.dataset_downloader.dataset_download_status_manager import (
    DatasetDownloadStatusManager,
    DOWNLOAD_STATUS_FILE,
)
from projectaria_tools.tools.dataset_downloader.dataset_downloader_utils import (
    load_data_groups_from_cdn,
)
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
from tqdm import tqdm

CHUCK_SIZE_BYTE = 8192
STATUS_CODE_DEFAULT = 404


MPS_DATA_TYPE_TO_SAVE_PATH = {
    "mps_eye_gaze": os.path.join("mps", "eye_gaze"),
    "mps_slam_trajectories": os.path.join("mps", "slam"),
    "mps_slam_points": os.path.join("mps", "slam"),
    "mps_slam_calibration": os.path.join("mps", "slam"),
    "mps_slam_summary": os.path.join("mps", "slam"),
    "mps_hand_tracking": os.path.join("mps", "hand_tracking"),
}


def calculate_file_sha1(file_path: str) -> str:
    sha1 = hashlib.sha1()
    with open(file_path, "rb") as file:
        chunk = file.read(4096)  # Read the file in 4KB chunks
        while chunk:
            sha1.update(chunk)
            chunk = file.read(4096)

    sha1_sum = sha1.hexdigest()
    return sha1_sum


class DatasetDownloader:
    __KEY_URL: str = "download_url"
    __KEY_FILENAME: str = "filename"
    __KEY_CHECKSUM: str = "sha1sum"

    def __init__(
        self,
        cdn_file: str,
        data_types: List[str],
        sequences: Optional[List[str]] = None,
        overwrite: bool = False,
    ):
        # add slam summary if any slam data is requested
        if "mps_slam_summary" not in data_types:
            if (
                "mps_slam_trajectories" in data_types
                or "mps_slam_calibration" in data_types
                or "mps_slam_points" in data_types
            ):
                data_types.append("mps_slam_summary")

        self.data_types_selected = data_types
        self.overwrite = overwrite

        # load cdn file
        with open(cdn_file, "r") as f:
            cdn_file_data = json.load(f)
            if "sequences" not in cdn_file_data:
                raise Exception("invalid cdn_file, missing key: data")
            self.sequences_data = cdn_file_data["sequences"]
            if "sequence_config" not in cdn_file_data:
                raise Exception("invalid cdn_file, missing key: sequence_config")
            self.sequence_config = cdn_file_data["sequence_config"]

        self.sequences = sequences
        if sequences is None:
            self.sequences = self.__get_all_sequences()

        self.data_types_all = load_data_groups_from_cdn(cdn_file)

    def download_data(self, output_folder: str):
        if not os.path.exists(output_folder):
            print(f"Creating local output folder {output_folder}")
            os.makedirs(output_folder)

        # download sequences
        responses = {}
        for sequence in self.sequences:
            responses[sequence] = self.__download_sequence(
                sequence=sequence,
                output_folder=output_folder,
            )
        num_success_sequences = sum(1 for sequence in responses if responses[sequence])
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

        if sequence not in self.sequences_data:
            print(f"sequence {sequence} is not available in the selected data group")
            return False

        if not os.path.exists(output_folder_seq):
            os.makedirs(output_folder_seq)

        # check last download status
        status_manager = DatasetDownloadStatusManager(self.data_types_all)
        status_json = os.path.join(output_folder_seq, DOWNLOAD_STATUS_FILE)
        if os.path.exists(status_json):
            status_manager.from_json(status_json)

        final_success = True
        for data_type in self.data_types_selected:
            # skip the data which has already been downloaded if overwrite is not set
            if not self.overwrite and status_manager.get_download_status(data_type):
                print(f"skip downloading datatype {data_type} since already downloaded")
                continue
            data_type_download_is_success = True
            is_success, status_code = self.__download_data_from_url(
                sequence=sequence,
                data_type=data_type,
                output_folder=output_folder_seq,
            )
            if not is_success:
                data_type_download_is_success = False
                final_success = False
                print(f"{data_type} is NOT downloaded with error code {status_code}")

            # log download status for each data_type
            status_manager.set_download_status(data_type, data_type_download_is_success)

        if "main_vrs" in self.sequences_data[sequence]:
            # Rename main recording according to sequence config
            main_vrs_filename = self.sequences_data[sequence]["main_vrs"][
                self.__KEY_FILENAME
            ]
            src = os.path.join(output_folder_seq, main_vrs_filename)
            if os.path.exists(src):
                dst = os.path.join(
                    output_folder_seq, self.sequence_config["main"]["recording"]
                )
                print(f"Renaming main vrs from {src} to {dst}")
                shutil.move(src, dst)

        # Rename main mps according to sequence config
        mps_path_src = os.path.join(output_folder_seq, "mps")
        if self.sequence_config["main"]["mps"] != "mps" and os.path.exists(
            mps_path_src
        ):
            mps_path_dst = os.path.join(
                output_folder_seq, self.sequence_config["main"]["mps"]
            )
            print(f"Renaming main mps folder from {mps_path_src} to {mps_path_dst}")
            shutil.move(mps_path_src, mps_path_dst)

        # write the download status to json
        status_manager.to_json(status_json)
        return final_success

    def __download_data_from_url(
        self,
        sequence: str,
        data_type: str,
        output_folder: str,
    ) -> Tuple[bool, int]:
        status_code = STATUS_CODE_DEFAULT
        is_success = True
        sequence_data = self.sequences_data[sequence]

        if data_type not in sequence_data:
            print(f"Data type {data_type} is not available for sequence: {sequence}")
            return is_success, status_code

        network_link = sequence_data[data_type][self.__KEY_URL]
        if not network_link:
            print(
                f"Url for sequence '{sequence}', data type '{data_type}' is not available"
            )
            return is_success, status_code

        try:
            with tempfile.TemporaryDirectory() as local_tmp_path:
                download_filename = sequence_data[data_type][self.__KEY_FILENAME]
                download_file_path = os.path.join(local_tmp_path, download_filename)
                sha1sum = sequence_data[data_type][self.__KEY_CHECKSUM]
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
                            f"Insufficient disk space for sequence '{sequence}' data type '{data_type}'. Required \
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
                    error = f"different checksum value for sequence '{sequence}' and data type '{data_type}'\n"
                    error += f"Calculated checksum: {calculated_checksum}, expected checksum: {sha1sum}"
                    raise Exception(error)

                if is_zipfile(download_file_path):
                    # unzip and reorganize
                    with ZipFile(download_file_path) as zip_ref:
                        unzipped_top_dir_name = sequence
                        unzipped_dir = os.path.join(
                            local_tmp_path, unzipped_top_dir_name
                        )
                        if not os.path.exists(unzipped_dir):
                            os.makedirs(unzipped_dir)

                        # MOVE MPS FILES
                        if data_type in MPS_DATA_TYPE_TO_SAVE_PATH.keys():
                            zip_ref.extractall(output_folder)
                            for local_path in zip_ref.namelist():
                                abs_path = os.path.join(output_folder, local_path)
                                if os.path.isfile(abs_path):
                                    move_filename = os.path.basename(abs_path)
                                    move_path = os.path.join(
                                        Path(abs_path).parent,
                                        MPS_DATA_TYPE_TO_SAVE_PATH[data_type],
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
        return list(self.sequences_data.keys())
