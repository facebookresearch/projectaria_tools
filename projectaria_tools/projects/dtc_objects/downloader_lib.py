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
import logging
import os
import shutil
import tempfile
from typing import Final, List, Optional, Tuple
from zipfile import is_zipfile, ZipFile

import requests
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
from tqdm import tqdm

CHUCK_SIZE_BYTE = 8192
SHA1SUM_CHUNK_SIZE = 4096
STATUS_CODE_DEFAULT = 404

DATA_TYPE_TO_SAVE_PATH = {}


logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(filename)s - %(lineno)d - %(levelname)s - %(message)s",
)


def calculate_file_sha1(file_path: str) -> str:
    """
    Calculate the SHA1 sum of a file.

    Args:
        file_path (str): Path to the file.

    Returns:
        str: SHA1 sum of the file.
    """
    sha1 = hashlib.sha1()
    with open(file_path, "rb") as file:
        chunk = file.read(SHA1SUM_CHUNK_SIZE)  # Read the file in 4KB chunks
        while chunk:
            sha1.update(chunk)
            chunk = file.read(SHA1SUM_CHUNK_SIZE)

    sha1_sum = sha1.hexdigest()
    return sha1_sum


class DigitalTwinCatalogObjectsDownloader:
    """
    Class responsible for downloading digital twin catalog objects from a CDN.
    """

    # Constants for CDN data keys
    __KEY_3D_ASSET_GLB: Final[str] = "3d-asset_glb"
    __KEY_LICENSE: Final[str] = "license"
    __KEY_METADATA: Final[str] = "metadata"
    __KEY_PREVIEWS_PREFIX: Final[str] = "previews_"

    # Constants for file metadata keys
    __KEY_URL: Final[str] = "download_url"
    __KEY_RELEASES: Final[str] = "releases"
    __KEY_OBJECTS: Final[str] = "objects"
    __KEY_FILENAME: Final[str] = "filename"
    __KEY_SHA1SUM: Final[str] = "sha1sum"

    def __init__(
        self,
        cdn_file: str,
        output_folder: str,
        file_keys: Optional[List[str]] = None,
        file_key_prefixes: Optional[List[str]] = None,
        objects: Optional[List[str]] = None,
        releases: Optional[List[str]] = None,
    ):
        """
        Initializes the downloader with CDN data and optional filters.

        Args:
            cdn_file (str): Path to the CDN data file
            output_folder (str): Output folder for downloaded data
            file_keys (Optional[List[str]]): List of file keys to download (optional)
            file_key_prefixes (Optional[List[str]]): List of file key prefixes to download (optional)
            objects (Optional[List[str]]): List of objects to download (optional)
            releases (Optional[List[str]]): List of releases to download (optional)
        """
        # Load CDN data from file
        if cdn_file:
            with open(cdn_file, "r") as file_obj:
                self.cdn_data = json.load(file_obj)

        # Set output folder
        self.output_folder = output_folder

        # Set filters for objects and releases
        self.releases = releases or list(self.cdn_data[self.__KEY_RELEASES].keys())
        self.objects = objects
        if not self.objects:
            self.objects = []
            for release in self.releases:
                self.objects.extend(
                    list(
                        self.cdn_data[self.__KEY_RELEASES][release][
                            self.__KEY_OBJECTS
                        ].keys()
                    )
                )
        self.file_keys = file_keys
        self.file_key_prefixes = file_key_prefixes

        # If no file keys or prefixes are specified, download all data types
        if not self.file_keys and not self.file_key_prefixes:
            self.file_keys = [
                self.__KEY_3D_ASSET_GLB,
                self.__KEY_LICENSE,
                self.__KEY_METADATA,
            ]
            self.file_key_prefixes = [self.__KEY_PREVIEWS_PREFIX]

    def get_object_cdn_entry(self, release: str, dtc_object: str) -> dict:
        """
        Returns the CDN data entry for a specific object in a release.

        Args:
            release (str): Release containing the object
            dtc_object (str): Object to get the CDN data entry for

        Returns:
            dict: CDN data entry for the object
        """
        release_cdn_data = self.cdn_data[self.__KEY_RELEASES][release]
        cdn_data_entry = release_cdn_data[self.__KEY_OBJECTS][dtc_object]
        return cdn_data_entry

    def download_data(self):
        """
        Downloads the filtered data to the specified output folder.
        """
        # Create output folder if it doesn't exist
        if not os.path.exists(self.output_folder):
            logger.info(f"Creating local output folder {self.output_folder}")
            os.makedirs(self.output_folder)

        responses = {}

        # If no file keys are specified, skip download
        if not self.file_keys:
            logger.info("No data types specified. Skipping download.")
            return

        # Download objects
        if self.objects is not None:
            for release in self.releases:
                for dtc_object in self.objects:
                    # Check if the object is in the release
                    if (
                        dtc_object
                        not in self.cdn_data[self.__KEY_RELEASES][release][
                            self.__KEY_OBJECTS
                        ]
                    ):
                        logger.info(
                            f"Object {dtc_object} is not available in release {release}"
                        )
                        continue
                    responses[dtc_object] = self.__download_object(
                        release=release,
                        dtc_object=dtc_object,
                    )
            num_success_objects = sum(
                1 for dtc_object in responses if responses[dtc_object]
            )
            logger.info(
                f"{num_success_objects} of {len(self.objects)} objects are successfully downloaded"
            )
        else:
            logger.info("No objects specified. Skipping download.")

    def __download_object(
        self,
        release: str,
        dtc_object: str,
    ) -> bool:
        """
        Downloads a single object to the specified output folder.

        Args:
            release (str): Release containing the object to download
            dtc_object (str): Object to download

        Returns:
            [bool] True if download was successful, False otherwise
        """
        object_output_folder = os.path.join(self.output_folder, dtc_object)
        if not os.path.exists(object_output_folder):
            logger.info(f"Creating local output folder {object_output_folder}")
            os.makedirs(object_output_folder)
        logger.info(f"downloading dtc_object {dtc_object} to {object_output_folder}")

        final_success = True
        cdn_data_entry = self.get_object_cdn_entry(release, dtc_object)
        file_keys = self.file_keys
        for prefix in self.file_key_prefixes:
            file_keys.extend([key for key in cdn_data_entry if key.startswith(prefix)])

        # the licence file must always be downloaded
        if self.__KEY_LICENSE not in file_keys:
            file_keys.append(self.__KEY_LICENSE)
        for file_key in self.file_keys:
            is_success, status_code = self.__download_data_from_url(
                release=release,
                dtc_object=dtc_object,
                file_key=file_key,
                object_output_folder=object_output_folder,
            )
            if not is_success:
                final_success = False
                logger.info(
                    f"{file_key} is not downloaded with error code {status_code}"
                )
        return final_success

    def __download_data_from_url(
        self,
        release: str,
        dtc_object: str,
        file_key: str,
        object_output_folder: str,
    ) -> Tuple[bool, int]:
        """
        Downloads data from a URL and saves it to a local folder.

        Args:
            dtc_object (str): The object to download data for.
            file_key (str): The key for the file to download.
            object_output_folder (str): The folder to save the downloaded data to.

        Returns:
            Tuple[bool, int]: A tuple containing a boolean indicating success and an integer status code.
        """

        # Get the CDN data entry for the specified object
        cdn_data_entry = self.get_object_cdn_entry(release, dtc_object)

        # Check if the file key is available in the CDN data entry
        if file_key not in cdn_data_entry:
            logger.info(f"Data type {file_key} is not available for {dtc_object}")
            return False, STATUS_CODE_DEFAULT

        # Get the network link for the file
        network_link = cdn_data_entry[file_key][self.__KEY_URL]

        # Check if the network link is available
        if not network_link:
            logger.info(f"Url for {dtc_object} {file_key} is not available")
            return False, STATUS_CODE_DEFAULT

        try:
            # Initialize variables
            status_code = STATUS_CODE_DEFAULT
            is_success = True

            # Create a temporary directory for downloading the file
            with tempfile.TemporaryDirectory() as local_tmp_path:
                # Get the filename and file path for the download
                download_filename = cdn_data_entry[file_key][self.__KEY_FILENAME]
                # Strip from the filename the prefix specifying the release, version, and object
                download_filename = download_filename.split(f"{dtc_object}_", 1)[-1]
                download_file_path = os.path.join(local_tmp_path, download_filename)
                # Get the SHA1 sum for the file
                sha1sum = cdn_data_entry[file_key][self.__KEY_SHA1SUM]

                # Create a requests session with retries
                session = requests.Session()
                retries = Retry(
                    total=5,
                    backoff_factor=2,  # final try is 2*(2**5) = 64 seconds
                    status_forcelist=[429, 500, 502, 503, 504],
                )
                session.mount("https://", HTTPAdapter(max_retries=retries))

                # Download the file
                with session.get(network_link, stream=True, timeout=10) as r:
                    # Get the total size of the file
                    total_size_in_bytes = int(r.headers.get("content-length", 0))

                    # Check if there is enough disk space available
                    _, _, free_disk_in_bytes = shutil.disk_usage("/")
                    if free_disk_in_bytes < total_size_in_bytes:
                        raise Exception(
                            f"Insufficient disk space for {dtc_object} {file_key}. Required \
                            {total_size_in_bytes} bytes, available {free_disk_in_bytes} bytes"
                        )

                    # Download the file in chunks
                    with open(download_file_path, "wb") as f:
                        logger.info(f"Serializing to {download_file_path}")
                        progress_bar = tqdm(
                            total=total_size_in_bytes, unit="iB", unit_scale=True
                        )
                        for chunk in r.iter_content(chunk_size=CHUCK_SIZE_BYTE):
                            # Serialize chunk of 8KB
                            progress_bar.update(len(chunk))
                            f.write(chunk)
                        progress_bar.close()

                    # Get the status code
                    status_code = r.status_code

                    # Raise an exception if the download fails
                    r.raise_for_status()

                # Calculate the SHA1 sum of the downloaded file
                calculated_checksum = calculate_file_sha1(download_file_path)

                # Check if the calculated checksum matches the expected checksum
                if sha1sum != calculated_checksum:
                    raise Exception(
                        f"different checksum value, validation failed. Calculated checksum: {calculated_checksum}, expected checksum: {sha1sum}"
                    )

                # Unzip the file if it is a ZIP file
                if is_zipfile(download_file_path):
                    # Unzip and reorganize
                    with ZipFile(download_file_path) as zip_ref:
                        unzipped_top_dir_name = dtc_object
                        unzipped_dir = os.path.join(
                            local_tmp_path, unzipped_top_dir_name
                        )
                        if not os.path.exists(unzipped_dir):
                            os.makedirs(unzipped_dir)
                        zip_ref.extractall(object_output_folder)
                else:
                    # Copy the file to the output folder
                    shutil.copyfile(
                        download_file_path,
                        os.path.join(object_output_folder, download_filename),
                    )
        except Exception as e:
            # Handle request exceptions (e.g., ConnectionError, TooManyRedirects, etc.)
            logger.info(f"An error occurred: {e}. Status code: {status_code}")
            is_success = False

        return is_success, status_code
