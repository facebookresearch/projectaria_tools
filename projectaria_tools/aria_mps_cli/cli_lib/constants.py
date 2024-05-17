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

from http import HTTPStatus
from pathlib import Path
from typing import Final, Sequence

CONFIG_DIR: Final[Path] = Path.home().joinpath(".projectaria")
CONFIG_FILE: Final[Path] = CONFIG_DIR.joinpath("mps.ini")
AUTH_TOKEN_FILE: Final[Path] = CONFIG_DIR.joinpath("auth_token")

# Common http status codes that should be retried
HTTP_RETRY_CODES: Final[Sequence[int]] = [
    HTTPStatus.BAD_GATEWAY,
    HTTPStatus.BAD_REQUEST,
    HTTPStatus.INTERNAL_SERVER_ERROR,
    HTTPStatus.PRECONDITION_FAILED,
    HTTPStatus.REQUEST_TIMEOUT,
    HTTPStatus.SERVICE_UNAVAILABLE,
    HTTPStatus.TOO_MANY_REQUESTS,
]


class ConfigSection:
    """
    Section of the config file
    """

    DEFAULT: str = "DEFAULT"
    UPLOAD: str = "UPLOAD"
    DOWNLOAD: str = "DOWNLOAD"
    ENCRYPTION: str = "ENCRYPTION"
    GRAPHQL: str = "GRAPHQL"
    HASH: str = "HASH"
    HEALTH_CHECK: str = "HEALTH_CHECK"


class ConfigKey:
    """
    Key of the config file
    """

    BACKOFF: str = "BACKOFF"
    CHUNK_SIZE: str = "CHUNK_SIZE"
    CONCURRENT_DOWNLOADS: str = "CONCURRENT_DOWNLOADS"
    CONCURRENT_ENCRYPTIONS: str = "CONCURRENT_ENCRYPTIONS"
    CONCURRENT_HASHES: str = "CONCURRENT_HASHES"
    CONCURRENT_UPLOADS: str = "CONCURRENT_UPLOADS"
    CONCURRENT_HEALTH_CHECKS: str = "CONCURRENT_HEALTH_CHECKS"
    DELETE_ENCRYPTED_FILES: str = "DELETE_ENCRYPTED_FILES"
    DELETE_ZIP: str = "DELETE_ZIP"
    INTERVAL: str = "INTERVAL"
    LOG_DIR: str = "LOG_DIR"
    MAX_CHUNK_SIZE: str = "MAX_CHUNK_SIZE"
    MIN_CHUNK_SIZE: str = "MIN_CHUNK_SIZE"
    RETRIES: str = "RETRIES"
    SMOOTHING_WINDOW_SIZE: str = "SMOOTHING_WINDOW_SIZE"
    STATUS_CHECK_INTERVAL: str = "STATUS_CHECK_INTERVAL"
    TARGET_CHUNK_UPLOAD_SECS: str = "TARGET_CHUNK_UPLOAD_SECS"
    UPLOAD_TIMEOUT_SECS: str = "UPLOAD_TIMEOUT_SECS"


class DisplayStatus:
    """
    Display status of the feature request in the UI.
    """

    CREATED = "Created"
    HASHING = "Hashing"
    WAITING = "Waiting"
    CHECKING = "Checking"
    DOWNLOADING = "Downloading"
    HEALTHCHECK = "HealthCheck"
    UPLOADING = "Uploading"
    ENCRYPTING = "Encrypting"
    SCHEDULED = "Scheduled"
    SUBMITTING = "Submitting"
    SUCCESS = "Success"
    ERROR = "Error"


class ErrorCode:
    """
    Error codes
    """

    # This means something else failed so the execution was stopped
    SOMETHING_ELSE_FAILURE: int = 100
    # An unknown exception was thrown while executing the state machine
    STATE_MACHINE_FAILURE: int = 101
    # Health checks for the recording failed. Could be due to missing streams, dropped data
    # or corrupted files
    HEALTH_CHECK_FAILURE: int = 102
    # Duplicate recordings were passed for Multi-SLAM
    DUPLICATE_RECORDING_FAILURE: int = 103
    # Encryption failed
    ENCRYPTION_FAILURE: int = 104
    # Multi-slam output path already contains output for a different multi-slam request
    INPUT_OUTPUT_MISMATCH_FAILURE: int = 105
    # Multi-slam output path exists but doesn't contain the expected output
    NON_EMPTY_OUTPUT_DIR_FAILURE: int = 106
    # Graphql queries failed
    GRAPHQL_FAILURE: int = 108
    # Hash computation failed
    HASH_COMPUTATION_FAILURE: int = 109
    # Existing outputs check failed
    EXISTING_OUTPUTS_CHECK_FAILURE: int = 110
    # Existing requests check failed
    EXISTING_REQUESTS_CHECK_FAILURE: int = 111
    # Existing recording check failed
    EXISTING_RECORDING_CHECK_FAILURE: int = 112
    # Upload failed
    UPLOAD_FAILURE: int = 113
    # Submit failed
    SUBMIT_FAILURE: int = 114
    # Processing failed
    PROCESSING_FAILURE: int = 115
    # Download failed
    DOWNLOAD_FAILURE: int = 116
