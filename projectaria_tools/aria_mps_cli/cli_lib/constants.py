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
ENCRYPTION_KEY_FILE: Final[Path] = CONFIG_DIR.joinpath(".encryption_key")

# GraphQL keys
KEY_ACCESS_TOKEN: Final[str] = "access_token"
KEY_ALIAS: Final[str] = "alias"
KEY_APP_ID: Final[str] = "app_id"
KEY_ARIA_MPS_REQUEST: Final[str] = "aria_mps_request"
KEY_CDN_URL: Final[str] = "cdn_url"
KEY_CONTACT_POINT: Final[str] = "contact_point"
KEY_CREATE_TOKEN: Final[str] = "create_token"
KEY_CREATE: Final[str] = "create"
KEY_CREATION_TIME: Final[str] = "creation_time"
KEY_CURSOR: Final[str] = "cursor"
KEY_DATA: Final[str] = "data"
KEY_DOC_ID: Final[str] = "doc_id"
KEY_END_CURSOR: Final[str] = "end_cursor"
KEY_ERROR_CODE: Final[str] = "error_code"
KEY_FEATURE: Final[str] = "feature"
KEY_FEATURES: Final[str] = "features"
KEY_FILE_HASH: Final[str] = "file_hash"
KEY_FILE_HASHES: Final[str] = "file_hashes"
KEY_HAS_NEXT_PAGE: Final[str] = "has_next_page"
KEY_ID: Final[str] = "id"
KEY_INPUT: Final[str] = "input"
KEY_KEY_ID: Final[str] = "key_id"
KEY_ME: Final[str] = "me"
KEY_MPS_RESULTS: Final[str] = "mps_results"
KEY_NAME: Final[str] = "name"
KEY_NODE: Final[str] = "node"
KEY_NODES: Final[str] = "nodes"
KEY_PAGE_INFO: Final[str] = "page_info"
KEY_PAGE_SIZE: Final[str] = "page_size"
KEY_PASSWORD: Final[str] = "password"
KEY_PROFILE_TOKENS: Final[str] = "profile_tokens"
KEY_PUBLIC_KEY: Final[str] = "public_key"
KEY_RECORDING_HASH: Final[str] = "recording_hash"
KEY_RECORDING_NAME: Final[str] = "recording_name"
KEY_RECORDINGS: Final[str] = "recordings"
KEY_REMAINING_TTL: Final[str] = "remaining_ttl"
KEY_REQUEST_ID: Final[str] = "request_id"
KEY_REQUESTS: Final[str] = "requests"
KEY_RESPONSE: Final[str] = "response"
KEY_RESULT_TYPE: Final[str] = "result_type"
KEY_SOURCE: Final[str] = "source"
KEY_PERSIST_ON_FAILURE: Final[str] = "persist_on_failure"
KEY_STATUS_MESSAGE: Final[str] = "status_message"
KEY_STATUS: Final[str] = "status"
KEY_VARIABLES: Final[str] = "variables"

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
    # Past outputs check failed
    PAST_OUTPUT_CHECK_FAILURE: int = 110
    # Past requests check failed
    PAST_REQUEST_CHECK_FAILURE: int = 111
    # Past recording check failed
    PAST_RECORDING_CHECK_FAILURE: int = 112
    # Upload failed
    UPLOAD_FAILURE: int = 113
    # Submit failed
    SUBMIT_FAILURE: int = 114
    # Processing failed
    PROCESSING_FAILURE: int = 115
    # Download failed
    DOWNLOAD_FAILURE: int = 116
