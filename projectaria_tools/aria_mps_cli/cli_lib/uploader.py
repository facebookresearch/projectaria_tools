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

import asyncio
import io
import logging
import time
from asyncio import Semaphore
from pathlib import Path
from statistics import median
from typing import Any, Dict, final, Final, List, Optional, Tuple

import aiofiles
import aiohttp
import xxhash

from .common import Config, CustomAdapter, get_pretty_size, retry
from .constants import ConfigKey, ConfigSection, HTTP_RETRY_CODES
from .graphql_query import GraphQLQueryExecutor
from .http_helper import HttpHelper
from .runner_with_progress import RunnerWithProgress

_OFFSET: Final[str] = "offset"
_URL_UPLOAD: Final[str] = "https://rupload.facebook.com/mps_recording_upload"
_MIN_REMAINING_TTL_SECS: int = 0  # 60 * 60  # 1hr

config = Config.get()


class UploadPending(Exception):
    """
    Raised when the upload is still pending
    """

    pass


class Uploader(RunnerWithProgress):
    """
    Uploads the VRS file to the MPS server via the resumable upload service
    The upload handle is supposed to be a unique identifier for the file being uploaded.
    We use the hash of the file along with the hash of the file name. This way, if
    there are multiple copies of the same file on the same machine, they will have different
    handles. Without this upload service will throw an error when two concurrent
    uploads have the same handle from the same user.
    The uploads are resumable. We first query the upload service for the offset.
    if the file handle corresponds to an upload that was previously interrupted,
    then the upload service will return a nonzero offset. We then seek to this offset
    and start sending the remaining bytes.
    The upload is done is chunks. The chunk size is chosen so that the average time to send
    a chunk is less than _TARGET_SECS_PER_CHUNK seconds.
    """

    # Limits the number of concurrent uploads
    semaphore_: Semaphore = Semaphore(
        value=config.getint(ConfigSection.UPLOAD, ConfigKey.CONCURRENT_UPLOADS)
    )

    def __init__(
        self,
        input_path: Path,
        input_hash: str,
        http_helper: HttpHelper,
    ):
        super().__init__()
        self._http_helper: HttpHelper = http_helper
        self._input_path: Path = input_path
        self._input_hash: str = input_hash
        self._logger: CustomAdapter = CustomAdapter(
            logging.getLogger(__name__), {"vrs": str(input_path)}
        )

    @classmethod
    @final
    def get_key(cls, input_path: Path, input_hash: str, http_helper) -> str:
        """Get a unique key for this Runner instance"""
        # We only need the file hash
        return input_hash

    @final
    @retry(
        exceptions=[
            aiohttp.client_exceptions.ClientOSError,
            asyncio.TimeoutError,
            UploadPending,
        ],
        error_codes=HTTP_RETRY_CODES,
        retries=config.getint(ConfigSection.UPLOAD, ConfigKey.RETRIES),
        interval=config.getfloat(ConfigSection.UPLOAD, ConfigKey.INTERVAL),
        backoff=config.getfloat(ConfigSection.UPLOAD, ConfigKey.BACKOFF),
    )
    async def _run(self) -> int:
        """
        Upload the file to the MPS server
        """
        # Limit the number of concurrent uploads per file

        query_exec: GraphQLQueryExecutor = GraphQLQueryExecutor(self._http_helper)
        async with Uploader.semaphore_:
            rec_fbid: int = await check_if_already_uploaded(
                file_hash=self._input_hash, query_exec=query_exec
            )
            self._total: Final[int] = (await aiofiles.os.stat(self._input_path)).st_size
            if rec_fbid:
                self._logger.info("Recording already uploaded")
                self._processed = self._total
                return rec_fbid

            path_hash = xxhash.xxh64(self._input_path.name).hexdigest()
            upload_url: Final[str] = f"{_URL_UPLOAD}/{self._input_hash}_{path_hash}"
            self._logger.info(f"Uploading to {upload_url}")

            # Get the offset
            offset: int = await self._fetch_offset(upload_url)
            self._logger.info(f"Offset: {offset}, File size: {self._total}")

            if offset > self._total:
                raise ValueError(
                    f"Offset ({offset}) cannot be greater than file size ({self._total})"
                )
            if offset >= self._total:
                # If we reach here, that means that the file has been fully uploaded but the
                # previous request timed out before the response was received from the
                # server.
                raise UploadPending("Upload timed out")
            self._processed = offset
            # Start the upload
            headers = {
                "Content-Type": "application/x-www-form-urlencoded",
                # This is the total size of the original file
                "X-Entity-Length": f"{self._total}",
                "X-Entity-Type": "bin_file",
                "X-custom-is-anonymized": "False",
                "X-custom-file-hash": self._input_hash,
                "X-entity-Name": self._input_path.name,
            }
            response: Dict[str, Any] = {}
            min_chunk_size: int = config.getint(
                ConfigSection.UPLOAD, ConfigKey.MIN_CHUNK_SIZE
            )
            max_chunk_size: int = config.getint(
                ConfigSection.UPLOAD, ConfigKey.MAX_CHUNK_SIZE
            )
            smoothing_window_size: int = config.getint(
                ConfigSection.UPLOAD, ConfigKey.SMOOTHING_WINDOW_SIZE
            )
            target_chunk_upload_secs: int = config.getint(
                ConfigSection.UPLOAD, ConfigKey.TARGET_CHUNK_UPLOAD_SECS
            )
            chunk_sizes: List[int] = [min_chunk_size] * smoothing_window_size
            chunk_size = min_chunk_size
            async with aiofiles.open(self._input_path, "rb") as f:
                await f.seek(self._processed)
                while chunk := await f.read(chunk_size):
                    headers[_OFFSET] = str(self._processed)
                    # This is length of the content we plan to send in this attempt
                    # It can be less than the total file size because this could be a resumption
                    # of a previous upload
                    headers["Content-Length"] = str(len(chunk))
                    start_time: float = time.time()
                    response = await self._http_helper.post(
                        url=upload_url, data=io.BytesIO(chunk), headers=headers
                    )
                    time_taken: float = time.time() - start_time
                    chunk_sizes.append(
                        target_chunk_upload_secs * len(chunk) / time_taken
                    )
                    chunk_sizes.pop(0)
                    # Chunk size is calculated based on the smoothed speed of the last few chunks
                    # The chunk size is capped between min_chunk_size and max_chunk_size
                    chunk_size = min(
                        max_chunk_size,
                        max(min_chunk_size, int(median(chunk_sizes))),
                    )
                    self._processed += len(chunk)
                    self._logger.info(
                        f"Uploading with chunk_size {get_pretty_size(chunk_size)} | {self.progress:.2f}% complete",
                    )
            self._logger.info("Finished uploading")
            return int(response["id"])

    async def _upload(self) -> int:
        """
        Actual upload implementation
        """

    async def _fetch_offset(self, upload_url: str):
        """
        Fetches the offset for the given upload URL. If the upload has never been attempted before,
        returns zero. If the upload has been attempted before, but was interrupted, returns the
        offset to resume from.
        """
        response = await self._http_helper.get(url=upload_url)
        return response[_OFFSET]


async def check_if_already_uploaded(
    file_hash: str, query_exec: GraphQLQueryExecutor
) -> Optional[int]:
    """
    Checks if a file with the given hash has already been successfully uploaded.
    """
    recording_info: Optional[Tuple[int, int]] = (
        await query_exec.query_recording_by_file_hash(file_hash)
    )
    if recording_info is not None and recording_info[1] > _MIN_REMAINING_TTL_SECS:
        return recording_info[0]
    return None
