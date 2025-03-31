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

import logging
import pathlib
import tempfile
from asyncio import Semaphore
from pathlib import Path
from typing import final, Final, List, Optional, Union
from urllib.parse import urlparse

import aiofiles

from .common import Config, CustomAdapter, retry, to_proc, unzip
from .constants import ConfigKey, ConfigSection, HTTP_RETRY_CODES
from .http_helper import HttpHelper
from .runner_with_progress import RunnerWithProgress

config = Config.get()


class Downloader(RunnerWithProgress):
    """
    Download the file from the given URL to the given local path and maybe unzip it, if
    needed.
    """

    semaphore_: Final[Semaphore] = Semaphore(
        value=config.getint(ConfigSection.DOWNLOAD, ConfigKey.CONCURRENT_DOWNLOADS)
    )

    def __init__(
        self,
        url_or_urls: Union[str, List[str]],
        download_dir: Path,
        http_helper: HttpHelper,
        download_filename: Optional[str] = None,
        unzip: bool = False,
    ):
        super().__init__()
        self._http_helper: HttpHelper = http_helper
        self._urls: List[str] = (
            [url_or_urls] if isinstance(url_or_urls, str) else url_or_urls
        )
        self._download_dir: Path = download_dir
        self._download_filename: str = download_filename
        self._unzip: bool = unzip

    @classmethod
    @final
    def get_key(
        cls,
        url: str,
        download_dir: Path,
        http_helper: HttpHelper,
        download_filename: Optional[str] = None,
        unzip: bool = False,
    ) -> str:
        """Get a unique key for this Runner instance"""
        return f"{url}_{download_dir}"

    @final
    @retry(
        error_codes=HTTP_RETRY_CODES,
        interval=config.getfloat(ConfigSection.DOWNLOAD, ConfigKey.INTERVAL),
        backoff=config.getfloat(ConfigSection.DOWNLOAD, ConfigKey.BACKOFF),
    )
    async def _run(self) -> None:
        self._processed = 0
        self._total = 0
        filename = None

        self._logger: CustomAdapter = CustomAdapter(
            logging.getLogger(__name__),
            {"path": self._download_dir},
        )

        async with self.semaphore_:
            with tempfile.TemporaryDirectory(dir=self._download_dir) as tmp_dir:
                tmp_dir_path = Path(tmp_dir)
                tmp_file = tmp_dir_path / "output"
                async with aiofiles.open(tmp_file, mode="wb") as f:
                    for url in self._urls:
                        self._logger.debug(f"Downloading output {url}")
                        async with self._http_helper.session.get(url) as response:
                            if not filename:
                                filename = (
                                    self._download_filename
                                    or pathlib.Path(urlparse(self._urls[0]).path).name
                                    or response.content_disposition.filename
                                )
                            self._total += response.content_length
                            chunk_size: int = config.getint(
                                ConfigSection.DOWNLOAD, ConfigKey.CHUNK_SIZE
                            )
                            self._logger.debug(
                                f"content length {response.content_length}"
                            )
                            async for chunk in response.content.iter_chunked(
                                chunk_size
                            ):
                                await f.write(chunk)
                                self._processed += len(chunk)
                                self._logger.debug(
                                    f"Downloaded {filename} : {self.progress}%"
                                )
                    self._logger.info(f"Finished downloading {filename}")
                tmp_file.rename(self._download_dir / filename)

        if self._unzip and not filename.endswith(".zip"):
            raise ValueError(f"Cannot unzip. File {filename} should be zip file")

        if self._unzip:
            zip_file = self._download_dir / filename
            dest_dir = (self._download_dir / filename).with_suffix("")
            self._logger.info(f"Unzipping {filename} to {dest_dir}...")
            await to_proc(unzip, zip_file, dest_dir)
            if config.getboolean(ConfigSection.DOWNLOAD, ConfigKey.DELETE_ZIP):
                self._logger.info(f"Deleting {filename}...")
                zip_file.unlink()
