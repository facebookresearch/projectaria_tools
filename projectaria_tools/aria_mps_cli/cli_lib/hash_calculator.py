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
from asyncio import Semaphore
from pathlib import Path
from typing import Final, final, Optional

import aiofiles
import aiofiles.os
import xxhash

from .common import Config, CustomAdapter
from .constants import ConfigKey, ConfigSection
from .runner_with_progress import RunnerWithProgress

config = Config.get()


class HashCalculator(RunnerWithProgress):
    """
    Class to calculate hash of a given input path
    """

    semaphore_: Final[Semaphore] = Semaphore(
        value=config.getint(ConfigSection.HASH, ConfigKey.CONCURRENT_HASHES)
    )

    def __init__(self, input_path: Path, debug_suffix: Optional[str] = None):
        super().__init__()
        self._input_path: Path = input_path
        self._debug_suffix: Optional[str] = debug_suffix
        self._logger: CustomAdapter = CustomAdapter(
            logging.getLogger(__name__), {"vrs": input_path}
        )

    @classmethod
    @final
    def get_key(cls, input_path: Path, debug_suffix: Optional[str]) -> str:
        """Get a unique key for this Runner instance"""
        # ignore the debug prefix
        return str(input_path)

    @final
    async def _run(self) -> str:
        """
        Calculate hash of the input file
        """
        self._logger.debug(f"Starting hash calculation for {self._input_path}")
        async with HashCalculator.semaphore_:
            x = xxhash.xxh64()
            chunk_size: int = config.getint(ConfigSection.HASH, ConfigKey.CHUNK_SIZE)
            self._total = (await aiofiles.os.stat(self._input_path)).st_size
            async with aiofiles.open(self._input_path, "rb") as f:
                while buf := await f.read(chunk_size):
                    x.update(buf)
                    self._processed += len(buf)
                    self._logger.debug(
                        f"Hashing {self.progress:.2f}% complete",
                    )

            return x.hexdigest() + (self._debug_suffix or "")
