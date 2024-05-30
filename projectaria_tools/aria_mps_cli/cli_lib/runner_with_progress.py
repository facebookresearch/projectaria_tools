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
import logging
from abc import ABC, abstractmethod
from typing import Any, Final, final, Mapping, Optional
from weakref import WeakValueDictionary

logger = logging.getLogger(__name__)


class RunnerWithProgress(ABC):
    """
    This class provides an interface to run long running functionality asynchronously
    and optionally query its progress
    """

    # We maintain weak references to Runner instances to avoid creating new ones
    # when the same input path is requested multiple times
    # Garbage collector will collect the Runner instance when it is no longer
    # referenced
    weak_instances_: Mapping[str, Any] = None
    mutex_: Final[asyncio.Lock] = None

    def __init__(self):
        self._processed: int = 0
        self._total: int = 0
        self._task: Optional[asyncio.Task] = None

    @classmethod
    async def get(cls, *args, **kwargs) -> Any:
        """
        Get the Runner instance of this class. If it doesn't exist, create a weakreference to it
        and cache it.
        """
        if cls.weak_instances_ is None:
            cls.mutex_ = asyncio.Lock()
            cls.weak_instances_ = WeakValueDictionary()
        key: str = cls.get_key(*args, **kwargs)
        async with cls.mutex_:
            if key not in cls.weak_instances_:
                logger.debug(f"Creating new {cls.__name__} for {key}")
                # This didn't work for some reason
                # cls.weak_instances_[key] = cls(input_path, debug_suffix)
                instance = cls(*args, **kwargs)
                cls.weak_instances_[key] = instance
            return cls.weak_instances_[key]

    @classmethod
    @abstractmethod
    def get_key(cls, *args, **kwargs) -> str:
        """Get a unique key for this Runner instance"""
        raise NotImplementedError()

    @final
    async def run(self) -> Any:
        """
        Run the long running functionality.
        If it is already running, await on that task instead of creating a new one
        """
        if not self._task or self._task.done() and not self._task.cancelled():
            self._task = asyncio.create_task(self._run())
        return await self._task

    @abstractmethod
    async def _run(self) -> Any:
        """
        Actual implementation of the long running functionality
        This method should be implemented by subclasses
        """
        raise NotImplementedError()

    @property
    def progress(self):
        """
        Get the progress of this Runner instance
        0.0 <= progress <= 10.0
        """
        return self._processed * 100 / self._total if self._total else 0
