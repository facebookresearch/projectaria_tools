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

from abc import ABC, abstractmethod
from typing import Any


class RunnerWithProgress(ABC):
    """
    This class provides an interface to run long running functionality asynchronously
    and query its progress
    """

    def __init__(self):
        self._processed: int = 0
        self._total: int = 0
        pass

    @abstractmethod
    async def run(self) -> Any:
        pass

    @property
    def progress(self):
        return self._processed * 100 / self._total if self._total else 0
