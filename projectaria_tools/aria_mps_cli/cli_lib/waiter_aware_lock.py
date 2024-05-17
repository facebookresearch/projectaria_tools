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


class WaiterAwareLock(asyncio.Lock):
    """A lock that lets us check if there are waiters for the lock."""

    def __init__(self):
        super().__init__()
        self._waiters_count = 0

    @property
    def waiters_count(self):
        """Return the number of waiters for this lock."""
        return self._waiters_count

    async def acquire(self):
        """Acquire the lock."""
        self._waiters_count += 1
        try:
            await super().acquire()
        finally:
            self._waiters_count -= 1

    def release(self):
        """Release the lock."""
        super().release()

    def has_waiters(self):
        """Return True if there are waiters for the lock."""
        return self._waiters_count > 0
