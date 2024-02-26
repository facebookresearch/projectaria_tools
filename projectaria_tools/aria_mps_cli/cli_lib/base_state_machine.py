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
from pathlib import Path
from typing import List, Mapping

from transitions.extensions.asyncio import AsyncMachine

from .types import ModelState, MpsFeature


class BaseStateMachine(AsyncMachine):
    """
    Base state machine for MPS. This class provides some common functionality that can
    be used across all the single sequence based state machines
    """

    def __init__(
        self,
        *args,
        **kwargs,
    ):
        super().__init__(
            *args,
            model=None,
            send_event=True,
            auto_transitions=False,
            queued="model",
            on_exception="on_exception",
            **kwargs,
        )
        self._tasks: List[asyncio.Task] = []

    @property
    def tasks(self):
        """
        Return list of tasks (usually one per model)
        """
        return self._tasks

    def fetch_current_model_states(
        self,
    ) -> Mapping[Path, Mapping[MpsFeature, ModelState]]:
        """
        Get the current state of each model
        """
        raise NotImplementedError()
