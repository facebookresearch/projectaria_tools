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

import argparse
import asyncio
import logging
from pathlib import Path
from typing import Dict, List, Mapping

from .http_helper import HttpHelper
from .multi_recording_request import MultiRecordingRequest
from .request_monitor import RequestMonitor
from .single_recording_request import SingleRecordingRequest
from .types import ModelState, MpsFeature

logger = logging.getLogger(__name__)

_SINGLE_COMMAND: str = "single"
_MULTI_COMMAND: str = "multi"


class Mps:
    """MPS main class"""

    def __init__(self, http_helper: HttpHelper):
        self._http_helper: HttpHelper = http_helper
        self._features: List[MpsFeature] = []
        self._requestor = None
        self._request_monitor = None
        self._log_path: Path = Path()

    async def run(self, args: argparse.Namespace, log_path: Path) -> None:
        """
        The core MPS functionality
        """
        logger.debug("Running MPS")
        # Create MPS Request Monitor and MPS Requestor
        self._features = (
            list(args.features)
            if args.mode == _SINGLE_COMMAND
            else [MpsFeature.MULTI_SLAM]
        )
        self._log_path = log_path
        self._request_monitor = RequestMonitor(self._http_helper)
        if args.mode == _MULTI_COMMAND:
            self._requestor: MultiRecordingRequest = MultiRecordingRequest(
                monitor=self._request_monitor,
                http_helper=self._http_helper,
            )
            # Add new VRS files to be processed
            await self._requestor.add_new_recordings(
                input_paths=args.input,
                output_dir=args.output_dir,
                force=args.force,
                retry_failed=args.retry_failed,
                suffix=args.suffix,
            )
        elif args.mode == _SINGLE_COMMAND:
            self._requestor: SingleRecordingRequest = SingleRecordingRequest(
                monitor=self._request_monitor,
                http_helper=self._http_helper,
            )
            # Add new VRS files to be processed
            await self._requestor.add_new_recordings(
                input_paths=args.input,
                features=args.features,
                force=args.force,
                retry_failed=args.retry_failed,
                suffix=args.suffix,
            )
        else:
            raise ValueError(f"Unknown mode {args.mode}")

        # Wait for all the requests to be submitted
        await asyncio.gather(*self._requestor.tasks)

        # Wait for all the requests to finish
        await asyncio.gather(*self._request_monitor.tasks)

    def get_status(self) -> Mapping[Path, Dict[MpsFeature, ModelState]]:
        """
        Refresh the status of all recordings
        """
        current_states: Mapping[Path, Dict[MpsFeature, ModelState]] = {}
        if self._requestor and self._request_monitor:
            for sm in [self._requestor, self._request_monitor]:
                sm_states = sm.fetch_current_model_states()
                # Merge the states from new state machines. The state from the new state
                # machine takes precedence over the current state
                current_states = {
                    rec: {**current_states.get(rec, {}), **sm_states.get(rec, {})}
                    for rec in set(current_states) | set(sm_states)
                }

        return current_states

    @property
    def features(self):
        return self._features
