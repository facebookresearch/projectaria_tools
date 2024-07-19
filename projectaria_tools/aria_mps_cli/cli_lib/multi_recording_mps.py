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
from pathlib import Path
from typing import Awaitable, Callable, List, Mapping, Optional, Union

from .common import log_exceptions
from .constants import DisplayStatus, ErrorCode
from .graphql_query import GraphQLQueryExecutor
from .http_helper import HttpHelper
from .multi_recording_request import MultiRecordingModel, MultiRecordingRequest
from .request_monitor import RequestMonitor, RequestMonitorModel
from .types import ModelState, MpsFeature, MpsRequest, MpsRequestSource

logger = logging.getLogger(__name__)


class MultiRecordingMps:
    """Manage multi recording MPS request end to end"""

    http_helper_: HttpHelper = None

    def __init__(
        self,
        recordings: List[Path],
        output_dir: Path,
        force: bool,
        retry_failed: bool,
        http_helper: HttpHelper,
        requestor: MultiRecordingRequest,
        request_monitor: RequestMonitor,
        source: MpsRequestSource,
        name: Optional[str] = None,
        suffix: Optional[str] = None,
        on_state_changed: Optional[
            Callable[[MultiRecordingModel, RequestMonitorModel], Awaitable[None]]
        ] = None,
    ):
        self._recordings: List[Path] = recordings
        self._output_dir: Path = output_dir
        self._force: bool = force
        self._retry_failed: bool = retry_failed
        self._query_exec: GraphQLQueryExecutor = GraphQLQueryExecutor(http_helper)
        self._requestor: MultiRecordingRequest = requestor
        self._request_monitor: RequestMonitor = request_monitor
        self._source: MpsRequestSource = source
        self._name: Optional[str] = name
        self._suffix: Optional[str] = suffix

        async def __noop(*args, **kwargs):
            pass

        self._on_state_changed: Callable[
            [MultiRecordingModel, RequestMonitorModel], Awaitable[None]
        ] = (on_state_changed or __noop)

        self._model: Optional[Union[MultiRecordingModel, RequestMonitorModel]] = None
        self._finish_status: Mapping[Path, ModelState] = {}
        self._running: bool = False

    @property
    def recordings(self) -> Path:
        return self._recordings

    @log_exceptions
    async def run(self) -> None:
        """
        Process a single VRS file through MPS
        The recording first goes through the requestor.
        The recording may get submitted to server for processing, if necessary.
        After that it will go through the request monitor.
        """
        #
        # First submit the recording to the requestor. One request per feature
        #
        self._model = await self._requestor.add_new_recordings(
            recordings=self._recordings,
            output_dir=self._output_dir,
            force=self._force,
            retry_failed=self._retry_failed,
            name=self._name,
            suffix=self._suffix,
        )
        await self._on_state_changed(self._model)
        logger.debug(f"{self._model}")
        logger.debug(f"{self._model.task}")

        # Set running = True after all the models are submitted
        self._running = True

        await self._model.task

        #
        # As requestor is done, it will either SUCCEED or FAIL.
        # If it succeeds there are 3 scenarios:
        # 1. This feature request was identified as a new request and needs to be
        #    submitted.
        # 2. This feature request was identified as a past request and needs to be
        #    tracked.
        # 3. Past output exists for this feature and nothings needs to be done
        #
        await self._on_state_changed(self._model)
        self._requestor.remove_model(self._model)
        if self._model.is_SUCCESS_PAST_OUTPUT() or self._model.is_FAILURE():
            # Set the state to SUCCESS or FAILURE
            self._finish_status = {
                r: self._model.get_status(r) for r in self._recordings
            }
            self._model = None
        elif self._model.is_SUCCESS_PAST_REQUEST():
            logger.debug(f"Past request found for {self._model}")
            # Pass the request to the monitor for tracking
            self._model = self._request_monitor.track_feature_request(
                self._model.recordings, self._model.feature_request
            )
        elif self._model.is_SUCCESS_NEW_REQUEST():
            # The feature request was identified as a new request and needs to be
            # submitted
            try:
                request: MpsRequest = await self._query_exec.submit_request(
                    name=self._name or f"{MpsFeature.MULTI_SLAM.name} request",
                    recording_ids=[rec.fbid for rec in self._model.recordings],
                    features=[MpsFeature.MULTI_SLAM],
                    source=self._source,
                )
                self._model = self._request_monitor.track_feature_request(
                    recordings=self._model.recordings,
                    feature_request=request.features[MpsFeature.MULTI_SLAM],
                )
            except Exception as e:
                logger.exception(f"Failed to submit request {e}")
                self._finish_status = {
                    r: ModelState(
                        status=DisplayStatus.ERROR,
                        error_code=str(ErrorCode.SUBMIT_FAILURE),
                    )
                    for r in self._recordings
                }

        #
        # Now wait for the request monitor to finish if there is a request to track
        #
        if self._model:
            await self._on_state_changed(self._model)
            logger.debug(f"Waiting for {self._model}")
            await self._model.task
            self._finish_status = {
                r: self._model.get_status(r) for r in self._recordings
            }
            self._request_monitor.remove_model(self._model)
            await self._on_state_changed(self._model)

    @log_exceptions
    def get_status(self, recording: Path) -> ModelState:
        """Get the status of a feature request"""
        if not self._running:
            return ModelState(status=DisplayStatus.CREATED)

        # First check the finish_status
        if recording in self._finish_status:
            return self._finish_status[recording]

        # If not found, check the model status
        return self._model.get_status(recording)
