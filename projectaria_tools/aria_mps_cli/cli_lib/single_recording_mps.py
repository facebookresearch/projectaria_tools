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
from pathlib import Path
from typing import Awaitable, Callable, List, Mapping, Optional, Set, Union

from .common import log_exceptions
from .constants import DisplayStatus, ErrorCode
from .http_helper import HttpHelper
from .request_monitor import RequestMonitor, RequestMonitorModel
from .single_recording_request import SingleRecordingModel, SingleRecordingRequest
from .types import ModelState, MpsFeature, MpsRequest, MpsRequestSource

logger = logging.getLogger(__name__)


class SingleRecordingMps:
    """Manage single recording MPS request for all features"""

    http_helper_: HttpHelper = None

    def __init__(
        self,
        recording: Path,
        # TODO: T190558252: Allow directly tracking requests through SingleRecordingMps
        features: Set[MpsFeature],
        force: bool,
        retry_failed: bool,
        http_helper: HttpHelper,
        requestor: SingleRecordingRequest,
        request_monitor: RequestMonitor,
        source: MpsRequestSource,
        persist_on_failure: bool = False,
        suffix: Optional[str] = None,
        on_state_changed: Optional[
            Callable[[SingleRecordingModel, RequestMonitorModel], Awaitable[None]]
        ] = None,
    ):
        self._recording: Path = recording
        self._features: Set[MpsFeature] = features
        self._force: bool = force
        self._retry_failed: bool = retry_failed
        self._persist_on_failure: bool = persist_on_failure
        self._http_helper: HttpHelper = http_helper
        self._requestor: SingleRecordingRequest = requestor
        self._request_monitor: RequestMonitor = request_monitor
        self._source: MpsRequestSource = source
        self._suffix: Optional[str] = suffix

        async def __noop(*args, **kwargs):
            pass

        self._on_state_changed: Callable[
            [SingleRecordingModel, RequestMonitorModel], Awaitable[None]
        ] = on_state_changed or __noop

        self._model_by_feature: Mapping[
            MpsFeature, Union[SingleRecordingModel, RequestMonitorModel]
        ] = {}
        self._finish_status: Mapping[MpsFeature, ModelState] = {}
        self._running: bool = False

    @property
    def recording(self) -> Path:
        return self._recording

    @log_exceptions
    async def run(self) -> None:
        """
        Process a single VRS file through MPS
        The recording first goes through the requestor.
        The recording may get submitted to server for processing, if necessary.
        After that it will go through the request monitor.
        """
        models_to_submit: Set[
            SingleRecordingModel
        ] = await self._process_through_requestor()
        await self._submit_request_and_add_to_monitor(models_to_submit)

        #
        # Now that the requestor is done, wait for all of the monitors to finish
        #
        await self._wait_for_monitor()

        logger.info("Done processing the VRS file")

    async def _process_through_requestor(self) -> None:
        #
        # First submit the recording to the requestor. One request per feature
        #
        model_by_task: Mapping[asyncio.Task, SingleRecordingModel] = {}
        for feature in self._features:
            model = await self._requestor.add_new_recording(
                recording=self._recording,
                feature=feature,
                force=self._force,
                retry_failed=self._retry_failed,
                persist_on_failure=self._persist_on_failure,
                suffix=self._suffix,
            )
            logger.debug(f"Done adding recording {self._recording} {feature}")
            self._model_by_feature[feature] = model
            model_by_task[model.task] = model
            await self._on_state_changed(model)
            logger.debug(f"{model}")
            logger.debug(f"{model.task}")

        # Set running = True after all the models are submitted
        self._running = True

        #
        # As requestor is done with each feature, it will either SUCCEED or FAIL.
        # If it succeeds there are 3 scenarios:
        # 1. This feature request was identified as a new request and needs to be
        #    submitted.
        # 2. This feature request was identified as a past request and needs to be
        #    tracked.
        # 3. Past output exists for this feature and nothings needs to be done
        #
        models_to_submit: Set[SingleRecordingModel] = set()
        requestor_tasks: List[asyncio.Task] = [
            m.task for m in self._model_by_feature.values()
        ]
        logger.debug(f"Requestor tasks: {requestor_tasks}")
        removed_models: Set[SingleRecordingModel] = set()
        while requestor_tasks:
            done, pending = await asyncio.wait(
                requestor_tasks, return_when=asyncio.FIRST_COMPLETED
            )
            requestor_tasks = pending
            for t in done:
                model = model_by_task[t]
                if model not in removed_models:
                    self._requestor.remove_model(model)
                    removed_models.add(model)
                if model.is_SUCCESS_PAST_OUTPUT() or model.is_FAILURE():
                    # Set the state to SUCCESS or FAILURE
                    self._finish_status[model.feature] = model.get_status()
                elif model.is_SUCCESS_PAST_REQUEST():
                    logger.debug(f"Past request found for {model.feature}")
                    # Pass the request to the monitor for tracking
                    self._model_by_feature[model.feature] = (
                        self._request_monitor.track_feature_request(
                            [model.recording], model.past_feature_request
                        )
                    )
                elif model.is_SUCCESS_NEW_REQUEST():
                    # The feature request was identified as a new request and needs to be
                    # submitted
                    models_to_submit.add(model)
                else:
                    raise RuntimeError(
                        f"Unexpected state for model {model.state} recording {model.recording} feature {model.feature}"
                    )
                await self._on_state_changed(model)
        return models_to_submit

    async def _submit_request_and_add_to_monitor(
        self, models_to_submit: Set[SingleRecordingModel]
    ) -> None:
        """Submit the request and add it to the monitor"""
        # If there are features to submit, then first submit them
        logger.info(f"Recordings to submit: {models_to_submit}")
        if models_to_submit:
            if len({m.recording.fbid for m in models_to_submit}) != 1:
                raise ValueError(
                    f"Recordings to submit {models_to_submit} have different fbids"
                )
            # Submit the request
            try:
                model = next(iter(models_to_submit))
                mps_request: MpsRequest = await self._http_helper.submit_request(
                    name=self._recording.name,
                    recording_ids=[model.recording.fbid],
                    features=[m.feature for m in models_to_submit],
                    source=self._source,
                    persist_on_failure=self._persist_on_failure,
                )
                for feature, request in mps_request.features.items():
                    self._model_by_feature[feature] = (
                        self._request_monitor.track_feature_request(
                            [model.recording], request
                        )
                    )
                    await self._on_state_changed(self._model_by_feature[feature])
            except Exception as e:
                # Set the status to error and eat the exception so that other models
                # can still be processed
                logger.exception(e)
                for feature in models_to_submit:
                    self._finish_status[feature.feature] = ModelState(
                        status=DisplayStatus.ERROR,
                        error_code=str(ErrorCode.SUBMIT_FAILURE),
                    )

    async def _wait_for_monitor(self) -> None:
        """Wait for all the monitors to finish"""
        model_by_task: Mapping[asyncio.Task, RequestMonitorModel] = {}
        for model in self._model_by_feature.values():
            if isinstance(model, RequestMonitorModel):
                model_by_task[model.task] = model

        monitor_tasks: List[asyncio.Task] = list(model_by_task.keys())
        logger.debug(f"Monitor tasks: {monitor_tasks}")
        while monitor_tasks:
            done, pending = await asyncio.wait(
                monitor_tasks, return_when=asyncio.FIRST_COMPLETED
            )
            monitor_tasks = pending
            for t in done:
                model = model_by_task[t]
                logger.debug(f"Monitor for {model} is done")
                self._finish_status[model.feature] = model.get_status(self._recording)
                self._request_monitor.remove_model(model)
                self._model_by_feature.pop(model.feature)
                await self._on_state_changed(model)

    @log_exceptions
    def get_status(self, feature: MpsFeature) -> ModelState:
        """Get the status of a feature request"""
        if not self._running:
            return ModelState(status=DisplayStatus.CREATED)

        # First check the finish_status map
        if feature in self._finish_status:
            return self._finish_status[feature]

        # If not found, check the model map
        model = self._model_by_feature[feature]
        if isinstance(model, RequestMonitorModel):
            return model.get_status(self._recording)
        elif isinstance(model, SingleRecordingModel):
            return model.get_status()

        raise ValueError("Feature request status cannot be determined")
