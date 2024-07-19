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
from enum import auto, Enum, unique
from pathlib import Path
from typing import Any, Dict, Final, List, Mapping, Optional, Sequence

from transitions.core import EventData

from .base_state_machine import BaseStateMachine
from .common import Config
from .constants import ConfigKey, ConfigSection, DisplayStatus, ErrorCode
from .downloader import Downloader
from .graphql_query import GraphQLQueryExecutor
from .http_helper import HttpHelper
from .types import (
    AriaRecording,
    GraphQLError,
    ModelState,
    MpsFeature,
    MpsFeatureRequest,
    MpsResult,
    MpsResultType,
    Status,
)

logger = logging.getLogger(__name__)

RESULT_TYPES_BY_FEATURE: Mapping[MpsFeature, List[MpsResultType]] = {
    MpsFeature.MULTI_SLAM: [MpsResultType.SLAM_ZIP],
    MpsFeature.SLAM: [MpsResultType.SLAM_ZIP],
    MpsFeature.EYE_GAZE: [MpsResultType.EYE_GAZE_ZIP],
    MpsFeature.HAND_TRACKING: [MpsResultType.HAND_TRACKING_ZIP],
}

config = Config.get()


class RequestMonitor(BaseStateMachine):
    """
    Once an MPS request has been submitted to the server, this monitors the status of
    the request and once its done the result will be downloaded on success and on
    failure the exception will be logged
    """

    @unique
    class States(Enum):
        CREATE = auto()
        WAIT = auto()
        DOWNLOAD = auto()
        SUCCESS = auto()
        FAILURE = auto()

    TRANSITIONS: Final[List[List[Any]]] = [
        ["next", "*", States.FAILURE, "has_error"],
        ["start", States.CREATE, States.WAIT],
        ["next", States.WAIT, States.DOWNLOAD],
        ["next", States.DOWNLOAD, States.SUCCESS],
    ]

    def __init__(
        self,
        http_helper: HttpHelper,
        *args,
        **kwargs,
    ):
        super().__init__(
            states=self.States,
            transitions=self.TRANSITIONS,
            initial=self.States.CREATE,
            **kwargs,
        )
        self._http_helper = http_helper

    def track_feature_request(self, *args, **kwargs) -> "RequestMonitorModel":
        """
        Track the status of a feature request by adding a new model to the state machine
        """

        model = RequestMonitorModel(*args, **kwargs, http_helper=self._http_helper)

        self.add_model(model)
        logger.debug(
            f"Adding {[r.path.name for r in model._recordings]} to state machine {self.__class__.__name__}"
        )
        self._tasks.append(asyncio.create_task(model.start()))
        model._task = self._tasks[-1]

        logger.debug("Done adding model")
        return model

    def fetch_current_model_states(
        self,
    ) -> Mapping[Path, Mapping[MpsFeature, ModelState]]:
        """
        Get the current state of each recording in each model
        Note: Each model contains exactly one feature but may contain one or more
        recordings attached to it
        """
        current_states = {}
        for model in self.models:
            for r in model.recordings:
                r = r.path
                if r not in current_states:
                    current_states[r] = {}
                current_states[r][model.feature] = model.get_status(r)
        return current_states

    def get_model_by_request_id(self, request_fbid: int) -> "RequestMonitorModel":
        """
        Get the model associated with a given feature request fbid.
        """
        return next(
            (m for m in self.models if m._feature_request.fbid == request_fbid), None
        )


class RequestMonitorModel:
    """
    Data model for a single MPS feature request. This is be used to track the status
    of a single feature request for single or multiple recordings
    """

    def __init__(
        self,
        recordings: Sequence[AriaRecording],
        feature_request: MpsFeatureRequest,
        http_helper: HttpHelper,
    ) -> None:
        self._recordings: Sequence[AriaRecording] = recordings
        self._feature_request: MpsFeatureRequest = feature_request
        self._http_helper: HttpHelper = http_helper
        self._query_exec: GraphQLQueryExecutor = GraphQLQueryExecutor(http_helper)
        self._progress: float = 0.0
        self._error_code: Optional[int] = None
        self._downloaders: Dict[Path, Downloader] = {}
        self._task: Optional[asyncio.Task] = None

    @property
    def feature(self) -> MpsFeature:
        """
        The feature that was requested
        """
        return self._feature_request.feature

    @property
    def recordings(self) -> Sequence[AriaRecording]:
        """
        All the recordings associated with this feature request
        """
        return self._recordings

    @property
    def feature_request(self) -> MpsFeatureRequest:
        """
        The feature request that was submitted to the server
        """
        return self._feature_request

    @property
    def task(self) -> Optional[asyncio.Task]:
        """
        The task associated with this model, if any
        """
        return self._task

    def get_status(self, recording: Path) -> str:
        """
        The current status of the request.
        We append the progress, where applicable
        """
        if self.is_CREATE():
            return ModelState(status="Submitted")
        elif self.is_WAIT():
            return ModelState(status=self._feature_request.status.capitalize())
        elif self.is_DOWNLOAD():
            downloader = self._downloaders.get(recording)
            progress = downloader.progress if downloader else 0
            return ModelState(status=DisplayStatus.DOWNLOADING, progress=progress)
        elif self.is_SUCCESS():
            return ModelState(status=DisplayStatus.SUCCESS)
        elif self.is_FAILURE():
            return ModelState(
                status=DisplayStatus.ERROR, error_code=str(self._error_code)
            )

        raise RuntimeError(f"Unknown state {self.state}")

    def has_error(self, event: EventData) -> bool:
        """
        Check if an error occurred during the state machine execution
        """
        logger.debug(event)
        logger.debug(f"has_error : {self._error_code}")
        return self._error_code is not None

    async def on_enter_WAIT(self, event: EventData) -> None:
        logger.debug(event)
        status_check_interval: int = config.getint(
            ConfigSection.DEFAULT, ConfigKey.STATUS_CHECK_INTERVAL
        )
        while self._feature_request.is_pending():
            await asyncio.sleep(status_check_interval)
            self._feature_request = await self._query_exec.query_feature_request(
                self._feature_request.fbid
            )

        if self._feature_request.status == Status.FAILED:
            self._error_code = self._feature_request.error_code
        await self.next()

    async def on_enter_DOWNLOAD(self, event: EventData) -> None:
        logger.debug(event)
        feature: MpsFeature = self._feature_request.feature
        await self._download_multi_slam_summary()

        for rec in self._recordings:
            results = [
                r
                for r in self._feature_request.results
                if r.recording_hash == rec.file_hash
                and r.result_type in RESULT_TYPES_BY_FEATURE[feature]
            ]
            if len(results) != 1:
                logger.error(f"{rec} has {len(results)} results: {results}")

            downloader: Downloader = Downloader(
                url=results[0].cdn_url,
                download_dir=rec.output_path,
                http_helper=self._http_helper,
                unzip=True,
            )
            self._downloaders[rec.path] = downloader
            logger.debug(f"Created downloader for {rec.path}")

        logger.debug(f"Starting downloads for {self._recordings}")
        await asyncio.gather(*[d.run() for d in self._downloaders.values()])
        logger.debug(f"Finished downloading {self._recordings}")
        await self.next()
        self._downloaders = {}  # Clear out references

    async def on_enter_SUCCESS(self, event: EventData) -> None:
        logger.debug(event)

    async def on_enter_FAILURE(self, event: EventData) -> None:
        logger.critical(event)

    async def on_exception(self, event: EventData) -> None:
        """
        This method is called whenever an exception occurs during execution of the
        state machine.
        """
        logger.error(f"Exception when processing {event}")
        # For future reference, here's how to log the exception without raising
        # it:
        # logger.error("".join(traceback.format_tb(event.error.__traceback__)))
        #
        # In order for the exception to be logged properly, we need to re-raise
        # the exception and then log it
        try:
            raise event.error
        except GraphQLError as e:
            logger.exception(e)
            self._error_code = ErrorCode.GRAPHQL_FAILURE
        except Exception as e:
            logger.exception(e)
        state_to_error_code: Dict[RequestMonitor.States, ErrorCode] = {
            RequestMonitor.States.WAIT.name: ErrorCode.PROCESSING_FAILURE,
            RequestMonitor.States.DOWNLOAD.name: ErrorCode.DOWNLOAD_FAILURE,
        }
        self._error_code = self._error_code or state_to_error_code.get(
            event.state.name, ErrorCode.STATE_MACHINE_FAILURE
        )
        await self.next()

    async def _download_multi_slam_summary(self) -> None:
        """
        Download the multi-slam summary file from the MPS server
        if this is a multi-slam request
        """
        if self.feature == MpsFeature.MULTI_SLAM:
            # Multi-SLAM request will have a summary file
            summary_result: MpsResult = next(
                (
                    r
                    for r in self._feature_request.results
                    if r.result_type == MpsResultType.SUMMARY
                    and r.recording_hash == str(self._feature_request.fbid)
                ),
                None,
            )
            downloader: Downloader = Downloader(
                url=summary_result.cdn_url,
                download_dir=self._recordings[0].output_path.parent,
                http_helper=self._http_helper,
                unzip=False,
            )
            self._downloaders["mst_summary"] = downloader
            logger.debug(f"Created downloader for summary {summary_result.cdn_url}")
