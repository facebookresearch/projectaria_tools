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
import glob
import logging
from enum import auto, Enum, unique
from pathlib import Path
from typing import Any, Dict, Final, List, Mapping, Optional, Set

from transitions.core import EventData

from .base_state_machine import BaseStateMachine
from .common import Config, CustomAdapter
from .constants import ConfigKey, ConfigSection, DisplayStatus, ErrorCode
from .encryption import VrsEncryptor
from .hash_calculator import HashCalculator
from .health_check import is_eligible, run_health_check
from .http_helper import HttpHelper
from .request_monitor import RequestMonitor
from .types import (
    AriaRecording,
    GraphQLError,
    ModelState,
    MpsFeature,
    MpsFeatureRequest,
    MpsRequest,
    Status,
)
from .uploader import check_if_already_uploaded, Uploader

config = Config.get()

## Min remaining TTL for a recording before it needs to be reupload
## TODO: Change this to 1hr once we have support for uploading same recording multiple
## times
_MIN_REMAINING_TTL_SECS: int = 0  # 60 * 60  # 1hr

## Names of the output folder per feature
OUTPUT_DIR_NAME: Dict[MpsFeature, str] = {
    MpsFeature.SLAM: MpsFeature.SLAM.name.lower(),
    MpsFeature.EYE_GAZE: MpsFeature.EYE_GAZE.name.lower(),
}


class SingleRecordingRequest(BaseStateMachine):
    """
    MPS State Machine to request MPS per aria recording
    """

    @unique
    class States(Enum):
        CREATED = auto()
        EXISTING_OUTPUTS_CHECK = auto()
        HASH_COMPUTATION = auto()
        EXISTING_REQUESTS_CHECK = auto()
        VALIDATION = auto()
        EXISTING_RECORDING_CHECK = auto()
        ENCRYPT = auto()
        UPLOAD = auto()
        SUBMIT = auto()
        SUCCESS = auto()
        FAILURE = auto()

    TRANSITIONS: Final[List[List[Any]]] = [
        # ["trigger", "source", "dest", "conditions"]
        ["start", States.CREATED, States.EXISTING_OUTPUTS_CHECK],
        ["next", "*", States.FAILURE, "has_error"],
        ["next", States.EXISTING_OUTPUTS_CHECK, States.HASH_COMPUTATION],
        ["next", States.HASH_COMPUTATION, States.EXISTING_REQUESTS_CHECK],
        ["next", States.EXISTING_REQUESTS_CHECK, States.VALIDATION],
        ["next", States.VALIDATION, States.EXISTING_RECORDING_CHECK],
        ["next", States.EXISTING_RECORDING_CHECK, States.ENCRYPT],
        ["next", States.ENCRYPT, States.UPLOAD],
        ["submit", States.EXISTING_RECORDING_CHECK, States.SUBMIT],
        ["next", States.UPLOAD, States.SUBMIT],
        [
            "finish",
            [
                States.EXISTING_OUTPUTS_CHECK,
                States.EXISTING_REQUESTS_CHECK,
                States.SUBMIT,
            ],
            States.SUCCESS,
        ],
    ]

    def __init__(
        self,
        monitor: RequestMonitor,
        http_helper: HttpHelper,
        cmd_args: argparse.Namespace,
        **kwargs,
    ):
        self._monitor: RequestMonitor = monitor
        self._http_helper: HttpHelper = http_helper
        self._cmd_args: argparse.Namespace = cmd_args
        super().__init__(
            states=self.States,
            transitions=self.TRANSITIONS,
            initial=self.States.CREATED,
            **kwargs,
        )

    async def add_new_recording(self, recording: Path) -> None:
        """
        Add new recording to the state machine
        """
        model = SingleRecordingModel(
            recording=recording,
            features=set(self._cmd_args.features),
            request_monitor=self._monitor,
            http_helper=self._http_helper,
            force=self._cmd_args.force,
            suffix=self._cmd_args.suffix,
            retry_failed=self._cmd_args.retry_failed,
            encryption_key=self._encryption_key,
            key_id=self._key_id,
        )

        self.add_model(model)
        logger = logging.getLogger(__name__)
        logger.debug(
            f"Adding {model.recording.name} to state machine {self.__class__.__name__}"
        )
        self._tasks.append(asyncio.create_task(model.start()))

        logger.debug("Done adding model")

    async def add_new_recordings(self, input_paths: List[Path]) -> None:
        """
        Search for all aria recordings recursively in all the input paths and add them
        to the state machine
        """
        (
            self._encryption_key,
            self._key_id,
        ) = await self._http_helper.query_encryption_key()

        for input_path in input_paths:
            if input_path.is_file():
                if input_path.suffix != ".vrs":
                    raise ValueError(f"Only .vrs file supported: {input_path}")
                await self.add_new_recording(input_path)
            elif input_path.is_dir():
                for rec in glob.glob(f"{input_path}/**/*.vrs", recursive=True):
                    await self.add_new_recording(Path(rec))
            else:
                raise ValueError(f"Invalid input path: {input_path}")

    def fetch_current_model_states(
        self,
    ) -> Mapping[Path, Mapping[MpsFeature, ModelState]]:
        """
        Get the current state of each model
        """
        return {
            model.recording: {f: model.get_status(f) for f in self._cmd_args.features}
            for model in self.models
        }


class SingleRecordingModel:
    """
    MPS Request model per aria recording
    """

    def __init__(
        self,
        recording: Path,
        request_monitor: RequestMonitor,
        features: Set[MpsFeature],
        http_helper: HttpHelper,
        force: bool,
        suffix: Optional[str] = None,
        retry_failed: bool = False,
        encryption_key=str,
        key_id=int,
    ) -> None:
        self._recording: AriaRecording = AriaRecording.create(recording)
        self._features: Set[MpsFeature] = features
        self._past_processed_features: Set[MpsFeature] = set()
        self._ineligible_features: Set[MpsFeature] = set()
        self._request_monitor: RequestMonitor = request_monitor
        self._http_helper: HttpHelper = http_helper
        self._force: bool = force
        self._suffix: Optional[str] = suffix
        self._retry_failed: bool = retry_failed

        self._encryption_key: str = encryption_key
        self._key_id: int = key_id
        self._error_code: Optional[int] = None

        self._encryptor: Optional[VrsEncryptor] = None
        self._uploader: Optional[Uploader] = None
        self._logger: CustomAdapter = CustomAdapter(
            logging.getLogger(__name__), {"vrs": str(self._recording.path)}
        )
        self._logger.debug(f"Creating output folder: {self._recording.output_path}")
        Path.mkdir(self._recording.output_path, parents=True, exist_ok=True)

    @property
    def recording(self) -> Path:
        """
        Get recording path
        """
        return self._recording.path

    def get_status(self, feature: MpsFeature) -> ModelState:
        """
        Get status of the request submission. We append the progress where applicable
        """
        if feature in self._ineligible_features:
            return ModelState(
                status=DisplayStatus.ERROR,
                error_code=str(ErrorCode.HEALTH_CHECK_FAILURE),
            )
        elif feature in self._past_processed_features:
            return ModelState(status=DisplayStatus.SUCCESS)
        elif self.is_CREATED():
            return ModelState(status=DisplayStatus.CREATED)
        elif self.is_HASH_COMPUTATION():
            progress = self._hash_calculator.progress if self._hash_calculator else 0
            return ModelState(status=DisplayStatus.HASHING, progress=progress)
        elif (
            self.is_EXISTING_REQUESTS_CHECK()
            or self.is_EXISTING_RECORDING_CHECK()
            or self.is_EXISTING_OUTPUTS_CHECK()
        ):
            return ModelState(status=DisplayStatus.CHECKING)
        elif self.is_VALIDATION():
            return ModelState(status=DisplayStatus.HEALTHCHECK)
        elif self.is_ENCRYPT():
            progress = self._encryptor.progress if self._encryptor else 0
            return ModelState(status=DisplayStatus.ENCRYPTING, progress=progress)
        elif self.is_UPLOAD():
            progress = self._uploader.progress if self._uploader else 0
            return ModelState(status=DisplayStatus.UPLOADING, progress=progress)
        elif self.is_SUBMIT():
            return ModelState(status=DisplayStatus.SUBMITTING)
        elif self.is_FAILURE():
            return ModelState(
                status=DisplayStatus.ERROR, error_code=str(self._error_code)
            )
        elif self.is_SUCCESS():
            return ModelState(status="Should not be here")

        raise RuntimeError(f"Unknown state {self.state}")

    def has_error(self, event: EventData) -> bool:
        """
        Check if an error occurred during the state machine execution
        """
        self._logger.debug(event)
        self._logger.debug(f"has_error : {self._error_code}")
        return self._error_code is not None

    async def on_enter_HASH_COMPUTATION(self, event: EventData) -> None:
        self._logger.debug(event)
        self._hash_calculator = HashCalculator(self._recording.path, self._suffix)
        self._recording.file_hash = await self._hash_calculator.run()
        self._logger.debug(f"File hash {self._recording.file_hash}")
        await self.next()
        # TODO: move this to after callback
        self._hash_calculator = None

    async def on_enter_EXISTING_OUTPUTS_CHECK(self, event: EventData) -> None:
        self._logger.debug(event)
        if self._force:
            self._logger.info(
                "Force flag is enabled, skipping pre-existing outputs check"
            )
        else:
            for feature in self._features:
                if (
                    self._recording.output_path / OUTPUT_DIR_NAME.get(feature)
                ).is_dir():
                    self._past_processed_features.add(feature)
                    self._logger.info(
                        f"Skipping feature {feature} because it has been computed before"
                    )
            self._features = self._features.difference(self._past_processed_features)
        if self._features:
            await self.next()
        else:
            await self.finish()

    async def on_enter_EXISTING_REQUESTS_CHECK(self, event: EventData) -> None:
        self._logger.debug(event)
        if not self._force:
            # check if there are any existing requests with this file hash
            prev_requested_features: List[MpsFeatureRequest] = (
                await self._http_helper.query_mps_requested_features_by_file_hash(
                    self._recording.file_hash
                )
            )
            # Ignore features not requested for this run
            prev_requested_features = [
                f for f in prev_requested_features if f.feature in self._features
            ]
            if self._retry_failed:
                # check if there are any failed requests with this file hash
                # If we find any, remove them from the list so that they can be
                # retried
                prev_requested_features = [
                    r for r in prev_requested_features if r.status != Status.FAILED
                ]

            for r in prev_requested_features:
                self._request_monitor.track_feature_request(
                    recordings=[self._recording], feature_request=r
                )
            self._features = self._features.difference(
                {r.feature for r in prev_requested_features}
            )
        if self._features:
            await self.next()
        else:
            await self.finish()

    async def on_enter_VALIDATION(self, event: EventData) -> None:
        self._logger.debug(event)
        if self._recording.health_check_path.exists():
            self._logger.warning(
                f"Health check output already exists at {self._recording.health_check_path}, skipping VrsHealthCheck"
            )
        else:
            await run_health_check(
                self._recording.path, self._recording.health_check_path
            )
            if not self._recording.health_check_path.exists():
                self._logger.error(
                    f"Failed to run VrsHealthCheck for {self._recording.path}"
                )
                raise RuntimeError("VrsHealthCheck failed")
        self._ineligible_features = {
            f for f in self._features if not is_eligible(f, self._recording)
        }
        if self._ineligible_features:
            self._logger.error(
                f"Ineligible features found: {self._ineligible_features}. Skipping them."
            )
        self._features = self._features.difference(self._ineligible_features)
        if not self._features:
            self._error_code = ErrorCode.HEALTH_CHECK_FAILURE
        await self.next()

    async def on_enter_EXISTING_RECORDING_CHECK(self, event: EventData) -> None:
        self._logger.debug(event)
        recording_fbid: Optional[int] = await check_if_already_uploaded(
            self._recording.file_hash, self._http_helper
        )
        if recording_fbid:
            self._logger.info(f"Found an existing recording with id {recording_fbid}")
            self._recording.fbid = recording_fbid
            await self.submit()
        else:
            await self.next()

    async def on_enter_ENCRYPT(self, event: EventData) -> None:
        self._logger.debug(event)
        if self._recording.encrypted_path.exists():
            # If encrypted file already exists, skip encryption. This means that
            # very likely the upload either failed or was interrupted.
            self._logger.warning(
                f"Encrypted file already exists at {self._recording.encrypted_path}, skipping encryption"
            )
        else:
            self._encryptor = VrsEncryptor(
                self._recording.path,
                self._recording.encrypted_path,
                self._encryption_key,
                self._key_id,
            )
            await self._encryptor.run()
        await self.next()

    async def on_enter_UPLOAD(self, event: EventData) -> None:
        self._logger.debug(event)
        self._uploader: Uploader = Uploader(
            input_path=self._recording.encrypted_path,
            input_hash=self._recording.file_hash,
            http_helper=self._http_helper,
        )

        self._recording.fbid = await self._uploader.run()
        self._logger.info(f"Recording ID: {self._recording.fbid}")
        if config.getboolean(
            ConfigSection.ENCRYPTION, ConfigKey.DELETE_ENCRYPTED_FILES
        ):
            self._logger.info(
                f"Deleting encrypted file {self._recording.encrypted_path}"
            )
            self._recording.encrypted_path.unlink()
        await self.next()

    async def on_enter_SUBMIT(self, event: EventData) -> None:
        self._logger.debug(event)
        mps_request: MpsRequest = await self._http_helper.submit_request(
            name=self._recording.path.name,
            recording_ids=[self._recording.fbid],
            features=self._features,
        )
        if not self._features:
            raise ValueError("No more features left to submit")
        for f in self._features:
            self._request_monitor.track_feature_request(
                recordings=[self._recording],
                feature_request=mps_request.features[f],
            )

        await self.finish()

    async def on_enter_SUCCESS(self, event: EventData) -> None:
        self._logger.debug(event)
        self._logger.info("Finished processing")

    async def on_enter_FAILURE(self, event: EventData) -> None:
        self._logger.critical(event)

    async def on_exception(self, event: EventData) -> None:
        """
        This method is called whenever an exception occurs during execution of the
        state machine.
        """
        self._logger.error(f"Exception when processing {event}")
        # For future reference, here's how to log the exception without raising
        # it:
        # self._logger.error("".join(traceback.format_tb(event.error.__traceback__)))
        #
        # In order for the exception to be logged properly, we need to re-raise
        # the exception and then log it
        try:
            raise event.error
        except GraphQLError as e:
            self._logger.exception(e)
            self._error_code = ErrorCode.GRAPHQL_FAILURE
        except Exception as e:
            self._logger.exception(e)

        self._logger.error(f"Error code: {self._error_code}")

        # Backup error code based on state.
        # These are only used if the current error code is None
        state_to_error: Dict[SingleRecordingRequest.States, ErrorCode] = {
            SingleRecordingRequest.States.EXISTING_OUTPUTS_CHECK.name: ErrorCode.EXISTING_OUTPUTS_CHECK_FAILURE,
            SingleRecordingRequest.States.HASH_COMPUTATION.name: ErrorCode.HASH_COMPUTATION_FAILURE,
            SingleRecordingRequest.States.EXISTING_REQUESTS_CHECK.name: ErrorCode.EXISTING_REQUESTS_CHECK_FAILURE,
            SingleRecordingRequest.States.VALIDATION.name: ErrorCode.HEALTH_CHECK_FAILURE,
            SingleRecordingRequest.States.EXISTING_RECORDING_CHECK.name: ErrorCode.EXISTING_RECORDING_CHECK_FAILURE,
            SingleRecordingRequest.States.ENCRYPT.name: ErrorCode.ENCRYPTION_FAILURE,
            SingleRecordingRequest.States.UPLOAD.name: ErrorCode.UPLOAD_FAILURE,
            SingleRecordingRequest.States.SUBMIT.name: ErrorCode.SUBMIT_FAILURE,
        }
        self._error_code = self._error_code or state_to_error.get(
            event.state.name, ErrorCode.STATE_MACHINE_FAILURE
        )
        await self.next()
