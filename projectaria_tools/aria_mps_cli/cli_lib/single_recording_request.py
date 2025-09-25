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
from typing import Any, Dict, Final, List, Mapping, Optional, Set

from projectaria_tools.core.data_provider import (
    create_vrs_data_provider,
    VrsDataProvider,
)

from transitions.core import EventData

from .base_state_machine import BaseStateMachine
from .common import Config, CustomAdapter
from .constants import (
    ConfigKey,
    ConfigSection,
    DisplayStatus,
    ErrorCode,
    KEY_DEVICE_TYPE,
)
from .encryption import VrsEncryptor
from .hash_calculator import HashCalculator
from .health_check import HealthCheckRunner, is_eligible
from .http_helper import HttpHelper
from .types import (
    AriaRecording,
    GraphQLError,
    ModelState,
    MpsAriaDevice,
    MpsFeature,
    MpsFeatureRequest,
    Status,
)
from .uploader import check_if_already_uploaded, Uploader

config = Config.get()

## Names of the output folder per feature
OUTPUT_DIR_NAME: Dict[MpsFeature, str] = {
    feature: feature.name.lower()
    for feature in (
        MpsFeature.SLAM,
        MpsFeature.EYE_GAZE,
        MpsFeature.HAND_TRACKING,
    )
}


class SingleRecordingRequest(BaseStateMachine):
    """
    MPS State Machine to request MPS per aria recording
    """

    @unique
    class States(Enum):
        CREATED = auto()
        PAST_OUTPUT_CHECK = auto()
        HASH_COMPUTATION = auto()
        PAST_REQUEST_CHECK = auto()
        DEVICE_TYPE_CHECK = auto()
        VALIDATION = auto()
        PAST_RECORDING_CHECK = auto()
        ENCRYPT = auto()
        UPLOAD = auto()

        SUCCESS_PAST_OUTPUT = auto()
        SUCCESS_NEW_REQUEST = auto()
        SUCCESS_PAST_REQUEST = auto()

        PROCESSING_NOT_REQUIRED = auto()

        FAILURE = auto()

    TRANSITIONS: Final[List[List[Any]]] = [
        # ["trigger", "source", "dest", "conditions"]
        ["start", States.CREATED, States.PAST_OUTPUT_CHECK],
        ["next", "*", States.FAILURE, "has_error"],
        ["next", States.PAST_OUTPUT_CHECK, States.HASH_COMPUTATION],
        ["next", States.HASH_COMPUTATION, States.PAST_REQUEST_CHECK],
        ["next", States.PAST_REQUEST_CHECK, States.DEVICE_TYPE_CHECK],
        ["next", States.DEVICE_TYPE_CHECK, States.VALIDATION],
        ["next", States.VALIDATION, States.PAST_RECORDING_CHECK],
        ["next", States.PAST_RECORDING_CHECK, States.ENCRYPT],
        ["next", States.ENCRYPT, States.UPLOAD],
        ["finish", States.PAST_RECORDING_CHECK, States.SUCCESS_NEW_REQUEST],
        ["finish", States.UPLOAD, States.SUCCESS_NEW_REQUEST],
        ["finish", States.PAST_OUTPUT_CHECK, States.SUCCESS_PAST_OUTPUT],
        ["finish", States.PAST_REQUEST_CHECK, States.SUCCESS_PAST_REQUEST],
        ["finish", States.DEVICE_TYPE_CHECK, States.PROCESSING_NOT_REQUIRED],
    ]

    def __init__(self, http_helper: HttpHelper, **kwargs):
        self._http_helper: HttpHelper = http_helper
        self._encryption_key: str = None
        self._key_id: str = None
        super().__init__(
            states=self.States,
            transitions=self.TRANSITIONS,
            initial=self.States.CREATED,
            **kwargs,
        )

    async def add_new_recording(
        self,
        recording: Path,
        feature: MpsFeature,
        force: bool,
        retry_failed: bool,
        persist_on_failure: bool,
        feedback_id: Optional[str] = None,
        suffix: Optional[str] = None,
    ) -> "SingleRecordingModel":
        """
        Add new recording to the state machine
        """
        if not self._encryption_key and not self._key_id:
            key, id = await self._http_helper.query_encryption_key()
            self._encryption_key, self._key_id = key, id

        model = SingleRecordingModel(
            recording=recording,
            feature=feature,
            http_helper=self._http_helper,
            force=force,
            suffix=suffix,
            retry_failed=retry_failed,
            persist_on_failure=persist_on_failure,
            feedback_id=feedback_id,
            encryption_key=self._encryption_key,
            key_id=self._key_id,
        )

        self.add_model(model)
        self._tasks.append(asyncio.create_task(model.start()))
        model._task = self._tasks[-1]

        logging.getLogger(__name__).debug(
            f"Started processing recording: {recording} with feature: {feature}"
        )
        logging.getLogger(__name__).debug(model._task)
        return model

    def fetch_current_model_states(
        self,
    ) -> Mapping[Path, Mapping[MpsFeature, ModelState]]:
        """
        Get the current state of each model
        """
        return {
            model.recording.path: model.get_status_for_all_features()
            for model in self.models
        }


class SingleRecordingModel:
    """
    MPS Request model per aria recording
    """

    def __init__(
        self,
        recording: Path,
        feature: MpsFeature,
        http_helper: HttpHelper,
        force: bool,
        persist_on_failure: bool,
        suffix: Optional[str] = None,
        retry_failed: bool = False,
        encryption_key=str,
        key_id=int,
        feedback_id: Optional[str] = None,
    ) -> None:
        self._recording: AriaRecording = AriaRecording.create(recording)
        self._feature: MpsFeature = feature
        # We modify the features set to remove the ones that are not eligible or already
        # processed. So we cache the features originally requested
        self._http_helper: HttpHelper = http_helper
        self._force: bool = force
        self._suffix: Optional[str] = suffix
        self._retry_failed: bool = retry_failed
        self._persist_on_failure: bool = persist_on_failure
        self._feedback_id: Optional[str] = feedback_id

        self._encryption_key: str = encryption_key
        self._key_id: int = key_id
        self._error_code: Optional[int] = None
        self._lock_taken: bool = False
        self._past_feature_request: Optional[MpsFeatureRequest] = None
        self._task: asyncio.Task = None

        self._encryptor: Optional[VrsEncryptor] = None
        self._uploader: Optional[Uploader] = None
        self._logger: CustomAdapter = CustomAdapter(
            logging.getLogger(__name__), {"vrs": str(self._recording.path)}
        )
        self._logger.debug(f"Creating output folder: {self._recording.output_path}")
        Path.mkdir(self._recording.output_path, parents=True, exist_ok=True)

    @property
    def recording(self) -> AriaRecording:
        """
        Get recording
        """
        return self._recording

    @property
    def past_feature_request(self) -> Optional[MpsFeatureRequest]:
        """
        The feature request associated with this model. This is used to submit the MPS job
        """
        return self._past_feature_request

    @property
    def feature(self) -> Set[MpsFeature]:
        """
        Get feature to be processed for this recording.
        """
        return self._feature

    @property
    def task(self) -> Optional[asyncio.Task]:
        """
        The task associated with this model, if any
        """
        return self._task

    @property
    def is_force(self) -> bool:
        """Get force flag."""
        return self._force

    @property
    def is_persisted_on_failure(self) -> bool:
        """Get persist on failure flag."""
        return self._persist_on_failure

    @property
    def is_retry_failed(self) -> bool:
        """Get retry failed flag."""
        return self._retry_failed

    def get_status(self) -> ModelState:
        """
        Get status of the request submission. We append the progress where applicable
        """
        if self.is_CREATED():
            return ModelState(status=DisplayStatus.CREATED)
        elif self.is_HASH_COMPUTATION():
            progress = self._hash_calculator.progress if self._hash_calculator else 0
            return ModelState(status=DisplayStatus.HASHING, progress=progress)
        elif (
            self.is_PAST_REQUEST_CHECK()
            or self.is_PAST_RECORDING_CHECK()
            or self.is_PAST_OUTPUT_CHECK()
            or self.is_SUCCESS_PAST_REQUEST()
            or self.is_DEVICE_TYPE_CHECK()
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
        elif self.is_SUCCESS_NEW_REQUEST():
            return ModelState(status=DisplayStatus.SUBMITTING)
        elif self.is_FAILURE():
            return ModelState(
                status=DisplayStatus.ERROR, error_code=str(self._error_code)
            )
        elif self.is_SUCCESS_PAST_OUTPUT() or self.is_PROCESSING_NOT_REQUIRED():
            return ModelState(status=DisplayStatus.SUCCESS)

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
        self._hash_calculator = await HashCalculator.get(
            self._recording.path, self._suffix
        )
        self._recording.file_hash = await self._hash_calculator.run()
        self._logger.debug(f"File hash {self._recording.file_hash}")
        await self.next()
        # TODO: move this to after callback
        self._hash_calculator = None

    async def on_enter_PAST_OUTPUT_CHECK(self, event: EventData) -> None:
        self._logger.debug(event)
        if self._force:
            self._logger.info(
                "Force flag is enabled, skipping pre-existing outputs check"
            )
            await self.next()
        elif (
            self._recording.output_path / OUTPUT_DIR_NAME.get(self._feature)
        ).is_dir():
            self._logger.info(
                f"Skipping feature {self._feature} because it has been computed before"
            )
            await self.finish()
        else:
            await self.next()

    async def on_enter_PAST_REQUEST_CHECK(self, event: EventData) -> None:
        self._logger.debug(event)
        finish: bool = False
        if self._force:
            self._logger.debug(
                "Force flag is enabled, skipping pre-existing requests check"
            )
        else:
            # check if there are any existing requests with this file hash
            # Once T190464177 lands, we can filter by file hash and feature
            past_requested_features: List[
                MpsFeatureRequest
            ] = await self._http_helper.query_mps_requested_features_by_file_hash(
                self._recording.file_hash
            )
            self._logger.info(
                f"Found {len(past_requested_features)} existing feature requests"
            )
            self._past_feature_request = next(
                iter(
                    [r for r in past_requested_features if r.feature == self._feature]
                ),
                None,
            )
            if self._past_feature_request:
                self._logger.info(
                    f"Found existing feature request {self._past_feature_request}"
                )
                finish = (
                    not self._retry_failed
                    or self._past_feature_request.status != Status.FAILED
                )
        await self.finish() if finish else await self.next()

    async def on_enter_DEVICE_TYPE_CHECK(self, event: EventData) -> None:
        self._logger.debug(event)

        provider: VrsDataProvider = create_vrs_data_provider(
            self._recording.path.as_posix()
        )
        tags: Dict[str, str] = provider.get_file_tags()
        self._recording.device_type = MpsAriaDevice.from_device_tag(
            tags[KEY_DEVICE_TYPE]
        )

        self._logger.debug(
            f"Processing recording from {self._recording.device_type.name} device"
        )

        # Check if the recording is eligible for the feature computation
        if (
            self._recording.device_type == MpsAriaDevice.ARIA_GEN2
            and self._feature == MpsFeature.EYE_GAZE
        ):
            self._logger.error(f"Eye gaze is not supported for {self._recording}")
            await self.finish()

        await self.next()

    async def on_enter_VALIDATION(self, event: EventData) -> None:
        self._logger.debug(event)
        if self._recording.health_check_path.exists():
            self._logger.warning(
                f"Health check output already exists at {self._recording.health_check_path}, skipping VrsHealthCheck"
            )
        else:
            vhc_runner: HealthCheckRunner = await HealthCheckRunner.get(
                vrs_path=self._recording.path,
                json_out=self._recording.health_check_path,
            )
            await vhc_runner.run()
            if not self._recording.health_check_path.exists():
                self._logger.error(
                    f"Failed to run VrsHealthCheck for {self._recording.path}"
                )
                raise RuntimeError("VrsHealthCheck failed")

        # Check if the recording is eligible for the feature computation
        if not is_eligible(self._feature, self._recording):
            self._logger.error(f"Health check failed for {self._feature}")
            self._error_code = ErrorCode.HEALTH_CHECK_FAILURE
        await self.next()

    async def on_enter_PAST_RECORDING_CHECK(self, event: EventData) -> None:
        self._logger.debug(event)
        recording_fbid: Optional[int] = await check_if_already_uploaded(
            self._recording.file_hash, self._http_helper
        )
        if recording_fbid:
            self._logger.info(f"Found an existing recording with id {recording_fbid}")
            self._recording.fbid = recording_fbid
            await self.finish()
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
            self._encryptor = await VrsEncryptor.get(
                self._recording.path,
                self._recording.encrypted_path,
                self._encryption_key,
                self._key_id,
            )
            await self._encryptor.run()
        await self.next()

    async def on_enter_UPLOAD(self, event: EventData) -> None:
        self._logger.debug(event)
        self._uploader: Uploader = await Uploader.get(
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
            self._recording.encrypted_path.unlink(missing_ok=True)
        await self.finish()

    async def on_enter_SUCCESS_NEW_REQUEST(self, event: EventData) -> None:
        self._logger.debug(event)
        self._logger.info(f"Finished processing {self.state}")

    async def on_enter_SUCCESS_PAST_OUTPUT(self, event: EventData) -> None:
        self._logger.debug(event)
        self._logger.info(f"Finished processing {self.state}")

    async def on_enter_SUCCESS_PAST_REQUEST(self, event: EventData) -> None:
        self._logger.debug(event)
        self._logger.info(f"Finished processing {self.state}")

    async def on_enter_PROCESSING_NOT_REQUIRED(self, event: EventData) -> None:
        self._logger.debug(event)
        self._logger.info(f"SKIPPING processing {self.state}")

    async def on_enter_FAILURE(self, event: EventData) -> None:
        self._logger.critical(event)
        self._logger.info(f"Finished processing with error code {self._error_code}")

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
            SingleRecordingRequest.States.PAST_OUTPUT_CHECK.name: ErrorCode.PAST_OUTPUT_CHECK_FAILURE,
            SingleRecordingRequest.States.HASH_COMPUTATION.name: ErrorCode.HASH_COMPUTATION_FAILURE,
            SingleRecordingRequest.States.PAST_REQUEST_CHECK.name: ErrorCode.PAST_REQUEST_CHECK_FAILURE,
            SingleRecordingRequest.States.VALIDATION.name: ErrorCode.HEALTH_CHECK_FAILURE,
            SingleRecordingRequest.States.PAST_RECORDING_CHECK.name: ErrorCode.PAST_RECORDING_CHECK_FAILURE,
            SingleRecordingRequest.States.ENCRYPT.name: ErrorCode.ENCRYPTION_FAILURE,
            SingleRecordingRequest.States.UPLOAD.name: ErrorCode.UPLOAD_FAILURE,
        }
        self._error_code = self._error_code or state_to_error.get(
            event.state.name, ErrorCode.STATE_MACHINE_FAILURE
        )
        await self.next()
