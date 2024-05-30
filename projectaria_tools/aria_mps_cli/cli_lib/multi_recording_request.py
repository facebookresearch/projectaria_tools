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
import functools
import glob
import json
import logging
from enum import auto, Enum, unique
from pathlib import Path
from typing import Any, Dict, Final, List, Mapping, Optional, Sequence, Set

from transitions.core import EventData

from .base_state_machine import BaseStateMachine
from .common import Config
from .constants import ConfigKey, ConfigSection, DisplayStatus, ErrorCode
from .encryption import VrsEncryptor
from .hash_calculator import HashCalculator
from .health_check import HealthCheckRunner, is_eligible
from .http_helper import HttpHelper
from .types import (
    AriaRecording,
    EncryptionError,
    GraphQLError,
    ModelState,
    MpsFeature,
    MpsFeatureRequest,
    Status,
    VrsHealthCheckError,
)
from .uploader import check_if_already_uploaded, Uploader

config = Config.get()
logger = logging.getLogger(__name__)

_VRS_TO_MULTI_SLAM_JSON: str = "vrs_to_multi_slam.json"
_SLAM: str = "slam"


class MultiRecordingRequest(BaseStateMachine):
    """
    MPS State Machine to run MPS on a group of aria recordings
    """

    @unique
    class States(Enum):
        CREATED = auto()
        PAST_OUTPUTS_CHECK = auto()
        HASH_COMPUTATION = auto()
        PAST_REQUESTS_CHECK = auto()
        VALIDATION = auto()
        UPLOAD = auto()
        SUBMIT = auto()
        SUCCESS_PAST_OUTPUT = auto()
        SUCCESS_NEW_REQUEST = auto()
        SUCCESS_PAST_REQUEST = auto()
        FAILURE = auto()

    TRANSITIONS: Final[List[List[Any]]] = [
        # ["trigger", "source", "dest", "conditions"]
        ["next", "*", States.FAILURE, "has_error"],
        ["start", States.CREATED, States.FAILURE, "has_error"],
        ["start", States.CREATED, States.PAST_OUTPUTS_CHECK],
        ["next", States.PAST_OUTPUTS_CHECK, States.HASH_COMPUTATION],
        ["next", States.HASH_COMPUTATION, States.PAST_REQUESTS_CHECK],
        ["next", States.PAST_REQUESTS_CHECK, States.VALIDATION],
        ["next", States.VALIDATION, States.UPLOAD],
        ["finish", States.PAST_OUTPUTS_CHECK, States.SUCCESS_PAST_OUTPUT],
        ["finish", States.PAST_REQUESTS_CHECK, States.SUCCESS_PAST_REQUEST],
        ["finish", States.UPLOAD, States.SUCCESS_NEW_REQUEST],
    ]

    def __init__(
        self,
        http_helper: HttpHelper,
        **kwargs,
    ):
        self._http_helper: HttpHelper = http_helper
        super().__init__(
            states=self.States,
            transitions=self.TRANSITIONS,
            initial=self.States.CREATED,
            **kwargs,
        )

    async def add_new_recordings(
        self,
        recordings: Set[Path],
        output_dir: Path,
        force: bool,
        retry_failed: bool,
        name: Optional[str] = None,
        suffix: Optional[str] = None,
    ) -> "MultiRecordingModel":
        """
        Search for all aria recordings recursively in all the input paths and add them
        to the state machine
        """
        (
            encryption_key,
            key_id,
        ) = await self._http_helper.query_encryption_key()

        model = MultiRecordingModel(
            recordings=recordings,
            http_helper=self._http_helper,
            force=force,
            suffix=suffix,
            retry_failed=retry_failed,
            output_dir=output_dir,
            encryption_key=encryption_key,
            key_id=key_id,
            name=name,
        )

        self.add_model(model)
        logger.debug(
            f"Adding {model._recordings} to state machine {self.__class__.__name__}"
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
                current_states[r][MpsFeature.MULTI_SLAM] = model.get_status(r)
        return current_states


class MultiRecordingModel:
    """
    MPS Request model for a group of aria recordings that are processed together
    """

    def __init__(
        self,
        recordings: List[Path],
        http_helper: HttpHelper,
        force: bool,
        suffix: Optional[str],
        retry_failed: bool,
        output_dir: Optional[Path],
        encryption_key: str,
        key_id: int,
        name: Optional[str] = None,
    ) -> None:
        self._feature: MpsFeature = MpsFeature.MULTI_SLAM
        self._http_helper: HttpHelper = http_helper
        self._force: bool = force
        self._suffix: Optional[str] = suffix
        self._retry_failed: bool = retry_failed
        self._hash_calculators: Mapping[Path, HashCalculator] = {}
        self._output_dir: Path = output_dir
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._recordings: List[AriaRecording] = []
        self._name: Optional[str] = name

        self._encryption_key: str = encryption_key
        self._key_id: int = key_id
        self._error_codes: Dict[Path, int] = {}
        self._feature_request: Optional[MpsFeatureRequest] = None
        self._task: Optional[asyncio.Task] = None

        output_mapping: Mapping[Path, str] = self._load_or_create_output_mapping(
            recordings
        )

        for rec in recordings:
            self._recordings.append(
                AriaRecording.create(
                    vrs_path=rec,
                    output_path=self._output_dir / output_mapping.get(rec, ""),
                )
            )

    def _load_or_create_output_mapping(
        self, recordings: List[Path]
    ) -> Mapping[Path, str]:
        """
        Load the existing output mapping from disk or create a new one
        """
        output_mapping_path: Path = self._output_dir / _VRS_TO_MULTI_SLAM_JSON
        output_mapping: Mapping[Path, str] = {}
        if output_mapping_path.exists():
            # if we have a previous mapping, load it and check if it matches our new
            # list. If it doesn't then we need to fail the entire request
            try:
                with open(output_mapping_path, "r") as fp:
                    output_mapping = {Path(k): v for k, v in json.load(fp).items()}
            except Exception as e:
                logger.exception(
                    f"Failed to load existing output mapping from {output_mapping_path}. Error: {e}"
                )

            if set(output_mapping.keys()) != set(recordings):
                logger.error(
                    f"Output mapping does not match the recordings provided. Expected {recordings} but got {output_mapping.keys()}"
                )
                self._error_codes = {
                    rec: ErrorCode.INPUT_OUTPUT_MISMATCH_FAILURE for rec in recordings
                }
        elif any(self._output_dir.iterdir()):
            # If there is already something in the output dir, then we can't continue
            logger.error(
                f"Output directory {self._output_dir} exists but is not empty. Please delete it before running again."
            )
            self._error_codes = {
                rec: ErrorCode.NON_EMPTY_OUTPUT_DIR_FAILURE for rec in recordings
            }
        else:
            # Create the mapping between original vrs files and the output directory
            for i, rec in enumerate(sorted(recordings)):
                output_mapping[rec] = str(i)

            # Save the mapping between original vrs files and the output directory
            with open(self._output_dir / _VRS_TO_MULTI_SLAM_JSON, "w") as fp:
                json.dump({str(k): v for k, v in output_mapping.items()}, fp, indent=2)
        return output_mapping

    @property
    def name(self) -> Optional[str]:
        return self._name

    @property
    def recordings(self) -> Sequence[AriaRecording]:
        """
        All the recordings associated with this feature request
        """
        return self._recordings

    @property
    def feature_request(self) -> Optional[MpsFeatureRequest]:
        """
        The feature request associated with this model.
        """
        return self._feature_request

    @property
    def task(self) -> Optional[asyncio.Task]:
        """
        The task associated with this model, if any
        """
        return self._task

    @property
    def is_force(self) -> bool:
        """
        Whether or not the force flag was set for this request.
        """
        return self._force

    @property
    def is_retry_failed(self) -> bool:
        """
        Whether or not the retry failed flag was set for this request.
        """
        return self._retry_failed

    def get_status(self, recording: Path) -> str:
        """
        The current status of the request.
        We append the progress, where applicable
        """
        if self.is_CREATED():
            return ModelState(status=DisplayStatus.CREATED)
        if self.is_HASH_COMPUTATION():
            hash_calculator = self._hash_calculators.get(recording)
            progress = hash_calculator.progress if hash_calculator else 0
            return ModelState(status=DisplayStatus.HASHING, progress=progress)
        if (
            self.is_PAST_REQUESTS_CHECK()
            or self.is_PAST_OUTPUTS_CHECK()
            or self.is_SUCCESS_PAST_REQUEST()
        ):
            return ModelState(status=DisplayStatus.CHECKING)
        if self.is_VALIDATION():
            return ModelState(status=DisplayStatus.HEALTHCHECK)
        if self.is_UPLOAD():
            uploader = self._uploaders.get(recording)
            if uploader:
                return ModelState(
                    status=DisplayStatus.UPLOADING, progress=uploader.progress
                )

            encryptor = self._encryptors.get(recording)
            if encryptor:
                return ModelState(
                    status=DisplayStatus.ENCRYPTING, progress=encryptor.progress
                )
            return ModelState(status=DisplayStatus.CHECKING)
        if self.is_SUCCESS_NEW_REQUEST():
            return ModelState(status=DisplayStatus.SUBMITTING)
        if self.is_SUCCESS_PAST_OUTPUT():
            return ModelState(status=DisplayStatus.SUCCESS)
        if self.is_FAILURE():
            return ModelState(
                status=DisplayStatus.ERROR,
                # If we don't have an error code, then that means that some other
                # recording failed
                error_code=str(
                    self._error_codes.get(recording, ErrorCode.SOMETHING_ELSE_FAILURE)
                ),
            )
        raise RuntimeError(f"Unknown state {self.state}")

    def has_error(self, event: EventData) -> bool:
        """
        Check if an error occurred during the state machine execution
        """
        logger.debug(event)
        logger.debug(f"has_error : {self._error_codes}")
        return bool(self._error_codes)

    async def on_enter_PAST_OUTPUTS_CHECK(self, event: EventData) -> None:
        logger.debug(event)
        if self._force:
            logger.info("Force flag is enabled, skipping pre-existing outputs check")
            await self.next()
        else:
            logger.debug("Checking for pre-existing outputs")
            outputs_exist: bool = True
            for rec in self._recordings:
                slam_output_path: Path = rec.output_path / _SLAM
                if slam_output_path.is_dir():
                    continue
                logger.debug(
                    f"{rec} does not have a slam folder in output {rec.output_path}"
                )
                outputs_exist = False
                break
            if outputs_exist:
                logger.debug("All outputs exist, skipping processing")
                await self.finish()
            else:
                logger.debug("Some outputs do not exist, continuing processing")
                await self.next()

    async def on_enter_HASH_COMPUTATION(self, event: EventData) -> None:
        logger.debug(event)
        hash_calculation_tasks: List[asyncio.Task] = []
        for rec in self._recordings:
            self._hash_calculators[rec.path] = await HashCalculator.get(
                rec.path, self._suffix
            )
            hash_calculation_tasks.append(
                asyncio.create_task(self._hash_calculators[rec.path].run())
            )

            def _assign_hash(r: AriaRecording, t: asyncio.Future) -> None:
                r.file_hash = t.result()

            hash_calculation_tasks[-1].add_done_callback(
                functools.partial(_assign_hash, rec)
            )
        await asyncio.gather(*hash_calculation_tasks)
        file_hashes: List[str] = [rec.file_hash for rec in self._recordings]
        for fh in file_hashes:
            if file_hashes.count(fh) > 1:
                self._error_codes[rec.path] = ErrorCode.DUPLICATE_RECORDING_FAILURE
        await self.next()
        self._hash_calculators = {}  # TODO: use before/after callback to clean up

    async def on_enter_PAST_REQUESTS_CHECK(self, event: EventData) -> None:
        logger.debug(event)
        if self._force:
            logger.info("Force flag is enabled, skipping pre-existing requests check")
            await self.next()
        else:
            mps_feature_request: Optional[MpsFeatureRequest] = (
                await self._http_helper.query_mps_requested_feature_by_file_hash_set(
                    list({r.file_hash for r in self._recordings})
                )
            )
            # Submit the request if it doesn't exist yet or if we are retrying failed requests
            if mps_feature_request is None or (
                mps_feature_request.status == Status.FAILED and self._retry_failed
            ):
                logger.info("Request not found or failed, submitting new request")
                await self.next()
            else:
                logger.info(
                    "Found an existing feature request with the same file hash. Skipping submission."
                )
                self._feature_request = mps_feature_request
                await self.finish()

    async def on_enter_VALIDATION(self, event: EventData) -> None:
        logger.debug(event)

        async def _health_check(rec: AriaRecording) -> None:
            if rec.health_check_path.is_file():
                logger.info(
                    f"Health check output already exists at {rec.health_check_path}, skipping VrsHealthCheck"
                )
            else:
                vhc_runner: HealthCheckRunner = await HealthCheckRunner.get(
                    vrs_path=rec.path,
                    json_out=rec.health_check_path,
                )
                await vhc_runner.run()
            if not rec.health_check_path.is_file():
                logger.error("Failed to run VrsHealthCheck for {rec.path}")
                self._error_codes[rec.path] = ErrorCode.HEALTH_CHECK_FAILURE
                raise VrsHealthCheckError()

            if not is_eligible(MpsFeature.MULTI_SLAM, rec):
                logger.error(f"{rec.path} is not eligible for multi-slam")
                self._error_codes[rec.path] = ErrorCode.HEALTH_CHECK_FAILURE
                raise VrsHealthCheckError()

        health_check_tasks: List[asyncio.Task] = []
        for rec in self._recordings:
            health_check_tasks.append(asyncio.create_task(_health_check(rec)))
        try:
            await asyncio.gather(*health_check_tasks)
        except VrsHealthCheckError:
            # Should be handled by error handling with next()
            ## Note that the pending tasks continue to run in the background
            pass
        await self.next()

    async def on_enter_UPLOAD(self, event: EventData) -> None:
        logger.debug(event)

        async def _encrypt_and_upload(rec: AriaRecording) -> None:
            """
            We combine encryption and upload into a single state here. This is to avoid
            waiting for encryption to finish on all files before starting the uploads.
            With this, a file upload will start immediately after encryption has finished.
            """
            recording_fbid: Optional[int] = await check_if_already_uploaded(
                rec.file_hash, self._http_helper
            )
            if recording_fbid:
                rec.fbid = recording_fbid
                logger.warning(
                    f"Found an existing recording with id {recording_fbid} for {rec.path}"
                )
            else:
                if rec.encrypted_path.is_file():
                    logger.warning(
                        f"Encrypted file already exists at {rec.encrypted_path}, skipping encryption"
                    )
                else:
                    self._encryptors[rec.path] = await VrsEncryptor.get(
                        rec.path,
                        rec.encrypted_path,
                        self._encryption_key,
                        self._key_id,
                    )
                    await self._encryptors[rec.path].run()

                if not rec.encrypted_path.is_file():
                    self._error_codes[rec.path] = ErrorCode.ENCRYPTION_FAILURE
                    raise EncryptionError()

                self._uploaders[rec.path] = await Uploader.get(
                    rec.encrypted_path, rec.file_hash, self._http_helper
                )
                if rec.path in self._encryptors:
                    self._encryptors.pop(rec.path)
                rec.fbid = await self._uploaders[rec.path].run()

                if config.getboolean(
                    ConfigSection.ENCRYPTION, ConfigKey.DELETE_ENCRYPTED_FILES
                ):
                    logger.info(f"Deleting encrypted file {rec.encrypted_path}")
                    rec.encrypted_path.unlink()

        self._uploaders: Mapping[Path, Uploader] = {}
        self._encryptors: Mapping[Path, VrsEncryptor] = {}
        upload_tasks: List[asyncio.Task] = [
            asyncio.create_task(_encrypt_and_upload(rec)) for rec in self._recordings
        ]

        #
        try:
            await asyncio.gather(*upload_tasks)
        except EncryptionError:
            # Should be handled by error handling with next()
            ## Note that the pending tasks continue to run in the background
            pass

        await self.finish()
        self._uploaders = {}  # TODO: Do this in before/after callback

    async def on_enter_SUCCESS_NEW_REQUEST(self, event: EventData) -> None:
        logger.debug(event)
        logger.debug(f"Finished processing {self.state}")

    async def on_enter_SUCCESS_PAST_OUTPUT(self, event: EventData) -> None:
        logger.debug(event)
        logger.debug(f"Finished processing {self.state}")

    async def on_enter_SUCCESS_PAST_REQUEST(self, event: EventData) -> None:
        logger.debug(event)
        logger.debug(f"Finished processing {self.state}")

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
            self._error_codes = {
                r.path: ErrorCode.GRAPHQL_FAILURE for r in self._recordings
            }
        except Exception as e:
            logger.exception(e)

        if not self._error_codes:
            # Backup error code based on state.
            # These are only used if the current error code is None
            state_to_error: Dict[MultiRecordingRequest.States, ErrorCode] = {
                MultiRecordingRequest.States.PAST_OUTPUTS_CHECK.name: ErrorCode.PAST_OUTPUTS_CHECK_FAILURE,
                MultiRecordingRequest.States.HASH_COMPUTATION.name: ErrorCode.HASH_COMPUTATION_FAILURE,
                MultiRecordingRequest.States.PAST_REQUESTS_CHECK.name: ErrorCode.PAST_REQUESTS_CHECK_FAILURE,
                MultiRecordingRequest.States.VALIDATION.name: ErrorCode.HEALTH_CHECK_FAILURE,
                MultiRecordingRequest.States.UPLOAD.name: ErrorCode.UPLOAD_FAILURE,
            }
            for rec in self._recordings:
                self._error_codes[rec.path] = state_to_error.get(
                    event.state.name, ErrorCode.STATE_MACHINE_FAILURE
                )
        logger.error(f"Error codes: {self._error_codes}")
        await self.next()
