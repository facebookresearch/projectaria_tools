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

import json
import logging
from asyncio import Task
from typing import Any, Dict, final, Optional

from projectaria_vrs_health_check.vrs_health_check import run_vrs_health_check

from .eligibility_check import (
    EligibilityChecker,
    EligibilityCheckerGen1,
    EligibilityCheckerGen2,
)
from .runner_with_progress import RunnerWithProgress
from .types import AriaRecording, MpsAriaDevice, MpsFeature

logger = logging.getLogger(__name__)


class HealthCheckRunner(RunnerWithProgress):
    """
    Run health check on a given vrs file.
    """

    def __init__(self, recording: AriaRecording) -> None:
        self._recording: AriaRecording = recording
        self._elgibility_checker: EligibilityChecker = (
            EligibilityCheckerGen1(recording)
            if recording.device_type == MpsAriaDevice.ARIA_GEN1
            else EligibilityCheckerGen2(recording)
        )

        # Declare for parent class
        self._task: Optional[Task] = None

    @classmethod
    @final
    def get_key(cls, recording: AriaRecording) -> str:
        """
        Get a unique key for this Runner instance.

        Args:
            recording (AriaRecording): The Aria Recording to get the key for.
        Returns:
            A unique key for this Runner instance.
        """

        return f"{recording.path}_{recording.health_check_path}"

    @final
    async def _run(self) -> None:
        """
        Run the health check on this vrs file.
        Repeatedly calling run will await on the same task
        """

        return run_vrs_health_check(
            path=self._recording.path.as_posix(),
            json_out_filename=self._recording.health_check_path.as_posix(),
            disable_logging=True,
            print_stats=False,
        )

    def is_eligible(self, feature: MpsFeature) -> bool:
        """
        Checks if an Aria Recording is eligible for processing with selected MPS feature
        based on the results of the VRS Health Check.

        Args:
            feature (MpsFeature): The MPS feature to check eligibility for.
        Returns:
            True if the recording is eligible for processing with the selected feature,
            False otherwise.
        """

        return self._elgibility_checker.is_eligible(feature)

    def check_vrs_output_and_remove_invalid(self) -> bool:
        """
        Check if the vrs health check output is valid.

        Returns:
            bool: True if the VRS HealthCcheck output exists and is valid, False otherwise.
            If the VRS Health Check output is invalid, remove it.
        """

        if not self._recording.health_check_path.exists():
            return False

        with open(self._recording.health_check_path) as vhc:
            vhc_json: Dict[str, Any] = json.load(vhc)

        if self._elgibility_checker.VHC_KEY_SECTION_DEFAULT not in vhc_json:
            logger.warning(
                f"Detected older version of health check output for {self._recording.path}, removing"
            )
            self._recording.health_check_path.unlink()
            return False

        return True
