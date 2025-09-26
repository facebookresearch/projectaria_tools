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
from typing import Any, Dict, final, Final, Optional

from projectaria_vrs_health_check.vrs_health_check import run_vrs_health_check

from .common import to_proc
from .runner_with_progress import RunnerWithProgress
from .types import AriaRecording, MpsAriaDevice, MpsFeature

logger = logging.getLogger(__name__)


class HealthCheckRunner(RunnerWithProgress):
    """
    Run health check on a given vrs file.
    """

    __VHC_KEY_SECTION_GEN1: Final[str] = "AriaGen1_Location"
    __VHC_KEY_SECTION_GEN2: Final[str] = "AriaGen2_Location"
    __VHC_KEY_FINAL_RESULT: Final[str] = "final_result"
    __VHC_KEY_FAILED_CHECKS: Final[str] = "failed_checks"
    __VHC_KEY_WARN_CHECKS: Final[str] = "warn_checks"

    __VHC_OUT_PASS: Final[str] = "pass"
    __VHC_OUT_WARN: Final[str] = "warn"
    __VHC_OUT_FAIL: Final[str] = "fail"

    def __init__(self, recording: AriaRecording) -> None:
        self._recording: AriaRecording = recording

        # Declare for parent class
        self._task: Optional[Task] = None

    @classmethod
    @final
    def get_key(cls, recording: AriaRecording) -> str:
        """
        Get a unique key for this Runner instance
        """

        return f"{recording.path}_{recording.health_check_path}"

    @final
    async def _run(self) -> None:
        """
        Run the health check on this vrs file.
        Repeatedly calling run will await on the same task
        """

        await to_proc(
            run_vrs_health_check,
            path=self._recording.path.as_posix(),
            json_out_filename=self._recording.health_check_path.as_posix(),
        )

    def is_eligible(self, feature: MpsFeature) -> bool:
        """
        Check if a feature is eligible for processing based on the results of the health check
        """

        if not self._recording.health_check_path.exists():
            raise FileNotFoundError(f"No health check found for {feature.file_path}")

        if self._recording.device_type == MpsAriaDevice.ARIA_GEN2 and feature in [
            MpsFeature.EYE_GAZE,
            MpsFeature.MULTI_SLAM,
        ]:
            return False

        with open(self._recording.health_check_path) as vhc:
            vhc_json: Dict[str, Any] = json.load(vhc)

            section: str = (
                HealthCheckRunner.__VHC_KEY_SECTION_GEN1
                if self._recording.device_type == MpsAriaDevice.ARIA_GEN1
                else HealthCheckRunner.__VHC_KEY_SECTION_GEN2
            )
            is_failed: bool = (
                vhc_json[section][HealthCheckRunner.__VHC_KEY_FINAL_RESULT]
                == HealthCheckRunner.__VHC_OUT_FAIL
            )

            logger.debug(f"Health check result: {vhc_json[section][
                    HealthCheckRunner.__VHC_KEY_FINAL_RESULT
                ]}")
            if is_failed:
                logger.debug(f"Failed checks: {vhc_json[section][
                    HealthCheckRunner.__VHC_KEY_FAILED_CHECKS
                ]}")
            if (
                vhc_json[section][HealthCheckRunner.__VHC_KEY_FINAL_RESULT]
                != HealthCheckRunner.__VHC_OUT_PASS
            ):
                logger.debug(f"Warning checks: {vhc_json[section][
                    HealthCheckRunner.__VHC_KEY_WARN_CHECKS
                ]}")

            return not is_failed

        raise NotImplementedError(f"Unknown feature type {feature}")
