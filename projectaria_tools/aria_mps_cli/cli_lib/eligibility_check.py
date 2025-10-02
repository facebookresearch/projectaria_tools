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
from abc import ABC, abstractmethod
from typing import Any, Dict, Final, List, Optional

from .types import AriaRecording, MpsFeature

logger = logging.getLogger(__name__)


class EligibilityChecker(ABC):
    """
    Base class for checking if a recording is eligible for processing with a given MPS feature.
    """

    VHC_KEY_SECTION_LOCATION: str = ""
    VHC_KEY_SECTION_DEFAULT: str = ""

    _VHC_KEY_FINAL_RESULT: Final[str] = "final_result"
    _VHC_KEY_FAILED_CHECKS: Final[str] = "failed_checks"
    _VHC_KEY_WARN_CHECKS: Final[str] = "warn_checks"
    _VHC_KEY_STREAM_CHECKS: Final[str] = "performed_checks_with_details"

    _VHC_OUT_PASS: Final[str] = "pass"
    _VHC_OUT_WARN: Final[str] = "warn"
    _VHC_OUT_FAIL: Final[str] = "fail"

    def __init__(self, recording: AriaRecording) -> None:
        self._recording: AriaRecording = recording

    @abstractmethod
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
        raise NotImplementedError()

    def _is_failed(self, vhc_json: Dict[str, Any]) -> bool:
        """
        Check, if the Aria Recording failed the VRS Health Check.

        Args:
            vhc_json (Dict[str, Any]): The JSON output of the VRS Health Check.
        Returns:
            True if the Aria Recording failed the VRS Health Check, False otherwise.
        """

        section: Dict[str, Any] = vhc_json[self.VHC_KEY_SECTION_LOCATION]
        is_failed: bool = section[self._VHC_KEY_FINAL_RESULT] == self._VHC_OUT_FAIL

        logger.debug(f"Health check result: {section[self._VHC_KEY_FINAL_RESULT]}")
        if is_failed:
            logger.debug(f"Failed checks: {section[self._VHC_KEY_FAILED_CHECKS]}")
        if section[self._VHC_KEY_FINAL_RESULT] != self._VHC_OUT_PASS:
            logger.debug(f"Warning checks: {section[self._VHC_KEY_WARN_CHECKS]}")

        return is_failed


class EligibilityCheckerGen1(EligibilityChecker):
    """
    Check if an Aria gen1 recording is eligible for processing with a given MPS feature.
    """

    VHC_KEY_SECTION_LOCATION: Final[str] = "AriaGen1_Location"
    VHC_KEY_SECTION_DEFAULT: Final[str] = "AriaGen1_Default"

    def is_eligible(self, feature: MpsFeature) -> bool:
        if not self._recording.health_check_path.exists():
            raise FileNotFoundError(f"No health check found for {feature.file_path}")

        with open(self._recording.health_check_path) as vhc:
            vhc_json: Dict[str, Any] = json.load(vhc)

        if feature in [MpsFeature.SLAM, MpsFeature.MULTI_SLAM]:
            return not self._is_failed(vhc_json)
        elif feature in [MpsFeature.EYE_GAZE, MpsFeature.HAND_TRACKING]:
            return self._is_eligible(vhc_json, feature)

        raise NotImplementedError(f"Unknown feature type {feature}")

    def _is_eligible(
        self, vhc_json: Dict[str, Any], feature: MpsFeature
    ) -> Optional[bool]:
        """
        Checks, if the Aria Recording is eligible for a given feature.

        Args:
            vhc_json (Dict[str, Any]): The JSON output of the VRS Health Check.
            feature (MpsFeature): The MPS feature to check eligibility for.
        Returns:
            True if the Aria Recording is eligible for the given feature, False otherwise.
        """

        section: Dict[str, Any] = vhc_json[self.VHC_KEY_SECTION_DEFAULT]

        if feature == MpsFeature.EYE_GAZE:
            return self._is_eligible_stream(section, "Eye Camera Class #1")
        elif feature == MpsFeature.HAND_TRACKING:
            return all(
                self._is_eligible_stream(section, f"Camera Data (SLAM) #{i}")
                for i in range(1, 2)
            )

        return False

    def _is_eligible_stream(self, section_json: Dict[str, Any], stream: str) -> bool:
        """
        Check if the Aria Recording's stream is eligible for MPS processing.

        Args:
            section_json (Dict[str, Any]): The selected section of JSON output of the VRS Health Check.
            stream (str): The stream to check eligibility for.
        Returns:
            True if the Aria Recording's stream is eligible for MPS processing, False otherwise.
        """

        has_stream: bool = stream in section_json[self._VHC_KEY_STREAM_CHECKS]
        if not has_stream:
            return False

        errors: List[str] = [
            check
            for check in section_json[self._VHC_KEY_FAILED_CHECKS]
            if check.startswith(stream)
        ]
        if errors:
            logger.debug(f"Failed checks: {errors}")
            return False

        warnings: List[str] = [
            check
            for check in section_json[self._VHC_KEY_WARN_CHECKS]
            if check.startswith(stream)
        ]
        if warnings:
            logger.debug(f"Warning checks: {warnings}")

        return True


class EligibilityCheckerGen2(EligibilityChecker):
    """
    Check if an Aria gen2 recording is eligible for processing with a given MPS feature.
    """

    VHC_KEY_SECTION_LOCATION: Final[str] = "AriaGen2_Location"
    VHC_KEY_SECTION_DEFAULT: Final[str] = "AriaGen2_Default"

    def is_eligible(self, feature: MpsFeature) -> bool:
        if not self._recording.health_check_path.exists():
            raise FileNotFoundError(f"No health check found for {feature.file_path}")

        if feature in [
            MpsFeature.EYE_GAZE,
            MpsFeature.HAND_TRACKING,
            MpsFeature.MULTI_SLAM,
        ]:
            # requestor should reject the Recording in a previous step
            return False

        with open(self._recording.health_check_path) as vhc:
            vhc_json: Dict[str, Any] = json.load(vhc)

        if feature == MpsFeature.SLAM:
            return not self._is_failed(vhc_json)

        raise NotImplementedError(f"Unknown feature type {feature}")
