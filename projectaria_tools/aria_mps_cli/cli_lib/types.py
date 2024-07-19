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

from dataclasses import dataclass
from enum import auto, Enum, unique
from pathlib import Path
from typing import Dict, List, Optional


@dataclass
class AriaRecording:
    """
    All the paths related to an Aria Recording.
    All the ids related to the recording are optional since they are not known at
    initialization
    """

    path: Path
    output_path: Path
    encrypted_path: Path
    health_check_path: Path
    health_check_slam_path: Path
    fbid: Optional[int] = None
    file_hash: Optional[str] = None

    @classmethod
    def create(
        cls,
        vrs_path: Path,
        output_path: Optional[Path] = None,
    ):
        """
        Setup all the relevant paths.
        If the root output path is not provided, it will be set to the same folder as the input vrs
        The output path will be <output_root>/mps_<vrs_file_name>_vrs
        """
        output_path = (
            output_path
            or vrs_path.parent / f'mps_{vrs_path.name.replace(".vrs", "_vrs")}'
        )
        output_path.mkdir(parents=True, exist_ok=True)

        return cls(
            path=vrs_path,
            output_path=output_path,
            encrypted_path=output_path / (vrs_path.name + ".enc"),
            health_check_path=output_path / "vrs_health_check.json",
            health_check_slam_path=output_path / "vrs_health_check_slam.json",
        )


@unique
class MpsFeature(str, Enum):
    """
    Feature types supported by MPS
    """

    def _generate_next_value_(name, start, count, last_values):
        return name.upper()

    SLAM = auto()
    EYE_GAZE = auto()
    MULTI_SLAM = auto()
    HAND_TRACKING = auto()


@unique
class Status(str, Enum):
    """
    Status of the feature request
    """

    def _generate_next_value_(name, start, count, last_values):
        return name.upper()

    SCHEDULED = auto()
    PROCESSING = auto()
    FAILED = auto()
    SUCCEEDED = auto()


class MpsResultType(str, Enum):
    """
    Type of the result
    """

    def _generate_next_value_(name, start, count, last_values):
        return name.upper()

    SUMMARY = auto()
    SLAM_ZIP = auto()
    EYE_GAZE_ZIP = auto()
    HAND_TRACKING_ZIP = auto()


@dataclass
class MpsResult:
    """
    Result of a feature computation
    """

    fbid: int
    cdn_url: str
    result_type: str
    recording_hash: str
    recording_name: Optional[str] = None


@dataclass
class MpsFeatureRequest:
    """
    Details of a MPS feature request
    """

    fbid: int
    feature: MpsFeature
    status: Status
    results: List[MpsResult]
    creation_time: int
    status_message: Optional[str] = None
    error_code: Optional[int] = None

    def is_pending(self) -> bool:
        return self.status in [Status.SCHEDULED, Status.PROCESSING]


@dataclass
class MpsRequest:
    """
    Details of a MPS request
    """

    fbid: int
    name: str
    creation_time: int
    features: Dict[MpsFeature, MpsFeatureRequest]
    recordings_fbids: List[int]


@dataclass
class ModelState:
    """
    State of the model.
    """

    status: str
    # Error code is a string for more flexibility.
    error_code: Optional[str] = None
    progress: Optional[float] = None


class MpsRequestSource(str, Enum):
    """
    Source of the MPS request.
    """

    def _generate_next_value_(name, start, count, last_values):
        return name.upper()

    MPS_CLI = auto()
    ARIA_STUDIO = auto()


class EncryptionError(Exception):
    """
    Raised when encryption fails for multi-trajectory
    """

    pass


class VrsHealthCheckError(Exception):
    """
    Raised when VrsHealthCheck fails for multi-trajectory
    """

    pass


class GraphQLError(Exception):
    """
    Raised when the GraphQL throws an error
    """

    pass
