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

from typing import Any, List, Mapping, Optional, Tuple

from .types import (
    MpsFeature,
    MpsFeatureRequest,
    MpsRequest,
    MpsResult,
    MpsResultType,
    Status,
)


class ResponseParser:
    """Class for parsing responses from GraphQL queries"""

    @staticmethod
    def parse_mps_request(
        response: Optional[Mapping[str, Any]]
    ) -> Optional[MpsRequest]:
        """Parse the given response into an MpsRequest object."""
        if not response:
            return None
        features = response["features"]["nodes"]
        recordings = response["recordings"]["nodes"]
        mps_request: MpsRequest = MpsRequest(
            fbid=response["id"],
            name=response["name"],
            recordings_fbids=[r["id"] for r in recordings],
            features={
                MpsFeature(f["feature"]): ResponseParser.parse_mps_feature_request(f)
                for f in features
            },
        )
        return mps_request

    @staticmethod
    def parse_mps_feature_request(response: Mapping[str, Any]) -> MpsFeatureRequest:
        """Parse the given response into an MpsFeatureRequest object."""
        return MpsFeatureRequest(
            fbid=response["id"],
            status=Status(response["status"]),
            status_message=response["status_message"],
            feature=MpsFeature(response["feature"]),
            error_code=response["error_code"],
            results=ResponseParser.parse_results(response["mps_results"]["nodes"]),
        )

    @staticmethod
    def parse_results(response: Mapping[str, Any]) -> List[MpsResult]:
        """Parse the given response into an list of MpsResult objects."""
        mps_results: List[MpsResult] = []
        for resp in response:
            mps_results.append(
                MpsResult(
                    fbid=resp["id"],
                    result_type=MpsResultType(resp["result_type"]),
                    cdn_url=resp["cdn_url"],
                    file_hash=resp["file_hash"],
                )
            )
        return mps_results

    @staticmethod
    def parse_recording_id_and_ttl(
        response: Mapping[str, Any]
    ) -> Optional[Tuple[int, int]]:
        """Parse the given response into an recording fbid and remaining ttl tuple"""
        recording = response["recording"]
        if recording:
            return int(recording["id"]), int(recording["remaining_ttl"])
        return None
