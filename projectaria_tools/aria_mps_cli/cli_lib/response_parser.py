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

from .constants import (
    KEY_CDN_URL,
    KEY_CREATION_TIME,
    KEY_ERROR_CODE,
    KEY_FEATURE,
    KEY_FEATURES,
    KEY_FILE_HASH,
    KEY_ID,
    KEY_MPS_OUTPUTS,
    KEY_MPS_RESULTS,
    KEY_NAME,
    KEY_NODES,
    KEY_PERSIST_ON_FAILURE,
    KEY_RECORDING_HASH,
    KEY_RECORDING_NAME,
    KEY_RECORDINGS,
    KEY_REMAINING_TTL,
    KEY_RESULT_TYPE,
    KEY_STATUS,
    KEY_STATUS_MESSAGE,
)

from .types import (
    MpsFeature,
    MpsFeatureRequest,
    MpsOutput,
    MpsRequest,
    MpsResult,
    MpsResultType,
    Status,
)


class ResponseParser:
    """Class for parsing responses from GraphQL queries"""

    @staticmethod
    def parse_mps_request(
        response: Optional[Mapping[str, Any]],
    ) -> Optional[MpsRequest]:
        """Parse the given response into an MpsRequest object."""
        if not response:
            return None
        features = response[KEY_FEATURES][KEY_NODES]
        # We have a mix of graphql queries. Some include recordings and some don't so we
        # need to check for that
        recordings = response.get(KEY_RECORDINGS, {}).get(KEY_NODES, [])
        mps_request: MpsRequest = MpsRequest(
            fbid=response[KEY_ID],
            name=response[KEY_NAME],
            creation_time=response[KEY_CREATION_TIME],
            recordings_fbids=[r[KEY_ID] for r in recordings],
            features={
                MpsFeature(f[KEY_FEATURE]): ResponseParser.parse_mps_feature_request(f)
                for f in features
            },
            persist_on_failure=response.get(KEY_PERSIST_ON_FAILURE, False),
        )
        return mps_request

    @staticmethod
    def parse_mps_feature_request(response: Mapping[str, Any]) -> MpsFeatureRequest:
        """
        Parse the given response into an MpsFeatureRequest object.
        The response may not contain results if they were not queried
        """
        return MpsFeatureRequest(
            fbid=response[KEY_ID],
            status=Status(response[KEY_STATUS]),
            feature=MpsFeature(response[KEY_FEATURE]),
            error_code=response[KEY_ERROR_CODE],
            status_message=response.get(KEY_STATUS_MESSAGE),
            creation_time=response[KEY_CREATION_TIME],
            results=ResponseParser.parse_results(
                response.get(KEY_MPS_RESULTS, {}).get(KEY_NODES, {})
            ),
            outputs=ResponseParser.parse_outputs(
                response.get(KEY_MPS_OUTPUTS, {}).get(KEY_NODES, {})
            ),
        )

    @staticmethod
    def parse_results(response: Mapping[str, Any]) -> List[MpsResult]:
        """Parse the given response into an list of MpsResult objects."""
        mps_results: List[MpsResult] = []
        for resp in response:
            mps_results.append(
                MpsResult(
                    fbid=resp["id"],
                    result_type=MpsResultType(resp[KEY_RESULT_TYPE]),
                    cdn_url=resp[KEY_CDN_URL],
                    recording_hash=(
                        resp[KEY_RECORDING_HASH]
                        if KEY_RECORDING_HASH in resp
                        # try to use deprecated value as a fallback
                        else resp[KEY_FILE_HASH]
                    ),
                    recording_name=resp.get(KEY_RECORDING_NAME),
                )
            )
        return mps_results

    @staticmethod
    def parse_outputs(response: Mapping[str, Any]) -> List[MpsOutput]:
        """Parse the given response into an list of MpsResult objects."""
        mps_outputs: List[MpsOutput] = []
        for resp in response:
            mps_outputs.append(
                MpsOutput(
                    fbid=resp["id"],
                    result_type=MpsResultType(resp[KEY_RESULT_TYPE]),
                    cdn_url=resp[KEY_CDN_URL],
                    recording_hash=resp[KEY_RECORDING_HASH],
                )
            )
        return mps_outputs

    @staticmethod
    def parse_recording_id_and_ttl(
        response: Mapping[str, Any],
    ) -> Optional[Tuple[int, int]]:
        """Parse the given response into an recording fbid and remaining ttl tuple"""
        recording = response["recording"]
        if recording:
            return int(recording[KEY_ID]), int(recording[KEY_REMAINING_TTL])
        return None
