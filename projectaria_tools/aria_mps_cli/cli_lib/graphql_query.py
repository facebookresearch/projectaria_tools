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
import os
from typing import Any, Dict, Final, List, Optional, Set, Tuple

from .common import Config, retry
from .constants import (
    ConfigKey,
    ConfigSection,
    HTTP_RETRY_CODES,
    KEY_ALIAS,
    KEY_ARIA_MPS_REQUEST,
    KEY_CREATE,
    KEY_CURSOR,
    KEY_DATA,
    KEY_DOC_ID,
    KEY_END_CURSOR,
    KEY_FEATURE,
    KEY_FEATURES,
    KEY_FILE_HASH,
    KEY_FILE_HASHES,
    KEY_HAS_NEXT_PAGE,
    KEY_ID,
    KEY_INPUT,
    KEY_KEY_ID,
    KEY_ME,
    KEY_NAME,
    KEY_NODE,
    KEY_NODES,
    KEY_PAGE_INFO,
    KEY_PAGE_SIZE,
    KEY_PUBLIC_KEY,
    KEY_RECORDINGS,
    KEY_REQUEST_ID,
    KEY_REQUESTS,
    KEY_RESPONSE,
    KEY_SOURCE,
    KEY_VARIABLES,
)

from .http_helper import HttpHelper
from .response_parser import ResponseParser
from .types import GraphQLError, MpsFeatureRequest, MpsRequest, MpsRequestSource

logger = logging.getLogger(__name__)

## Doc ids for various graphql queries and mutations
_DOC_ID_SUBMIT_MPS_REQUEST: Final[int] = 7600943133300278
_DOC_ID_QUERY_MPS_REQUESTED_FEATURE_BY_FILE_HASH: Final[int] = 7587879547939670
_DOC_ID_QUERY_MPS_REQUESTED_FEATURE_BY_FILE_HASH_SET: Final[int] = 25450657081245347
_DOC_ID_QUERY_FEATURE_REQUEST: Final[int] = 7490684447654027
_DOC_ID_GET_HORIZON_PROFILE_TOKEN: Final[int] = 24299011599746673
_DOC_ID_QUERY_RECORDING_BY_FILE_HASH: Final[int] = 6818108551598913
_DOC_ID_QUERY_ME: Final[int] = 7092145450831175
_DOC_ID_QUERY_MPS_REQUEST: Final[int] = 8119818841375479
_DOC_ID_QUERY_MPS_REQUESTS: Final[int] = 7092586410844521
_DOC_ID_QUERY_PUBLIC_ENCRYPTION_KEY: Final[int] = 7360371104027087
# hard, server-side limit for the number of requests to be returned in a single response is 10 000
_QUERY_DEFAULT_PAGE_SIZE: Final[int] = 1000

_GQL_URL: Final[str] = os.environ.get("GQL_URL", "https://graph.oculus.com/graphql")
_AUTHORIZATION: Final[str] = "Authorization"
_AUTH_TOKEN: Final[str] = "auth_token"

config = Config.get()


class GraphQLQueryExecutor:
    """
    Helper class to send GraphQL queries to the MPS service.
    Uses HttpHelper to send the requests and parse the responses.
    """

    def __init__(self, http_helper: HttpHelper):
        self._http_helper: HttpHelper = http_helper

    async def submit_request(
        self,
        name: str,
        recording_ids: List[int],
        features: Set[str],
        source: MpsRequestSource,
    ) -> MpsRequest:
        """
        Submit a request to the MPS service to process the given recording id.
        """
        logger.debug(
            f"Submitting MPS request. Name: {name}, Recordings: {recording_ids}, Features: {features}"
        )
        input = {
            KEY_NAME: name,
            KEY_RECORDINGS: recording_ids,
            KEY_FEATURES: list(features),
            KEY_SOURCE: source.value,
        }
        if extra_input := os.environ.get("MPS_EXTRA_INPUT"):
            input = {**input, **json.loads(extra_input)}
        response = await self.query_graph(
            doc_id=_DOC_ID_SUBMIT_MPS_REQUEST, variables={KEY_INPUT: input}
        )
        return ResponseParser.parse_mps_request(
            response[KEY_DATA][KEY_CREATE][KEY_ARIA_MPS_REQUEST]
        )

    async def query_me(self, auth_token: Optional[str] = None) -> str:
        """
        Get the info about the logged in user and return the alias.
        The main purpose of this query is to validate the auth token.
        """
        logger.debug("Querying me")
        response = await self.query_graph(
            doc_id=_DOC_ID_QUERY_ME, variables={}, auth_token=auth_token
        )
        return response[KEY_DATA][KEY_ME][KEY_ALIAS]

    async def query_encryption_key(self) -> Tuple[str, int]:
        """
        Obtain public key used to encrypt Aria recording before sending it for MPS processing.
        """
        logger.debug("Querying Encryption public key")
        response = await self.query_graph(
            doc_id=_DOC_ID_QUERY_PUBLIC_ENCRYPTION_KEY,
        )
        return (
            response[KEY_DATA][KEY_RESPONSE][KEY_PUBLIC_KEY],
            int(response[KEY_DATA][KEY_RESPONSE][KEY_KEY_ID]),
        )

    async def query_recording_by_file_hash(
        self, file_hash: str
    ) -> Optional[Tuple[int, int]]:
        """
        Given a file hash, find the recording id associated with it.
        """
        logger.debug(f"Querying recording by file hash {file_hash}")
        response = await self.query_graph(
            doc_id=_DOC_ID_QUERY_RECORDING_BY_FILE_HASH,
            variables={KEY_FILE_HASH: file_hash},
        )
        return ResponseParser.parse_recording_id_and_ttl(response[KEY_DATA])

    async def query_mps_requested_features_by_file_hash(
        self, file_hash: str
    ) -> List[MpsFeatureRequest]:
        """
        Query the latest MPS requested features associated with the given
        file hash that cover all the single sequence features (SLAM and EYE Gaze).

        Args:
            file_hash: A file hash.
        Returns:
            A list of MpsFeatureRequest objects unique per type.
        """
        logger.debug(f"Querying mps feature requests by file hash {file_hash}")
        response = await self.query_graph(
            doc_id=_DOC_ID_QUERY_MPS_REQUESTED_FEATURE_BY_FILE_HASH,
            variables={KEY_FILE_HASH: file_hash},
        )
        return [
            ResponseParser.parse_mps_feature_request(feature)
            for feature in response[KEY_DATA][KEY_FEATURES][KEY_NODES]
        ]

    async def query_mps_requested_feature_by_file_hash_set(
        self, file_hashes: Set[str]
    ) -> Optional[MpsFeatureRequest]:
        """
        Query the last MPS feature request associated exclusively with the
        given file hash set.

        Args:
            file_hashes: A set of file hashes.
        Returns:
            An MpsFeatureRequest object or None if no such request exists.
        """
        logger.debug(f"Querying mps feature request by file hash set {file_hashes}")
        response = await self.query_graph(
            doc_id=_DOC_ID_QUERY_MPS_REQUESTED_FEATURE_BY_FILE_HASH_SET,
            variables={KEY_FILE_HASHES: file_hashes},
        )

        feature = response[KEY_DATA][KEY_FEATURE]
        return (
            ResponseParser.parse_mps_feature_request(feature)
            if feature is not None
            else None
        )

    async def query_feature_request(
        self, requested_feature_id: int
    ) -> MpsFeatureRequest:
        """
        Query the status of requested feature id
        """
        logger.debug(f"Querying feature request {requested_feature_id}")
        response = await self.query_graph(
            doc_id=_DOC_ID_QUERY_FEATURE_REQUEST,
            variables={KEY_REQUEST_ID: requested_feature_id},
        )
        return ResponseParser.parse_mps_feature_request(response[KEY_DATA][KEY_NODE])

    async def query_mps_request(self, request_id: int) -> MpsRequest:
        """
        Query the status of the given MPS request
        """
        response = await self.query_graph(
            doc_id=_DOC_ID_QUERY_MPS_REQUEST,
            variables={KEY_ID: request_id},
        )
        return ResponseParser.parse_mps_request(response[KEY_DATA][KEY_NODE])

    async def query_all_mps_requests(
        self,
    ) -> List[MpsRequest]:
        """Query all MPS requests by the user"""
        logger.debug("Querying all mps requests")
        requests: List[MpsRequest] = []
        has_next: bool = True
        cursor: Optional[str] = None

        while has_next:
            response = await self.query_graph(
                doc_id=_DOC_ID_QUERY_MPS_REQUESTS,
                variables={
                    KEY_PAGE_SIZE: _QUERY_DEFAULT_PAGE_SIZE,
                    KEY_CURSOR: cursor,
                },
            )
            for r in response[KEY_DATA][KEY_REQUESTS][KEY_NODES]:
                request = ResponseParser.parse_mps_request(r)
                # We currently don't need recording fbids in the response
                request.recordings_fbids = None
                requests.append(request)

            page_info = response[KEY_DATA][KEY_REQUESTS][KEY_PAGE_INFO]
            has_next = page_info[KEY_HAS_NEXT_PAGE]
            cursor = page_info[KEY_END_CURSOR]
        return requests

    @retry(
        error_codes=HTTP_RETRY_CODES,
        exceptions=[GraphQLError],
        retries=config.getint(ConfigSection.GRAPHQL, ConfigKey.RETRIES),
        interval=config.getfloat(ConfigSection.GRAPHQL, ConfigKey.INTERVAL),
        backoff=config.getfloat(ConfigSection.GRAPHQL, ConfigKey.BACKOFF),
    )
    async def query_graph(
        self, doc_id: int, variables: Optional[Dict[str, Any]] = None, **kwargs
    ) -> Dict[str, Any]:
        """
        Query the graph data for the given parameters
        Args:
            doc_id: GraphQL document ID.
            variables: Variables to be passed to the query.
        """
        response = await self._http_helper.post(
            url=_GQL_URL,
            json={
                KEY_DOC_ID: doc_id,
                KEY_VARIABLES: variables,
            },
            **kwargs,
        )
        logger.debug(f"GraphQL response {json.dumps(response, indent=2)}")
        if "errors" in response:
            logger.error(f"GraphQL errors: {response['errors']}")
            raise GraphQLError(response["errors"])
        return response
