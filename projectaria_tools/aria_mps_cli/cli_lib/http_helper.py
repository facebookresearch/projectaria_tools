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
from typing import Any, Final, Mapping

from aiohttp import ClientSession
from aiohttp.client_exceptions import ContentTypeError

_AUTHORIZATION: Final[str] = "Authorization"
_AUTH_TOKEN: Final[str] = "auth_token"

logger = logging.getLogger(__name__)


class HttpHelper:
    """
    Helper class to make requests to the MPS service and authentication.
    It leverages aiohttp for async http requests.
    """

    def __init__(self):
        logger.debug("Creating http session")
        self._http_session: ClientSession = ClientSession(raise_for_status=True)

    @property
    def session(self) -> ClientSession:
        """
        Get the http session
        """
        return self._http_session

    async def __aenter__(self) -> "HttpHelper":
        """
        Enter context manager and return the instance itself
        """
        return self

    async def __aexit__(self, exc_type, exc_value, traceback) -> None:
        """
        Exit context manager and close the http session
        """
        await self.close()

    async def close(self):
        """
        Close the http session
        """
        logger.debug("Closing http session")
        await self._http_session.close()

    def set_auth_token(self, auth_token: str) -> None:
        """
        Set the authentication token header for Http Session
        """
        logger.debug("Setting http session auth header")
        self._http_session.headers["Authorization"] = f"OAuth {auth_token}"

    async def post(self, **kwargs) -> Mapping[str, Any]:
        """
        Helper to make POST request to the given URL with the given parameters.
        """
        return await self._run_method("post", **kwargs)

    async def get(self, **kwargs) -> Mapping[str, Any]:
        """
        Helper to make GET request to the given URL with the given parameters.
        """
        return await self._run_method("get", **kwargs)

    async def _run_method(self, method: str, **kwargs) -> Mapping[str, Any]:
        """
        Helper function to get/post to an endpoint and return the JSON response
        Insert the authorization header if it's not present.
        """
        headers = kwargs.pop("headers", {})
        if auth_token := kwargs.pop(_AUTH_TOKEN, None):
            headers[_AUTHORIZATION] = f"OAuth {auth_token}"
        m = getattr(self._http_session, method)
        async with m(headers=headers, **kwargs) as r:
            try:
                return await r.json()
            except ContentTypeError:
                return json.loads(await r.text())
            except Exception as e:
                logger.exception(e)
                raise e
