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

import base64
import json
import logging
import time
from http import HTTPStatus
from typing import Any, Final, Mapping, Optional

from Crypto.Cipher import AES, PKCS1_v1_5
from Crypto.PublicKey import RSA
from Crypto.Random import get_random_bytes

from .common import retry
from .constants import AUTH_TOKEN_FILE
from .graphql_query import GraphQLQueryExecutor
from .http_helper import HttpHelper

logger = logging.getLogger(__name__)

# OAuth FRL|App ID| Client Token
_CLIENT_TOKEN: Final[str] = "FRL|844405224048903|ed1d75011c9a461b1f6c83c91f1fecb9"
_DOC_ID_GET_HORIZON_PROFILE_TOKEN: int = 24299011599746673
_URL_ACCOUNTS_LOGIN: str = "https://meta.graph.meta.com/accounts_login"
_URL_ACCOUNTS_LOGOUT: str = "https://graph.oculus.com/logout"
_URL_META_GQL: str = "https://meta.graph.meta.com/graphql"
_URL_ENCRYPTION_KEY: str = "https://meta.graph.meta.com/passwords_encryption?version=2"


class AuthenticationError(RuntimeError):
    """
    Raised when authentication fails
    """

    pass


class Authenticator:
    """
    Class to handle authentication with the MPS backend.
    It supports:
    - Logging in using Meta account
    - Logging out
    - Cache the authentication token
    - Loading and validating cached token

    """

    def __init__(self, http_helper: HttpHelper, client_token: str = _CLIENT_TOKEN):
        self._http_helper: HttpHelper = http_helper
        self._query_exec: GraphQLQueryExecutor = GraphQLQueryExecutor(http_helper)
        self._client_token: str = client_token
        self._auth_token: Optional[str] = None
        self._user_alias: Optional[str] = None

    @property
    def user(self) -> str:
        """
        Return the alias of the logged in user
        """
        return self._user_alias

    @property
    def auth_token(self) -> str:
        """
        Return the auth token
        """
        return self._auth_token

    def is_logged_in(self) -> bool:
        """
        Returns True if the user is logged in
        """
        return self._auth_token is not None

    async def load_and_validate_token(self) -> bool:
        """
        Reads the token from the local disk if present and validates it.
        Returns:
            True if the token is found and is valid
        """
        if not AUTH_TOKEN_FILE.is_file():
            logger.debug("No cached token found")
            return False
        logger.debug("Reading cached token")
        with AUTH_TOKEN_FILE.open("r") as f:
            self._auth_token = f.read().strip()

        self._user_alias = await self._get_user_alias()
        return self._user_alias is not None

    async def login(self, username: str, password: str, save_token: bool) -> bool:
        """
        Authenticate using the provided credentials and returns the authentication token
        The login is done in 2 steps:
        1. Login using Meta account and get the access token
        2. Use the access token to get the profile scoped access token
        Args:
            username: Username
            password: Password
            save_token: If true, saves the token to the local disk
        Returns:
            True if the login was successful
        """
        ## Step 1 : Login using Meta account
        response: Mapping[str, Any] = {}
        # 1.1 Get public key
        response = await self._http_helper.post(
            url=_URL_ENCRYPTION_KEY,
            auth_token=self._client_token,
        )
        if "key_id" not in response or "public_key" not in response:
            raise AuthenticationError(
                f"Getting public key failed with response '{json.dumps(response, indent=2)}'"
            )

        # 1.2 Encrypt password
        encrypted_password: bytearray = self._encrypt_password(
            key_id=response["key_id"],
            pub_key=response["public_key"],
            raw_password=password,
        )

        # 1.3 Login
        try:
            response = await self._http_helper.post(
                url=_URL_ACCOUNTS_LOGIN,
                auth_token=self._client_token,
                json={
                    "contact_point": _get_email(username),
                    "password": encrypted_password,
                },
            )
        except Exception as e:
            raise AuthenticationError(f"Login failed with exception {e}")
        if "access_token" not in response:
            raise AuthenticationError(
                f"Login failed with response '{json.dumps(response, indent=2)}"
            )
        logger.debug("Got meta account access token")
        user_access_token = response["access_token"]

        # Step 2 : Get Horizon profile scoped access token
        response = await self._http_helper.post(
            url=_URL_META_GQL,
            json={"doc_id": _DOC_ID_GET_HORIZON_PROFILE_TOKEN},
            auth_token=user_access_token,
        )

        try:
            self._auth_token = response["data"]["xfr_create_profile_token"][
                "profile_tokens"
            ][0]["access_token"]
            if save_token:
                self._cache_token()
        except KeyError as e:
            raise AuthenticationError(
                f"Getting profile scoped access token failed with response '{response}'"
            ) from e

        logger.debug("Got profile scoped access token")
        self._user_alias = await self._get_user_alias()
        return self._user_alias is not None

    @retry(
        error_codes=[
            HTTPStatus.REQUEST_TIMEOUT,
            HTTPStatus.SERVICE_UNAVAILABLE,
            HTTPStatus.BAD_REQUEST,
        ],
    )
    async def logout(self) -> bool:
        """
        Log out the current user.
        """
        logger.info("Logging out...")
        if not self._auth_token and not await self.load_and_validate_token():
            logger.info("Not logged in and cached token invalid or not found")
            return False

        try:
            await self._http_helper.post(
                url=_URL_ACCOUNTS_LOGOUT, auth_token=self._auth_token
            )
            logger.info("Logged out successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to log out: {e}")
            return False
        finally:
            self._auth_token = None
            logger.info("Removing cached token")
            AUTH_TOKEN_FILE.unlink(missing_ok=True)

    def _cache_token(self) -> None:
        """
        Cache the token locally
        """
        logger.info(f"Caching token to {AUTH_TOKEN_FILE}")
        AUTH_TOKEN_FILE.parent.mkdir(parents=True, exist_ok=True)
        with AUTH_TOKEN_FILE.open("w") as f:
            f.write(self._auth_token)

    def _encrypt_password(self, key_id: int, pub_key: str, raw_password: str) -> str:
        """Encrypts the password using the public key

        Args:
            key_id: The key id used to identify the key
            pub_key: The public key used to encrypt the password
            raw_password: The password to encrypt

        Returns:
            The encrypted password in the format expected by the auth service
        """
        version_byte: int = 1
        encrypted_data: bytearray = bytearray([version_byte, key_id])
        iv = get_random_bytes(12)
        encrypted_data.extend(iv)

        # Generate a random symmetric key and encrypt it using the public key
        symmetric_key = get_random_bytes(32)  # for AES-256
        encrypted_key = PKCS1_v1_5.new(RSA.import_key(pub_key)).encrypt(symmetric_key)
        # Write encrypted key length (256 bytes) in little endian
        encrypted_data.extend(len(encrypted_key).to_bytes(2, byteorder="little"))
        # Write encrypted key
        encrypted_data.extend(encrypted_key)

        # Initialize a cipher with the symmetric key
        cipher = AES.new(symmetric_key, AES.MODE_GCM, iv)
        aad = str(int(time.time()))
        cipher.update(aad.encode())
        cipher_text, tag = cipher.encrypt_and_digest(raw_password.encode())
        encrypted_data.extend(tag)
        encrypted_data.extend(cipher_text)
        return f"#PWD_ENC:2:{aad}:{base64.urlsafe_b64encode(encrypted_data).decode()}"

    async def _get_user_alias(self) -> Optional[str]:
        """
        Gets the alias of the currently logged in user
        """
        try:
            ## This will throw if the token is invalid
            return await self._query_exec.query_me(auth_token=self._auth_token)
        except Exception:
            logger.warning("Token is invalid.")
            return None


def _get_email(username: str) -> str:
    """
    Converts the username to email format if necessary.
    This function is needed because academic partner accounts are provisioned by us and
    they all have tfbnw.net domain. When logging in to the mobile CA, they are used to
    only entering the username without the @tfbnw.net suffix. However, we need the full
    email address to authenticate
    """
    _DOMAIN: str = "tfbnw.net"
    if username.lower().endswith(f"@{_DOMAIN}"):
        return username
    if username.count("@") >= 1:
        raise ValueError(
            f"Invalid email address: {username}. Expected format: <username>@tfbnw.net"
        )
    return f"{username}@{_DOMAIN}"
