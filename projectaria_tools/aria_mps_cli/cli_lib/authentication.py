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
import os
import stat
import time
from http import HTTPStatus
from typing import Any, Final, Mapping, Optional

try:
    import keyring

    KEYRING_AVAILABLE = True
except ImportError:
    KEYRING_AVAILABLE = False

from Crypto.Cipher import AES, PKCS1_v1_5
from Crypto.PublicKey import RSA
from Crypto.Random import get_random_bytes

from .common import retry
from .constants import (
    AUTH_TOKEN_FILE,
    ENCRYPTION_KEY_FILE,
    KEY_ACCESS_TOKEN,
    KEY_APP_ID,
    KEY_CONTACT_POINT,
    KEY_CREATE_TOKEN,
    KEY_DATA,
    KEY_DOC_ID,
    KEY_KEY_ID,
    KEY_PASSWORD,
    KEY_PROFILE_TOKENS,
    KEY_PUBLIC_KEY,
    KEY_VARIABLES,
)
from .http_helper import HttpHelper

logger = logging.getLogger(__name__)

# Keyring constants
_KEYRING_SERVICE_NAME: Final[str] = "projectaria_tools"
_KEYRING_SESSION_TOKEN_KEY: Final[str] = "session_token_key"

# OAuth FRL|FRL App ID| Client Token
_CLIENT_TOKEN: Final[str] = "FRL|844405224048903|ed1d75011c9a461b1f6c83c91f1fecb9"
# OC App ID
_CLIENT_APPLICATION: Final[int] = 6715036791927135
_DOC_ID_GET_HORIZON_PROFILE_TOKEN: Final[int] = 7806394482748426
_URL_ACCOUNTS_LOGIN: Final[str] = "https://meta.graph.meta.com/accounts_login"
_URL_ACCOUNTS_LOGOUT: Final[str] = "https://graph.oculus.com/logout"
_URL_META_GQL: Final[str] = "https://meta.graph.meta.com/graphql"
_URL_ENCRYPTION_KEY: Final[str] = (
    "https://meta.graph.meta.com/passwords_encryption?version=2"
)


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

    def __init__(
        self,
        http_helper: HttpHelper,
        client_token: str = _CLIENT_TOKEN,
        client_app: int = _CLIENT_APPLICATION,
    ):
        self._http_helper: HttpHelper = http_helper
        self._client_token: str = client_token
        self._client_app: int = client_app
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

    async def set_auth_token(self, token: str, save_token: bool = False) -> None:
        """
        Sets the authentication token.

        Args:
            token (str): The authentication token to be set.
            save_token (bool): Optional; Whether to save the token. Defaults to False.

        """

        # Validate the input token
        if not isinstance(token, str):
            raise ValueError("Token must be a string")

        self._auth_token = token
        self._clear_cached_token()

        self._user_alias = await self._get_user_alias()
        if not self._user_alias:
            logger.error("Failed to get user alias: Token is invalid.")
            self._user_alias = None
            self._auth_token = None
            raise ValueError("Token is invalid")

        # Save the token if required
        if save_token:
            self._cache_token()

    async def load_and_validate_token(self) -> bool:
        """
        Reads the token from the local disk if present and validates it.
        Handles both encrypted tokens (newer versions) and unencrypted tokens (older versions).

        Returns:
            True if the token is found and is valid
        """
        if not AUTH_TOKEN_FILE.is_file() or not os.access(AUTH_TOKEN_FILE, os.R_OK):
            logger.debug("No cached token found")
            return False

        try:
            logger.debug("Reading cached token")
            with AUTH_TOKEN_FILE.open("rb") as f:
                # Read the first byte to check if it's a version byte
                first_byte = f.read(1)

                # If the first byte is 0x01, it's an encrypted token (version 1)
                if first_byte == b"\x01":
                    # Continue with decryption process for encrypted tokens
                    iv = f.read(12)
                    tag = f.read(16)
                    cipher_text = f.read()

                    key = self.get_encryption_key()
                    cipher = AES.new(key, AES.MODE_GCM, iv)
                    self._auth_token = cipher.decrypt_and_verify(
                        cipher_text, tag
                    ).decode()
                else:
                    # Handle unencrypted token from older versions
                    # Seek back to the beginning of the file
                    f.seek(0)
                    self._auth_token = f.read().decode().strip()
                    logger.info(
                        "Found unencrypted token from older version - upgrading to encrypted format"
                    )
                    self._cache_token()
                    logger.info("Token successfully upgraded to encrypted format")

                self._user_alias = await self._get_user_alias()
                return True
        except (ValueError, KeyError, OSError) as e:
            logger.error(f"Token loading failed: {e}")
            AUTH_TOKEN_FILE.unlink(missing_ok=True)
            return False
        except Exception as e:
            logger.error(f"Unexpected error while loading token: {e}")
            # removing the auth token cache file
            AUTH_TOKEN_FILE.unlink(missing_ok=True)
            return False

    def get_encryption_key(self) -> bytes:
        """
        Generate or retrieve the encryption key used for token encryption/decryption.

        This function attempts to retrieve an existing encryption key from the system keyring
        if available. If the key doesn't exist in the keyring, it generates a new random 32-byte key
        and stores it in the keyring for future use.

        If the keyring is not available (e.g., due to missing dependencies or permissions),
        it falls back to using a local file (ENCRYPTION_KEY_FILE) to store and retrieve the key.
        If the file doesn't exist, it creates a new random key, saves it to the file with
        appropriate permissions (read/write for the owner only), and returns it.

        Returns:
            bytes: A 32-byte encryption key for AES-256 encryption
        """
        if KEYRING_AVAILABLE:
            try:
                key_str = keyring.get_password(
                    _KEYRING_SERVICE_NAME, _KEYRING_SESSION_TOKEN_KEY
                )
                if not key_str:
                    key = get_random_bytes(32)
                    # Store as base64 string to handle Windows keyring string conversion
                    key_b64 = base64.b64encode(key).decode("ascii")
                    keyring.set_password(
                        _KEYRING_SERVICE_NAME, _KEYRING_SESSION_TOKEN_KEY, key_b64
                    )
                    return key
                else:
                    try:
                        return base64.b64decode(key_str.encode("ascii"))
                    except Exception:
                        # If decoding fails, delete the corrupted key
                        logger.warning(
                            "Invalid key format in keyring, deleting corrupted key"
                        )
                        try:
                            keyring.delete_password(
                                _KEYRING_SERVICE_NAME, _KEYRING_SESSION_TOKEN_KEY
                            )
                        except Exception as delete_error:
                            logger.error(
                                f"Failed to delete corrupted key from keyring: {delete_error}"
                            )
                        # Fall through to file-based fallback
            except Exception as e:
                logger.error(f"Failed to get encryption key from keyring: {e}")
        # Fallback to the local file if keyring is not available
        return get_random_bytes(32)

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
        if KEY_KEY_ID not in response or KEY_PUBLIC_KEY not in response:
            raise AuthenticationError(
                f"Getting public key failed with response '{json.dumps(response, indent=2)}'"
            )

        # 1.2 Encrypt password
        encrypted_password: bytearray = self._encrypt_password(
            key_id=response[KEY_KEY_ID],
            pub_key=response[KEY_PUBLIC_KEY],
            raw_password=password,
        )

        # 1.3 Login
        try:
            response = await self._http_helper.post(
                url=_URL_ACCOUNTS_LOGIN,
                auth_token=self._client_token,
                json={
                    KEY_CONTACT_POINT: _get_email(username),
                    KEY_PASSWORD: encrypted_password,
                },
            )
        except Exception as e:
            raise AuthenticationError(f"Login failed with exception {e}")
        if KEY_ACCESS_TOKEN not in response:
            raise AuthenticationError(
                f"Login failed with response '{json.dumps(response, indent=2)}"
            )
        logger.debug("Got meta account access token")
        user_access_token = response[KEY_ACCESS_TOKEN]

        # Step 2 : Get Horizon profile scoped access token
        response = await self._http_helper.post(
            url=_URL_META_GQL,
            json={
                KEY_DOC_ID: _DOC_ID_GET_HORIZON_PROFILE_TOKEN,
                KEY_VARIABLES: {KEY_APP_ID: self._client_app},
            },
            auth_token=user_access_token,
        )

        try:
            self._auth_token = response[KEY_DATA][KEY_CREATE_TOKEN][KEY_PROFILE_TOKENS][
                0
            ][KEY_ACCESS_TOKEN]
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
        key = self.get_encryption_key()
        iv = get_random_bytes(12)
        cipher = AES.new(key, AES.MODE_GCM, iv)
        cipher_text, tag = cipher.encrypt_and_digest(self._auth_token.encode())

        with AUTH_TOKEN_FILE.open("wb") as f:
            f.write(b"\x01")  # Version
            f.write(iv)
            f.write(tag)
            f.write(cipher_text)
        os.chmod(AUTH_TOKEN_FILE, stat.S_IRUSR | stat.S_IWUSR)

    def _clear_cached_token(self) -> None:
        """
        Clear the locally cached token
        """
        try:
            if AUTH_TOKEN_FILE.exists():
                logger.info(f"Clearing cached token from {AUTH_TOKEN_FILE}")
                AUTH_TOKEN_FILE.unlink()
            else:
                logger.debug(f"No cached token found at {AUTH_TOKEN_FILE}")
        except OSError as e:
            logger.error(f"Failed to clear cached token: {e}")

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
            return await self._http_helper.query_me(auth_token=self._auth_token)
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
