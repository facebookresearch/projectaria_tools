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

import asyncio
import io
import logging
import shutil
import tempfile
from asyncio import Semaphore
from multiprocessing import connection, Pipe
from pathlib import Path
from typing import final, Final

import aiofiles
from Crypto.Cipher import AES, PKCS1_v1_5
from Crypto.PublicKey import RSA
from Crypto.Random import get_random_bytes

from .common import Config, CustomAdapter, to_proc
from .constants import ConfigKey, ConfigSection
from .runner_with_progress import RunnerWithProgress

config = Config.get()


class VrsEncryptor(RunnerWithProgress):
    """
    Encrypt a vrs file and report progress
    """

    semaphore_: Final[Semaphore] = Semaphore(
        config.getint(ConfigSection.ENCRYPTION, ConfigKey.CONCURRENT_ENCRYPTIONS)
    )

    def __init__(
        self, src_path: Path, dest_path: Path, encryption_key: str, key_id: int
    ):
        super().__init__()
        self._src_path = src_path
        self._dest_path = dest_path
        self._encryption_key = encryption_key
        self._encryption_key_id = key_id
        self._logger: CustomAdapter = CustomAdapter(
            logging.getLogger(__name__), {"vrs": str(src_path)}
        )

    @final
    async def run(self) -> str:
        """
        Encrypt the source file and write it to destination
        Encryption is a CPU intensive operation, so in order to avoid blocking the event
        loop, we run it in a ProcessPoolExecutor.
        The encryption progress is reported back via a pipe.
        """

        self._total = (await aiofiles.os.stat(self._src_path)).st_size
        async with self.semaphore_:
            receiver, sender = Pipe(duplex=False)
            encrypt_task: asyncio.Task = asyncio.create_task(
                to_proc(
                    _encrypt_file,
                    self._src_path,
                    self._dest_path,
                    self._encryption_key,
                    self._encryption_key_id,
                    config.getint(ConfigSection.ENCRYPTION, ConfigKey.CHUNK_SIZE),
                    sender,
                )
            )
            done = None
            while not done:
                done, pending = await asyncio.wait([encrypt_task], timeout=1)
                while receiver.poll():
                    self._processed = receiver.recv()
                self._logger.debug(f"Encrypted {self.progress:.2f}%")
            receiver.close()
            sender.close()


class FileEncryptor:
    """
    This class is responsible for performing envelope encryption of the vrs file.
    TODO: Handle files bigger than 64 GB
    """

    def __init__(
        self,
        encryption_key: str,
        key_id: int,
        chunk_size: int,
        src_file: Path,
        dest_file: Path,
    ):
        self._crypto_version: int = 1
        self._symmetric_key_length: int = 32
        self._k_iv_len: int = 12
        self._k_aria_encryption_version: int = 1
        self._chunk_size = chunk_size

        self._crypto_key_id: int = key_id
        self._pub_key: RSA.RSAKey = RSA.import_key(encryption_key)
        self._src_file: Path = src_file
        self._dest_file: Path = dest_file
        self._logger: CustomAdapter = CustomAdapter(
            logging.getLogger(__name__), {"vrs": str(src_file)}
        )

    def _header(self, dest_stream: io.BytesIO):
        """
        output format: [version: 1byte][key ID: 1byte][IV: 12 bytes][key size: 2bytes][key:
        256bytes][tag: 16 bytes][ciphertext: 45 bytes] ciphertext: [symmetric key: 32 bytes][IV: 12
        bytes][Aria Encryption Version: 1 byte]
        """
        self._logger.debug("Writing encryption header")

        # Even though the payload to encrypt is just 45 bytes
        # (32 bytes symmetric key + 12 bytes IV + 1 byte), much smaller than
        # 245 bytes, the limit of RSA-2056 encryption, we need to perform an envelope
        # encryption  because the Crypto service used for remote decryption on backend
        # expects it.
        encrypted_data: bytearray = bytearray(
            [self._crypto_version, self._crypto_key_id]
        )

        envelope_iv = get_random_bytes(self._k_iv_len)
        encrypted_data.extend(envelope_iv)

        # Generate a random symmetric key and encrypt it using the public key
        envelope_symmetric_key = get_random_bytes(self._symmetric_key_length)
        ciphertext = PKCS1_v1_5.new(self._pub_key).encrypt(envelope_symmetric_key)

        # Write encrypted key length (256 bytes) in little endian
        encrypted_data.extend(len(ciphertext).to_bytes(2, byteorder="little"))
        # Write encrypted key
        encrypted_data.extend(ciphertext)
        envelope_cipher = AES.new(envelope_symmetric_key, AES.MODE_GCM, envelope_iv)

        symmetric_key = get_random_bytes(self._symmetric_key_length)
        iv = get_random_bytes(self._k_iv_len)

        self._cipher = AES.new(symmetric_key, AES.MODE_GCM, iv)

        plain_text: bytearray = bytearray()
        plain_text.extend(symmetric_key)
        plain_text.extend(iv)
        plain_text.append(self._k_aria_encryption_version)

        cipher_text, tag = envelope_cipher.encrypt_and_digest(plain_text)
        encrypted_data.extend(tag)
        encrypted_data.extend(cipher_text)

        dest_stream.write(encrypted_data)

    def _write_data(self, dest_stream: io.BytesIO, conn: connection.Connection):
        """Encrypt the data from the source file and write to the destination stream"""
        with self._src_file.open("rb") as fr:
            processed_bytes: int = 0
            while chunk := fr.read(self._chunk_size):
                if len(chunk) == 0:
                    break
                cipher_text = self._cipher.encrypt(chunk)
                dest_stream.write(cipher_text)
                processed_bytes += len(chunk)
                self._logger.debug(f"Encrypted {processed_bytes} bytes")
                conn.send(processed_bytes)
            dest_stream.write(self._cipher.digest())
        self._logger.debug("Finished writing encrypted data")

    def encrypt(self, conn: connection.Connection) -> None:
        """Perform encryption"""
        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = Path(tmp_dir) / "encrypted.vrs"
            with tmp_path.open("wb") as dest:
                self._header(dest)
                self._write_data(dest, conn)
            # once encryption is done, move the file to its final location
            shutil.move(str(tmp_path), str(self._dest_file))


def _encrypt_file(
    src_file: Path,
    dest_file: Path,
    encryption_key: str,
    key_id: int,
    chunk_size: int,
    conn: connection.Connection,
) -> None:
    """Helper function to pass to the process pool
    Args:
        src_file: The file to encrypt
        dest_file: The encrypted file to create
        encryption_key: The RSA public key to encrypt the file
        key_id: The ID of provided encryption key
        chunk_size: The size of each chunk to encrypt
        conn: The connection (Pipe) to send progress updates over

    We use a temporary directory to avoid having to deal with partially encrypted file
    """
    FileEncryptor(encryption_key, key_id, chunk_size, src_file, dest_file).encrypt(conn)
