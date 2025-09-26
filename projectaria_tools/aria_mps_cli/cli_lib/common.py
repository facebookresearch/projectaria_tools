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
import concurrent
import functools
import logging
import multiprocessing
import platform
import shutil
import tempfile
import zipfile
from configparser import ConfigParser
from functools import wraps
from pathlib import Path
from typing import (
    Any,
    Callable,
    Dict,
    MutableMapping,
    Optional,
    Sequence,
    Tuple,
    TypeVar,
)

import aiohttp

from .constants import CONFIG_FILE, ConfigKey, ConfigSection


logger = logging.getLogger(__name__)


T = TypeVar("T")


class CustomAdapter(logging.LoggerAdapter):
    """Custom logging adapter to print the contextual information like vrs file"""

    def process(
        self, msg: Any, kwargs: MutableMapping[str, str]
    ) -> Tuple[str, MutableMapping[str, str]]:
        suffix = " ".join([f"[{k}:{v}]" for k, v in self.extra.items()])
        return " ".join([str(suffix), str(msg)]), kwargs


async def to_proc(func: Callable[..., T], /, *args: Any, **kwargs: Any) -> T:
    """
    Use this for synchronous cpu bound tasks
    Main functionality is to maintain context when offloading to OS threads.
    """
    if not hasattr(to_proc, "process_pool"):
        mp_context = (
            multiprocessing.get_context("spawn")
            if platform.system() == "Windows"
            else multiprocessing.get_context("fork")
        )
        to_proc.process_pool = concurrent.futures.ProcessPoolExecutor(
            mp_context=mp_context,
        )
    loop = asyncio.get_running_loop()
    func_call = functools.partial(func, *args, **kwargs)
    return await loop.run_in_executor(to_proc.process_pool, func_call)


def unzip(zip_filepath: Path, dest_dir: Path) -> None:
    """
    Unzips a zip file into a destination directory.
    Initially unzip files into temporary directory and then moves them to final
    destination. It prevents showing incomplete output in the destination directory.
    """
    # First unzip to temporary folder
    with tempfile.TemporaryDirectory() as tmp_dir:
        with zipfile.ZipFile(zip_filepath, "r") as zip_ref:
            zip_ref.extractall(tmp_dir)
        if dest_dir.is_dir():
            shutil.rmtree(dest_dir)
        elif dest_dir.is_file():
            dest_dir.unlink()
        shutil.move(tmp_dir, dest_dir)


def retry(
    exceptions: Optional[Sequence[Exception]] = None,
    error_codes: Optional[Sequence[int]] = None,
    retries: int = 3,
    interval: int = 1,
    backoff: float = 1.5,
):
    """
    Decorator for retrying functions up to `retries` retries if exceptions are thrown.
    Does exponential backoff between retries.
    ClientResponseError from aiohttp is treated special and will only be retried if
    the status code is in `error_codes`.
    At least one of `exceptions` or `error_codes` must be provided.

    Args:
        exceptions: List of exceptions to catch
        error_codes: List of HTTP status codes to catch
        retries: Number of times to retry
        interval: Interval between retries in seconds
        backoff: Backoff multiplier

    Usage:
    @retry(exceptions=[ValueError, TypeError], retries=5, interval=2, backoff=2)
    async def my_async_function():
        # code that might raise an exception

    """
    if not exceptions and not error_codes:
        raise ValueError(
            "At least one of `exceptions` or `error_codes` must be provided."
        )
    exceptions = exceptions or []
    if error_codes and aiohttp.ClientResponseError not in exceptions:
        exceptions.append(aiohttp.ClientResponseError)

    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            attempts = 0
            while True:
                try:
                    return await func(*args, **kwargs)
                except tuple(exceptions) as e:
                    if (
                        isinstance(e, aiohttp.ClientResponseError)
                        and error_codes
                        and e.status not in error_codes
                    ):
                        logger.exception(e)
                        raise e from e
                    if attempts > retries:
                        logger.error(f"Maximum number of retries reached ({retries}).")
                        raise e from e
                    logger.warning(f"Exception encountered: {e}")
                    sleep_time = interval * (backoff ** (attempts))
                    attempts += 1
                    logger.warning(
                        f"Retrying attempt {attempts}/{retries+1} in {sleep_time} seconds..."
                    )
                    await asyncio.sleep(sleep_time)

        return wrapper

    return decorator


def log_exceptions(func):
    """Decorator to log exceptions and re-raise them"""

    @functools.wraps(func)
    def sync_wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            logger.exception(f"Exception occurred in {func.__name__} \n {e}")
            raise

    @functools.wraps(func)
    async def async_wrapper(*args, **kwargs):
        try:
            return await func(*args, **kwargs)
        except Exception as e:
            logger.exception(f"Exception occurred in {func.__name__} \n {e}")
            raise

    # Check if the function is a coroutine (async)
    if asyncio.iscoroutinefunction(func):
        return async_wrapper
    else:
        return sync_wrapper


def get_pretty_size(num_bytes: int, suffix="B"):
    """
    Returns a human readable size string
    """
    factor = 1024
    for unit in ["", "K", "M", "G", "T", "P", "E", "Z"]:
        if num_bytes < factor:
            return f"{num_bytes:.2f} {unit}{suffix}"
        num_bytes /= factor


class Config(ConfigParser):
    """
    Simple class to access the config object
    We use a singleton pattern here so we can have one instance of the config object
    across all modules
    The actual config file is located at ~/.projectaria/mps.ini
    We leverage the ConfigParser library to parse the config file
    If the config file does not exist, we create it with default values
    Users can then overwrite the values by editing the config file directly
    """

    _config = None
    # Default config values
    DEFAULT_CONFIG: Dict[str, Dict[str, str]] = {
        ConfigSection.DEFAULT: {
            ConfigKey.LOG_DIR: "/tmp/logs/projectaria/mps/ # Path to log directory",
            ConfigKey.STATUS_CHECK_INTERVAL: "30 # Status check interval in seconds",
        },
        ConfigSection.HASH: {
            ConfigKey.CONCURRENT_HASHES: "4 # Maximum number of recordings whose hashes will be calculated concurrently",
            ConfigKey.CHUNK_SIZE: "10485760 # 10 * 2**20 (10MB)",
        },
        ConfigSection.ENCRYPTION: {
            ConfigKey.CHUNK_SIZE: "52428800 # 50 * 2**20 (50MB)",
            ConfigKey.CONCURRENT_ENCRYPTIONS: "5 # Maximum number of recordings that will be encrypted concurrently",
            ConfigKey.DELETE_ENCRYPTED_FILES: "true # Delete encrypted files after upload is done",
        },
        ConfigSection.UPLOAD: {
            ConfigKey.BACKOFF: "1.5 # Backoff factor for retries",
            ConfigKey.CONCURRENT_UPLOADS: "4 # Maximum number of concurrent uploads",
            ConfigKey.INTERVAL: "20 # Interval between runs",
            ConfigKey.MAX_CHUNK_SIZE: "104857600 # 100 * 2**20 (100 MB)",
            ConfigKey.MIN_CHUNK_SIZE: "5242880 # 5 * 2**20 (5MB)",
            ConfigKey.RETRIES: "10 # Number of times to retry a failed upload",
            ConfigKey.SMOOTHING_WINDOW_SIZE: "10 # Size of the smoothing window",
            ConfigKey.TARGET_CHUNK_UPLOAD_SECS: "3 # Target duration to upload a chunk",
        },
        ConfigSection.DOWNLOAD: {
            ConfigKey.BACKOFF: "1.5 # Backoff factor for retries",
            ConfigKey.CHUNK_SIZE: "10485760 # 10 * 2**20 (10MB)",
            ConfigKey.CONCURRENT_DOWNLOADS: "10 # Maximum number of concurrent downloads",
            ConfigKey.DELETE_ZIP: "true # Delete zip files after extracting",
            ConfigKey.INTERVAL: "20 # Interval between runs",
            ConfigKey.RETRIES: "10 # Number of times to retry a failed upload",
        },
        ConfigSection.GRAPHQL: {
            ConfigKey.BACKOFF: "1.5 # Backoff factor for retries",
            ConfigKey.INTERVAL: "4 # Interval between runs",
            ConfigKey.RETRIES: "3 # Number of times to retry a failed upload",
        },
    }

    @staticmethod
    def get() -> "Config":
        """
        Get the config object
        Loads the config from ~/.projectaria/mps.ini. if it doesn't exist, creates it
        with default values
        """
        if Config._config is None:
            Config._config = ConfigParser(inline_comment_prefixes="#")

            if not CONFIG_FILE.is_file():
                c = ConfigParser()
                c.read_dict(Config.DEFAULT_CONFIG)
                CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
                with CONFIG_FILE.open("w") as fd:
                    c.write(fd)
            Config._config.read(CONFIG_FILE)
            logger.info(f"Loaded config file {CONFIG_FILE}")
        return Config._config

    @staticmethod
    def reload() -> None:
        """
        Reload the config object from the file.
        Clears the current configuration and reads the updated config file.
        Used in Aria Studio to reload the config after it is updated using UI

        This method also updates all registered components that depend on configuration values.

        Raises:
            ConfigUpdateError: If updating components fails after config reload.
        """
        if Config._config is not None:
            # Clear the current configuration
            Config._config.clear()
            # Re-read the configuration file
            Config._config.read(CONFIG_FILE)
            logger.info(f"Reloaded config file {CONFIG_FILE}")

            # Update all components semaphore configuration values
            try:
                from .config_updatable import update_all_components

                update_all_components()
                logger.info("Updated all components with new configuration values")
            except Exception as e:
                logger.error(f"Error updating components: {e}")
                raise ConfigUpdateError(
                    f"Failed to update components after config reload: {e}"
                ) from e


class ConfigUpdateError(Exception):
    """Exception raised when configuration update fails."""

    pass
