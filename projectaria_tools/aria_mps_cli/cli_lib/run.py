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

import argparse
import asyncio
import logging
from datetime import datetime
from pathlib import Path
from typing import List

from .authentication import Authenticator
from .common import Config
from .constants import ConfigKey, ConfigSection
from .http_helper import HttpHelper
from .mps import Mps
from .mps_app import MpsApp
from .types import MpsFeature

logger = logging.getLogger(__name__)

_SINGLE_COMMAND: str = "single"
_MULTI_COMMAND: str = "multi"
_LOGOUT_COMMAND: str = "logout"


def _add_common_args(parser: argparse.ArgumentParser):
    """
    Add common arguments to the parser
    """
    parser.add_argument(
        "-i",
        "--input",
        help="Path to the input VRS file or directory containing VRS files.",
        action="append",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--retry-failed",
        help="Retry failed requests if any for the input VRS files.",
        action="store_true",
        dest="retry_failed",
    )
    parser.add_argument(
        "--force",
        help="Force reprocess all the VRS files even if they were already processed or in progress.",
        action="store_true",
    )
    parser.add_argument(
        "-u",
        "--username",
        help="Username to use when connecting to MPS. This can be an email address, or a username.",
        type=str,
    )
    parser.add_argument(
        "-p",
        "--password",
        help="Password to use when connecting to MPS.",
        type=str,
    )
    parser.add_argument(
        "--no-save-token",
        help="Do not save the authentication token to disk. Additionally invalidate the token on exit. Can only be used with --no-ui option.",
        action="store_false",
        dest="save_token",
    )
    parser.add_argument(
        "--no-ui",
        help="Do not display the UI showing status of each MPS Request.",
        action="store_false",
        dest="show_ui",
    )
    # For debugging only to re-upload the same file by appending the suffix to the
    # file hash
    parser.add_argument("-s", "--suffix", help=argparse.SUPPRESS, type=str)


def _parse_args() -> argparse.Namespace:
    """
    Helpers to parse command line arguments
    """
    parser = argparse.ArgumentParser(
        description="Aria Machine Perception Services (MPS) Command Line Interface (CLI) tool. This tool can be used to upload Aria recordings to MPS backend, generated Machine Perception post-processed data and download the generated results."
    )

    # Define the subparsers
    subparsers = parser.add_subparsers(
        dest="mode",
        help="The CLI supports the following modes of operation. Run --help with each mode to get help specific to the mode.",
        required=True,
    )
    # Define the single_sequence subcommand
    parser_single = subparsers.add_parser(
        _SINGLE_COMMAND,
        help="Single sequence MPS. MPS will process each VRS file separately.",
    )
    _add_common_args(parser_single)
    parser_single.add_argument(
        "--features",
        help="List of MP feature(s) to generate",
        nargs="+",
        choices=[
            MpsFeature.EYE_GAZE.value,
            MpsFeature.SLAM.value,
            MpsFeature.HAND_TRACKING.value,
        ],
        type=MpsFeature,
        default=[MpsFeature.EYE_GAZE, MpsFeature.SLAM, MpsFeature.HAND_TRACKING],
    )
    # Define the multi_sequence subcommand
    parser_multi = subparsers.add_parser(
        _MULTI_COMMAND,
        help="Multi sequence MPS. MPS will process the group of VRS files together to produce a trajectories in a common frame of reference.",
    )
    _add_common_args(parser_multi)

    parser_multi.add_argument(
        "-n",
        "--name",
        help="Name of the multi MPS request",
        type=str,
    )
    parser_multi.add_argument(
        "-o",
        "--output",
        help="Output directory where the results will be stored.",
        dest="output_dir",
        type=Path,
        required=True,
    )

    # Define the logout subcommand
    subparsers.add_parser(
        _LOGOUT_COMMAND,
        help="Logout from MPS. The cached auth token will be removed.",
    )

    return parser.parse_args()


def _configure_logging(verbose: bool) -> Path:
    """
    Setup logging to file and remove default logger unless verbose mode is enabled
    """
    config = Config.get()
    log_dir = Path(config.get(ConfigSection.DEFAULT, ConfigKey.LOG_DIR))
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file: Path = log_dir / f"{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.log"
    handlers: List[logging.StreamHandler] = [logging.FileHandler(log_file)]
    if verbose:
        handlers.append(logging.StreamHandler())
    logging.basicConfig(
        handlers=handlers,
        level=logging.DEBUG,
        format="%(asctime)s [%(process)d] [%(levelname)s] [%(filename)s:%(lineno)d] - %(message)s",
    )

    logger.info(f"log file : {log_file}")
    return log_file


async def _run_async(args: argparse.Namespace, log_path: Path) -> None:
    """Asynchronous entry point for the CLI"""
    asyncio.current_task().set_name("main")
    async with HttpHelper() as http_helper:
        authenticator: Authenticator = Authenticator(http_helper)

        if args.mode == _LOGOUT_COMMAND:
            await authenticator.logout()
        else:
            if args.username and args.password:
                await authenticator.login(args.username, args.password, args.save_token)
            elif not await authenticator.load_and_validate_token():
                logger.error(
                    """

                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    !!!                                                               !!!
                    !!! Failed to login to MPS - please provide username and password !!!
                    !!!                                                               !!!
                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    """
                )
                return

            http_helper.set_auth_token(authenticator.auth_token)
            mps: Mps = Mps(http_helper)
            await mps.run(args, log_path)

            if args.username and args.password and not args.save_token:
                await authenticator.logout()


def run():
    """Synchronous entry point for the CLI"""
    args = _parse_args()
    if args.mode == _LOGOUT_COMMAND or not args.show_ui:
        log_path: Path = _configure_logging(True)
        asyncio.get_event_loop().run_until_complete(_run_async(args, log_path))
    else:
        # show UI here
        if not args.save_token:
            logger.error("Cannot use --no-save-token option without --no-ui option.")
            return

        log_path: Path = _configure_logging(False)
        mps_app = MpsApp(args, log_path)
        mps_app.run()
