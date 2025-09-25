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
import logging
from pathlib import Path
from time import monotonic
from typing import Any, Dict, Mapping, Optional

import aiofiles
from rich import box
from rich.spinner import Spinner
from rich.table import Table
from rich.text import Text
from textual import work
from textual.app import App, ComposeResult
from textual.containers import Horizontal, ScrollableContainer
from textual.reactive import reactive
from textual.widgets import Footer, Header, Rule, Static

from .authentication import Authenticator
from .common import get_pretty_size
from .constants import DisplayStatus
from .http_helper import HttpHelper
from .login_screen import LoginScreen
from .mps import Mps
from .quit_screen import QuitMode, QuitScreen
from .types import ModelState, MpsFeature

logger = logging.getLogger(__name__)
from_markup = Text.from_markup


class ElapsedTime(Static):
    """
    A static widget that displays elapsed time since start
    """

    elapsed_time = reactive(0)

    def reset(self) -> None:
        """
        Reset the timer
        """
        self._start_time = monotonic()

    def on_mount(self) -> None:
        """
        Called when the widget is mounted
        """
        self._start_time = monotonic()
        self.set_interval(1, self.update_elapsed_time)

    def update_elapsed_time(self) -> None:
        """
        Update the elapsed time
        """
        self.elapsed_time = monotonic() - self._start_time

    def watch_elapsed_time(self) -> None:
        """
        Update the UI when elapsed time is updated
        """
        self.update(
            f"{Text.from_markup(':clock1:')} Time elapsed: [bold][cyan]{self._get_pretty_time()}"
        )

    def _get_pretty_time(self) -> str:
        """
        Convert time delta to human readable format
        """
        seconds = int(self.elapsed_time)
        minutes, seconds = divmod(seconds, 60)
        hours, minutes = divmod(minutes, 60)
        days, hours = divmod(hours, 24)
        if days > 0:
            return f"{days}d:{hours}h:{minutes}m:{seconds}s"
        return f"{hours}h:{minutes}m:{seconds}s"


class StatusTable(Static):
    """
    A static widget that displays status of all recordings
    """

    mps_status: Mapping[Path, Dict[MpsFeature, ModelState]] = reactive({})

    def __init__(self, mps: Mps) -> None:
        self._mps: Mps = mps
        self._file_sizes: Dict[Path, int] = {}
        self._table: Optional[Table] = None
        super().__init__()

    def on_mount(self) -> None:
        """
        Called when the widget is mounted
        """
        self.set_interval(1 / 2, self.update_status)
        self.set_interval(1 / 5, self.refresh_ui)

    def refresh_ui(self) -> None:
        """
        Refresh the UI
        """
        if self._table:
            self.update(self._table)

    def update_status(self) -> None:
        """
        Get latest status from MPS
        """
        self.mps_status = self._mps.get_status()

    async def _get_file_size(self, vrs_path: Path) -> int:
        """
        Get file size of given VRS file
        """
        if vrs_path not in self._file_sizes:
            self._file_sizes[vrs_path] = (await aiofiles.os.stat(vrs_path)).st_size
        return self._file_sizes[vrs_path]

    async def watch_mps_status(self) -> None:
        """
        Update the table on status update
        """
        vrs_paths = sorted(self.mps_status.keys())
        if not vrs_paths:
            return
        table = Table(
            expand=True,
            box=box.SQUARE_DOUBLE_HEAD,
            style="grey37",
        )
        table.add_column("ID")
        table.add_column("RECORDING", overflow="fold")
        table.add_column("FILE SIZE", justify="right")
        for feature in self._mps.features:
            table.add_column(feature.value.upper())
        for i, vrs_path in enumerate(vrs_paths, 1):
            feature_status = self.mps_status[vrs_path]
            table.add_row(
                str(i),
                str(vrs_path),
                f"{get_pretty_size(await self._get_file_size(vrs_path))}",
                *[
                    self._apply_style(feature_status[feature])
                    for feature in self._mps.features
                ],
            )
        self._table = table

    def _apply_style(self, state: ModelState) -> Any:
        """
        Apply styling to the given state
        """
        if state.status == DisplayStatus.ERROR:
            return Text.from_markup(
                f":cross_mark: [red]{state.status}({state.error_code})"
            )
        elif state.status == DisplayStatus.SUCCESS:
            return Text.from_markup(f":white_check_mark: [green]{state.status}")
        elif state.status == DisplayStatus.SCHEDULED:
            return Text.from_markup(f":clock1: {state.status}")
        elif state.status in [DisplayStatus.UPLOADING, DisplayStatus.DOWNLOADING]:
            return Spinner("point", f"{state.status} {state.progress:.2f}%")
        elif state.status == DisplayStatus.ENCRYPTING:
            return Text.from_markup(f":lock: {state.status} {state.progress:.2f}%")
        elif state.status == DisplayStatus.HASHING:
            return Text.from_markup(f":key: {state.status} {state.progress:.2f}%")
        return Spinner("dots", state.status)


class MpsApp(App):
    """MPS App that shows the current status of all recordings"""

    BINDINGS = [
        ("ctrl+q", "request_quit", "Quit   "),
        ("d", "toggle_dark", "Toggle dark mode"),
    ]
    CSS = """
    #top {
        height: 2;
    }
    #elapsed_time {
        width: auto;
        height: auto;
        dock: left;
        content-align: center middle;
        padding: 1 5;
    }
    #user {
        width: auto;
        height: auto;
        dock: right;
        content-align: center middle;
        padding: 1 5;
    }
    #stages_title {
        content-align: center middle;
        height:auto;
        padding: 1 0 0 0;
    }
    #stages {
        content-align: center middle;
        height:auto;
        padding: 1 0;
    }
    #status, #status_title{
        content-align: center middle;
    }
    #log {
        height: auto;
        padding: 1 5;
    }
    """

    def __init__(self, args: argparse.Namespace, log_path: Path) -> None:
        self._args: argparse.Namespace = args
        self._log_path: Path = log_path
        super().__init__()

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        logger.info("Composing")
        yield Header()
        yield Horizontal(
            ElapsedTime(name="elapsed time", id="elapsed_time"),
            Static(
                "",
                name="user",
                id="user",
            ),
            id="top",
        )
        yield Static("EXPECTED STAGES", id="stages_title")
        yield Static(
            "[magenta]Hashing ->  HealthCheck  ->  Encryption  ->  Upload  ->  Scheduled  -> Processing  ->  Download Results  ->  Success",
            id="stages",
        )
        yield Rule()
        yield Static("[b]MPS REQUESTS", id="status_title")
        yield ScrollableContainer(StatusTable(self._mps), id="status")
        yield Rule()
        yield Static(
            f":scroll: Log: [bold][cyan]{self._log_path}", name="log file", id="log"
        )

        yield Footer()

    async def on_load(self) -> None:
        """Run when the app is loaded."""
        logger.info("on_load")
        self.title = "ARIA MACHINE PERCEPTION SERVICES"
        self._http_helper: HttpHelper = HttpHelper()
        self._authenticator: Authenticator = Authenticator(self._http_helper)
        args = self._args
        if args.username and args.password:
            await self._authenticator.login(
                args.username, args.password, args.save_token
            )
        elif await self._authenticator.load_and_validate_token():
            logger.debug("Using cached token.")
        self._mps: Mps = Mps(self._http_helper)

    async def on_mount(self) -> None:
        """Run when the app is mounted."""
        logger.info("on_mount")
        header = self.query_one(Header)
        header.title = "Aria Machine Perception Service"
        header.tall = True
        # Run MPS after login
        if self._authenticator.is_logged_in():
            self.run_mps()
        else:

            def __login_callback(success: bool) -> None:
                if self._authenticator.is_logged_in():
                    self.run_mps()
                else:
                    logger.debug("Failed to log in!")
                    self.exit()

            self.push_screen(LoginScreen(), __login_callback)

    def _update_username(self) -> None:
        """Update the user name."""
        logger.debug(f"Setting username to {self._authenticator.user}")
        self.query_one("#user", Static).update(
            Text.from_markup(f"Username: [bold][cyan]{self._authenticator.user}"),
        )

    @work
    async def run_mps(self) -> None:
        """
        MPS worker task
        """
        self._http_helper.set_auth_token(self._authenticator.auth_token)
        self._update_username()
        self.query_one("#status_title", Static).update(
            f"[b] MPS REQUESTS - {self._args.mode.upper()}",
        )
        self.query_one("#elapsed_time", ElapsedTime).reset()
        await self._mps.run(self._args)

    async def action_request_quit(self) -> bool:
        """An action to quit the application."""

        async def __check_quit(quit_mode: QuitMode) -> None:
            """Called when QuitScreen is dismissed."""
            if quit_mode == QuitMode.LOGOUT_AND_QUIT:
                logger.debug("Logging out...")
                await self._authenticator.logout()
            if quit_mode in (QuitMode.QUIT, QuitMode.LOGOUT_AND_QUIT):
                logger.debug("Quitting...")
                await self._http_helper.close()
                return self.exit()
            if quit_mode == QuitMode.CANCEL:
                logger.debug("Dismissing QuitScreen...")

        self.push_screen(QuitScreen(), __check_quit)

    def action_toggle_dark(self) -> None:
        """An action to toggle dark mode."""
        self.dark = not self.dark
