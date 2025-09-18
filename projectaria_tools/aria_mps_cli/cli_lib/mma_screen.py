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
import logging
from typing import Optional

from rich.text import Text

from textual.app import ComposeResult
from textual.containers import Grid
from textual.screen import ModalScreen
from textual.widgets import Button, Static

from .authentication import AuthenticationError


logger = logging.getLogger(__name__)


class MMAScreen(ModalScreen[bool]):
    """MMA authentication screen that displays a link and code for user login"""

    CSS: str = """
    MMAScreen {
        align: center middle;
        background: $background 100%;
    }

    #dialog {
        grid-size: 2 10;
        grid-gutter: 1 1;
        grid-rows: 2 1 1 1 1 1 2 3 3 2;
        padding: 1 1 1 1;
        width: 80;
        height: 26;
        border: thick $background 100%;
        background: $surface;
    }

    .centered-content {
        column-span: 2;
        text-align: center;
        margin: 0 2;
    }

    #title {
        column-span: 2;
        content-align: center middle;
    }

    #link {
        color: rgb(0, 127, 255);
    }

    Button {
        width: 100%;
        height: auto;
        text-align: center;
        margin: 0 2;
    }

    #status {
        column-span: 2;
        text-align: center;
    }
    """

    def __init__(
        self,
        login_url: Optional[str] = "https://work.meta.com/cli",
        user_code: Optional[str] = None,
        device_code: Optional[str] = None,
        expires_in: Optional[int] = 600,  # 10 minutes
        polling_interval: Optional[int] = 5,
    ):
        """
        Initialize the MMA authentication screen.

        Args:
            login_url: The URL where the user should complete authentication
            user_code: The code the user should enter on the login page
            device_code: The device code used for polling the authentication status
            expires_in: Time in seconds until the codes expire
            polling_interval: Polling interval in seconds (default: 5)
        """
        super().__init__()
        self._login_url = login_url
        self._user_code = user_code
        self._device_code = device_code
        self._expires_in = expires_in
        self._polling_interval = polling_interval
        self._polling_task: Optional[asyncio.Task] = None
        self._is_cancelled = False

    def compose(self) -> ComposeResult:
        """Create child widgets for the screen."""
        yield Grid(
            Static("[b]MMA Authentication Required", id="title"),
            Static(
                "Please complete authentication by:",
                id="subtitle",
                classes="centered-content",
            ),
            Static("1. Visit the link below:", id="step1", classes="centered-content"),
            Static(
                Text.from_markup(f"[underline]{self._login_url}[/]"),
                id="link",
                classes="centered-content",
            ),
            Static("2. Enter the code:", id="step2", classes="centered-content"),
            Static(
                Text.from_markup(f"[b bright_yellow]{self._user_code}[/]"),
                id="code",
                classes="centered-content",
            ),
            Static(
                "3. CLI would automatically detect that the login is complete and will open the login screen",
                id="step3",
                classes="centered-content",
            ),
            Button("Cancel", variant="default", id="cancel"),
            Button("Refresh Status", variant="primary", id="refresh"),
            Static("Waiting for authentication...", id="status"),
            id="dialog",
        )

    async def on_mount(self) -> None:
        """Start polling for authentication completion when the screen is mounted."""
        logger.info("Running on_mount in mma_screen")
        device_flow_info = await self.app._authenticator.get_mma_device_flow_info()
        logger.info(f"Got MMA code: {device_flow_info}")
        self._user_code = device_flow_info["user_code"]
        self._device_code = device_flow_info["device_code"]
        # Update the code field to match the new device code
        self.query_one("#code").update(
            Text.from_markup(f"[b bright_yellow]{self._user_code}[/]")
        )
        self._polling_task = asyncio.create_task(self._poll_for_completion())

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        """Handle button press events."""
        if event.button.id == "cancel":
            self._is_cancelled = True
            if self._polling_task:
                self._polling_task.cancel()
            self.dismiss(False)
        elif event.button.id == "refresh":
            # Manually check authentication status
            await self._check_authentication_status()

    async def _poll_for_completion(self) -> None:
        """
        Poll the authentication endpoint periodically to check if login is completed.
        """
        start_time = asyncio.get_event_loop().time()

        while not self._is_cancelled:
            try:
                # Check if the code has expired
                elapsed_time = asyncio.get_event_loop().time() - start_time
                if elapsed_time >= self._expires_in:
                    self.query_one("#status").update(
                        Text.from_markup(
                            "[red]Authentication code has expired. Please try again.[/]"
                        )
                    )
                    await asyncio.sleep(2)  # Show message briefly
                    self.dismiss(False)
                    return

                # Check authentication status
                if await self._check_authentication_status():
                    return

                # Wait for the specified interval before next check
                await asyncio.sleep(self._polling_interval)

            except asyncio.CancelledError:
                logger.debug("Polling task cancelled")
                return
            except Exception as e:
                logger.error(f"Error during polling: {e}")
                self.query_one("#status").update(
                    Text.from_markup(
                        f"[red]Error checking authentication status: {str(e)}[/]"
                    )
                )
                await asyncio.sleep(self._polling_interval)

    async def _check_authentication_status(self) -> bool:
        """
        Check if the authentication has been completed.

        Returns:
            True if authentication is complete, False otherwise
        """
        try:
            self.query_one("#status").update("Checking authentication status...")

            # Poll for auth code using device code (step 3 of MMA flow)
            auth_code = await self.app._authenticator.check_mma_device_authorization(
                self._device_code
            )

            if auth_code:
                self.query_one("#status").update("Getting access token...")

                # Get access token using auth code (step 4 of MMA flow)
                access_token = await self.app._authenticator.get_mma_access_token(
                    auth_code
                )

                # Set the access token in the authenticator
                await self.app._authenticator.set_auth_token(
                    access_token, save_token=True
                )

                self.query_one("#status").update(
                    Text.from_markup("[green]Authentication successful![/]")
                )
                await asyncio.sleep(1)  # Show success message briefly
                self.dismiss(True)
                return True
            else:
                self.query_one("#status").update("Waiting for authentication...")
                return False

        except AuthenticationError as e:
            logger.error(f"Authentication error: {e}")
            self.query_one("#status").update(
                Text.from_markup(f"[red]Authentication failed: {str(e)}[/]")
            )
            return False
        except Exception as e:
            logger.error(f"Unexpected error checking authentication: {e}")
            self.query_one("#status").update(
                Text.from_markup(f"[red]Error: {str(e)}[/]")
            )
            return False

    def on_unmount(self) -> None:
        """Clean up when the screen is unmounted."""
        if self._polling_task:
            self._polling_task.cancel()
