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

from enum import Enum

from textual.app import ComposeResult
from textual.containers import Grid
from textual.screen import ModalScreen
from textual.widgets import Button, Label


class QuitMode(int, Enum):
    """
    Enum class representing different modes of quitting the app
    """

    QUIT = 1
    LOGOUT_AND_QUIT = 2
    CANCEL = 3


class QuitScreen(ModalScreen[QuitMode]):
    """Screen with a dialog to quit."""

    CSS = """
        QuitScreen {
            align: center middle;
            background: $background 80%;
        }

        #dialog {
            grid-size: 3 2;
            grid-gutter: 1 2;
            grid-rows: 3 3;
            padding: 0 1;
            width: 60;
            height: 11;
            border: thick $background 80%;
            background: $surface;
        }

        #question {
            column-span: 3;
            height: 1fr;
            width: 1fr;
            content-align: center middle;
        }

        Button {
            width: 100%;
        }

    """
    BINDINGS = [("escape", "app.pop_screen", "Pop screen")]

    def compose(self) -> ComposeResult:
        """
        Create child widgets for the screen.
        """
        yield Grid(
            Label("[b]Are you sure you want to quit?", id="question"),
            Button("Cancel", variant="default", id="cancel"),
            Button("Log out & Quit", variant="default", id="logout_quit"),
            Button("Quit", variant="primary", id="quit"),
            id="dialog",
        )

    def on_mount(self) -> None:
        """
        Called when the widget is mounted
        """
        self.query_one("#quit").focus()

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        """
        Handle button press events.
        """
        if event.button.id == "cancel":
            quit_mode: QuitMode = QuitMode.CANCEL
        elif event.button.id == "logout_quit":
            quit_mode: QuitMode = QuitMode.LOGOUT_AND_QUIT
        elif event.button.id == "quit":
            quit_mode: QuitMode = QuitMode.QUIT
        self.dismiss(quit_mode)
