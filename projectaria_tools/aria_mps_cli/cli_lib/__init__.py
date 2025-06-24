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

import importlib.util
import subprocess
import sys


def is_module_installed(module_name: str) -> bool:
    """
    Check whether a module is installed or not
    """
    spec = importlib.util.find_spec(module_name)
    return spec is not None


## These modules get installed on importing the cli_lib module
def _install_deps():
    """These are all the necessary modules to be installed before running the CLI"""
    _required_modules = {
        "aiofiles",
        "aiohttp",
        "projectaria_tools",
        "rich",
        "textual",
        "transitions",
        "xxhash",
        "keyring",
    }

    _modules_to_install = [m for m in _required_modules if not is_module_installed(m)]
    # Unlike other modules, pycryptodome is installed as Crypto but the pip package name
    # is different (pycryptodome)
    if not is_module_installed("Crypto"):
        _modules_to_install.append("pycryptodome")
    if _modules_to_install:
        print(
            f"The following modules need to be installed before running the CLI:\n{_modules_to_install}"
        )
        subprocess.run(
            [sys.executable, "-m", "pip", "install"] + list(_modules_to_install)
        )


_install_deps()

from .run import run  # noqa
