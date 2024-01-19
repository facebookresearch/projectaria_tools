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

import platform
import subprocess

system = platform.system()
command = "gsed"  # this is for macos
if system == "Linux":
    command = "sed"

# replace _core_pybinds, etc with projectaria_tools
replace_core_pybinds = f"find projectaria_tools-stubs/ -name '*.pyi' | xargs {command} -i 's/_core_pybinds/projectaria_tools.core/g'"
replace_adt_pybinds = f"find projectaria_tools-stubs/ -name '*.pyi' | xargs {command} -i 's/_adt_pybinds/projectaria_tools.project.adt/g'"
replace_ase_pybinds = f"find projectaria_tools-stubs/ -name '*.pyi' | xargs {command} -i 's/_ase_pybinds/projectaria_tools.project.ase/g'"

command_list = [
    "mkdir -p projectaria_tools-stubs",
    "mkdir -p projectaria_tools-stubs/projectaria_tools",
    "pybind11-stubgen projectaria_tools -o projectaria_tools-stubs --ignore-all-errors",
    "pybind11-stubgen _core_pybinds -o projectaria_tools-stubs --ignore-all-errors",
    "cp -r projectaria_tools-stubs/_core_pybinds/* projectaria_tools-stubs/projectaria_tools/core/",
    "rm -r projectaria_tools-stubs/_core_pybinds",
    "pybind11-stubgen _adt_pybinds -o projectaria_tools-stubs --ignore-all-errors",
    "mkdir -p projectaria_tools-stubs/projectaria_tools/projects/",
    "mv projectaria_tools-stubs/_adt_pybinds.pyi projectaria_tools-stubs/projectaria_tools/projects/adt.pyi",
    "pybind11-stubgen _ase_pybinds -o projectaria_tools-stubs --ignore-all-errors",
    "mv projectaria_tools-stubs/_ase_pybinds.pyi projectaria_tools-stubs/projectaria_tools/projects/ase.pyi",
    replace_core_pybinds,
    replace_adt_pybinds,
    replace_ase_pybinds,
]

for command in command_list:
    subprocess.run(command, shell=True, check=True)
