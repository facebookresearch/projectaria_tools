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

# This makes the modules discoverable when doing dir(projectaria_tools.utils)

"""
projectaria_tools utils
"""

# Lazy installation of the rerun-sdk dependency for the utils module
# If module not installed yet, it will install it, and refresh the current python session
# to take into account the new installed module
import subprocess
import sys

try:
    import rerun as rr
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "rerun-sdk>=0.12.0"])
finally:
    import site
    from importlib import reload

    reload(site)
    import rerun as rr  # noqa
