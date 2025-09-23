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

# This makes the modules discoverable when doing dir(projectaria_tools.core)

"""
A pybind11 binding for projectaria_tools core module
"""

from . import (  # noqa
    calibration,
    data_provider,
    image,
    mps,
    sensor_data,
    sophus,
    stream_id,
    vrs,
)
