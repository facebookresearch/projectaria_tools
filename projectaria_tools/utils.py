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

import numpy as _np
from projectaria_tools.core.sensor_data import PixelFrame as _PixelFrame


# Convert from Aria image format to numpy array
#
# Inputs:
# - pixel_frame: PixelFrame
# Output:
# - image_array: numpy.ndarray
def to_image_array(pixel_frame: _PixelFrame) -> _np.ndarray:
    normalizedFrame = pixel_frame.normalizeFrame(False)
    img_buf = normalizedFrame.getBuffer()
    width = normalizedFrame.getWidth()
    height = normalizedFrame.getHeight()
    buffer_array = _np.array(img_buf, dtype=_np.uint8)
    # reshape into [col, row] -> [height, width]
    image_array = buffer_array.reshape((height, width, -1))
    return image_array
