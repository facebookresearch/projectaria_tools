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


"""
Python parity test for random-access P-frame decoding.

Mirrors the C++ ImageSensorPlayer.RandomAccessParity test at the PyBind
boundary. This test fails iff random access from Python returns different
pixels than sequential access.
"""

import os
import random
import unittest

import numpy as np
from projectaria_tools.core import data_provider
from projectaria_tools.core.sensor_data import SensorDataType

TEST_FOLDER = os.getenv("TEST_FOLDER")
TEST_FIXTURE = os.path.join(
    os.getenv("TEST_FOLDER_GEN2"), "aria_gen2_unit_test_sequence_with_pframe.vrs"
)

# Per-component LSB drift between independent decodes — covers both lossless
# (where the drift is 0) and YUV/NV12 paths (where decoder rounding can drift
# by 1 LSB). Strictly weaker than the C++ test's split tolerance, but still
# orders of magnitude tighter than the failure mode this test guards against
# (broken P-frame decode returns garbage / undefined buffer content).
PIXEL_TOLERANCE = 1


def sample_indices(num_frames, n=20):
    """Boundary indices + uniformly-spaced interior samples. Mirrors
    sampleIndices() in ImageSensorPlayerTest.cpp."""
    if num_frames <= 0:
        return []
    sample = {0}
    if num_frames >= 2:
        sample.add(1)
        sample.add(num_frames - 1)
    if num_frames >= 3:
        sample.add(num_frames - 2)
    if num_frames > 4 and n > 4:
        interior = n - 4
        for i in range(interior):
            idx = int((i + 1) * (num_frames - 2) / (interior + 1))
            if 1 < idx < num_frames - 2:
                sample.add(idx)
    return sorted(sample)


def get_image_array(provider, stream_id, index):
    """Return a deep-copied numpy array for the image at (stream_id, index).

    `to_numpy_array()` may return a view into pybind-managed memory whose
    lifetime is tied to the ImageData object. We copy so the snapshot survives
    the next decode call into the same player.
    """
    image_data, _ = provider.get_image_data_by_index(stream_id, index)
    if not image_data.is_valid():
        return None
    return np.array(image_data.to_numpy_array(), copy=True)


class PFrameRandomAccessTest(unittest.TestCase):
    def test_random_access_parity(self):
        ref_provider = data_provider.create_vrs_data_provider(TEST_FIXTURE)
        rand_provider = data_provider.create_vrs_data_provider(TEST_FIXTURE)
        self.assertIsNotNone(
            ref_provider, f"failed to open ref provider for {TEST_FIXTURE}"
        )
        self.assertIsNotNone(
            rand_provider, f"failed to open rand provider for {TEST_FIXTURE}"
        )

        stream_ids = ref_provider.get_all_streams()
        image_streams_checked = 0

        for stream_id in stream_ids:
            if ref_provider.get_sensor_data_type(stream_id) != SensorDataType.IMAGE:
                continue
            image_streams_checked += 1

            num_frames = ref_provider.get_num_data(stream_id)
            indices = sample_indices(num_frames, n=20)
            if not indices:
                continue
            index_set = set(indices)

            print(
                f"stream {stream_id}: numFrames={num_frames}, "
                f"sampling {len(indices)} indices"
            )

            # Pass 1 — sequential reference. Walk every frame so the decoder
            # state stays clean (no random access on ref_provider). Snapshot
            # the sampled indices into owned numpy arrays.
            reference = {}
            for j in range(num_frames):
                arr = get_image_array(ref_provider, stream_id, j)
                if j in index_set and arr is not None:
                    reference[j] = arr

            # Pass 2 — random-access reads on a fresh provider in a
            # deterministically shuffled order, exercising forward jumps,
            # backward jumps, and cross-GOP seeks.
            shuffled = list(indices)
            rng = random.Random(0xA1B2C3D4)
            rng.shuffle(shuffled)

            for idx in shuffled:
                rand_arr = get_image_array(rand_provider, stream_id, idx)
                ref_arr = reference.get(idx)
                self.assertIsNotNone(
                    rand_arr,
                    f"random-access read returned invalid image at "
                    f"stream={stream_id} idx={idx}",
                )
                self.assertIsNotNone(
                    ref_arr,
                    f"sequential reference missing for stream={stream_id} idx={idx}",
                )
                self.assertEqual(
                    rand_arr.shape,
                    ref_arr.shape,
                    f"shape mismatch at stream={stream_id} idx={idx}: "
                    f"{rand_arr.shape} vs {ref_arr.shape}",
                )
                self.assertEqual(
                    rand_arr.dtype,
                    ref_arr.dtype,
                    f"dtype mismatch at stream={stream_id} idx={idx}: "
                    f"{rand_arr.dtype} vs {ref_arr.dtype}",
                )
                self.assertTrue(
                    np.allclose(
                        rand_arr.astype(np.int32),
                        ref_arr.astype(np.int32),
                        atol=PIXEL_TOLERANCE,
                        rtol=0,
                    ),
                    f"random vs sequential pixel mismatch at "
                    f"stream={stream_id} idx={idx} "
                    f"(max abs diff = "
                    f"{np.max(np.abs(rand_arr.astype(np.int32) - ref_arr.astype(np.int32)))})",
                )

        self.assertGreater(
            image_streams_checked,
            0,
            "no image streams found in fixture",
        )
