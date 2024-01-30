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


import os
import unittest
from pathlib import Path

from projectaria_tools.core.mps import MpsDataPathsProvider

from projectaria_tools.projects.aea import (
    AriaEverydayActivitiesDataPathsProvider,
    AriaEverydayActivitiesDataProvider,
)

TEST_FOLDER = os.getenv("TEST_FOLDER")
sequence_path = os.path.join(str(TEST_FOLDER), "aria_everyday_activities_test_data/")


class TestAeaPybindings(unittest.TestCase):
    def test_data_paths_provider(self):
        paths_provider = AriaEverydayActivitiesDataPathsProvider(sequence_path)

        ## Check query functions
        self.assertEqual(paths_provider.get_location_number(), 1)
        self.assertEqual(paths_provider.get_script_number(), 1)
        self.assertEqual(paths_provider.get_sequence_number(), 1)
        self.assertEqual(paths_provider.get_recording_number(), 1)
        self.assertEqual(len(paths_provider.get_concurrent_recordings()), 0)
        self.assertEqual(paths_provider.get_dataset_name(), "AEA_2024")
        self.assertEqual(paths_provider.get_dataset_version(), "1.0")

        ## Check data paths
        data_paths = paths_provider.get_data_paths()
        self.assertTrue(os.path.exists(data_paths.aria_vrs))
        self.assertTrue(os.path.exists(data_paths.speech))
        self.assertTrue(os.path.exists(data_paths.metadata))

        # Check MPS data paths
        self.assertTrue(os.path.exists(data_paths.mps.eyegaze.summary))
        self.assertTrue(os.path.exists(data_paths.mps.eyegaze.general_eyegaze))
        self.assertTrue(os.path.exists(data_paths.mps.eyegaze.general_eyegaze))
        self.assertFalse(data_paths.mps.slam.closed_loop_trajectory)
        self.assertFalse(data_paths.mps.slam.open_loop_trajectory)
        self.assertFalse(data_paths.mps.slam.semidense_points)
        self.assertFalse(data_paths.mps.slam.semidense_observations)
        self.assertFalse(data_paths.mps.slam.online_calibrations)
        self.assertFalse(data_paths.mps.slam.summary)

        # Create separate MPS data paths provider and make sure they match
        mps_root_path = Path(data_paths.mps.eyegaze.general_eyegaze).parent.parent
        mps_data_paths_provider = MpsDataPathsProvider(str(mps_root_path))
        mps_data_paths = mps_data_paths_provider.get_data_paths()
        self.assertEqual(str(mps_data_paths), str(data_paths.mps))

    def test_data_provider(self):
        paths_provider = AriaEverydayActivitiesDataPathsProvider(sequence_path)
        data_paths = paths_provider.get_data_paths()
        dp = AriaEverydayActivitiesDataProvider(data_paths)
        self.assertTrue(dp.has_aria_data())
        self.assertTrue(dp.has_speech_data())
        self.assertTrue(len(dp.vrs.get_all_streams()) > 0)
        self.assertTrue(dp.vrs.get_device_calibration())

        start_ts_ns = [
            101217000000,
            128657000000,
            129657000000,
            129817000000,
            129977000000,
            130217000000,
            130377000000,
            196337000000,
            197337000000,
            197497000000,
            197657000000,
            198217000000,
            198377000000,
        ]
        end_ts_ns = [
            102217000000,
            129657000000,
            129817000000,
            129977000000,
            130217000000,
            130377000000,
            131017000000,
            197337000000,
            197497000000,
            197657000000,
            198217000000,
            198377000000,
            198937000000,
        ]

        ts = start_ts_ns[0]
        s1 = dp.speech.get_sentence_data_by_timestamp_ns(ts)
        self.assertTrue(s1)
        self.assertEqual(s1.to_string(), "and what is there like looking at?")
        self.assertEqual(s1.start_timestamp_ns, start_ts_ns[0])
        self.assertEqual(s1.end_timestamp_ns, end_ts_ns[6])

        s2 = dp.speech.get_sentence_data_by_timestamp_ns(start_ts_ns[7])
        self.assertTrue(s2)
        self.assertEqual(s2.to_string(), "I'm in the masters, right now.")
        self.assertEqual(s2.start_timestamp_ns, start_ts_ns[7])
        self.assertEqual(s2.end_timestamp_ns, end_ts_ns[len(end_ts_ns) - 1])

    def test_mps_integration(self):
        paths_provider = AriaEverydayActivitiesDataPathsProvider(sequence_path)
        data_paths = paths_provider.get_data_paths()
        dp = AriaEverydayActivitiesDataProvider(data_paths)
        self.assertTrue(dp.has_mps_data())
        self.assertTrue(dp.mps.has_general_eyegaze())

        eyegaze_ts_ns = 86435306000
        eg1 = dp.mps.get_general_eyegaze(eyegaze_ts_ns)
        self.assertTrue(eg1)
        self.assertAlmostEqual(eg1.yaw, -0.36566, 5)
        self.assertAlmostEqual(eg1.pitch, -0.121962, 5)


if __name__ == "__main__":
    unittest.main()
