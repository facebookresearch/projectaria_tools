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

from projectaria_tools.core.stream_id import StreamId

from projectaria_tools.projects.adt import (
    AriaDigitalTwinDataPathsProvider,
    AriaDigitalTwinDataProvider,
)

TEST_FOLDER = os.getenv("TEST_FOLDER")
sequence_path = os.path.join(str(TEST_FOLDER), "aria_digital_twin_test_data")


class TestAeaPybindings(unittest.TestCase):
    def test_data_paths_provider(self):
        paths_provider = AriaDigitalTwinDataPathsProvider(sequence_path)

        # test metadata
        self.assertEqual(paths_provider.get_scene_name(), "Apartment")
        self.assertEqual(paths_provider.get_device_serial_number(), "1WM103600M1292")
        self.assertFalse(paths_provider.get_concurrent_sequence_name())
        self.assertEqual(paths_provider.is_multi_person(), False)
        self.assertEqual(paths_provider.get_num_skeletons(), 1)

        # test data paths
        data_paths = paths_provider.get_datapaths()
        self.assertTrue(data_paths)
        self.assertTrue(os.path.exists(data_paths.aria_vrs_filepath))
        self.assertTrue(os.path.exists(data_paths.aria_trajectory_filepath))
        self.assertTrue(os.path.exists(data_paths.object_trajectories_filepath))
        self.assertTrue(os.path.exists(data_paths.object_boundingbox_3d_filepath))
        self.assertTrue(os.path.exists(data_paths.boundingboxes_2d_filepath))
        self.assertTrue(os.path.exists(data_paths.segmentations_filepath))
        self.assertTrue(os.path.exists(data_paths.depth_images_filepath))
        self.assertTrue(os.path.exists(data_paths.synthetic_vrs_filepath))
        self.assertTrue(os.path.exists(data_paths.eyegazes_filepath))
        for _, filepath in data_paths.skeletons_filepaths.items():
            self.assertTrue(os.path.exists(filepath))
        self.assertTrue(os.path.exists(data_paths.skeleton_metadata_filepath))
        self.assertTrue(os.path.exists(data_paths.metadata_filepath))
        self.assertTrue(os.path.exists(data_paths.instances_filepath))

    def test_data_provider(self):
        paths_provider = AriaDigitalTwinDataPathsProvider(sequence_path)
        data_paths = paths_provider.get_datapaths()
        dp = AriaDigitalTwinDataProvider(data_paths)

        # check data exists
        self.assertTrue(dp.has_aria_data())
        self.assertTrue(dp.has_aria_3d_poses())
        self.assertTrue(dp.has_object_3d_boundingboxes())
        self.assertTrue(dp.has_instance_2d_boundingboxes())
        self.assertTrue(dp.has_segmentation_images())
        self.assertTrue(dp.has_depth_images())
        self.assertTrue(dp.has_synthetic_images())
        self.assertTrue(dp.has_eyegaze())
        self.assertTrue(dp.has_skeleton())
        self.assertTrue(dp.has_instances_info())
        self.assertTrue(dp.has_mps())
        mps_dp = dp.mps_data_provider_ptr()
        self.assertTrue(mps_dp.has_general_eyegaze())
        self.assertFalse(mps_dp.has_personalized_eyegaze())
        self.assertFalse(mps_dp.has_open_loop_poses())
        self.assertFalse(mps_dp.has_closed_loop_poses())
        self.assertFalse(mps_dp.has_online_calibrations())
        self.assertFalse(mps_dp.has_semidense_point_cloud())
        self.assertFalse(mps_dp.has_semidense_observations())
        self.assertFalse(mps_dp.has_wrist_and_palm_poses())

        # check we can query data
        stream_id = StreamId("214-1")  # use RGB camera for testing
        image_timestamps_ns = dp.get_aria_device_capture_timestamps_ns(stream_id)
        ts1 = image_timestamps_ns[0]
        self.assertEqual(len(image_timestamps_ns), 3)
        maybe_first_image = dp.get_aria_image_by_timestamp_ns(ts1, stream_id)
        self.assertTrue(maybe_first_image.is_valid())
        self.assertAlmostEqual(abs(maybe_first_image.dt_ns()), 0, 3)
        self.assertTrue(dp.get_aria_camera_calibration(stream_id))
        first_tc_time = dp.get_timecode_from_device_time_ns(ts1)
        self.assertTrue(
            abs(ts1 - dp.get_device_time_from_timecode_ns(first_tc_time)) < 1000
        )
        self.assertTrue(dp.get_aria_3d_pose_by_timestamp_ns(ts1).is_valid())
        self.assertTrue(dp.get_object_3d_boundingboxes_by_timestamp_ns(ts1).is_valid())
        self.assertTrue(
            dp.get_object_2d_boundingboxes_by_timestamp_ns(ts1, stream_id).is_valid()
        )
        self.assertTrue(
            dp.get_skeleton_2d_boundingboxes_by_timestamp_ns(ts1, stream_id).is_valid()
        )
        self.assertTrue(
            dp.get_segmentation_image_by_timestamp_ns(ts1, stream_id).is_valid()
        )
        self.assertTrue(dp.get_depth_image_by_timestamp_ns(ts1, stream_id).is_valid())
        self.assertTrue(
            dp.get_synthetic_image_by_timestamp_ns(ts1, stream_id).is_valid()
        )
        self.assertTrue(dp.get_eyegaze_by_timestamp_ns(ts1).is_valid())


if __name__ == "__main__":
    unittest.main()
