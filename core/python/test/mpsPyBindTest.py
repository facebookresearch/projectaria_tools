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

from projectaria_tools.core import mps

TEST_FOLDER = os.getenv("TEST_FOLDER")


closed_loop_trajectory_file = os.path.join(
    TEST_FOLDER, "mps_sample/trajectory/closed_loop_trajectory.csv"
)
open_loop_trajectory_file = os.path.join(
    TEST_FOLDER, "mps_sample/trajectory/open_loop_trajectory.csv"
)


class MPSTrajectory(unittest.TestCase):
    """
    Tests for reading trajectory files
    """

    def test_open_loop(self) -> None:
        mps_trajectory = mps.read_open_loop_trajectory(open_loop_trajectory_file)
        assert len(mps_trajectory) > 0

    def test_open_loop_invalid_file(self) -> None:
        mps_trajectory = mps.read_open_loop_trajectory("")
        assert len(mps_trajectory) == 0

    def test_closed_loop(self) -> None:
        mps_trajectory = mps.read_closed_loop_trajectory(closed_loop_trajectory_file)
        assert len(mps_trajectory) > 0

    def test_closed_loop_invalid_file(self) -> None:
        mps_trajectory = mps.read_closed_loop_trajectory("")
        assert len(mps_trajectory) == 0


global_points_file = os.path.join(
    TEST_FOLDER, "mps_sample/trajectory/global_points.csv.gz"
)
semidense_observations_file = os.path.join(
    TEST_FOLDER, "mps_sample/trajectory/semidense_observations.csv.gz"
)


class MPSPointCloudAndObservations(unittest.TestCase):
    """
    Tests for reading point cloud and observations from csv.gz files
    """

    def test_global_points(self) -> None:
        mps_global_points = mps.read_global_point_cloud(
            global_points_file, mps.StreamCompressionMode.GZIP
        )
        assert len(mps_global_points) > 0

        mps_global_points2 = mps.read_global_point_cloud(global_points_file)
        assert len(mps_global_points2) > 0

    def test_global_points_invalid_file(self) -> None:
        mps_global_points = mps.read_global_point_cloud("")
        assert len(mps_global_points) == 0

    def test_semidense_observations(self) -> None:
        mps_semidense_observations = mps.read_point_observations(
            semidense_observations_file, mps.StreamCompressionMode.GZIP
        )
        assert len(mps_semidense_observations) > 0

        mps_semidense_observations2 = mps.read_point_observations(
            semidense_observations_file
        )
        assert len(mps_semidense_observations2) > 0

    def test_semidense_observations_invalid_file(self) -> None:
        mps_semidense_observations = mps.read_point_observations("")
        assert len(mps_semidense_observations) == 0


online_calibration_file = os.path.join(
    TEST_FOLDER, "mps_sample/trajectory/online_calibration.jsonl"
)
online_calibration_file_v1_1_0 = os.path.join(
    TEST_FOLDER, "mps_sample/trajectory/online_calibration_v1_1_0.jsonl"
)


class MPSOnlineCalibration(unittest.TestCase):
    """
    Tests for reading online calibration from jsonl files
    """

    def test_online_calibration(self) -> None:
        mps_online_calibration = mps.read_online_calibration(online_calibration_file)
        assert len(mps_online_calibration) > 0

        mps_online_calibration_v1_1_0 = mps.read_online_calibration(
            online_calibration_file_v1_1_0
        )
        assert len(mps_online_calibration_v1_1_0) > 0

        rgb_camera_calibration = mps_online_calibration_v1_1_0[0].get_camera_calib(
            "camera-rgb"
        )
        assert rgb_camera_calibration.get_readout_time_sec() > 0
        assert rgb_camera_calibration.get_time_offset_sec_device_camera() < 0
        assert rgb_camera_calibration.get_image_size()[0] == 1408
        assert rgb_camera_calibration.get_image_size()[1] == 1408

    def test_online_calibration_invalid_file(self) -> None:
        mps_online_calibration = mps.read_online_calibration("")
        assert len(mps_online_calibration) == 0


class MPSHandTracking(unittest.TestCase):
    """
    Tests for reading hand tracking data from csv files
    """

    def test_wrist_and_palm_valid_file_v1(self) -> None:
        wrist_and_palm_file = os.path.join(
            TEST_FOLDER, "mps_sample/hand_tracking/wrist_and_palm_poses.csv"
        )
        mps_wrist_and_palm_poses = mps.hand_tracking.read_wrist_and_palm_poses(
            wrist_and_palm_file
        )
        self.assertGreater(len(mps_wrist_and_palm_poses), 0)

    def test_wrist_and_palm_valid_file_v2(self) -> None:
        wrist_and_palm_file = os.path.join(
            TEST_FOLDER, "mps_sample/hand_tracking/wrist_and_palm_poses_v2.csv"
        )
        mps_wrist_and_palm_poses = mps.hand_tracking.read_wrist_and_palm_poses(
            wrist_and_palm_file
        )
        self.assertGreater(len(mps_wrist_and_palm_poses), 0)

        wrist_and_palm_pose: mps.hand_tracking.WristAndPalmPose = (
            mps_wrist_and_palm_poses[0]
        )
        self.assertTrue(wrist_and_palm_pose.left_hand is not None)
        self.assertTrue(
            wrist_and_palm_pose.left_hand.wrist_and_palm_normal_device is not None
        )

    def test_wrist_and_palm_valid_file_v3(self) -> None:
        wrist_and_palm_file = os.path.join(
            TEST_FOLDER, "mps_sample/hand_tracking/wrist_and_palm_poses_v3.csv"
        )
        mps_wrist_and_palm_poses = mps.hand_tracking.read_wrist_and_palm_poses(
            wrist_and_palm_file
        )
        self.assertEqual(len(mps_wrist_and_palm_poses), 63)
        # Test the second pose and randomly picked landmark from the file.
        wrist_and_palm_pose: mps.hand_tracking.WristAndPalmPose = (
            mps_wrist_and_palm_poses[1]
        )
        self.assertAlmostEqual(
            wrist_and_palm_pose.left_hand.landmark_positions_device[5][0], 0.187604
        )
        self.assertAlmostEqual(
            wrist_and_palm_pose.left_hand.landmark_positions_device[5][1], -0.190168
        )
        self.assertAlmostEqual(
            wrist_and_palm_pose.left_hand.landmark_positions_device[5][2], 0.242241
        )
        self.assertTrue(
            wrist_and_palm_pose.left_hand.wrist_and_palm_normal_device is None
        )

    def test_wrist_and_palm_invalid_file(self) -> None:
        wrist_and_palm_file = ""
        mps_wrist_and_palm_poses = mps.hand_tracking.read_wrist_and_palm_poses(
            wrist_and_palm_file
        )
        self.assertEqual(len(mps_wrist_and_palm_poses), 0)


class MPSEyeGaze(unittest.TestCase):
    """
    Tests for reading eye gaze data from csv files
    """

    def test_eyegaze_valid_file(self) -> None:
        eye_gaze_file = os.path.join(TEST_FOLDER, "mps_sample/eye_gaze/eyegaze.csv")
        mps_eye_gazes = mps.read_eyegaze(eye_gaze_file)
        self.assertGreater(len(mps_eye_gazes), 0)
        self.assertEqual(mps_eye_gazes[0].session_uid, "")
        self.assertEqual(mps_eye_gazes[0].vergence.left_yaw, 0.0)

    def test_eyegaze_valid_file_with_session_id(self) -> None:
        eye_gaze_file = os.path.join(
            TEST_FOLDER, "mps_sample/eye_gaze/generalized_eye_gaze.csv"
        )
        mps_eye_gazes = mps.read_eyegaze(eye_gaze_file)
        self.assertGreater(len(mps_eye_gazes), 0)
        self.assertNotEqual(mps_eye_gazes[0].session_uid, "")
        self.assertEqual(mps_eye_gazes[0].vergence.left_yaw, 0.0)

    def test_eyegaze_vergence_valid_file(self) -> None:
        eye_gaze_file = os.path.join(
            TEST_FOLDER, "mps_sample/eye_gaze_vergence/generalized_gaze.csv"
        )
        mps_eye_gazes = mps.read_eyegaze(eye_gaze_file)
        self.assertGreater(len(mps_eye_gazes), 0)
        self.assertNotEqual(mps_eye_gazes[0].session_uid, "")
        self.assertAlmostEqual(mps_eye_gazes[0].vergence.tx_left_eye, 0.0315)
        self.assertAlmostEqual(mps_eye_gazes[0].vergence.tx_right_eye, -0.0315)

    def test_eyegaze_invalid_file(self) -> None:
        eye_gaze_file = ""
        mps_eye_gazes = mps.read_eyegaze(eye_gaze_file)
        self.assertEqual(len(mps_eye_gazes), 0)

    def test_get_eyegaze_point_at_depth(self) -> None:
        gaze_x, gaze_y, gaze_z = mps.get_eyegaze_point_at_depth(
            -0.102910490016660, -0.288851886987686, 1.179526637404006
        )
        self.assertAlmostEqual(gaze_x, -0.116201334110103)
        self.assertAlmostEqual(gaze_y, -0.334355951558995)
        self.assertAlmostEqual(gaze_z, 1.12516063)

    def test_compute_depth_and_combined_gaze_direction(self) -> None:
        depth, combined_yaw, combined_pitch = (
            mps.compute_depth_and_combined_gaze_direction(
                -0.1264797, -0.07706982, -0.26359045
            )
        )
        self.assertAlmostEqual(depth, 1.31310134)
        self.assertAlmostEqual(combined_yaw, -0.10183712)
        self.assertAlmostEqual(combined_pitch, -0.26359045)

    def test_get_gaze_intersection_point(self) -> None:
        gaze_x, gaze_y, gaze_z = mps.get_gaze_intersection_point(
            -0.1264797, -0.07706982, -0.26359045
        )
        self.assertAlmostEqual(gaze_x, -0.12892598)
        self.assertAlmostEqual(gaze_y, -0.34047373)
        self.assertAlmostEqual(gaze_z, 1.26162232)

    def test_get_gaze_directions(self) -> None:
        left_gaze, right_gaze = mps.get_gaze_vectors(
            -0.1264797, -0.07706982, -0.26359045
        )
        self.assertAlmostEqual(left_gaze[0], -0.12185171)
        self.assertAlmostEqual(left_gaze[1], -0.25860713)
        self.assertAlmostEqual(left_gaze[2], 0.95826641)
        self.assertAlmostEqual(right_gaze[0], -0.07434921)
        self.assertAlmostEqual(right_gaze[1], -0.25982753)
        self.assertAlmostEqual(right_gaze[2], 0.96278858)

        """
        (-0.12185171, leftDirection.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.25860713, leftDirection.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.95826641, leftDirection.z(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.07434921, rightDirection.x(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(-0.25982753, rightDirection.y(), EYEGAZE_ERROR_TOLERANCE);
  ASSERT_NEAR(0.96278858
        """


mps_root_path = os.path.join(TEST_FOLDER, "mps_sample")


class MPSDataProvider(unittest.TestCase):
    """
    Tests for loading and querying data with MpsDataProvider
    """

    def test_data_paths_provider(self) -> None:
        data_paths_provider = mps.MpsDataPathsProvider(mps_root_path)
        data_paths = data_paths_provider.get_data_paths()
        assert os.path.exists(data_paths.eyegaze.general_eyegaze)
        assert os.path.exists(data_paths.eyegaze.summary)
        assert os.path.exists(data_paths.slam.closed_loop_trajectory)
        assert os.path.exists(data_paths.slam.open_loop_trajectory)
        assert os.path.exists(data_paths.slam.semidense_points)
        assert os.path.exists(data_paths.slam.semidense_observations)
        assert os.path.exists(data_paths.slam.online_calibrations)
        assert os.path.exists(data_paths.slam.summary)

    def test_data_provider(self) -> None:
        """
        Test loading and querying data with MpsDataProvider
        """
        data_paths_provider = mps.MpsDataPathsProvider(mps_root_path)
        data_paths = data_paths_provider.get_data_paths()
        dp = mps.MpsDataProvider(data_paths)

        assert dp.has_general_eyegaze()
        assert not dp.has_personalized_eyegaze()
        assert dp.has_open_loop_poses()
        assert dp.has_closed_loop_poses()
        assert dp.has_online_calibrations()
        assert dp.has_semidense_point_cloud()
        assert dp.has_semidense_observations()

        assert len(dp.get_semidense_point_cloud()) > 0
        assert len(dp.get_semidense_observations()) > 0

        assert dp.get_open_loop_pose(0)
        assert dp.get_closed_loop_pose(0)
        assert dp.get_general_eyegaze(0)
        assert dp.get_online_calibration(0)

        TEST_VERSION = "1.2.3"
        assert dp.get_slam_version() == TEST_VERSION
        assert dp.get_eyegaze_version() == TEST_VERSION
        assert dp.get_hand_tracking_version() == TEST_VERSION

    def test_data_provider_missing_path(self) -> None:
        """
        Test loading and querying data with MpsDataProvider when a path is missing
        """
        data_paths_provider = mps.MpsDataPathsProvider("/path/to/missing/data")
        data_paths = data_paths_provider.get_data_paths()
        dp = mps.MpsDataProvider(data_paths)

        assert not dp.has_general_eyegaze()
        assert not dp.has_personalized_eyegaze()
        assert not dp.has_open_loop_poses()
        assert not dp.has_closed_loop_poses()
        assert not dp.has_online_calibrations()
        assert not dp.has_semidense_point_cloud()
        assert not dp.has_semidense_observations()

        assert dp.get_slam_version() is None
        assert dp.get_eyegaze_version() is None
        assert dp.get_hand_tracking_version() is None
