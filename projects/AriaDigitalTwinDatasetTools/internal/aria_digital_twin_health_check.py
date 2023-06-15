#!/usr/bin/env python3
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

import argparse
import logging
import os
import traceback

from projectaria_tools.core.stream_id import StreamId

from projectaria_tools.projects.adt import (
    AriaDigitalTwinDataPathsProvider,
    AriaDigitalTwinDataProvider,
)

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


class AriaDigitalTwinHealthCheck:
    """
    A class to check the health of a ADT sequence
    """

    __DELTA_TIME_TOLERANCE_NS = 100e6  # 100 ms

    def __init__(
        self,
        sequence_path: str,
    ):
        self.sequence_path = sequence_path
        self.sequence_name = os.path.basename(sequence_path)
        self.health_flag = True
        self.error_msg = f"- error message for {self.sequence_name} - \n"

    def __log_error(self, new_error_msg):
        """
        A internal helper function to log error message, and set health flag to False
        """
        self.error_msg += new_error_msg + "\n"
        self.health_flag = False

    def __check_data_provider_query_function(
        self, query_function, timestamp, stream_id=None
    ):
        """
        An internal helper function to check if a specific query API in AdtDataProvider returns valid results,
        and also if the delta_time is within valid range (for 30 Hz data)
        """
        if stream_id is None:
            data_with_dt = query_function(timestamp)
        else:
            data_with_dt = query_function(timestamp, stream_id)

        # Check data validity
        if not data_with_dt.is_valid():
            self.__log_error(
                f"data querying {query_function.__name__} from timestamp {timestamp} has returned invalid result. "
            )
            return False

        # Check delta T
        dt = data_with_dt.dt_ns()
        if abs(dt) > self.__DELTA_TIME_TOLERANCE_NS:
            self.__log_error(
                f"Queried data {query_function.__name__} from timestamp {timestamp} "
                f"has dtNs of {abs(dt)}, which is larger than threshold of {self.__DELTA_TIME_TOLERANCE_NS}. "
            )
            return False

        return True

    def __check_path_provider(self):
        """
        This function checks if we can successfully create a AdtDataPathsProvider with correct content
        """
        self.error_msg += "--- AdtDataPathsProvider --- \n"

        # Try create a DataPathProvider
        try:
            self.paths_provider = AriaDigitalTwinDataPathsProvider(self.sequence_path)
        except Exception:
            self.__log_error(traceback.format_exc())
            return False

        all_devices = self.paths_provider.get_device_serial_numbers()

        # Check for device count is not 0
        if len(all_devices) == 0:
            self.__log_error(" Device serial count is 0")
            return False

        # Ensure can get valid DataPaths for each SN
        self.num_skeleton = self.paths_provider.get_num_skeletons()
        skeleton_flag = self.num_skeleton > 0
        for device_id in range(len(all_devices)):
            data_paths_via_id = self.paths_provider.get_datapaths_by_device_num(
                device_id, skeleton_flag
            )
            data_paths_via_sn = self.paths_provider.get_datapaths_by_device_serial(
                all_devices[device_id], skeleton_flag
            )

            # check both DataPaths are valid
            if data_paths_via_id is None:
                self.__log_error(
                    f"Cannot obtain valid DataPath via device id {device_id}"
                )
                return False
            if data_paths_via_sn is None:
                self.__log_error(
                    f"Cannot obtain valid DataPath via device serial {all_devices[device_id]}"
                )
                return False

            # check both data paths point to the same directory
            if str(data_paths_via_id) != str(data_paths_via_sn):
                self.__log_error(
                    f"The DataPaths representation obtained from id and serial are different: \n"
                    f"DataPaths from device id: {str(data_paths_via_id)}"
                    f"DataPaths from device sn: {str(data_paths_via_sn)}"
                )
                return False

        # No error
        self.error_msg += " No error \n"
        return True

    def __check_data_provider(self, data_path, device_serial):
        """
        This function checks the contents in `ADTDataProvider`.
        """
        self.error_msg += f"--- AdtDataProvider for {device_serial}--- \n"

        # Try create a ADT data provider
        try:
            data_provider = AriaDigitalTwinDataProvider(data_path)
        except Exception:
            self.__log_error(traceback.format_exc())
            return False

        # Check validity of various data provider pointers
        if data_provider.raw_data_provider_ptr() is None:
            self.__log_error("Raw data provider ptr is null.")
            return False
        if data_provider.segmentation_data_provider_ptr() is None:
            self.__log_error("Segmentation data provider ptr is null.")
            return False
        if data_provider.depth_data_provider_ptr() is None:
            self.__log_error("Depth data provider ptr is null.")
            return False
        if data_provider.synthetic_data_provider_ptr() is None:
            self.__log_error("Synthetic data provider ptr is null.")
            return False

        # Check streams exist
        all_streams = data_provider.get_aria_all_streams()
        if len(all_streams) == 0:
            self.__log_error("Aria recording should have at least 1 stream")
            return False

        # loop over all streams
        camera_streamids = [StreamId("214-1"), StreamId("1201-1"), StreamId("1201-2")]
        for stream_id in [s for s in all_streams if s in camera_streamids]:
            # Get middle timestamp from the first stream
            timestamps = data_provider.get_aria_device_capture_timestamps_ns(stream_id)
            if len(timestamps) == 0:
                self.__log_error(
                    "Aria recording should have more than 0 timestamp samples for stream"
                )
                return False

            # Check a number of query functions on a middle sample
            sample_timestamp = timestamps[len(timestamps) // 2]

            query_function_list_w_streamid = [
                data_provider.get_aria_image_by_timestamp_ns,
                data_provider.get_object_2d_boundingboxes_by_timestamp_ns,
                data_provider.get_segmentation_image_by_timestamp_ns,
                data_provider.get_depth_image_by_timestamp_ns,
                data_provider.get_synthetic_image_by_timestamp_ns,
            ]

            if self.num_skeleton > 0:
                query_function_list_w_streamid.append(
                    data_provider.get_skeleton_2d_boundingboxes_by_timestamp_ns
                )

            query_function_list_wo_streamid = [
                data_provider.get_aria_3d_pose_by_timestamp_ns,
                data_provider.get_object_3d_boundingboxes_by_timestamp_ns,
                data_provider.get_eyegaze_by_timestamp_ns,
            ]

            for function_w_streamid in query_function_list_w_streamid:
                query_check_flag = self.__check_data_provider_query_function(
                    function_w_streamid, sample_timestamp, stream_id
                )
                if not query_check_flag:
                    return False
            for function_wo_streamid in query_function_list_wo_streamid:
                query_check_flag = self.__check_data_provider_query_function(
                    function_wo_streamid, sample_timestamp
                )
                if not query_check_flag:
                    return False

        if self.num_skeleton > 0:
            # Check if skeletons count is none-zero
            skeleton_ids = data_provider.get_skeleton_ids()
            if len(skeleton_ids) == 0:
                self.__log_error("Get skeleton ids returns 0.")
                return False

            # Check if query skeleton through sample_timestamp return at least one valid result
            for skeleton_id in skeleton_ids:
                query_skeleton = data_provider.get_skeleton_by_timestamp_ns(
                    sample_timestamp, skeleton_id
                )
                if not query_skeleton.is_valid():
                    self.__log_error(
                        f"Query skeleton {skeleton_id} from the dataset returns empty result!"
                    )
                    return False

        return True

    def run_health_check(self) -> bool:
        """
        This function is the main entry point to perform data health check
        """

        path_provider_flag = self.__check_path_provider()
        if not path_provider_flag:
            return False

        for device_serial in self.paths_provider.get_device_serial_numbers():
            data_provider_flag = self.__check_data_provider(
                self.paths_provider.get_datapaths_by_device_serial(
                    device_serial, self.num_skeleton > 0
                ),
                device_serial,
            )
            if not data_provider_flag:
                return False

        return True

    def print_summary(self):
        """
        Printing health check summary towards the end
        """
        if self.health_flag:
            logging.info(f"{self.sequence_name} has PASSED all health checks!")
        else:
            logging.error(f"{self.sequence_name} has FAILED health checks!")
            logging.error(f"The full error message is:\n{self.error_msg}")


def parse_args():
    """
    A helper function for parsing input arguments from command line
    """
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        "-s",
        "--sequence-path",
        dest="sequence_path",
        type=str,
        required=True,
        default=None,
        help="Path to the ADT sequence to validate",
    )

    return parser.parse_args()


def adt_health_check_wrapper(sequence_path: str) -> bool:
    health_checker = AriaDigitalTwinHealthCheck(
        sequence_path=sequence_path,
    )

    success = health_checker.run_health_check()

    health_checker.print_summary()

    # return whether it is successful or not. Hardcoded as True
    # for now
    return success


if __name__ == "__main__":
    args = parse_args()
    adt_health_check_wrapper(sequence_path=args.sequence_path)
