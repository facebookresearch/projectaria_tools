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
import os

from projectaria_tools.core.sensor_data import TimeQueryOptions

from projectaria_tools.core.stream_id import StreamId

from projectaria_tools.projects.adt import (
    AriaDigitalTwinDataPathsProvider,
    AriaDigitalTwinDataProvider,
    AriaDigitalTwinSkeletonProvider,
    InstanceType,
    MotionType,
    RigidityType,
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sequence-path",
        dest="sequence_path",
        type=str,
        required=True,
        help="path to the data sequence to run",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    paths_provider = AriaDigitalTwinDataPathsProvider(args.sequence_path)
    all_device_serials = paths_provider.get_device_serial_numbers()
    print("all devices used in sequence: ", os.path.basename(args.sequence_path))
    print(all_device_serials)
    print("recording in scene: ", paths_provider.get_scene_name())
    print("is a multi-person sequence? ", paths_provider.is_multi_person())
    print("number of skeletons: ", paths_provider.get_num_skeletons())
    for idx, device_serial in enumerate(all_device_serials):
        # test data path provider
        print(idx, ": ", device_serial)
        data_paths = paths_provider.get_datapaths_by_device_num(idx)
        print("print all ground truth files")
        print(data_paths)
        print("access each members of data_paths")
        print("aria_vrs_filepath: ", data_paths.aria_vrs_filepath)
        print("aria_trajectory_filepath: ", data_paths.aria_trajectory_filepath)
        print("object_trajectories_filepath: ", data_paths.object_trajectories_filepath)
        print(
            "object_boundingbox_3d_filepath: ",
            data_paths.object_boundingbox_3d_filepath,
        )
        print("boundingboxes_2d_filepath: ", data_paths.boundingboxes_2d_filepath)
        print("segmentations_filepath: ", data_paths.segmentations_filepath)
        print("depth_images_filepath: ", data_paths.depth_images_filepath)
        print("eyegazes_filepath: ", data_paths.eyegazes_filepath)
        print("skeletons_filepaths: ", data_paths.skeletons_filepaths)
        print("metadata_filepath: ", data_paths.metadata_filepath)
        print("instances_filepath: ", data_paths.instances_filepath)

        stream_id = StreamId("214-1")  # use RGB camera for testing

        # test data provider
        gt_provider = AriaDigitalTwinDataProvider(data_paths)

        # test raw data access
        transform_device_rgb = gt_provider.get_aria_transform_device_camera(stream_id)
        print("transform_device_rgb: ", transform_device_rgb)
        timestamp_ns_rgb = gt_provider.get_aria_device_capture_timestamps_ns(stream_id)
        print("time range: [{}, {}]".format(timestamp_ns_rgb[0], timestamp_ns_rgb[-1]))
        eg_timestamp_ns_rgb = timestamp_ns_rgb[int(len(timestamp_ns_rgb) / 2)]
        tc_ts = gt_provider.get_timecode_from_device_time_ns(eg_timestamp_ns_rgb)
        print("{} mapped to time code time {}".format(eg_timestamp_ns_rgb, tc_ts))
        # TODO test other functions in image provider
        # test time range
        print(
            "GT time range [{}, {}]".format(
                gt_provider.get_start_time_ns(), gt_provider.get_end_time_ns()
            )
        )

        # test has functions
        print("has_aria_3d_poses? ", gt_provider.has_aria_3d_poses())
        print(
            "has_object_3d_boundingboxes? ", gt_provider.has_object_3d_boundingboxes()
        )
        print(
            "has_instance_2d_boundingboxes? ",
            gt_provider.has_instance_2d_boundingboxes(),
        )
        print("has_segmentation_images? ", gt_provider.has_segmentation_images())
        print("has_depth_images? ", gt_provider.has_depth_images())
        print("has_synthetic_images? ", gt_provider.has_synthetic_images())
        print("has_eyegaze? ", gt_provider.has_eyegaze())
        print("has_skeleton? ", gt_provider.has_skeleton())
        print("has_instances_info? ", gt_provider.has_instances_info())

        # test instances info
        instance_ids = gt_provider.get_instance_ids()
        print("Number of instances: ", len(instance_ids))
        first_instance_id = instance_ids[0]
        print(
            "first instance id exist: ", gt_provider.has_instance_id(first_instance_id)
        )
        print(
            "first instance: ", gt_provider.get_instance_info_by_id(first_instance_id)
        )

        print("first symmetric object")
        for instance_id in instance_ids:
            instance_info = gt_provider.get_instance_info_by_id(instance_id)
            if (
                instance_info.rotational_symmetry.is_annotated
                and instance_info.rotational_symmetry.axes
            ):
                print(instance_info)
                break

        print("first dynamic object")
        for instance_id in instance_ids:
            instance_info = gt_provider.get_instance_info_by_id(instance_id)
            if instance_info.motion_type == MotionType.DYNAMIC:
                print(instance_info)
                break

        print("first rigid object")
        for instance_id in instance_ids:
            instance_info = gt_provider.get_instance_info_by_id(instance_id)
            if instance_info.rigidity_type == RigidityType.RIGID:
                print(instance_info)
                break

        print("first human object")
        for instance_id in instance_ids:
            instance_info = gt_provider.get_instance_info_by_id(instance_id)
            if instance_info.instance_type == InstanceType.HUMAN:
                print(instance_info)
                break

        print("skeleton info")
        skeleton_ids = gt_provider.get_skeleton_ids()
        for skeleton_id in skeleton_ids:
            skeleton_info = gt_provider.get_instance_info_by_id(skeleton_id)
            print("skeleton id queried: ", skeleton_id)
            print("skeleton info returned:")
            print(skeleton_info)

        print("number of objects: ", len(gt_provider.get_object_ids()))

        # test gt getter
        # raw data
        aria_image_with_dt = gt_provider.get_aria_image_by_timestamp_ns(
            eg_timestamp_ns_rgb, stream_id
        )
        print("aria image valid: ", aria_image_with_dt.is_valid())
        print("aria image dt: ", aria_image_with_dt.dt_ns())
        a_img = aria_image_with_dt.data()
        print("aria image dim: [{}, {}]".format(a_img.get_width(), a_img.get_height()))
        print(
            "aria image value:[{}, {}, {}]".format(
                a_img.at(938, 938, 0), a_img.at(938, 938, 1), a_img.at(938, 938, 2)
            )
        )
        a_img_np = a_img.to_numpy_array()
        print("aria image numpy shape: {}".format(a_img_np.shape))
        print("aria image numpy value: {}".format(a_img_np[938, 938]))

        # Aria 3d pose
        aria_3d_pose_with_dt = gt_provider.get_aria_3d_pose_by_timestamp_ns(
            eg_timestamp_ns_rgb
        )
        aria_3d_pose = aria_3d_pose_with_dt.data()
        print("aria 3d pose valid: ", aria_3d_pose_with_dt.is_valid())
        print("aria 3d pose dt: ", aria_3d_pose_with_dt.dt_ns())
        print(
            "aria 3d pose transform_scene_device: ", aria_3d_pose.transform_scene_device
        )
        print("aria device linear velo: ", aria_3d_pose.device_linear_velocity)
        print("aria device rotational velo: ", aria_3d_pose.device_rotational_velocity)

        # Object 3d bbox
        obj_3d_bboxes_with_dt = gt_provider.get_object_3d_boundingboxes_by_timestamp_ns(
            eg_timestamp_ns_rgb
        )
        obj_3d_bboxes = obj_3d_bboxes_with_dt.data()
        print("obj 3d bboxes valid: ", obj_3d_bboxes_with_dt.is_valid())
        print("obj 3d bboxes dt: ", obj_3d_bboxes_with_dt.dt_ns())
        print("obj 3d bboxes len: ", len(obj_3d_bboxes))
        for objId in obj_3d_bboxes_with_dt.data():
            obj_3d_bbox = obj_3d_bboxes[objId]
            print("obj id: ", objId)
            print("obj transform_scene_object: ", obj_3d_bbox.transform_scene_object)
            print("obj aabb: ", obj_3d_bbox.aabb)
            break

        # Object 2d bbox
        obj_2d_bboxes_with_dt = gt_provider.get_object_2d_boundingboxes_by_timestamp_ns(
            eg_timestamp_ns_rgb, stream_id
        )
        obj_2d_bboxes = obj_2d_bboxes_with_dt.data()
        print("obj 2d bboxes valid: ", obj_2d_bboxes_with_dt.is_valid())
        print("obj 2d bboxes dt: ", obj_2d_bboxes_with_dt.dt_ns())
        print("obj 2d bboxes len: ", len(obj_2d_bboxes))
        for objId in obj_2d_bboxes_with_dt.data():
            obj_2d_bbox = obj_2d_bboxes[objId]
            print("obj id: ", objId)
            print("obj box_range: ", obj_2d_bbox.box_range)
            print("obj visibility_ratio: ", obj_2d_bbox.visibility_ratio)
            break

        # Object segmentation
        seg_with_dt = gt_provider.get_segmentation_image_by_timestamp_ns(
            eg_timestamp_ns_rgb, stream_id
        )
        seg = seg_with_dt.data()
        print("Segmentation valid: ", seg_with_dt.is_valid())
        print("Segmentation dt: ", seg_with_dt.dt_ns())
        print("Segmentation dim: [{}, {}]".format(seg.get_width(), seg.get_height()))
        print("Segmentation vis value:{}".format(seg.get_visualizable().at(938, 938)))
        print("Segmentation value: {}".format(seg.at(938, 938)))
        seg_np = seg.to_numpy_array()
        print("Segmentation numpy shape: {}".format(seg_np.shape))
        print("Segmentation numpy value: {}".format(seg_np[938, 938]))

        # Object depth
        dep_with_dt = gt_provider.get_depth_image_by_timestamp_ns(
            eg_timestamp_ns_rgb, stream_id
        )
        dep = dep_with_dt.data()
        print("Depth map valid: ", dep_with_dt.is_valid())
        print("Depth map dt: ", dep_with_dt.dt_ns())
        print("Depth map dim: [{}, {}]".format(dep.get_width(), dep.get_height()))
        print("Depth map vis value:{}".format(dep.get_visualizable().at(938, 938)))
        print("Depth map value: {}".format(dep.at(938, 938)))
        dep_np = dep.to_numpy_array()
        print("Depth map numpy shape: {}".format(dep_np.shape))
        print("Depth map numpy dtype: {}".format(dep_np.dtype))
        print("Depth map numpy value: {}".format(dep_np[938, 938]))

        # Object synthetic
        syn_with_dt = gt_provider.get_synthetic_image_by_timestamp_ns(
            eg_timestamp_ns_rgb, stream_id
        )
        syn = syn_with_dt.data()
        print("Synthetic valid: ", syn_with_dt.is_valid())
        print("Synthetic dt: ", syn_with_dt.dt_ns())
        print("Synthetic dim: [{}, {}]".format(syn.get_width(), syn.get_height()))
        syn_with_dt = gt_provider.get_synthetic_image_by_timestamp_ns(
            eg_timestamp_ns_rgb, stream_id, TimeQueryOptions.BEFORE
        )
        print("Synthetic query before dt: ", syn_with_dt.dt_ns())

        # Eye gaze data
        eyegaze_with_dt = gt_provider.get_eyegaze_by_timestamp_ns(eg_timestamp_ns_rgb)
        eyegaze = eyegaze_with_dt.data()
        print("Eye gaze valid: ", eyegaze_with_dt.is_valid())
        print("Eye gaze dt: ", eyegaze_with_dt.dt_ns())
        print(
            f"eye gaze depth: {eyegaze.depth}, eye gaze angles: {eyegaze.yaw}, {eyegaze.pitch}"
        )

        # Skeleton data
        if skeleton_ids:
            skeleton_with_dt = gt_provider.get_skeleton_by_timestamp_ns(
                eg_timestamp_ns_rgb, skeleton_ids[0]
            )
            print("Skeleton valid: ", skeleton_with_dt.is_valid())
            skeleton = skeleton_with_dt.data()
            print("number of joints: ", len(skeleton.joints))
            print("number of markers: ", len(skeleton.markers))

        print(AriaDigitalTwinSkeletonProvider.get_joint_connections())
        print(AriaDigitalTwinSkeletonProvider.get_joint_labels())
        print(AriaDigitalTwinSkeletonProvider.get_marker_labels())

        # Test getters of data provider pointers
        raw_data_provider = gt_provider.raw_data_provider_ptr()
        print(f"raw vrs data have streams: {raw_data_provider.get_all_streams()}")
        segmentation_data_provider = gt_provider.segmentation_data_provider_ptr()
        print(
            f"segmentation vrs data have streams: {segmentation_data_provider.get_all_streams()}"
        )
        depth_data_provider = gt_provider.depth_data_provider_ptr()
        print(f"depth vrs data have streams: {depth_data_provider.get_all_streams()}")
        synthetic_data_provider = gt_provider.synthetic_data_provider_ptr()
        print(
            f"synthetic vrs data have streams: {synthetic_data_provider.get_all_streams()}"
        )
