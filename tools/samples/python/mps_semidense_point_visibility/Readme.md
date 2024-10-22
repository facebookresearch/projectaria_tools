# Code Sample - Python - Using MPS Semi Dense Point cloud observation

## Description

This is a code sample showing how you can use [semi dense point cloud observations](https://facebookresearch.github.io/projectaria_tools/docs/data_formats/mps/slam/mps_pointcloud#point-observations), in other words, how to query MPS point cloud observations that are visible at a given time and for a given camera.

### How to use

Assuming projectaria_tools has been installed in your python environment. Please run:

```bash
python mps_semidense_point_visibility_demo.py
```

## Key learnings

1. MPS Point cloud data consists of

- A global point cloud (3D points)
- Point cloud observations for the SLAM cameras (visibility information for camera_serial and timestamps)

> [!TIP]
> Global points and their observations are linked together by their unique 3D point ids

2. MPS data is connected to corresponding VRS image by using the timestamp

- MPS point cloud data observations is indexed by camera_serial and can be slam_left or slam_right
- MPS data is indexed by timestamp in microseconds
- VRS data is indexed by timestamp in nanoseconds

3. This code sample show how to keep a list of visible tracks at the current timestamp

- I.E point observations are accumulated and hashed by their global point unique ids
- if a point is not visible on the current frame, the track is removed
