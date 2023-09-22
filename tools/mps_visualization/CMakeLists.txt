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

message("--- Compiling mps_eyegaze_viewer.")
add_executable(mps_eyegaze_viewer main_eyegaze.cpp EyeGazeAriaPlayer.cpp EyeGazeAriaViewer.cpp EyeGazeAriaVisualizationData.h EyeGazeAriaPlayer.h EyeGazeAriaViewer.h)
target_link_libraries(mps_eyegaze_viewer
    PRIVATE
        aria_viewer_lib
        mps
        CLI11::CLI11
        Threads::Threads)

message("--- Compiling mps_3d_scene_viewer.")
add_library(mps_3d_scene_viewer_lib STATIC
    MPSData3DGui.h MPSData3DGui.cpp
    MPS3DSceneViewer.h MPS3DSceneViewer.cpp
)
target_include_directories(mps_3d_scene_viewer_lib PUBLIC "./")
target_link_libraries(mps_3d_scene_viewer_lib
    PRIVATE
        mps
        CLI11::CLI11
        ${Pangolin_LIBRARIES}
)
add_executable(mps_3d_scene_viewer
    main_3d_scene_viewer.cpp
)
target_link_libraries(mps_3d_scene_viewer
    PRIVATE
        mps_3d_scene_viewer_lib
)
message("--- Compiling mps_3d_replay_viewer.")
add_executable(mps_3d_replay_viewer
    main_3d_replay_viewer.cpp
    Data3DGui.cpp Data3DGui.h
    Boundary.h CachedDataProviders.h
    PangolinHelper.h
)
target_link_libraries(mps_3d_replay_viewer
    PRIVATE
        aria_viewer_lib
        mps
        CLI11::CLI11
        ${Pangolin_LIBRARIES}
)
