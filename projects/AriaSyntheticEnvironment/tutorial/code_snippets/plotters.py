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

import numpy as np
import plotly.graph_objects as go

from .constants import UNIT_CUBE_LINES_IDXS, UNIT_CUBE_VERTICES
from .interpreter import language_to_bboxes
from .readers import read_language_file, read_points_file, read_trajectory_file


PLOTTING_COLORS = {
    "wall": "#FBFAF5",
    "door": "#F7C59F",
    "window": "#53F4FF",
    "points": "#C7DAE8",
    "trajectory": "#F92A82",
}


# This function plots a wire frame for each language entity bounding box loaded from lang
def plot_box_wireframe(box):
    box_verts = UNIT_CUBE_VERTICES * box["scale"]
    box_verts = (box["rotation"] @ box_verts.T).T
    box_verts = box_verts + box["center"]

    lines_x = []
    lines_y = []
    lines_z = []
    for pair in UNIT_CUBE_LINES_IDXS:
        for idx in pair:
            lines_x.append(box_verts[idx, 0])
            lines_y.append(box_verts[idx, 1])
            lines_z.append(box_verts[idx, 2])
        lines_x.append(None)
        lines_y.append(None)
        lines_z.append(None)

    class_name = box["class"]
    wireframe = go.Scatter3d(
        x=lines_x,
        y=lines_y,
        z=lines_z,
        name=box["id"],
        mode="lines",
        line={
            "color": PLOTTING_COLORS[class_name],
            "width": 10,
        },
    )
    return wireframe


def plot_point_cloud(points, max_points_to_plot=500_000):
    if len(points) > max_points_to_plot:
        print(
            f"The number of points ({len(points)}) exceeds the maximum that can be reliably plotted."
        )
        print(f"Randomly subsampling {max_points_to_plot} points for the plot.")
        sampled = np.random.choice(len(points), max_points_to_plot, replace=False)
        points = points[sampled]
    return go.Scatter3d(
        x=points[:, 0],
        y=points[:, 1],
        z=points[:, 2],
        mode="markers",
        name="Semi-dense Points",
        marker={
            "size": 1.0,
            "opacity": 0.3,
            "color": PLOTTING_COLORS["points"],
        },
    )


def plot_trajectory(trajectory):
    device_positions = trajectory["ts"]
    return go.Scatter3d(
        x=device_positions[:, 0],
        y=device_positions[:, 1],
        z=device_positions[:, 2],
        name="Device Poses",
        mode="lines+markers",
        marker={
            "size": 3,
            "opacity": 1.0,
            "color": PLOTTING_COLORS["trajectory"],
        },
        line={
            "color": PLOTTING_COLORS["trajectory"],
            "width": 3,
        },
    )


# Main plotting function
def plot_3d_scene(language_path=None, points_path=None, trajectory_path=None):
    traces = []
    if points_path is not None:
        points = read_points_file(points_path)
        traces.append(plot_point_cloud(points))

    if trajectory_path is not None:
        trajectory = read_trajectory_file(trajectory_path)
        traces.append(plot_trajectory(trajectory))

    if language_path is not None:
        entities = read_language_file(language_path)
        boxes = language_to_bboxes(entities)
        for box in boxes:
            traces.append(plot_box_wireframe(box))

    assert traces, "Nothing to visualize."
    fig = go.Figure(data=traces)
    fig.update_layout(
        template="plotly_dark",
        scene={
            "xaxis": {"showticklabels": False, "title": ""},
            "yaxis": {"showticklabels": False, "title": ""},
            "zaxis": {"showticklabels": False, "title": ""},
        },
    )
    fig.show()
