"""Shared helpers for planner_benchmark.py and path_capture.py: static map loading and
a blocking ComputePathToPose call."""

import math
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import ComputePathToPose
from PIL import Image
from scipy import ndimage


def resolve_map_yaml(name=None):
    """Absolute path of a map yaml; bare names are looked up in webots_spot/map."""
    name = name or "map.yaml"
    if os.path.isabs(name):
        return name
    return os.path.join(get_package_share_directory("webots_spot"), "map", name)


class StaticMap:
    """map_server-equivalent occupancy of a trinary map, plus a clearance EDT in metres."""

    def __init__(self, map_yaml=None):
        self.yaml_path = resolve_map_yaml(map_yaml)
        with open(self.yaml_path) as handle:
            meta = yaml.safe_load(handle)
        self.image_path = os.path.join(os.path.dirname(self.yaml_path), meta["image"])
        self.res = float(meta["resolution"])
        self.origin = meta["origin"]
        img = np.array(Image.open(self.image_path))
        # map_server trinary: occupied if (255 - p) / 255 > occupied_thresh
        occupied = (255.0 - img) / 255.0 > float(meta["occupied_thresh"])
        free = (255.0 - img) / 255.0 < float(meta["free_thresh"])
        self.occ = np.flipud(occupied)  # row 0 = lowest y
        self.free = np.flipud(free)
        self.clearance = ndimage.distance_transform_edt(~self.occ) * self.res

    @property
    def extent(self):
        """[xmin, xmax, ymin, ymax] for imshow with origin='lower'."""
        h, w = self.occ.shape
        x0, y0 = self.origin[0], self.origin[1]
        return [x0, x0 + w * self.res, y0, y0 + h * self.res]

    def to_world(self, r, c):
        return (
            self.origin[0] + (c + 0.5) * self.res,
            self.origin[1] + (r + 0.5) * self.res,
        )

    def to_cell(self, x, y):
        return int(math.floor((y - self.origin[1]) / self.res)), int(
            math.floor((x - self.origin[0]) / self.res)
        )

    def is_free(self, x, y):
        r, c = self.to_cell(x, y)
        h, w = self.free.shape
        return 0 <= r < h and 0 <= c < w and bool(self.free[r, c])

    def clearance_at(self, x, y):
        return float(sample_grid(self.clearance, x, y, self.res, self.origin[0], self.origin[1]))


def sample_grid(grid, x, y, res, ox, oy):
    """Bilinear sample of a cell-centred grid at world (x, y)."""
    r, c = (y - oy) / res - 0.5, (x - ox) / res - 0.5
    return ndimage.map_coordinates(grid, [[r], [c]], order=1, mode="nearest")[0]


def _yaw_quaternion(yaw):
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def compute_path(node, client, planner_id, start, goal):
    """Blocking ComputePathToPose. start/goal are (x, y) or (x, y, yaw) in the map frame.

    Returns (Nx3 array of x, y, yaw or None on failure, wall time in s).
    """
    goal_msg = ComputePathToPose.Goal()
    goal_msg.planner_id = planner_id
    goal_msg.use_start = True
    for pose, xy in ((goal_msg.start, start), (goal_msg.goal, goal)):
        pose.header.frame_id = "map"
        pose.pose.position.x = float(xy[0])
        pose.pose.position.y = float(xy[1])
        qz, qw = _yaw_quaternion(float(xy[2]) if len(xy) > 2 else 0.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
    t0 = time.perf_counter()
    send = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if not handle.accepted:
        return None, time.perf_counter() - t0
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res)
    wall = time.perf_counter() - t0
    result = res.result().result
    if len(result.path.poses) < 2:
        return None, wall
    pts = np.array(
        [
            [
                p.pose.position.x,
                p.pose.position.y,
                2.0 * math.atan2(p.pose.orientation.z, p.pose.orientation.w),
            ]
            for p in result.path.poses
        ]
    )
    return pts, wall
