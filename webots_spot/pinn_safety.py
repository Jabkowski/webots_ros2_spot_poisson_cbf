#!/usr/bin/env python3

import os
import sys
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


GRID_SIZE = 512
OCCUPIED_THRESH = 65


class PinnSafety(Node):
    def __init__(self):
        super().__init__("pinn_safety")

        params = self._load_params()
        pinn_root = params["pinn_root"]
        if pinn_root not in sys.path:
            sys.path.insert(0, pinn_root)

        import torch
        from models.double_unet import UNetDoublePoisson, denormalize_pred_3ch

        self._torch = torch
        self._denormalize_pred_3ch = denormalize_pred_3ch

        weights_path = self._resolve_path(pinn_root, params["weights_path"])
        norm_stats_path = self._resolve_path(pinn_root, params["norm_stats_path"])
        self._dilate_map = int(params["dilate_map"])

        self.get_logger().info("Loading PINN on CPU")
        self._device = torch.device("cpu")
        self._model = UNetDoublePoisson().to(self._device)
        state_dict = torch.load(weights_path, map_location=self._device)
        self._model.load_state_dict(state_dict)
        self._model.eval()

        norm_payload = torch.load(norm_stats_path, map_location="cpu")
        self._normalization_stats = norm_payload["stats"]

        self._map_msg = None
        self.h = None

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self._h_pub = self.create_publisher(OccupancyGrid, "/pinn_safety/h", map_qos)
        self.create_service(Trigger, "run", self._on_run)

        self.get_logger().info(
            "Waiting for /map. Call trigger: ros2 service call /pinn_safety/run std_srvs/srv/Trigger"
        )

    @staticmethod
    def _load_params():
        params_path = os.path.join(
            get_package_share_directory("webots_spot"),
            "params",
            "pinn_safety.yaml",
        )
        with open(params_path, "r") as handle:
            return yaml.safe_load(handle)["pinn_safety"]["ros__parameters"]

    @staticmethod
    def _resolve_path(pinn_root, path):
        if os.path.isabs(path):
            return path
        return os.path.join(pinn_root, path)

    def _on_map(self, msg):
        self._map_msg = msg
        self.get_logger().info(
            f"Received /map {msg.info.width}x{msg.info.height}, "
            f"resolution={msg.info.resolution:.3f}"
        )

    def _occupancy_to_grid(self, msg):
        raw = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )
        grid = (raw >= OCCUPIED_THRESH).astype(np.float32)
        return grid

    def _dilate_grid(self, grid_tensor):
        if self._dilate_map <= 0:
            return grid_tensor
        x = grid_tensor
        for _ in range(self._dilate_map):
            x = self._torch.nn.functional.max_pool2d(
                x, kernel_size=3, stride=1, padding=1
            )
        return x

    def _on_run(self, request, response):
        del request
        if self._map_msg is None:
            response.success = False
            response.message = "No /map received yet"
            return response

        torch = self._torch
        grid = self._occupancy_to_grid(self._map_msg)
        input_grid = torch.from_numpy(grid)[None, None, ...].to(self._device)
        input_grid = self._dilate_grid(input_grid)
        if input_grid.shape[-2:] != (GRID_SIZE, GRID_SIZE):
            input_grid = torch.nn.functional.interpolate(
                input_grid,
                size=(GRID_SIZE, GRID_SIZE),
                mode="nearest",
            )

        start = time.perf_counter()
        with torch.no_grad():
            pred_3ch = self._model(input_grid)
            pred_3ch = self._denormalize_pred_3ch(pred_3ch, self._normalization_stats)
        elapsed = time.perf_counter() - start

        pred_3ch = pred_3ch.cpu().numpy().squeeze(0)
        self.h = pred_3ch[2]
        h_min = float(self.h.min())
        h_max = float(self.h.max())
        h_mean = float(self.h.mean())

        self._publish_h_map(self.h, grid)

        response.success = True
        response.message = (
            f"dilate_map={self._dilate_map} h shape={tuple(self.h.shape)} "
            f"min={h_min:.4f} max={h_max:.4f} mean={h_mean:.4f} time={elapsed:.3f}s"
        )
        self.get_logger().info(response.message)
        return response

    def _publish_h_map(self, h_pred, occupancy):
        torch = self._torch
        h_tensor = torch.from_numpy(h_pred)[None, None, ...]
        h_map = torch.nn.functional.interpolate(
            h_tensor,
            size=occupancy.shape,
            mode="bilinear",
            align_corners=False,
        )
        h_map = h_map.cpu().numpy().squeeze()

        h_min = float(h_map.min())
        h_max = float(h_map.max())
        if h_max > h_min:
            # RViz costmap colors reserve 99 (cyan) and 100 (lethal).
            scaled = (h_map - h_min) / (h_max - h_min) * 98.0
        else:
            scaled = np.zeros_like(h_map)
        scaled[occupancy >= 1.0] = 0.0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_msg.header.frame_id
        msg.info = self._map_msg.info
        msg.data = np.clip(np.rint(scaled), 0, 98).astype(np.int8).flatten().tolist()
        self._h_pub.publish(msg)
        self.get_logger().info("Published /pinn_safety/h")


def main(args=None):
    rclpy.init(args=args)
    node = PinnSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
