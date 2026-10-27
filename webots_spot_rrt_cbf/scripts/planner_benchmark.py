#!/usr/bin/env python3
"""Compare NavFn (GridBased) and RrtCbf global planners on the Spot map.

Requires `planner_bench_launch.py` (or a full Nav2 stack) to be running and the PINN
safety field to be published on /pinn_safety/field (latched). Calls the
ComputePathToPose action for random start/goal pairs, computes path metrics from the
static map and the PINN field, and writes one CSV row per (planner, pair).

Metrics: success, length, min/mean obstacle clearance (static map), min/mean normalised
h along the path, planning wall time, and a CBF violation count
(samples with v * dh/ds + kappa * h <= 0) so the two planners are scored with the same CBF.
"""

import argparse
import csv
import math
import time

import numpy as np
import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from webots_spot_msgs.msg import SafetyField

from bench_common import StaticMap, compute_path, sample_grid


class Bench(Node):
    def __init__(self, args):
        super().__init__("planner_benchmark")
        self.args = args
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.field = None
        self.field_msgs = 0
        self.create_subscription(SafetyField, "/pinn_safety/field", self._on_field, qos)
        self.client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

        self.map = StaticMap()
        self.res = self.map.res
        self.origin = self.map.origin
        self.free = self.map.free
        self.clearance = self.map.clearance

    def _on_field(self, msg):
        self.field_msgs += 1
        h = np.array(msg.h, dtype=np.float64).reshape(msg.info.height, msg.info.width)
        self.field = {
            "h": h / h.max(),
            "dhdx": np.array(msg.dhdx).reshape(h.shape) / h.max(),
            "dhdy": np.array(msg.dhdy).reshape(h.shape) / h.max(),
            "res": msg.info.resolution,
            "ox": msg.info.origin.position.x,
            "oy": msg.info.origin.position.y,
        }

    # --- map helpers -----------------------------------------------------------------
    def _sample(self, grid, x, y, res, ox, oy):
        return sample_grid(grid, x, y, res, ox, oy)

    def sample_pairs(self, n, seed, min_clear, min_dist, max_dist):
        rng = np.random.default_rng(seed)
        ok = self.free & (self.clearance > min_clear)
        ok = ndimage.binary_erosion(ok, iterations=1)
        rows, cols = np.nonzero(ok)
        pairs = []
        while len(pairs) < n:
            i, j = rng.integers(len(rows), size=2)
            s = self._to_world(rows[i], cols[i])
            g = self._to_world(rows[j], cols[j])
            d = math.dist(s, g)
            if min_dist <= d <= max_dist:
                pairs.append((s, g))
        return pairs

    def _to_world(self, r, c):
        return self.map.to_world(r, c)

    # --- planning --------------------------------------------------------------------
    def plan(self, planner_id, start, goal):
        pts, wall = compute_path(self, self.client, planner_id, start, goal)
        return (None if pts is None else pts[:, :2]), wall

    def metrics(self, pts):
        seg = np.hypot(*np.diff(pts, axis=0).T)
        # Resample at 2.5 cm for fair clearance / h statistics.
        s = np.concatenate([[0], np.cumsum(seg)])
        t = np.arange(0, s[-1], 0.025)
        xs, ys = np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])
        clr = np.array(
            [self._sample(self.clearance, x, y, self.res, self.origin[0], self.origin[1])
             for x, y in zip(xs, ys)]
        )
        out = {"length": s[-1], "clr_min": clr.min(), "clr_mean": clr.mean()}
        if self.field:
            f = self.field
            hv = np.array([self._sample(f["h"], x, y, f["res"], f["ox"], f["oy"]) for x, y in zip(xs, ys)])
            gx = np.array([self._sample(f["dhdx"], x, y, f["res"], f["ox"], f["oy"]) for x, y in zip(xs, ys)])
            gy = np.array([self._sample(f["dhdy"], x, y, f["res"], f["ox"], f["oy"]) for x, y in zip(xs, ys)])
            th = np.arctan2(np.gradient(ys), np.gradient(xs))
            cbf = self.args.v * (gx * np.cos(th) + gy * np.sin(th)) + self.args.kappa * hv
            out.update(h_min=hv.min(), h_mean=hv.mean(), cbf_violation_ratio=float(np.mean(cbf <= 0)))
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pairs", type=int, default=20)
    ap.add_argument("--repeats", type=int, default=1, help="plans per pair and planner")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--min-clear", type=float, default=0.4)
    ap.add_argument("--min-dist", type=float, default=3.0)
    ap.add_argument("--max-dist", type=float, default=12.0)
    ap.add_argument("--planners", default="GridBased,RrtCbf")
    ap.add_argument("--v", type=float, default=0.5, help="CBF v used for scoring")
    ap.add_argument("--kappa", type=float, default=2.0, help="CBF kappa used for scoring")
    ap.add_argument("--out", default="planner_benchmark.csv")
    args = ap.parse_args()

    rclpy.init()
    node = Bench(args)
    node.get_logger().info("Waiting for compute_path_to_pose action server")
    node.client.wait_for_server()
    t0 = time.time()
    while node.field is None and time.time() - t0 < 120.0:
        rclpy.spin_once(node, timeout_sec=0.5)
    if node.field is None:
        node.get_logger().error("No /pinn_safety/field received; h metrics disabled")

    pairs = node.sample_pairs(args.pairs, args.seed, args.min_clear, args.min_dist, args.max_dist)
    fields = ["pair", "planner", "rep", "success", "wall_s", "length", "clr_min", "clr_mean",
              "h_min", "h_mean", "cbf_violation_ratio", "sx", "sy", "gx", "gy"]
    rows = []
    with open(args.out, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for i, (s, g) in enumerate(pairs):
            for planner in args.planners.split(","):
                for rep in range(args.repeats):
                    pts, wall = node.plan(planner, s, g)
                    row = {"pair": i, "planner": planner, "rep": rep, "success": int(pts is not None),
                           "wall_s": round(wall, 4), "sx": s[0], "sy": s[1], "gx": g[0], "gy": g[1]}
                    if pts is not None:
                        row.update({k: round(float(v), 4) for k, v in node.metrics(pts).items()})
                    writer.writerow(row)
                    rows.append(row)
                    handle.flush()

    print(f"\nSafety field messages seen by benchmark: {node.field_msgs} (latched, 1 expected)")
    print(f"{'planner':10s} {'ok':>5s} {'len[m]':>8s} {'clr_min':>8s} {'clr_mean':>9s} "
          f"{'h_mean':>7s} {'cbf_viol':>9s} {'wall[s]':>8s}")
    for planner in args.planners.split(","):
        ok = [r for r in rows if r["planner"] == planner and r["success"]]
        n = len([r for r in rows if r["planner"] == planner])

        def mean(key):
            vals = [r[key] for r in ok if key in r]
            return float(np.mean(vals)) if vals else float("nan")

        print(f"{planner:10s} {len(ok):>2d}/{n:<2d} {mean('length'):8.2f} {mean('clr_min'):8.3f} "
              f"{mean('clr_mean'):9.3f} {mean('h_mean'):7.3f} {mean('cbf_violation_ratio'):9.3f} "
              f"{mean('wall_s'):8.3f}")
    print(f"CSV: {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
