#!/usr/bin/env python3
"""Capture repeated RrtCbf plans for fixed start/goal scenarios and several values of `c`.

Each plan is a single ComputePathToPose call (no NavigateToPose, so no BT replanning and no
robot motion). Before every call the planner's runtime parameters are set over
/planner_server/set_parameters: `RrtCbf.c` and `RrtCbf.seed = seed0 + rep` (seed_mode fixed),
so repeats differ and a run is reproducible from its seeds.

  ros2 launch webots_spot_rrt_cbf planner_bench_launch.py
  ros2 run webots_spot_rrt_cbf path_capture.py --pick --scenarios scenarios.yaml
  ros2 run webots_spot_rrt_cbf path_capture.py --scenarios scenarios.yaml \
      --c 0,1 --repeats 30 --seed0 0 --out captures/run_001

Output directory: meta.yaml, scenarios.yaml, summary.csv, paths.csv (long format:
scenario, c, rep, seed, idx, x, y, yaw) and preview_<scenario>.png.
"""

import argparse
import csv
import datetime
import hashlib
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from webots_spot_msgs.msg import SafetyField

from bench_common import StaticMap, compute_path

LATCHED = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)
SERIES_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]


def to_param_value(value):
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    if isinstance(value, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    if isinstance(value, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value))


def from_param_value(pv):
    return {
        ParameterType.PARAMETER_BOOL: lambda: pv.bool_value,
        ParameterType.PARAMETER_INTEGER: lambda: pv.integer_value,
        ParameterType.PARAMETER_DOUBLE: lambda: pv.double_value,
        ParameterType.PARAMETER_STRING: lambda: pv.string_value,
        ParameterType.PARAMETER_DOUBLE_ARRAY: lambda: list(pv.double_array_value),
        ParameterType.PARAMETER_INTEGER_ARRAY: lambda: list(pv.integer_array_value),
        ParameterType.PARAMETER_STRING_ARRAY: lambda: list(pv.string_array_value),
    }.get(pv.type, lambda: None)()


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def sha1(path):
    with open(path, "rb") as handle:
        return hashlib.sha1(handle.read()).hexdigest()


def load_scenarios(path):
    if not os.path.exists(path):
        return {"map": "map.yaml", "scenarios": []}
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    data.setdefault("map", "map.yaml")
    data.setdefault("scenarios", [])
    return data


class Capture(Node):
    def __init__(self, planner_node, planner_id):
        super().__init__("path_capture")
        self.planner_id = planner_id
        self.client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.set_cli = self.create_client(SetParameters, f"{planner_node}/set_parameters")
        self.get_cli = self.create_client(GetParameters, f"{planner_node}/get_parameters")
        self.list_cli = self.create_client(ListParameters, f"{planner_node}/list_parameters")
        self.field = None
        self.map_msg = None
        self.create_subscription(SafetyField, "/pinn_safety/field", self._on_field, LATCHED)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, LATCHED)

    def _on_field(self, msg):
        h = np.asarray(msg.h, dtype=np.float64)
        self.field = {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": float(msg.info.resolution),
            "origin": [msg.info.origin.position.x, msg.info.origin.position.y],
            "h_max": float(h.max()) if h.size else None,
        }

    def _on_map(self, msg):
        self.map_msg = msg

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"service {client.srv_name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_params(self, names):
        res = self._call(self.get_cli, GetParameters.Request(names=names))
        return {n: from_param_value(v) for n, v in zip(names, res.values)}

    def planner_params(self):
        res = self._call(
            self.list_cli, ListParameters.Request(prefixes=[self.planner_id], depth=0)
        )
        return self.get_params(sorted(res.result.names))

    def set_params(self, values):
        request = SetParameters.Request(
            parameters=[
                Parameter(name=f"{self.planner_id}.{k}", value=to_param_value(v))
                for k, v in values.items()
            ]
        )
        res = self._call(self.set_cli, request)
        failed = [r.reason for r in res.results if not r.successful]
        if failed:
            raise RuntimeError(f"set_parameters {values} rejected: {failed}")

    def wait_latched(self, timeout):
        t0 = time.time()
        while (self.field is None or self.map_msg is None) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)

    def check_map(self, static_map):
        """Warn if the map served on /map is not the scenarios' map."""
        m = self.map_msg
        if m is None:
            self.get_logger().warn("No /map received; cannot confirm the served map")
            return
        h, w = static_map.occ.shape
        same = (
            m.info.width == w
            and m.info.height == h
            and abs(m.info.resolution - static_map.res) < 1e-6
            and abs(m.info.origin.position.x - static_map.origin[0]) < 1e-3
            and abs(m.info.origin.position.y - static_map.origin[1]) < 1e-3
        )
        if not same:
            self.get_logger().warn(
                f"/map ({m.info.width}x{m.info.height} @ {m.info.resolution}) does not match "
                f"{static_map.yaml_path} ({w}x{h} @ {static_map.res})"
            )


# --- pick mode ----------------------------------------------------------------------------
def pick(args):
    data = load_scenarios(args.scenarios)
    static_map = StaticMap(data["map"])
    node = Node("path_capture_pick")
    state = {"start": None, "goal": None}

    def on_start(msg):
        p = msg.pose.pose
        state["start"] = [round(p.position.x, 3), round(p.position.y, 3),
                          round(yaw_from_quat(p.orientation), 3)]
        print(f"start = {state['start']}")

    def on_goal(msg):
        p = msg.pose
        state["goal"] = [round(p.position.x, 3), round(p.position.y, 3),
                         round(yaw_from_quat(p.orientation), 3)]
        print(f"goal  = {state['goal']}")

    node.create_subscription(PoseWithCovarianceStamped, "/initialpose", on_start, 10)
    node.create_subscription(PoseStamped, "/goal_pose", on_goal, 10)
    print("RViz: '2D Pose Estimate' sets the start, '2D Goal Pose' the goal. Ctrl+C to stop.")
    try:
        while True:
            while state["start"] is None or state["goal"] is None:
                rclpy.spin_once(node, timeout_sec=0.2)
            problems = validate(static_map, state["start"], state["goal"], args.min_clear)
            for p in problems:
                print(f"  invalid: {p}")
            name = input("scenario name (empty = discard, keep picking): ").strip()
            if name and not problems:
                names = {s["name"] for s in data["scenarios"]}
                if name in names:
                    print(f"  '{name}' already exists, discarded")
                else:
                    data["scenarios"].append(
                        {"name": name, "start": state["start"], "goal": state["goal"]}
                    )
                    with open(args.scenarios, "w") as handle:
                        yaml.safe_dump(data, handle, sort_keys=False, default_flow_style=None)
                    print(f"  saved '{name}' to {args.scenarios}")
            state["start"] = state["goal"] = None
    except (KeyboardInterrupt, EOFError):
        pass
    node.destroy_node()


def validate(static_map, start, goal, min_clear):
    problems = []
    for label, pose in (("start", start), ("goal", goal)):
        x, y = pose[0], pose[1]
        if not static_map.is_free(x, y):
            problems.append(f"{label} ({x:.2f}, {y:.2f}) is not on a free map cell")
        elif static_map.clearance_at(x, y) < min_clear:
            problems.append(
                f"{label} ({x:.2f}, {y:.2f}) clearance {static_map.clearance_at(x, y):.2f} m "
                f"< {min_clear:.2f} m"
            )
    return problems


# --- capture mode -------------------------------------------------------------------------
def preview(static_map, scenario, rows_by_c, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8 * static_map.occ.shape[0] / static_map.occ.shape[1] + 0.5))
    ax.imshow(
        np.where(static_map.occ, 0.25, np.where(static_map.free, 1.0, 0.8)),
        cmap="gray", vmin=0, vmax=1, origin="lower", extent=static_map.extent,
        interpolation="nearest",
    )
    for i, (c, paths) in enumerate(rows_by_c.items()):
        color = SERIES_COLORS[i % len(SERIES_COLORS)]
        for j, pts in enumerate(paths):
            ax.plot(pts[:, 0], pts[:, 1], color=color, linewidth=1.0, alpha=0.5,
                    label=f"c = {c:g} ({len(paths)} paths)" if j == 0 else None)
    for label, pose, marker, size in (("start", scenario["start"], "o", 10), ("goal", scenario["goal"], "*", 16)):
        ax.plot(pose[0], pose[1], marker, color="#222222", markersize=size,
                markeredgecolor="white", markeredgewidth=1.5)
        ax.annotate(label, (pose[0], pose[1]), xytext=(6, 6), textcoords="offset points",
                    color="#222222", fontsize=9)
    ax.set_title(scenario["name"], loc="left", fontsize=11)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="best", frameon=True, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def capture(args):
    data = load_scenarios(args.scenarios)
    if not data["scenarios"]:
        sys.exit(f"No scenarios in {args.scenarios} (use --pick to create some)")
    if args.only:
        wanted = set(args.only.split(","))
        data["scenarios"] = [s for s in data["scenarios"] if s["name"] in wanted]
    static_map = StaticMap(data["map"])
    c_values = [float(v) for v in args.c.split(",")]

    node = Capture(args.planner_node, args.planner_id)
    log = node.get_logger()
    log.info("Waiting for compute_path_to_pose action server")
    node.client.wait_for_server()
    node.wait_latched(args.field_timeout)
    if node.field is None:
        log.warn("No /pinn_safety/field seen yet (planner fails until it has one)")
    node.check_map(static_map)

    original = node.planner_params()
    keys = {k: f"{args.planner_id}.{k}" for k in ("c", "seed", "seed_mode")}
    if keys["c"] not in original:
        sys.exit(f"{args.planner_node} has no {keys['c']} parameter; is {args.planner_id} loaded?")
    min_clear = args.min_clear
    if min_clear is None:
        min_clear = float(original.get(f"{args.planner_id}.start_clear_radius_m") or 0.0)

    bad = False
    for s in data["scenarios"]:
        for problem in validate(static_map, s["start"], s["goal"], min_clear):
            log.error(f"{s['name']}: {problem}")
            bad = True
    if bad:
        sys.exit("Fix or remove the invalid scenarios (or lower --min-clear)")

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "scenarios.yaml"), "w") as handle:
        yaml.safe_dump(data, handle, sort_keys=False, default_flow_style=None)
    started = datetime.datetime.now().isoformat(timespec="seconds")

    summary_f = open(os.path.join(args.out, "summary.csv"), "w", newline="")
    paths_f = open(os.path.join(args.out, "paths.csv"), "w", newline="")
    summary = csv.writer(summary_f)
    paths = csv.writer(paths_f)
    summary.writerow(["scenario", "c", "rep", "seed", "success", "wall_s", "length_m", "n_poses"])
    paths.writerow(["scenario", "c", "rep", "seed", "idx", "x", "y", "yaw"])

    total = len(data["scenarios"]) * len(c_values) * args.repeats
    done = 0
    collected = {}
    try:
        node.set_params({"seed_mode": "fixed"})
        for s in data["scenarios"]:
            collected[s["name"]] = {c: [] for c in c_values}
            for c in c_values:
                for rep in range(args.repeats):
                    seed = args.seed0 + rep
                    node.set_params({"c": c, "seed": seed})
                    pts, wall = compute_path(node, node.client, args.planner_id, s["start"], s["goal"])
                    ok = pts is not None
                    length = float(np.sum(np.hypot(*np.diff(pts[:, :2], axis=0).T))) if ok else ""
                    summary.writerow([s["name"], c, rep, seed, int(ok), round(wall, 4),
                                      round(length, 4) if ok else "", len(pts) if ok else 0])
                    if ok:
                        for idx, (x, y, yaw) in enumerate(pts):
                            paths.writerow([s["name"], c, rep, seed, idx,
                                            round(x, 4), round(y, 4), round(yaw, 4)])
                        collected[s["name"]][c].append(pts)
                    summary_f.flush()
                    paths_f.flush()
                    done += 1
                    log.info(f"[{done}/{total}] {s['name']} c={c:g} rep={rep} seed={seed} "
                             f"{'ok' if ok else 'FAILED'} {length if ok else ''}")
    finally:
        summary_f.close()
        paths_f.close()
        restore = {k: original[full] for k, full in keys.items() if original.get(full) is not None}
        try:
            node.set_params(restore)
            log.info(f"Restored planner params {restore}")
        except Exception as exc:  # noqa: BLE001 - report, but keep the captured data
            log.error(f"Could not restore planner params: {exc}")

    meta = {
        "started": started,
        "finished": datetime.datetime.now().isoformat(timespec="seconds"),
        "map_yaml": static_map.yaml_path,
        "map_image": static_map.image_path,
        "map_image_sha1": sha1(static_map.image_path),
        "planner_node": args.planner_node,
        "planner_id": args.planner_id,
        "c_values": c_values,
        "repeats": args.repeats,
        "seed0": args.seed0,
        "min_clear": min_clear,
        "planner_params": {k: original[k] for k in sorted(original)},
        "safety_field": node.field,
    }
    with open(os.path.join(args.out, "meta.yaml"), "w") as handle:
        yaml.safe_dump(meta, handle, sort_keys=False)

    if not args.no_preview:
        for s in data["scenarios"]:
            preview(static_map, s, collected[s["name"]],
                    os.path.join(args.out, f"preview_{s['name']}.png"))

    print(f"\n{'scenario':16s} {'c':>5s} {'ok':>7s} {'len_mean':>9s} {'len_std':>8s}")
    for name, by_c in collected.items():
        for c, plist in by_c.items():
            lens = [float(np.sum(np.hypot(*np.diff(p[:, :2], axis=0).T))) for p in plist]
            mean = f"{np.mean(lens):9.2f}" if lens else f"{'-':>9s}"
            std = f"{np.std(lens):8.2f}" if lens else f"{'-':>8s}"
            print(f"{name:16s} {c:5g} {len(plist):>3d}/{args.repeats:<3d} {mean} {std}")
    print(f"Output: {os.path.abspath(args.out)}")
    node.destroy_node()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenarios", required=True, help="scenarios yaml (created by --pick)")
    ap.add_argument("--pick", action="store_true", help="record scenarios from RViz clicks")
    ap.add_argument("--c", default="0,1", help="comma separated c values")
    ap.add_argument("--repeats", type=int, default=10, help="plans per scenario and c")
    ap.add_argument("--seed0", type=int, default=0, help="rep i uses seed seed0 + i")
    ap.add_argument("--only", default="", help="comma separated scenario names to run")
    ap.add_argument("--out", default="", help="output dir (default captures/<timestamp>)")
    ap.add_argument("--min-clear", type=float, default=None,
                    help="min map clearance [m] of start/goal (default RrtCbf.start_clear_radius_m)")
    ap.add_argument("--planner-node", default="/planner_server")
    ap.add_argument("--planner-id", default="RrtCbf")
    ap.add_argument("--field-timeout", type=float, default=30.0,
                    help="s to wait for latched /pinn_safety/field and /map")
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args()
    if not args.out:
        args.out = os.path.join("captures", datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))

    rclpy.init()
    try:
        if args.pick:
            if args.min_clear is None:
                args.min_clear = 0.0
            pick(args)
        else:
            capture(args)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
