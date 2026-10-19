"""Simulator-free Nav2 planning stack: map_server + planner_server (+ global costmap)
+ pinn_safety, with static TF. Used to benchmark RrtCbf against NavFn on the Spot map.

  ros2 launch webots_spot_rrt_cbf planner_bench_launch.py
  ros2 run webots_spot_rrt_cbf planner_benchmark.py --out /tmp/bench.csv
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    spot_dir = get_package_share_directory("webots_spot")
    map_yaml = os.path.join(spot_dir, "map", "map.yaml")
    params_path = os.path.join(spot_dir, "params", "nav2_params.yaml")

    with open(params_path, "r") as handle:
        params = yaml.safe_load(handle)
    # No simulator clock here.
    for node_params in params.values():
        if isinstance(node_params, dict) and "ros__parameters" in node_params:
            node_params["ros__parameters"]["use_sim_time"] = False
    params["global_costmap"]["global_costmap"]["ros__parameters"]["use_sim_time"] = False
    params["map_server"]["ros__parameters"]["yaml_filename"] = map_yaml

    overrides = params["planner_server"]["ros__parameters"]["RrtCbf"]
    csv_path = LaunchConfiguration("stats_csv").perform(context)
    if csv_path:
        overrides["stats_csv_path"] = csv_path
    for item in LaunchConfiguration("rrt_overrides").perform(context).split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            overrides[key.strip()] = yaml.safe_load(value)

    out = tempfile.NamedTemporaryFile(
        mode="w", prefix="planner_bench_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(params, out)
    out.close()

    def tf(parent, child):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", parent, child],
        )

    return [
        tf("map", "odom"),
        tf("odom", "base_link"),
        Node(
            package="nav2_map_server",
            executable="map_server",
            output="screen",
            parameters=[out.name],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            output="screen",
            parameters=[out.name],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_bench",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": ["map_server", "planner_server"],
                }
            ],
        ),
        Node(
            package="webots_spot",
            executable="pinn_safety",
            name="pinn_safety",
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("stats_csv", default_value="", description="RrtCbf stats CSV"),
            DeclareLaunchArgument(
                "rrt_overrides",
                default_value="",
                description="Comma separated RrtCbf param overrides, e.g. kappa=1.0,c=0.5",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
