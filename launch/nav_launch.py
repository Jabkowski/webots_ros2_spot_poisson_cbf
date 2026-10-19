import os
import tempfile

import launch
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _build_nav2_params(package_dir, planner):
    """Return the nav2 params file path, selecting the global planner.

    planner:=navfn     -> params file unchanged (BT uses planner_id GridBased).
    planner:=rrt_cbf   -> bt_navigator uses a BT with planner_id RrtCbf.
    """
    params_path = os.path.join(package_dir, "params", "nav2_params.yaml")
    if planner != "rrt_cbf":
        return params_path

    with open(params_path, "r") as handle:
        params = yaml.safe_load(handle)
    bt_xml = os.path.join(package_dir, "behavior_trees", "navigate_to_pose_rrt_cbf.xml")
    params["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = bt_xml

    out = tempfile.NamedTemporaryFile(
        mode="w", prefix="nav2_params_rrt_cbf_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(params, out)
    out.close()
    return out.name


def _launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory("webots_spot")
    planner = LaunchConfiguration("planner").perform(context)
    if planner not in ("navfn", "rrt_cbf"):
        raise RuntimeError(f"planner must be 'navfn' or 'rrt_cbf', got '{planner}'")

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    nav2_map = os.path.join(package_dir, "map", "map.yaml")
    nav2_params = _build_nav2_params(package_dir, planner)

    use_rviz = LaunchConfiguration("rviz", default=True)
    rviz_config = os.path.join(package_dir, "resource", "nav2.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["--display-config=" + rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz),
    )

    initial_pose = LaunchConfiguration("set_initial_pose", default=False)
    spot_initial_pose = Node(
        package="webots_spot",
        executable="set_initial_pose",
        output="screen",
        condition=launch.conditions.IfCondition(initial_pose),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments=[
            ("map", nav2_map),
            ("use_sim_time", use_sim_time),
            ("params_file", nav2_params),
        ],
    )

    actions = [rviz, spot_initial_pose, nav2]

    # PINN safety field: inference runs once after the first /map (auto_run_once)
    # and the result is latched on /pinn_safety/field for the whole run.
    run_pinn = LaunchConfiguration("pinn_safety").perform(context).lower()
    if run_pinn == "true" or (run_pinn == "auto" and planner == "rrt_cbf"):
        actions.append(
            Node(
                package="webots_spot",
                executable="pinn_safety",
                name="pinn_safety",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "planner",
                default_value="navfn",
                description="Global planner: navfn (default) or rrt_cbf",
            ),
            DeclareLaunchArgument(
                "pinn_safety",
                default_value="auto",
                description="Start pinn_safety node: true, false or auto (only with planner:=rrt_cbf)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
