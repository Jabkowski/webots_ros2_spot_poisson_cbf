# webots_spot_rrt_cbf

Nav2 `nav2_core::GlobalPlanner` plugin (`webots_spot_rrt_cbf/RrtCbfPlanner`) that runs Safe CBF-RRT*
on the global costmap, using the PINN Poisson safety field `h` from `webots_spot/pinn_safety`.

## Data flow

- `pinn_safety` runs the PINN once (`auto_run_once`, after the first `/map`) and publishes raw `h`,
  `dhdx`, `dhdy` (per metre, map grid) on the latched topic `/pinn_safety/field`.
- The plugin caches the first field (`lock_field: true`) and never triggers inference. Reset with
  `ros2 service call /planner_server/RrtCbf/reset_field std_srvs/srv/Trigger`.
- Obstacles come from the global costmap (cost >= 253). `h` only drives the CBF edge check
  `v * dh/ds + kappa * h > 0` and the safety part of the edge cost. `h` is normalised by `max(h)`
  (or `h_scale`), so near an obstacle roughly `v / kappa` metres of clearance are kept.

## Vendored core

`include/safe_cbf_rrt`, `src/safe_cbf_rrt` come from the external `rrt_star_cbf/cpp_version` sources
(checksums of the origin files in `VENDORED_SOURCES.md5`). Local changes: `Params::maxTimeMs`,
`Params::recordTree` / `Result::treeEdges`, `Result::timedOut`, and `EdgeChecker` is public so
shortcutting reuses the planner's own validity rules. The Poisson solver and pybind module are not used.

## Use

```bash
ros2 launch webots_spot nav_launch.py planner:=rrt_cbf      # planner:=navfn is the default
```

Without the simulator (map server + planner server + PINN), compare planners:

```bash
ros2 launch webots_spot_rrt_cbf planner_bench_launch.py rrt_overrides:="kappa=2.0,c=0.2"
ros2 run webots_spot_rrt_cbf planner_benchmark.py --pairs 30 --out /tmp/bench.csv
```

### Runtime parameters

`c`, `seed`, `seed_mode`, `kappa`, `v`, `goal_bias`, `max_iter` and `max_planning_time` can be changed
without restarting the planner, e.g. `ros2 param set /planner_server RrtCbf.c 1.0`. The rest are read
at configure.

### Capturing repeated paths (c = 0 vs c = 1)

`path_capture.py` calls `compute_path_to_pose` directly, so there is one plan per request: no BT
replanning and no robot motion. Before each request it sets `RrtCbf.c` and `RrtCbf.seed = seed0 + rep`
(`seed_mode: fixed`), so repeats differ and are reproducible. Original values are restored at the end.

```bash
ros2 launch webots_spot_rrt_cbf planner_bench_launch.py      # + rviz2 with a Map display to pick
# record named start/goal pairs: RViz "2D Pose Estimate" = start, "2D Goal Pose" = goal
ros2 run webots_spot_rrt_cbf path_capture.py --pick --scenarios scenarios.yaml
# capture
ros2 run webots_spot_rrt_cbf path_capture.py --scenarios scenarios.yaml \
    --c 0,1 --repeats 30 --seed0 0 --out captures/run_001
```

`scenarios.yaml`:

```yaml
map: map.yaml            # bare name = webots_spot/map/<name>, or an absolute path
scenarios:
  - name: corridor_A
    start: [0.0, 0.0, 0.0]   # x, y, yaw (map frame)
    goal:  [6.5, -3.0, 0.0]
```

Output directory: `meta.yaml` (map path + sha1, full `RrtCbf.*` parameter snapshot, safety field info,
seeds), `scenarios.yaml`, `summary.csv` (one row per plan), `paths.csv` (long format
`scenario,c,rep,seed,idx,x,y,yaw`) and `preview_<scenario>.png`. Start and goal must be on free map
cells with clearance >= `--min-clear` (default `RrtCbf.start_clear_radius_m`).

Unit tests: `colcon test --packages-select webots_spot_rrt_cbf` (or run `build/webots_spot_rrt_cbf/test_safe_cbf_rrt`).
