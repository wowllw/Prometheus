# ros_run_notes.md

**[File name]** `doc/ros_run_notes.md`
**[File purpose]** Operating notes for the ROS layer of `jps_vo_planner`:
- build;
- offline unit tests;
- the simulation start sequence;
- the closed-loop verification checklist;
- troubleshooting for deadlocks, timestamps, grid alignment, data races and B-spline format.

The experiment procedure itself (seeds, groups, invalid runs) is in `experiment_run_notes.md`.

---

## 1. Files of the ROS layer

| File | Purpose |
|---|---|
| `src/jps_vo_planner_node.cpp` | `main()`; documents the thread / lock / topic layout |
| `src/planner_node.cpp`, `include/jps_vo_planner/planner_node.h` | topics, goal and control-state handling, reference state, B-spline and gate publishing (Subtask-2 contract), diagnostics, ego-node conflict detection |
| `src/planner_node_coupled.cpp` | parameter → `CoreConfig` mapping, 10 Hz `localStep` (main thread), `globalStep` thread, CSV logs, runtime checks |
| `src/map_input.cpp` | octomap cloud → 0.1 m `OccupancyGrid` (double-buffered), `/gazebo/model_states` → obstacle states + trail masks |
| `launch/jps_vo_planner.launch` | the node only; every ablation switch can be overridden by an argument; `use_sim_time=false` |
| `launch/sitl_jps_vo_planner.launch` | `jps_vo_planner.launch` + the unmodified `traj_server_for_prometheus` (used by the start script) |
| `config/jps_vo_params.yaml` | all constants and engineering parameters (defaults = g6) |
| `config/groups/g1..g6.yaml` | the 6 ablation modes (switches only) |

The core (`coupling`, `jps3d`, `global_path`, `local_planner`, `planner_core`, `bspline_utils`, `occupancy_grid`) is
ROS-free and is built as the static library `jps_vo_core`.

## 2. Topics and threads

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/uav1/mavros/local_position/odom` | `nav_msgs/Odometry` | first plan, safety distances, logs only (the planner works on the reference) |
| in | `/uav1/octomap_point_cloud_centers` | `sensor_msgs/PointCloud2` | frame `world`, 0.05 m centres → 0.1 m grid, rebuilt ≤ 5 Hz |
| in | `/gazebo/model_states` | `gazebo_msgs/ModelStates` | names `dyn_obs_*`; no header, stamped on receipt |
| in | `/uav1/prometheus/motion_planning/goal` | `geometry_msgs/PoseStamped` | Prometheus RViz "3D Nav Goal", `send_goal.py` |
| in | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz default "2D Nav Goal"; z < 0.1 → current height |
| in | `/uav1/prometheus/control_state` | `prometheus_msgs/UAVControlState` | 2 = COMMAND_CONTROL |
| out | `/uav1/planning/bspline` | `traj_utils/Bspline` | order 3, uniform knots, start_time ≤ now |
| out | `/uav1/prometheus/command/ego_command_stop_pub` | `std_msgs/Bool` | true at 10 Hz while in COMMAND with an active trajectory |
| out | `/uav1/prometheus/command` | `prometheus_msgs/UAVCommand` | only with `pre_rotate_yaw:=true` |
| out | `/uav1/jps_vo/path` | `nav_msgs/Path` | RViz |

| Thread | Runs | Shared state |
|---|---|---|
| main (`ros::spin`, single-threaded) | all callbacks; the 10 Hz local cycle (`localStep`), 10 Hz gate, 20 Hz odom/GT log, 0.5 Hz diagnostics | writes the global snapshot under `snap_mtx_` |
| global | loop every Δt: copies the snapshot under `snap_mtx_`, calls `globalStep` without that lock, hands back the result under `snap_mtx_` | `PlannerCore` guards goal / path / κ / events with its own mutex; the G_J search runs on copies |

No code path holds `snap_mtx_` and the core mutex at the same time. The grid is shared as an immutable
`shared_ptr` snapshot.

## 3. Build

```bash
cd ~/Prometheus && source devel/setup.bash
catkin_make --source Modules/jps_vo_planning --build build/jps_vo_planning
rospack find jps_vo_planner
# optional install: catkin_make --source Modules/jps_vo_planning --build build/jps_vo_planning install
```

Dependencies: `roscpp std_msgs geometry_msgs nav_msgs sensor_msgs gazebo_msgs traj_utils prometheus_msgs`,
Eigen3, Threads. `traj_utils` and `prometheus_msgs` come from the Prometheus devel space. The ego baseline is
`~/Prometheus_baseline` (git worktree) and is not needed to build this package.

## 4. Offline unit tests (no ROS master)

```bash
cd ~/Prometheus
for t in test_occupancy_grid test_jps test_vo test_coupling; do ./devel/lib/jps_vo_planner/$t; done
# each prints [PASS]/[FAIL] lines and ends with "ALL CHECKS PASSED (0 failures)"
python3 Modules/jps_vo_planning/jps_vo_planner/scripts/check_scenarios.py \
  Modules/jps_vo_planning/jps_vo_planner/config/scenarios/scenario_{static_only,single_dynamic,multi_dynamic}.yaml --d_min 0.2
```

| Test | Rows |
|---|---|
| `test_occupancy_grid` | octomap/0.1 m alignment, ego index identity, inflation ball, trail masks, bounds |
| `test_jps` | ①-a (JPS == A* cost, random + planning_easy), ①-b, ①-c, both eq. (7) tie rules, direction-augmented search |
| `test_vo` | ②-a, ②-b, ②-c |
| `test_coupling` | ③-a (λ ≡ 0 → JPS), ③-b (λ ≡ 1 → classic VO), ③-c, ③-d (Prop. 1 vs enumerated G_J), ③-e (exact penalty, G_θ), ③-f, B-1…B-5 |

`roslaunch --dump-params jps_vo_planner jps_vo_planner.launch group:=g3` shows the resolved parameters without
starting anything.

## 5. Simulation start sequence

Order matters: simulation first, then arming, then obstacles, then the goal.

```bash
# 0. clean (own shell: kill_sim.sh pkills 'rostopic echo' and would kill a shell that contains it)
bash ~/Prometheus/Scripts/experiment/kill_sim.sh

# 1. simulation + planner (5 tabs; planner tab = sitl_jps_vo_planner.launch)
LOG_DIR=~/.ros/runs/<name> GROUP=g6 PRE_ROTATE=false \
  ~/Prometheus/Scripts/simulation/ego_planner/new_test_jps_vo_p450_d435i.sh

# 2. wait until
#    planner tab: "[jps_vo] coupled mode, group g6: ... tau1 2.214 tau2 12.043 ..."
#    rostopic echo -n1 /uav1/prometheus/state  -> connected True, odom_valid True
#    rostopic echo -n1 /uav1/mavros/home_position/home  -> one message (GPS home set)

# 3. arm + COMMAND_CONTROL (takeoff to about 1.5 m, heading forced to yaw 0 by uav_control)
~/Prometheus/Scripts/simulation/NO_RC/arm_and_command.sh

# 4. dynamic obstacles (skip for static_only)
rosrun jps_vo_planner dyn_obstacles.py _scenario:=$(rospack find jps_vo_planner)/config/scenarios/<file>.yaml \
       _variant:=<variant> _seed:=<k> _log_dir:=$LOG_DIR &

# 5. goal (or RViz 3D Nav Goal / 2D Nav Goal)
rosrun jps_vo_planner send_goal.py _scenario:=<same file> _variant:=<variant>

# 6. after "[jps_vo] goal reached" (checks.yaml written) or the scenario timeout
rosrun jps_vo_planner verify_run.py $LOG_DIR --timeout <scenario timeout>
```

Single-run overrides (not for the ablation tables) go directly to the launch file, e.g.
`roslaunch jps_vo_planner sitl_jps_vo_planner.launch group:=g6`, or
`roslaunch jps_vo_planner jps_vo_planner.launch group:=g6 tie_rule:=literal label:=g6_literal`.

## 6. Closed-loop verification checklist (one run)

| # | Check | How | Pass |
|---|---|---|---|
| V-1 | one planner only | `rosnode list` | no `uav1_ego_planner_node`; diag line `conflict=0` |
| V-2 | parameters as intended | first planner line, `planner_meta.yaml` | group, d_min 0.2, τ₁ 2.214, τ₂ 12.043, tie_rule, pre_rotate |
| V-3 | map input | diag line | cloud frame `world`, pts > 0, age < 0.5 s |
| V-4 | obstacle input | `rostopic echo -n1 /gazebo/model_states/name` | contains `dyn_obs_*`; `checks.yaml: stale_obstacle_drops 0` |
| V-5 | single B-spline publisher | `rostopic info /uav1/planning/bspline` | publisher `/uav1_jps_vo_planner_node` only |
| V-6 | gate | `rostopic echo /uav1/prometheus/command/ego_command_stop_pub` | `true` at 10 Hz during flight |
| V-7 | timestamps | `checks.yaml` | `continuity_violations 0`, `max_start_jump_m < 0.01`; traj_server never prints `invalid time` |
| V-8 | format | `checks.yaml` | `format_violations 0` |
| V-9 | timing | `checks.yaml` | `cycles_over_dt 0`, `max_cycle_gap_s` ≈ 0.1; record `max_global_ms` as measured T_g |
| V-10 | flight | `/uav1/prometheus/state` | OFFBOARD the whole run, no `Failsafe` in `uav_control.log` |
| V-11 | outcome rows | `verify_run.py` | all PASS rows of the run's scenario type |

## 7. Troubleshooting

### Thread deadlock / freeze
- **Symptom:** the local cycle stops (no new `cycles.csv` rows, the gate stops), and the node does not exit on
  Ctrl-C.
- **Checks:**
  - `max_cycle_gap_s` in `checks.yaml`;
  - `gdb -p $(pgrep jps_vo_planner) -batch -ex "thread apply all bt"` to see both thread stacks.
- **Notes:**
  - by design no path holds two mutexes;
  - shutdown waits for a running G_J search to finish (up to the worst global step, ≈ 2.6 s measured);
  - a freeze longer than that is a bug: report the backtrace.

### Timestamp errors
- **The vehicle dives towards (0, 0, 0):** traj_server received a `start_time` in the future. The node clamps
  `start_time ≤ now` at publish. Check that no other node publishes on `/uav1/planning/bspline` (V-5) and that
  `/use_sim_time` is false (the node warns at startup if it is not).
- **Obstacles ignored:** `obstacle '…' state is … s old`. Gazebo or `dyn_obstacles.py` stalled. That run is
  invalid (I-4).
- **CSV times all equal / 10 s steps:** an old binary from before the fix. Rebuild (`t` is written with 6
  decimals).

### Grid alignment
- **Map origin:** grid origin (−25, −25, 0.5) and δ = 0.1 must match ego's `map_origin` / `ground_height`
  (`test_occupancy_grid` row 1–2). If the map box changes, change `map/*` in `jps_vo_params.yaml` and in the
  ego launch alike.
- **Cloud frame:** must be `world` (diag line, `checks.yaml: cloud_frame`); `rosrun tf tf_echo world map` should
  be identity.
- **Phantom walls behind moving spheres:** octomap trails not masked. Raise `dyn/mask_history` or
  `dyn/mask_margin` (engineering parameters; state any change in T1).
- **Path through a cylinder at the start:** the camera has not seen it (unknown = free). See the takeoff-heading
  note N-13 in `experiment_run_notes.md` and `pre_rotate_yaw`.
- **Reference stuck near an obstacle with `fallback_active=1`:** the reference is inside the inflation. [ENG-10]
  now escapes towards the nearest free cell; if it persists, check `r_infl` (0.6 m) against the corridor width.

### Data races
- All ROS callbacks run on the single-threaded spinner. Do **not** switch to `ros::AsyncSpinner` / a
  MultiThreadedSpinner: `MapInput`, the reference state and the CSV writers assume serialised callbacks.
- The global thread only touches `PlannerCore` (internally locked) and its snapshot copy. Never pass references
  to node members into it.
- A suspected race can be checked by building with `-DCMAKE_CXX_FLAGS="-fsanitize=thread -g"` in a separate
  build folder.

### B-spline format errors
- **`format_violations > 0`:** knots ≠ control points + 4 (B-1), last three control points not equal (B-4), or
  a control-point speed above v_max (B-5). Every published trajectory comes from `buildVelocityBridge` or
  `buildStraightLine`; both are checked offline by `test_coupling`.
- **traj_server prints nothing / the drone ignores goals:** the gate is false. Check that COMMAND_CONTROL is
  active (`control_state: 2`), there is no node conflict, and pre-rotation is not still running (planner log
  `pre-rotation done`).
- **Jumps at re-planning:** `continuity_violations`. The node always starts from the previous reference; a
  violation means the reference state was lost, e.g. a COMMAND re-entry. See the planner log at that time.
