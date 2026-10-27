# experiment_run_notes.md

**[File name]** `doc/experiment_run_notes.md`
**[File purpose]** Operating rules for the ablation runs: precautions, the per-run procedure, failure criteria
(valid run, planner failed), invalid-run conditions (infrastructure fault, rerun) and data preprocessing rules.
Metric definitions are in `subtask4_ablation_experiment_spec.md` §4.

---

## 1. Precautions (N-x)

- **N-1 Order.** Start the simulation script first, wait until it is ready (§2 step 3), then run
  `arm_and_command.sh`. The reverse order loses the setup messages and arming fails silently.
- **N-2 One planner only.** `rosnode list` must not show `uav1_ego_planner_node` during jps_vo runs, and
  `rostopic info /uav1/planning/bspline` must show exactly one publisher.
- **N-3 Never change the spawn point** (`uav1_init_x/y = 0`). With `location_source: 4` (GPS), the odom frame
  origin is the spawn point. The scenarios, `planning_easy_cylinders.yaml` and `/gazebo/model_states` are in
  the Gazebo world frame, and the two coincide only for spawn = origin.
- **N-4 No manual input during a run.** Any RViz "3D Nav Goal" click is published to the planner. Do not
  touch RViz, the RC or the joystick while a run is active.
- **N-5 Full restart per run.** Run `Scripts/experiment/kill_sim.sh`, then relaunch. Octomap, PX4/EKF state
  and the planner's memory must not carry over between runs.
- **N-6 Identical software for all groups.** One binary and one constants file (T1). Group differences are
  launch parameters only (spec §3). Record the git commit in `run_meta.yaml`.
- **N-7 Ground-truth masking statement** (thesis experimental setup, user decision 2). This method removes
  dynamic obstacles and their octomap trails from its static grid using `/gazebo/model_states`. The
  ego-planner reference, if run, sees them only through D435i/octomap.
- **N-8 Reference vs odom** (user decision 4).
  - The planner works on the B-spline reference (p_ref, v_ref).
  - Odom is used for the initial snap, logging and safety monitoring.
  - All metrics except safety distances use the reference.
  - PX4 trails the reference by about 0.8 s, so odom positions lag.
- **N-9 Timeouts.** These come from the scenario files (70 s, or 80 s for multi): about 3 × the longest
  nominal route time plus a margin, the same for every group.
- **N-10 Static-safety interpretation [ENG-6].** The static-obstacle check is always active; λ only switches
  the dynamic part of (C-s). State this in the thesis.
- **N-11 R3/R5 are recorded, not fixed.** VO side switching and corridor flipping are logged as diagnostics
  and written into the limitations. Never respond with tuning or model changes.
- **N-12 Machine load.** No other heavy processes during a campaign. Record the mean Gazebo real-time factor
  in T0.
- **N-13 Takeoff heading.** Prometheus `Init_Pos_Hover` always commands yaw 0 (`uav_controller.cpp:483`), so
  after `arm_and_command.sh` the drone faces +x whatever the spawn yaw.
  - On north-bound routes (S2, single_dynamic, multi_dynamic) the forward D435i is 90° off the route at t₀.
    Unknown space is free, so the first plan can run through unseen cylinders (Subtask-3 smoke run 1: collision).
  - `PRE_ROTATE=true` turns towards the goal before planning, as ego_planner does. The default `false` follows
    decision 1. **Pending decision before the experiments.**
  - Whatever is chosen must be identical for all 6 groups and stated in the thesis.
- **N-14 Global step timing.** With [ENG-11] the global step runs asynchronously. Its time appears in
  `plan_ms_global` of the cycle that receives the result, so M1 can exceed Δt while no local cycle is delayed.
  `checks.yaml: cycles_over_dt` counts local overruns. Record the measured worst global step as T_g in T1.

## 2. Per-run procedure

```bash
# variables for one run
GROUP=g6; SCEN=scenario_single_dynamic; VAR=head_on; SEED=7
LOG_DIR=$HOME/.ros/ablation/${SCEN}/${VAR}/${GROUP}/seed${SEED}_attempt1

# 1. clean start
bash ~/Prometheus/Scripts/experiment/kill_sim.sh

# 2. simulation + planner with the group's switches (config/groups/${GROUP}.yaml, spec §3)
LOG_DIR=$LOG_DIR GROUP=$GROUP PRE_ROTATE=false ./Scripts/simulation/ego_planner/new_test_jps_vo_p450_d435i.sh

# 3. ready check (all must hold):
#    /uav1/prometheus/state: connected True, odom_valid True; /uav1/mavros/home_position/home received;
#    planner diag shows cloud frame 'world' with pts > 0

# 4. arm + COMMAND, then wait until z_odom is in [1.35, 1.65] and |v| < 0.1 for 5 s
./Scripts/simulation/NO_RC/arm_and_command.sh

# 5. obstacles (skip for static_only, or run it to log an empty seed file)
rosrun jps_vo_planner dyn_obstacles.py _scenario:=$(rospack find jps_vo_planner)/config/scenarios/${SCEN}.yaml \
       _variant:=${VAR} _seed:=${SEED} _log_dir:=${LOG_DIR} &

# 6. goal (= RViz 3D Nav Goal topic); this is t0
rosrun jps_vo_planner send_goal.py _scenario:=$(rospack find jps_vo_planner)/config/scenarios/${SCEN}.yaml _variant:=${VAR}

# 7. end of run: goal reached (the node writes checks.yaml), failure (F-x) or timeout
# 8. evaluation (verification rows + M1..M6 -> $LOG_DIR/metrics.yaml)
rosrun jps_vo_planner verify_run.py $LOG_DIR --timeout <scenario timeout>
# 9. bash ~/Prometheus/Scripts/experiment/kill_sim.sh   (in its own shell: it pkills 'rostopic echo')
```

Order of runs: interleave the groups per seed (seed k: ①②③④⑤⑥, then seed k+1). Slow drift of the machine or
simulator then affects all groups equally. Do not run all 20 seeds of one group in a block.

## 3. Outcome rules

### Success (S-1)
- The goal is reached: ‖p_ref − goal‖ ≤ 0.3 m and ‖v_ref‖ ≤ 0.1 m/s, before the timeout.
- No failure condition occurred.

### Failure criteria: the run is VALID and counts as unsuccessful (F-x)
- **F-1 Collision.**
  - Static: the minimum surface distance between the UAV sphere (odom, r_q) and any cylinder in
    `planning_easy_cylinders.yaml` is ≤ 0.
  - Dynamic: the minimum surface distance (odom vs ground truth, r_q + r_o) is ≤ 0.
  - Gazebo contact followed by an `Odom invalid` failsafe is also F-1.
- **F-2 Timeout:** the goal is not reached within the scenario timeout.
- **F-3 Stall:** the reference has moved less than 0.2 m in 15 s while not at the goal (local minimum, an
  empty F/fallback loop, or JPS finding no path).
- **F-4 Planner fault:** the planner node crashes, publishes NaN or an invalid B-spline, or stops publishing
  for more than 1 s while in COMMAND.
- **F-5 Left the flight volume:** p_ref is outside the map box (x, y outside ±25 m, or z outside [0.5, 3.0]).

A violation of d_min that does not reach contact (0 < surface distance < d_min) is **not** a failure. It is
logged in the min-surface-distance columns.

### Invalid-run conditions: infrastructure fault, rerun with the same seed (I-x)
- **I-1** Gazebo real-time factor averaged over the run is < 0.9.
- **I-2** PX4/EKF/GPS fault without contact: an `Odom invalid` failsafe, or an odom jump > 1 m between
  consecutive samples, with the minimum surface distance to every obstacle > 0.
- **I-3** Arming or takeoff failed, or the hover was not in [1.35, 1.65] m before t₀.
- **I-4** Obstacle state stale: the planner logged `obstacle '…' state is … s old` (age > 0.5 s), or
  `dyn_obstacles.py` crashed.
- **I-5** A trigger never fired **and** the UAV reference did pass through the trigger region (a script or
  timing fault). If the planner's own route avoided the trigger region, the run is valid; record it in Notes.
- **I-6** Manual interference (N-4), or another goal publisher appeared on the goal topic.
- **I-7** A process crash outside the planner: Gazebo, PX4, mavros, uav_control, octomap or traj_server.
- **I-8** Log files are missing or truncated (`cycles.csv` does not cover [t₀, t_end]).
- **I-9** The octomap cloud is stale (age > 1 s) for more than 2 s during the run.

Rerun policy:
- Up to 3 attempts per (group, variant, seed). If all 3 are invalid, mark the cell "excluded" in T6.
- Never rerun a **valid** run, whether it succeeded or failed.

### Outliers
There is no removal based on metric values. Values beyond Q3 + 3·IQR are flagged in T2 and discussed, and the
median is reported next to the mean (spec §6.3).

## 4. Data preprocessing rules (P-x)

- **P-1 Clock.** All stamps are wall clock (`use_sim_time=false`). `/gazebo/model_states` has no header, so
  ground-truth samples are stamped at receipt.
- **P-2 Window.** t₀ = goal publish time. t_end = first sample that satisfies S-1, the time of failure, or
  t₀ + timeout. Only cycles in [t₀, t_end] are used.
- **P-3 M1.**
  - Use `plan_ms_local + plan_ms_global` for every cycle in the window.
  - The first global search after t₀ is included. Report it separately in Notes if it exceeds the P95.
- **P-4 M2.** Sum ‖Δp_ref‖ over consecutive cycles. Drop jumps larger than v_max·Δt·1.5, which indicate a
  logging gap; if one occurs, the run is I-8.
- **P-5 M3.** Resample v_ref to 10 Hz. Compute the angle between v_ref(t) and v_ref(t − 1 s) where both speeds
  are ≥ v_min = 0.1 m/s, and take the maximum.
- **P-6 Safety distances (M4, F-1).**
  - Interpolate the obstacle ground truth to the odom timestamps (20 Hz).
  - Dynamic surface distance = ‖p_odom − p_o‖ − r_q − r_o.
  - Static surface distance = horizontal distance to the cylinder axis − r_cyl − r_q, evaluated only when the
    UAV's z range overlaps the cylinder's [z_min, z_max] (from `planning_easy_cylinders.yaml`).
- **P-7 M5.** Lateral distance from p_ref to the polyline Π₀ (`pi0.csv`); report the mean and max over the
  window.
- **P-8 M6.**
  - Read `paths.csv`.
  - Ignore changes with `replan_reason = goal`.
  - A flip-back is a change to a path whose Hausdorff distance to some path active within the previous 10 s
    (other than the current one) is < 0.3 m.
  - Count the flip-backs.
- **P-9 Aggregation.** Per (group, variant): mean ± std and median [IQR] over valid runs; success rate with
  its Wilson 95 % CI.
- **P-10 Paired tests.** Pair runs by seed. If one side of a pair is excluded (T6), drop the pair from that
  test and report the number of pairs.
- **P-11 Traceability.** Keep the raw run folders, and keep the preprocessing script under version control
  with its commit in T0. Every number in the thesis must be reproducible from the raw files.

## 5. Folder and file naming

```
~/.ros/ablation/<scenario>/<variant>/<group g1..g6>/seed<k>_attempt<a>/
    run_meta.yaml  cycles.csv  pi0.csv  paths.csv  odom_gt.csv  dyn_obstacles_seed<k>.yaml
    planner.log  uav_control.log  gazebo.log  octomap.log  [run.bag]
```
