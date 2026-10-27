# closed_loop_sim_guide.md

**[File name]** `doc/closed_loop_sim_guide.md`
**[File purpose]** Step-by-step guide for running the Gazebo closed-loop simulation of `jps_vo_planner` locally:
- the configuration to confirm before starting;
- the recommended run order;
- the exact commands per run;
- the V-1 … V-11 checklist (from `ros_run_notes.md`) with where to read each item;
- what to watch live and when to abort;
- which log files and fields to collect and send back.

This guide is for functional closed-loop verification, not the ablation campaign (Subtask 4).

---

## 0. Confirmed configuration (2026-10-26)

| Item | Value | Where it shows up |
|---|---|---|
| v_max / v_c / a_max | 1.0 m/s / 0.8 m/s / 0.8 m/s² | first planner line `v_max 1.00 a_max 0.80`; `planner_meta.yaml: constants` |
| d_min → r_infl, τ₁, τ₂ | 0.2 m → 0.60 m, 2.214 s, 12.043 s | first planner line |
| eq. (7) tie rule | `strict` | `tie_rule strict` |
| yaw pre-rotation | off (`pre_rotate_yaw false`) | `planner_meta.yaml: engineering` |
| gate | `true` at 10 Hz only in COMMAND with an active trajectory; nothing before takeoff | V-6 |
| group | `g6` (full C1/C2/C3) for this guide | `group g6` |

## 1. Preconditions (once)

```bash
cd ~/Prometheus && source devel/setup.bash
catkin_make --source Modules/jps_vo_planning --build build/jps_vo_planning
for t in test_occupancy_grid test_jps test_vo test_coupling; do ./devel/lib/jps_vo_planner/$t | tail -1; done
#   -> 4 x "ALL CHECKS PASSED (0 failures)"
python3 Modules/jps_vo_planning/jps_vo_planner/scripts/check_scenarios.py \
  Modules/jps_vo_planning/jps_vo_planner/config/scenarios/scenario_{static_only,single_dynamic,multi_dynamic}.yaml --d_min 0.2
#   -> every variant "OK", exit 0
```

- Close RViz goal tools before starting. A click during a run sends a goal (invalid run I-6).
- Make sure no other heavy load is running (Gazebo real-time factor ≥ 0.9).

## 2. Recommended run order

Start with the run where the takeoff heading matches the route, then increase difficulty. **Stop at the first
run that fails a V-item or aborts (§5) and send me its logs before continuing.**

| Run | Scenario file / variant | Why this order | Heading at t₀ vs route |
|---|---|---|---|
| R1 | `scenario_static_only` / `S1_east` | static planning, camera looks along the route | aligned (east) |
| R2 | `scenario_static_only` / `S3_climb_ne` | 3D goal (z 2.2 m) | ~22° off |
| R3 | `scenario_static_only` / `S2_north` | first route that starts 90° off the camera | **90° off**: see risk 1 |
| R4 | `scenario_single_dynamic` / `head_on` | one dynamic obstacle, C2/C3 active | 90° off |
| R5 | `scenario_single_dynamic` / `crossing_90` | crossing encounter | 90° off |
| R6 | `scenario_multi_dynamic` / `M1_three_way` | three obstacles, corridor patrol | 90° off |

Use seed `1` for every run in this guide. Timeouts: 70 s (R1–R5), 80 s (R6).

## 3. Per-run procedure (copy for each run)

Each numbered block goes in its **own terminal**. `kill_sim.sh` kills `rostopic echo`, so never put it in the
same shell line as other commands.

```bash
# ---- variables (terminal A) ----
SCEN=scenario_static_only; VAR=S1_east; SEED=1; TMO=70; GROUP=g6
export LOG_DIR=$HOME/.ros/closed_loop/${SCEN}_${VAR}_${GROUP}_seed${SEED}_$(date +%Y%m%d_%H%M%S)
SC=$(rospack find jps_vo_planner)/config/scenarios/${SCEN}.yaml

# ---- 1. clean start (terminal A, alone) ----
bash ~/Prometheus/Scripts/experiment/kill_sim.sh

# ---- 2. simulation + planner (terminal A) ----
LOG_DIR=$LOG_DIR GROUP=$GROUP PRE_ROTATE=false RECORD=1 \
  ~/Prometheus/Scripts/simulation/ego_planner/new_test_jps_vo_p450_d435i.sh

# ---- 3. readiness (terminal B, same variables exported) ----
grep -a "coupled mode" $LOG_DIR/planner.log                   # V-2: check the values of section 0
rostopic echo -n1 /uav1/prometheus/state | grep -E "connected|odom_valid"     # True / True
rostopic echo -n1 /uav1/mavros/home_position/home > /dev/null && echo "home set"
rosnode list                                                   # V-1

# ---- 4. arm + takeoff (terminal B) ----
~/Prometheus/Scripts/simulation/NO_RC/arm_and_command.sh
#   wait until the hover is stable (about 15 s): mode OFFBOARD, control_state 2, z_odom 1.35..1.65
rostopic echo -n1 /uav1/prometheus/state | grep -E "armed|mode|position"

# ---- 5. dynamic obstacles (terminal C; skip for static_only) ----
rosrun jps_vo_planner dyn_obstacles.py _scenario:=$SC _variant:=$VAR _seed:=$SEED _log_dir:=$LOG_DIR

# ---- 6. goal = t0 (terminal B) ----
rosrun jps_vo_planner send_goal.py _scenario:=$SC _variant:=$VAR

# ---- 7. wait for "[jps_vo] goal reached" in the planner tab, an abort condition (section 5) or TMO seconds;
#         then stop the obstacle script (Ctrl-C in terminal C) and the rosbag tab (Ctrl-C) ----

# ---- 8. evaluation (terminal B) ----
rosrun jps_vo_planner verify_run.py $LOG_DIR --timeout $TMO

# ---- 9. pack the run (section 6) and shut down (terminal A, alone) ----
bash ~/Prometheus/Scripts/experiment/kill_sim.sh
```

## 4. Verification checklist V-1 … V-11 (fill one per run)

Run: ____ Scenario / variant: ____ Seed: ____ LOG_DIR: ____

| # | Check | Command / file | Pass criterion | Result |
|---|---|---|---|---|
| V-1 | one planner only | `rosnode list`; planner diag line | no `uav1_ego_planner_node`; `conflict=0` | |
| V-2 | parameters | first `[jps_vo] coupled mode` line, `planner_meta.yaml` | group g6, v_max 1.00, a_max 0.80, d_min 0.20, r_infl 0.60, tau1 2.214, tau2 12.043, tie_rule strict, pre_rotate_yaw 0 | |
| V-3 | map input | diag line every 2 s | cloud frame `world`, `pts > 0`, cloud age < 0.5 s | |
| V-4 | obstacle input | `rostopic echo -n1 /gazebo/model_states/name`; `checks.yaml` | `dyn_obs_*` present (dynamic runs); `stale_obstacle_drops: 0` | |
| V-5 | single B-spline publisher | `rostopic info /uav1/planning/bspline` | publisher `/uav1_jps_vo_planner_node` only | |
| V-6 | gate | `rostopic echo /uav1/prometheus/command/ego_command_stop_pub` | nothing before the goal; `data: True` at ~10 Hz during flight (`rostopic hz` ≈ 10) | |
| V-7 | timestamps / continuity | `checks.yaml`; traj_server output | `continuity_violations: 0`, `max_start_jump_m < 0.01`; no `invalid time` line | |
| V-8 | B-spline format | `checks.yaml` | `format_violations: 0` | |
| V-9 | timing | `checks.yaml` | `cycles_over_dt: 0`, `max_cycle_gap_s` ≤ 0.15; note `max_global_ms` (= measured T_g) | |
| V-10 | flight | `/uav1/prometheus/state`; `uav_control.log` | OFFBOARD for the whole run; no `Failsafe` / `Odom invalid` | |
| V-11 | outcome | `verify_run.py` | all rows PASS for the run type (static: 1-d, 1-e, 1-f, B, 2-g/3-i, 2-d, 3-f, 3-g; dynamic adds 2-e, 3-h as INFO) | |

## 5. What to watch live, and when to abort

### 5.1 Live monitor (terminal D)

```bash
# one line per local cycle: time since start, lambda, kappa, s_g*, fallback, replan reason, global/local ms,
# min dynamic surface distance on odom, lateral deviation from Pi0
tail -n +2 -f $LOG_DIR/cycles.csv | awk -F, 'NR==1{t0=$1} {printf "t=%6.1f lam=%.2f kap=%.2f sg=%.3f fb=%s reason=%-9s glob=%7.1fms loc=%5.2fms dmin_odom=%s devPi0=%.2f\n", $1-t0,$17,$20,$19,$30,$29,$3,$4,$25,$28; fflush()}'
```

| Signal | Source | Normal | Watch for |
|---|---|---|---|
| PX4 mode | `rostopic echo /uav1/mavros/state/mode` | `OFFBOARD` | any other mode |
| gate | `rostopic hz /uav1/prometheus/command/ego_command_stop_pub` | ~10 Hz in flight | 0 Hz while flying |
| command rate | `rostopic hz /uav1/prometheus/command` | ~100 Hz (traj_server) | drops |
| λ (col 17) | monitor | 0 in static runs; rises towards 1 as an obstacle approaches, falls after | λ > 0 in a static run |
| κ (col 20), s_g* (col 19) | monitor | mostly 0; > 0 only when VO must leave the guidance cone | κ = 1 for many cycles (fallback) |
| fallback (col 30) | monitor | rare, isolated cycles | long runs of `1` (stall, F-3) |
| replan reason (col 29) | monitor | `periodic` ~1 Hz, `map` while the map grows, `risk_Dk`/`kappa` near obstacles | many `fail` |
| global ms (col 3) | monitor | tens of ms static; up to seconds near obstacles (asynchronous) | > 9800 ms (eq. 32 margin) |
| local ms (col 4) | monitor | < 1 ms | > 100 ms |
| min dyn. surface distance (col 25) | monitor | > 0, ideally > d_min = 0.2 m | ≤ 0 (contact) |
| reference vs odom | `p_ref` vs `p_odom` columns | lag ≈ 0.6–0.9 m at 0.8 m/s | > 2 m (vehicle not following) |

### 5.2 Abort conditions

Stop the run, collect the logs, and do not continue to the next run:
- `Failsafe` / `Odom invalid` / mode leaves OFFBOARD;
- the drone visibly touches a cylinder or a sphere (col 25 ≤ 0, or static contact);
- `conflict=1` in the diag line;
- traj_server prints `invalid time`;
- fallback stays `1` for more than 15 s (stall);
- the timeout is reached.

## 6. Logs to collect and send

The node writes everything into `$LOG_DIR`. Pack the whole folder:

```bash
cd $(dirname $LOG_DIR) && tar czf $(basename $LOG_DIR).tgz $(basename $LOG_DIR)
```

| File | Written by | Needed for |
|---|---|---|
| `cycles.csv` | planner, 10 Hz | all metrics and checks (fields below) |
| `paths.csv` | planner, per global path | M6 flip-backs, path switches by reason |
| `pi0.csv` | planner, first path after the goal | M5 |
| `odom_gt.csv` | planner, 20 Hz | safety distances (odom vs ground truth), V-10 |
| `planner_meta.yaml` | planner, at the goal | V-2 (constants, switches, engineering settings, t0) |
| `checks.yaml` | planner, at goal reached / shutdown | V-4, V-7, V-8, V-9 |
| `metrics.yaml` | `verify_run.py` | V-11, M1–M6 |
| `dyn_obstacles_seed<k>.yaml` | `dyn_obstacles.py` | the actual obstacle start/velocity after seed jitter |
| `planner.log`, `uav_control.log`, `gazebo.log`, `octomap.log`, `roscore.log` | start script tabs | diagnostics, failsafes, `invalid time` |
| `run.bag` (`RECORD=1`) | rosbag tab | replay if something is unclear |

**`cycles.csv` fields** (32 columns, in this order; spec §8):

```
 1 t                    2 mode                   3 plan_ms_global        4 plan_ms_local
 5-7 p_ref_x/y/z        8-10 v_ref_x/y/z         11-13 p_odom_x/y/z      14-16 v_star_x/y/z
17 lambda              18 xi                    19 s_g_star             20 kappa
21 path_id             22 path_hash             23 n_jump_pts           24 min_surface_dist_ref
25 min_surface_dist_odom  26 turn_angle         27 lateral_dev_ref_to_Pi
--- extension fields ---
28 lateral_dev_ref_to_Pi0  29 replan_reason     30 fallback_active      31 nearest_dyn_obs   32 encounter_flag
```

Also note by hand for each run:
- the V-1 … V-11 results (§4);
- any abort reason and the time it happened;
- anything seen in Gazebo or RViz that the logs may not show (e.g. "touched cylinder at (0.2, 2.1)").
