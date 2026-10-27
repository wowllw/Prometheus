# experiment_batch_run_notes.md

**[File name]** `doc/experiment_batch_run_notes.md`

**[Purpose]** Instructions for running the Subtask-4 ablation as an automated batch. It covers:
- what the batch shell driver must do, with a skeleton;
- how groups are switched;
- log output paths;
- post-processing of the batch data;
- exclusion rules for anomalous samples;
- the statistical tests.

It also covers how the stall-hold override is handled in the data, and the mandatory `precheck_run.sh` before
every run.

It complements `doc/subtask4_ablation_experiment_run_spec.md`, which holds the binding definitions; this file is
the "how". The per-run steps are the ones used in the R1–R6 closed-loop validation (2026-10-26). It replaces the
batch-relevant parts of `doc/experiment_run_notes.md`, whose precautions N-1 … N-14 still apply.

**No driver or post-processing script exists yet.** The skeletons below are specifications for writing them. They
have not been run, and they produce no data by themselves.

---

## 1. Batch rules (B-x)

- **B-1 Clean environment before every run.** Every run starts with `Scripts/experiment/kill_sim.sh`, then
  `Scripts/experiment/precheck_run.sh`.
  - Launch only if precheck exits 0 and prints `PRECHECK OK`. Otherwise do not launch, log I-0, and run kill_sim
    again (at most 3 attempts, then stop the batch).
  - Reason: a leftover `dyn_obstacles.py` from a previous run deletes the same-named obstacle of the next run
    through its `on_shutdown` hook. This invalidated a validation run (R5, first attempt).
- **B-2 Never stop `dyn_obstacles.py` by hand.** Start it in the background and let `kill_sim.sh` stop it after
  rosmaster and Gazebo are down.
- **B-3 One unique directory per attempt:** `~/.ros/ablation/<variant>/<group>/seed<k>_a<attempt>/`. Create it
  before the launch and never reuse it. The directory name is the only place the seed and attempt are recorded
  besides `dyn_obstacles_seed<k>.yaml`.
- **B-4 Group switching = the `GROUP` variable only.**
  - The start script passes `group:=$GROUP` to `sitl_jps_vo_planner.launch`, which loads
    `config/groups/$GROUP.yaml`.
  - Nothing else may differ between groups: no other variable, file or rebuild.
- **B-5 Fixed variables for every run:** `PLANNER_MODE=coupled PRE_ROTATE=true REPLAN_PERIOD=1.0 RECORD=0`.
  `RECORD=1` only for pre-declared illustration runs (disk: about 200 MB per dynamic bag, 32 GB free at writing).
- **B-6 Order.** Loop over seed (outer) → variant → group (inner), rotating the group order per seed (seed k
  starts at group ((k − 1) mod 6) + 1). Slow drift then affects all groups equally.
- **B-7 Configuration assertion after the launch:**
  - `rosparam get /uav1_jps_vo_planner_node/map/z_min` = 0.8;
  - `…/pre_rotate_yaw` = true;
  - the planner's `coupled mode, group <GROUP>` line matches;
  - after the run, `planner_meta.yaml` switches match the group file.

  A mismatch is I-10. The parameter namespace is `/uav1_jps_vo_planner_node/` (underscore, not `/uav1/…`).
- **B-8 One planner, one B-spline publisher** (V-1, V-5). `rosnode list` shows `uav1_jps_vo_planner_node` and no
  `ego_planner_node`, and `/uav1/planning/bspline` has exactly one publisher. Otherwise I-11.
- **B-9 Display.** The start script opens `gnome-terminal` tabs, so the batch must run inside a logged-in desktop
  session (or with `DISPLAY` set). Starting it over a plain SSH session fails silently.
- **B-10 No manual input** during the batch (N-4): no RViz goal clicks, no RC, and no other heavy processes (N-12).
- **B-11 Abort the whole batch**, rather than continuing, when:
  - precheck fails 3 times in a row;
  - disk free < 5 GB;
  - the same infrastructure code (I-x) occurs in 5 consecutive attempts;
  - a planner crash (F-4) occurs in a group's first 3 runs. That suggests a build or config fault rather than a
    result; report it.

## 2. Per-run procedure (driver steps)

| Step | Action | Pass condition / on failure |
|---|---|---|
| 1 | `kill_sim.sh`; `precheck_run.sh` | `PRECHECK OK`, else I-0 (B-1) |
| 2 | create `LOG_DIR`, write `attempt.yaml` (variant, group, seed, attempt, start wall time, driver commit) | — |
| 3 | `LOG_DIR=… GROUP=… PLANNER_MODE=coupled PRE_ROTATE=true RECORD=0 bash -ic './Scripts/simulation/ego_planner/new_test_jps_vo_p450_d435i.sh'` | — |
| 4 | wait ≤ 90 s for `coupled mode` in `planner.log`; `/uav1/prometheus/state` connected and odom_valid; home position received | else I-7 |
| 5 | B-7 / B-8 assertions | else I-10 / I-11 |
| 6 | `sleep 6`; publish the two `UAVSetup` messages of `arm_and_command.sh` directly on `/uav1/prometheus/setup`: arm (`cmd 0`) and, after 3 s, `COMMAND_CONTROL` (`cmd 3`). Confirm each (`armed: True`, then `control_state 2`) and retry up to 3 times. The script's gnome-terminal publish lost the COMMAND message once in the smoke test. | tries recorded in `attempt.yaml`; failure ends in I-3 at step 7 |
| 7 | wait until odom z > 1.35 (≤ 40 s), then 10 s settle; sample z for 5 s; `armed True`, mode OFFBOARD, `control_state 2` | z in [1.35, 1.65] and |v| < 0.1, else I-3 (spec Q-A) |
| 8 | gate check: 0 messages on `/uav1/prometheus/command/ego_command_stop_pub` in 5 s | else I-11 |
| 9 | start `dyn_obstacles.py _scenario:=scenario_ablation_all.yaml _variant:=V _seed:=k _log_dir:=$LOG_DIR` in the background (static variants too: empty seed file) | dynamic: every expected `dyn_obs_*` in `/gazebo/model_states` after 5 s, and exactly 1 script process, else I-4 |
| 10 | `send_goal.py _scenario:=… _variant:=V` (t₀) | — |
| 11 | after 6 s: gate 10 Hz, all `true`; mode OFFBOARD | else note in T9 (gate fault = V-6 fail, report) |
| 12 | wait until `checks.yaml` appears (goal reached) or t₀ + timeout + 5 s | timeout is the variant's `timeout` |
| 13 | `kill_sim.sh` (closes the bag if any, stops the master, then the obstacle script) | the node writes `checks.yaml` on shutdown if the goal was not reached |
| 14 | `verify_run.py $LOG_DIR --timeout <timeout>` → `metrics.yaml` | missing files: I-8 |
| 15 | append one line to `batch_index.csv` (B-12); optionally `tar czf` the run directory | — |

**B-12 `~/.ros/ablation/batch_index.csv`:** one line per attempt with variant, group, seed, attempt, LOG_DIR,
start/end time, precheck result, hover z (mean of step 7), invalid code (or empty), end_reason. The post-processing
reads this index, never a directory listing.

## 3. Driver skeleton (specification; adapt and test on one run before the batch)

```bash
#!/bin/bash
# run_ablation_batch.sh  (to be written; skeleton only)
set -u
ROOT=$HOME/Prometheus; PKG=$ROOT/Modules/jps_vo_planning/jps_vo_planner
SC=$PKG/config/scenarios/scenario_ablation_all.yaml
OUT=$HOME/.ros/ablation; INDEX=$OUT/batch_index.csv
VARIANTS="S1_east S2_north S3_climb_ne head_on crossing_90 M1_three_way"
GROUPS_ALL=(g1 g2 g3 g4 g5 g6)
declare -A TMO=([S1_east]=70 [S2_north]=70 [S3_climb_ne]=70 [head_on]=70 [crossing_90]=70 [M1_three_way]=80)

run_attempt() {   # $1 variant  $2 group  $3 seed  $4 attempt  -> echoes invalid code or empty
  local V=$1 G=$2 K=$3 A=$4
  local LOG_DIR=$OUT/$V/$G/seed${K}_a${A}
  bash $ROOT/Scripts/experiment/kill_sim.sh >/dev/null 2>&1
  bash $ROOT/Scripts/experiment/precheck_run.sh | grep -q "PRECHECK OK" || { echo I-0; return; }
  mkdir -p $LOG_DIR
  (cd $ROOT && LOG_DIR=$LOG_DIR GROUP=$G PLANNER_MODE=coupled PRE_ROTATE=true RECORD=0 REPLAN_PERIOD=1.0 \
     bash -ic './Scripts/simulation/ego_planner/new_test_jps_vo_p450_d435i.sh' >/dev/null 2>&1)
  # ... steps 4-8 as in the table (each failing check: kill_sim, echo <I-x>, return)
  source $ROOT/devel/setup.bash
  PYTHONUNBUFFERED=1 rosrun jps_vo_planner dyn_obstacles.py _scenario:=$SC _variant:=$V _seed:=$K \
     _log_dir:=$LOG_DIR > $LOG_DIR/dyn_obstacles.log 2>&1 &        # never killed here (B-2)
  # ... step 9 check
  rosrun jps_vo_planner send_goal.py _scenario:=$SC _variant:=$V
  local t_end=$(( $(date +%s) + ${TMO[$V]} + 5 ))
  while [ ! -f $LOG_DIR/checks.yaml ] && [ $(date +%s) -lt $t_end ]; do sleep 2; done
  bash $ROOT/Scripts/experiment/kill_sim.sh >/dev/null 2>&1
  python3 $PKG/scripts/verify_run.py $LOG_DIR --timeout ${TMO[$V]} > $LOG_DIR/verify.txt 2>&1 || true
  # ... post-run I-x checks from files (section 5), echo code or empty
}

for K in $(seq 1 20); do
  for V in $VARIANTS; do
    for i in 0 1 2 3 4 5; do
      G=${GROUPS_ALL[$(( (i + K - 1) % 6 ))]}
      for A in 1 2 3; do
        code=$(run_attempt $V $G $K $A)
        # append to $INDEX; break if valid (empty code); after 3 invalid attempts mark the cell excluded
        [ -z "$code" ] && break
      done
    done
  done
done
```

Key points for writing it:
- Each call to `kill_sim.sh` must be its own process, as above. `pkill -f` patterns inside the caller's command
  line can kill the caller; `kill_sim.sh` and `precheck_run.sh` already use anchored patterns.
- Use `bash -ic` for the start script so the user's ROS environment is loaded.
- The ROS master changes per run. Source `devel/setup.bash` again in each attempt, and never keep `rostopic`
  processes across runs.
- Make the driver resumable: on start, read `batch_index.csv` and skip (variant, group, seed) cells that are
  already valid or excluded.
- Log the driver's own stdout to `$OUT/batch_<date>.log`.
- Expected duration: about 2.5–3.5 min per attempt, so 720 valid runs take ≈ 30–42 h plus reruns. Plan sessions
  of whole seeds (36 runs ≈ 2 h) so an interruption never leaves a seed half-done across days.

## 4. Log output paths

```
~/.ros/ablation/
  batch_index.csv                       one line per attempt (B-12)
  batch_<date>.log                      driver log
  <variant>/<group>/seed<k>_a<attempt>/
      attempt.yaml                      driver: variant, group, seed, attempt, times, driver commit
      cycles.csv  checks.yaml  planner_meta.yaml  pi0.csv  paths.csv  odom_gt.csv   (planner node)
      dyn_obstacles_seed<k>.yaml  dyn_obstacles.log                              (obstacle script)
      metrics.yaml  verify.txt                                                   (verify_run.py)
      planner.log  uav_control.log  gazebo.log  octomap.log  roscore.log        (start script)
  results/
      ablation_runs.csv                 one row per attempt (T2 columns), produced by post-processing
      aggregates_<variant>.csv          T3/T4 values
      tests_<variant>.csv               T5 values
```

The R1–R6 validation logs stay in `~/.ros/closed_loop/` and are **not** part of the ablation data.

## 5. Post-processing (to be implemented as one versioned script, commit recorded in T0)

1. **Read `batch_index.csv`.** For each attempt, load `metrics.yaml`, `checks.yaml`, `planner_meta.yaml`,
   `attempt.yaml` and `dyn_obstacles_seed<k>.yaml`.
2. **Assert identity:** group switches in `planner_meta.yaml` = `config/groups/<group>.yaml`; constants equal T1;
   `engineering.pre_rotate_yaw = 1`. Otherwise I-10.
3. **File-based invalid checks:**

   | Code | Check |
   |---|---|
   | I-4 | `stale_obstacle_drops > 0`; obstacle rows missing in `odom_gt.csv` in dynamic variants |
   | I-8 | any required file missing; `cycles.csv` not covering [t₀, t_end] |
   | I-9 | `no_cloud_cycles > 20` |
   | I-2 | odom jump > 1 m between consecutive `odom_gt.csv` samples without contact |
   | I-1 | mean real-time factor < 0.9 (from `gazebo.log`, if logged; otherwise record "not measured" in T0) |

4. **Read `cycles.csv` safely.** Drop rows whose `stall_hold_remaining` is empty (a partial last row).
5. **Derived stall fields per run:**
   - `override_cycles = stall_breaks + stall_hold_cycles`;
   - `override_share = override_cycles / cycles`;
   - `trigger_any = stall_breaks > 0`;
   - `success_wo_trigger = M4_success ∧ ¬trigger_any`. In static variants, use goal reached ∧ ¬trigger_any.

   Cross-check: the sum of `stall_break_triggered` in `cycles.csv` must equal `stall_breaks`, and the number of
   rows with `stall_break_triggered = 0` whose previous row has `stall_hold_remaining > 0` must equal
   `stall_hold_cycles`. A mismatch is a logging fault: I-8.
6. **Write `ablation_runs.csv`** with exactly the T2 columns.
7. **Aggregate valid rows** per (variant, group) for T3/T4, and run the tests for T5 (§7).
8. **Never edit raw run directories.** All corrections live in the script, so every thesis number can be reproduced
   from the raw files (P-11).

## 6. Exclusion rules for anomalous samples

| Situation | Treatment |
|---|---|
| Infrastructure fault I-0 … I-11 | invalid; rerun the same seed (≤ 3 attempts); then the cell is "excluded" and listed in T6 |
| Collision, timeout, stall, planner fault, band exit (F-1 … F-5) | **valid failure**, kept |
| Stall-breaker triggered / hold active / fallback active | **valid**, kept; reported in T4 (§8 of the spec) |
| Extreme metric value (> Q3 + 3·IQR) | kept; flagged † in T2/T3 and discussed |
| Hover outside [1.35, 1.65] m before t₀ | I-3, rerun (spec Q-A) |
| One side of a seed pair excluded | drop that pair from the paired test only; report n pairs |

Rules:
- Never exclude a run because its metrics look wrong.
- Never rerun a valid run.
- Report invalid and excluded counts per group (T6). An unequal number of invalid runs between groups is itself a
  finding to discuss.

## 7. Statistical tests (declared before the first run)

| Quantity | Statistic | Test vs g6 (paired by seed) |
|---|---|---|
| M1 mean / P95, M2, M3, M5 mean, M5 max, M6 | mean ± std, median [IQR] | Wilcoxon signed-rank, two-sided, `zero_method='pratt'` (ties at 0 are frequent for M6) |
| M4, goal success, success w/o trigger | k/n, Wilson 95 % CI | exact McNemar (binomial on discordant pairs) |
| stall_breaks, override_share | mean ± std, median [IQR], share of runs with ≥ 1 trigger | Wilcoxon (descriptive priority; reported, not used for ranking) |
| Effect size | median of paired differences (g6 − gX) | bootstrap 95 % CI, 10 000 resamples, fixed RNG seed recorded in T0 |
| Multiplicity | — | Holm–Bonferroni over the 5 comparisons (g6 vs g1…g5) per metric and variant |

- α = 0.05 after correction.
- With n ≤ 20, a non-significant result is "no evidence of a difference".
- If all 20 pairs of a static variant are identical (deterministic runs), report the values without a p-value.
- **Override-aware analysis (mandatory):**
  1. *Primary:* all valid runs, override included.
  2. *Sensitivity:* the M4 / goal-success test repeated with `success_wo_trigger`. If the sign or significance of
     any g6-vs-gX comparison changes, state it in the results text.
  3. *Descriptive:* T4 per group (Σ stall_breaks, Σ stall_hold_cycles, trigger share, mean override_share).
  4. *Failure types:* combine F-2 and F-3 as "no progress" for runs with `stall_breaks > 0`, because a hold can
     turn a stall into a slow creep that ends as a timeout.
  5. *Not allowed:* removing runs or cycles with override from the M-metrics.
- **Per variant, not pooled.** A pooled table across variants may be shown only in addition, with variant as the
  pairing stratum.

## 8. Checklist before starting the batch

- [ ] Spec §1 entry gate E-1 … E-6 met (E-5: smoke run g1–g5 on S1_east seed 1).
- [ ] Spec §11 open points Q-A … Q-E confirmed.
- [ ] Driver tested end-to-end on one (variant, group, seed), and its output checked by hand against the R1–R6
      procedure.
- [ ] Post-processing tested on the R1–R6 validation directories. Its M-values must equal those in `metrics.yaml`;
      its stall fields must equal `checks.yaml` (R2: 6 / 30; R6: 1 / 5).
- [ ] T0 and T1 in `ablation_result_record_table.md` filled.
- [ ] Disk ≥ 20 GB free, `RECORD=0`.
