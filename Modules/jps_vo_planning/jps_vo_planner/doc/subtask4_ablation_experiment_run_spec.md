# subtask4_ablation_experiment_run_spec.md

**[File name]** `doc/subtask4_ablation_experiment_run_spec.md`

**[Purpose]** The binding specification for the Subtask-4 ablation experiment. It covers:
- the 6 control groups (g1–g6) and their launch parameters;
- 20 repetitions per group and variant, and the seed rules;
- invalid-run criteria;
- the equal tuning budget;
- the log schema, which reuses `cycles.csv` including the stall-breaker fields;
- how the stall-hold override is accounted for;
- the analysis and write-up points for the 3 innovation points.

It **supersedes** `doc/subtask4_ablation_experiment_spec.md` (written before the closed-loop validation). That file
is kept for history.

This document is a template and specification. **It contains no results.** Every number below is a configured
constant, a derived constant or a requirement, never a measurement.

Related files:

| File | Role |
|---|---|
| `doc/ablation_result_record_table.md` | blank result tables |
| `doc/experiment_batch_run_notes.md` | batch execution, post-processing, exclusion and statistics |
| `config/scenarios/scenario_ablation_all.yaml` | all 6 scenario variants in one file |
| `config/groups/g1.yaml … g6.yaml` | group switches (exist, unchanged) |
| `scripts/verify_run.py` | per-run verification rows and M1–M6 → `metrics.yaml` |
| `scripts/check_scenarios.py` | offline scenario geometry check |
| `Scripts/experiment/precheck_run.sh`, `kill_sim.sh` | environment cleanup before and after every run |

---

## 1. Entry gate (all must hold before the first ablation run)

| # | Condition | Status at writing (2026-10-26) |
|---|---|---|
| E-1 | Closed-loop validation R1–R6 passed V-1 … V-11 on the unified configuration (§2) | met (`~/.ros/closed_loop/summary_all.md`) |
| E-2 | 4 offline test suites: ALL CHECKS PASSED | met at the R1–R4 re-test |
| E-3 | `check_scenarios.py config/scenarios/scenario_ablation_all.yaml --d_min 0.2 --seeds 1-20` exits 0 | met (6/6 OK) |
| E-4 | Core sources unchanged since the validation (`coupling.*`, `jps3d.*`, `local_planner.*`, `planner_core.*`; timestamps 00:35–01:14 on 2026-10-26) and the git commit recorded in T0 | to be checked at campaign start |
| E-5 | Each group g1–g5 has completed one smoke run on `S1_east` (seed 1) with a complete log set. Only g6 was run in closed loop so far. | **open** |
| E-6 | Free disk ≥ 20 GB with `RECORD=0` (§9) | to be checked at campaign start |

If E-5 shows a software fault in a group (crash, NaN, missing log rows), stop and report. A planner *failure*
(collision, stall, timeout) in a smoke run is not a fault; it is a possible result.

## 2. Frozen configuration (identical for all groups and runs)

**Model (Stage 2, frozen):** J_total, ξ, λ(ξ), Γ, C1, C2, C3. They are not changed and not parameterised beyond the
group switches of §3.

**Constants** (from `planner_meta.yaml` of the validation runs):

| r_q | e_cut | v_c | Δt | τ_h | m | L_min | ψ_max | θ_max | v_min | w_θ | w_a | ρ_g | ε_w | T_c | T_g | Δv* |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.30 m | 0.10 m | 0.80 m/s | 0.10 s | 4.0 s | 1.20 m | 2.0 m | 60° | 60° | 0.10 m/s | 1.0 | 1.0 | 10.0 | 0.05 | 0.80 s | 1.50 s | 1.60 m/s |

**Engineering parameters:**

| Parameter | Value |
|---|---|
| v_max | 1.0 m/s |
| a_max | 0.8 m/s² |
| d_min | 0.2 m |
| r_infl | 0.60 m (derived) |
| τ₁ | 2.214 s (derived, eq. 27) |
| τ₂ | 12.043 s (derived, eq. 32) |
| ℓ_w | 0.838 m (derived) |

**Upper-level / engineering configuration (fixed after the R1–R6 validation):**

| Item | Value | Where |
|---|---|---|
| `pre_rotate_yaw` | true | `config/jps_vo_params.yaml`, launch default, start script `PRE_ROTATE=true` |
| `map/z_min`, `map/z_max` | 0.8 m, 3.0 m | `config/jps_vo_params.yaml` (`flight_band` record in the scenario file) |
| JPS | `tie_rule: strict`, search margin 5 m, `async_global: true` | `config/jps_vo_params.yaml` |
| replan | period 1.0 s, switch threshold 0.3 m | start script / yaml |
| stall breaker | trigger after `kStallCycles = 12` cycles with ‖v*‖ < 1e-6 and goal > 0.5 m; escape held `kStallHoldCycles = 6` cycles | `planner_node.h` (compiled in, always on, identical for all groups) |
| goal reached | ‖p_ref − goal‖ ≤ 0.3 m and ‖v_ref‖ ≤ 0.1 m/s | yaml |
| gate | true at 10 Hz only in COMMAND with an active trajectory | node |

## 3. Control groups and launch parameters

All groups run **the same binary** with `planner_mode: coupled`. They differ **only** in the switch block loaded
from `config/groups/<GROUP>.yaml` (selected by the start-script variable `GROUP`). Every group differs from g6 in
the listed component only (nested models, Stage-2 §5.2).

| Group | Name | Stage-2 ID | Switches different from g6 | Component removed relative to g6 |
|---|---|---|---|---|
| g1 | Pure 3D-JPS | A1 | `lambda_mode: zero` | VO: λ ≡ 0, so w_e = ℓ_e/L_s, Γ = 0, κ = 0, θ_c = 0, and the dynamic part of (C-s) is off |
| g2 | Pure VO | A2 | `lambda_mode: one` (`use_rho_in_pure_vo: false`) | global layer in the local decision: λ ≡ 1, so J_A drops out, θ_c = π, Γ = 0; classic VO, eq. (16) |
| g3 | One-way series JPS → VO | A10 | `enable_c2: false`, `enable_c3: false` | both coupling channels: no risk/κ in the edge weights, no D_k re-expansion, no guidance cone |
| g4 | Coupling, constant λ | A3 | `lambda_mode: "const:0.5"` | adaptivity of λ(ξ) |
| g5 | Coupling without Γ | A7 | `enable_gamma: false` | coupling penalty Γ, eq. (34)(35) |
| g6 | Full C1/C2/C3 (proposed) | — | — | — |

g6 switch block (`config/groups/g6.yaml`):

```yaml
switches:
  lambda_mode: adaptive        # zero | one | const:<c> | adaptive
  use_rho_in_pure_vo: false
  enable_c2: true
  enable_c3: true
  c3_soft_tracking: 0.0
  enable_gamma: true
  lambda_shape: quintic_log
  theta_c_mode: linear
  xi_source: reference
```

Launch command per run (start script variables; nothing else differs between groups):

```bash
LOG_DIR=<run dir> PLANNER_MODE=coupled GROUP=<g1..g6> PRE_ROTATE=true RECORD=0 REPLAN_PERIOD=1.0 \
  ./Scripts/simulation/ego_planner/new_test_jps_vo_p450_d435i.sh
```

Each run's `planner_meta.yaml` records the group, the switches (numeric), the constants and the engineering block.
The post-processing **must check** that it matches the intended group (batch notes B-7).

Definitions to state in the thesis:
- g2 is the classic VO (16), argmin over F of ‖v − v_pref‖. The smooth-TTC variant (17) is not one of the groups.
- g3 is A10: J_A = V_Π(q(v)) stays in the local cost, but nothing flows from local to global and the tangent is
  not a constraint.
- g4 uses λ = 0.5, where Γ is largest.
- **C2 and C3 are not separated**: g3 removes both (decision Q2: no A4/A5). The thesis writes "C2 and C3
  jointly".

**External reference (outside the 6 groups):** the clean ego-planner in `~/Prometheus_baseline` (decision Q5),
with `ground_height` 0.8, the same band.
- Run it on the same variants and seeds and report it in a separate table. It is not part of the paired tests
  between groups.
- **Its configuration has not been flown yet** (gate before its first run: one smoke run).

## 4. Metrics (definitions final; implemented in `verify_run.py`)

**Window:** [t₀, t_end]. t₀ is the goal publish time; t_end is goal reached, failure or timeout.
**Signals:** metrics use the reference (p_ref, v_ref); safety distances use odom vs ground truth.

| ID | Metric | Definition | `metrics.yaml` key(s) |
|---|---|---|---|
| M1 | Single-step planning time [ms] | `plan_ms_local + plan_ms_global` per cycle; the global term is non-zero only in cycles receiving a global result | `M1_mean_ms`, `M1_p95_ms`, `M1_max_ms` |
| M2 | Total path length [m] | Σ‖Δp_ref‖ over the window | `M2_path_length_m` |
| M3 | Maximum turning angle [deg] | max ∠(v_ref(t), v_ref(t − 1 s)), where both speeds ≥ v_min | `M3_max_turn_deg_1s` |
| M4 | Dynamic-obstacle avoidance success | 1 if goal reached before timeout, and min dynamic surface distance > 0, and min static surface distance > 0; else 0. N/A in static variants | `M4_success` |
| M5 | Deviation from the global path [m] | lateral distance p_ref → Π₀ (initial global path); mean and max | `M5_mean_m`, `M5_max_m` |
| M6 | Path-switching oscillation count | flip-backs A → B → A (Hausdorff < 0.3 m to a path active in the last 10 s); N/A for g2 | `M6_flip_backs` |

**Required per-run diagnostics (reported, not ranked as outcome metrics):**

| Key | Source | Meaning |
|---|---|---|
| `stall_breaks` | `checks.yaml` | number of stall-breaker triggers |
| `stall_hold_cycles` | `checks.yaml` | cycles after a trigger in which the stored escape velocity was reused (≤ 5 per trigger) |
| `override_cycles` | derived: `stall_breaks + stall_hold_cycles` | cycles whose commanded v* is not the core's v* |
| `override_share` | derived: `override_cycles / cycles` | fraction of the run under override |
| `fallback_cycles` | `checks.yaml` | cycles with the fallback (40) active inside the core |
| `max_global_ms`, `cycles_over_dt`, `max_cycle_gap_s` | `checks.yaml` | timing |
| `end_reason` | `metrics.yaml` | goal / collision (F-1) / timeout (F-2) / stall (F-3) / fault (F-4) / band (F-5) |
| `min_static_surface_dist_odom_m`, `min_dyn_surface_dist_odom_m` | `metrics.yaml` | safety |
| `diag_path_switches`, `diag_encounter_flag` | `metrics.yaml` | mechanism evidence |

## 5. Repetitions and seeds

- **20 valid runs** per (group, variant): 6 groups × 6 variants × 20 = **720 valid runs**, plus reruns of invalid
  runs.
- **Evaluation seeds 1–20**, identical for every group, so every comparison is paired by seed.
- **Tuning seeds 101–110**, used only under P1 (§7), and disjoint from the evaluation seeds.
- The seed is passed only to `dyn_obstacles.py` (`_seed:=k`). It jitters obstacle start (± `jitter_pos`) and speed
  (± `jitter_speed`), and the drawn values are logged in `dyn_obstacles_seed<k>.yaml`.
- **Static variants have no seeded input.** Their 20 repetitions measure only run-to-run nondeterminism (Gazebo /
  PX4 / thread timing), not scenario variation. State this in the thesis. Run `dyn_obstacles.py` in static
  variants too: it logs an empty seed file and keeps the procedure uniform.
- **Run order:** interleave groups per seed and variant (seed k: g1…g6 in a per-seed rotated order, then seed
  k+1), so slow drift of the machine or simulator affects all groups equally. Never run all 20 seeds of one
  group in a block.
- The seed is not in `planner_meta.yaml`. It is carried by the run directory name and the seed file (batch notes
  B-3).

## 6. Valid, failed and invalid runs

**Success (S-1):**
- goal reached (‖p_ref − goal‖ ≤ 0.3 m, ‖v_ref‖ ≤ 0.1 m/s) before the timeout (70 s; 80 s for M1_three_way);
- no failure condition.

**Failure: valid run, counted as unsuccessful.**

| Code | Condition |
|---|---|
| F-1 | Collision: static or dynamic minimum surface distance (odom) ≤ 0, or contact followed by an odom failsafe |
| F-2 | Timeout |
| F-3 | Stall: reference moved < 0.2 m in 15 s while not at the goal |
| F-4 | Planner fault: crash, NaN, invalid B-spline, or > 1 s without publishing in COMMAND |
| F-5 | p_ref outside the map box: x, y outside ±25 m, or **z outside [0.8, 3.0]** |

A d_min violation without contact (0 < distance < d_min) is not a failure; it is logged.

**Invalid: infrastructure fault, rerun with the same seed (max 3 attempts, then "excluded").**

| Code | Condition |
|---|---|
| I-0 | `precheck_run.sh` did not print `PRECHECK OK` before the launch (the run must not be started; if it was, it is invalid) |
| I-1 | Gazebo real-time factor averaged over the run < 0.9 |
| I-2 | PX4/EKF/GPS fault without contact (odom failsafe, odom jump > 1 m) |
| I-3 | Arming/takeoff failed, or hover before t₀ outside [1.35, 1.65] m (see risk Q-A, §11) |
| I-4 | Obstacle stale: `stale_obstacle_drops > 0` in `checks.yaml`, `dyn_obstacles.py` crashed, a `dyn_obs_*` model missing from `/gazebo/model_states` at t₀ + 1 s in a dynamic variant, or more than one `dyn_obstacles.py` process |
| I-5 | A trigger never fired although the UAV reference passed through the trigger region |
| I-6 | Manual interference, or a second goal publisher |
| I-7 | Crash outside the planner: Gazebo, PX4, mavros, uav_control, octomap, traj_server |
| I-8 | Logs missing or truncated: `cycles.csv`, `checks.yaml`, `planner_meta.yaml` or `odom_gt.csv` absent, or `cycles.csv` not covering [t₀, t_end] |
| I-9 | Octomap cloud stale (> 1 s) for more than 2 s (`no_cloud_cycles` > 20) |
| I-10 | `planner_meta.yaml` group, switches or engineering block differ from the intended group, or `map/z_min` ≠ 0.8 |
| I-11 | V-1 or V-5 violated: a second planner node, or a second publisher on `/uav1/planning/bspline` |

Rules:
- A collision, timeout, stall, fallback, stall-breaker trigger or planner crash is **never** an invalid run; it
  is a result.
- **No metric-based outlier removal.** Values beyond Q3 + 3·IQR are flagged and discussed, and stay in the mean.
- Never rerun a valid run.

## 7. Equal tuning budget (separating structural gains from tuning effects)

- **P0 (main experiment): zero tuning for every group, including g6.**
  - All groups use §2 unchanged.
  - Every number in the thesis results tables comes from P0.
  - The configuration was fixed during the R1–R6 validation, **using g6 only**. That is a tuning advantage for
    g6 and must be stated (§11, Q-C). All fixes were engineering-layer and group-independent: pre-rotation,
    floor height, stall hold, gate. None of them changes a group switch.
- **P1 (only if a calibratable parameter must change, e.g. d_min):**
  - every group gets the same budget: 6 configurations from the same range by the same method;
  - selection on tuning seeds 101–110 with a selection score declared in T1 before any P1 run;
  - evaluation on seeds 1–20;
  - all trials logged in the appendix.
- **Optional sensitivity:** a d_min sweep {0.1, 0.2, 0.3} applied to all groups identically, reported as curves.
- **No model or switch change after the first evaluation run.** A defect becomes a limitation, not a retune.

## 8. Effect of the stall-hold override on the results (mandatory analysis)

**What the mechanism does:**
- After 12 consecutive cycles with v* ≈ 0 (goal > 0.5 m away), the node computes an escape velocity v_esc. It
  uses the fallback rule (40) over moving, statically free candidates of the core's last local problem.
- v_esc is commanded for 6 cycles (0.6 s) instead of the core's v*. The core still runs each cycle, and its
  global feedback (κ, re-searches) is unaffected.
- The mechanism is engineering-layer and identical for all groups. **It is not part of the Stage-2 model**, and
  the commanded velocity during those cycles is not the J_total optimum.

**Logging facts needed for correct analysis:**
- `stall_break_triggered` = 1 on the trigger cycle.
- `stall_hold_remaining` is the value **after** that cycle: 5 on the trigger cycle, then 4 … 0.
- `stall_hold_cycles` in `checks.yaml` counts the reuse cycles after the trigger (5 per completed hold). So
  override cycles = `stall_breaks + stall_hold_cycles`.
- In override cycles, the `v_star_*` columns hold the **commanded** v_esc; the core's own v* of that cycle is not
  logged. `turn_angle` is computed by the core before the override. `lambda`, `kappa`, `s_g_star` are the core's
  values.

**Expected group dependence (to be measured, not assumed):**
- g1 (λ ≡ 0) has a ray-shaped cone and is the most likely to hit v* = 0 at static corners.
- g2 has no global guidance.
- The override share may therefore differ strongly between groups.

**Required reporting per (group, variant):**
1. Total and mean ± std of `stall_breaks` and `stall_hold_cycles`.
2. The share of runs with ≥ 1 trigger, and the mean `override_share`.
3. The share of **successful** runs that needed ≥ 1 trigger. A success that depended on the override is reported
   as such.
4. Primary analysis on **all valid runs**. Runs are **never** excluded for having triggers: the breaker is part of
   the evaluated system, and excluding them would bias towards groups that stall more.
5. Sensitivity analysis: repeat the paired M4 test with "success **without** any trigger" as the outcome. If a
   group ranking changes, the thesis must say so.

**Interaction with the failure codes:**
- A hold advances the reference by about ‖v_esc‖ · 0.6 s per trigger, so a real deadlock can turn into a slow
  creep. Example: in validation run R2, 6 triggers moved it ≈ 0.24 m in ≈ 12 s.
- A deadlock that formerly ended as F-3 (stall) may therefore end as F-2 (timeout). Both are failures, so M4 is
  unaffected, but failure-type statistics must group F-2 and F-3 as "no progress" when `stall_breaks > 0`.
- M2 and M3 include the escape motion. M1 is unaffected: the escape is computed in the node, and its time is not
  in `plan_ms_local`.

## 9. Log schema (reuses the validated `cycles.csv`)

One directory per run, `~/.ros/ablation/<variant>/<group>/seed<k>_a<attempt>/`, containing:

| File | Writer | Content |
|---|---|---|
| `cycles.csv` | planner node | one row per local cycle (10 Hz), 34 columns (below) |
| `checks.yaml` | planner node, at the end | counters: cycles, continuity/format violations, `cycles_over_dt`, `max_local_ms`, `max_global_ms`, `max_cycle_gap_s`, global searches/fails, `fallback_cycles`, **`stall_breaks`, `stall_hold_cycles`**, invariant violations, `no_cloud_cycles`, `stale_obstacle_drops`, cloud frame |
| `planner_meta.yaml` | planner node | group, goal, t₀, L_s, constants, switches, engineering block |
| `pi0.csv`, `paths.csv` | planner node | Π₀ and every global path |
| `odom_gt.csv` | planner node | 20 Hz odom and obstacle ground truth |
| `dyn_obstacles_seed<k>.yaml`, `dyn_obstacles.log` | `dyn_obstacles.py` | drawn obstacle parameters |
| `metrics.yaml` | `verify_run.py` | verification rows, M1–M6, safety distances, end reason |
| `planner.log`, `uav_control.log`, `gazebo.log`, `octomap.log`, `roscore.log` | start script | process logs |
| `run.bag` | start script, only with `RECORD=1` | **off in the batch** (≈ 200 MB per dynamic run) |

`cycles.csv` columns, in file order:

```
t, mode, plan_ms_global, plan_ms_local,
p_ref_x, p_ref_y, p_ref_z, v_ref_x, v_ref_y, v_ref_z,
p_odom_x, p_odom_y, p_odom_z,
v_star_x, v_star_y, v_star_z,
lambda, xi, s_g_star, kappa,
path_id, path_hash, n_jump_pts,
min_surface_dist_ref, min_surface_dist_odom,
turn_angle, lateral_dev_ref_to_Pi,
lateral_dev_ref_to_Pi0, replan_reason, fallback_active, nearest_dyn_obs, encounter_flag,
stall_break_triggered, stall_hold_remaining
```

- Columns 1–27 are the Subtask-3 fields; 28–32 the approved extensions; 33–34 the stall-breaker fields.
- `stall_hold_cycles` is a per-run total in `checks.yaml`, not a column.
- Times are wall-clock with 6 decimals.
- A row is complete only if `stall_hold_remaining` is non-empty, so readers must drop a trailing partial row.

## 10. Analysis and write-up points for the 3 innovation points

The bullets are **hypotheses to test**, with the comparison that tests each. Every hypothesis is reported
whether it is supported or not.

**IP1: unified-cost coupling (C1: one J_total, shared ρ, λ(ξ), Γ).**
- *Comparisons:* g6 vs g1 and g2 (interior vs the two boundaries of one cost function); g6 vs g4 (adaptive λ);
  g6 vs g5 (Γ).
- *Hypotheses:*
  - H1a: g1 has lower M4 in head_on, crossing_90 and M1_three_way.
  - H1b: g2 has larger M2/M5 and lower success in S1–S3.
  - H1c: g6 reaches the M4 of the better boundary while keeping M2/M5 close to g1.
  - H1d: g4 has larger M3/M5 than g6 when no threat is present.
  - H1e: g5 has larger M3 than g6.
- *Write-up:*
  - M4 and M5 side by side (safety vs path fidelity);
  - one λ(t) plot from the run with the median M2 (pre-declared rule, marked as an illustration);
  - the stall/override table next to it, since g1/g2 may rely on the breaker.

**IP2: local risk written back to the global planner (C2: ρ_e in w_e, κ from s_g*, D_k re-expansion).**
- *Comparisons:* g6 vs g3 in M1_three_way and the single-dynamic variants. Mechanism signature required:
  `replan_reason` ∈ {risk_Dk, kappa} **before** the closest approach (`verify_run.py` row 3-h).
- *Hypotheses:*
  - H2a: g6 re-plans before the encounter; g3 does not.
  - H2b: M4(g6) ≥ M4(g3).
  - H2c: M6 of g6 stays low.
- *Write-up:* switch time relative to CPA; M6 next to M4.
  - Validation already showed M6 up to 4 (R6) and high C2 re-search counts for g6. Corridor flipping (Stage-2
    risk R5) must be reported, not filtered.
  - Attribution is to "C2 and C3 jointly".

**IP3: global tangent as a hard constraint (C3: guidance cone, exact penalty).**
- *Comparisons:* g6 vs g3 and g2 in the dynamic variants.
- *Hypotheses:*
  - H3a: M5(g6) < M5(g3), M5(g2).
  - H3b: fewer VO side flips than g2 (diagnostic).
  - H3c: s_g* > 0 only where cone ∩ F = ∅.
- *Write-up:* M5 distribution; the share of cycles with s_g* > 0 and its timing relative to encounters.

**Interpretation rules:**
- An outcome difference without the matching mechanism signature is "unexplained".
- A non-significant difference is "no evidence of difference", not "equal".
- Results are given per variant, never pooled across variants only.

**Limitations that must appear in the thesis (agreed):**
1. Static-corner escape is slow: the escape velocity can be ≈ 0.08 m/s, so release can take several triggers (R2:
   6 triggers, goal time 38.3 → 52.3 s).
2. The hold overrides the J_total-optimal velocity for 6 cycles (0.6 s) per trigger. It is engineering-layer and
   the same for all groups. Its frequency is reported per group (§8).
3. Closed-loop validation used a single seed. The ablation's 20 seeds are the first multi-seed evidence, and
   static variants have no seeded input.
4. Global search time exceeds T_g = 1.5 s in dynamic scenarios (validation max 4.23 s), within the eq. (32)
   bound T_g ≤ 9.8 s. M1 max will show it.
5. The ego baseline with z_min/ground_height 0.8 is not yet verified in simulation.
6. Further limitations:
   - JPS optimality only on G_J;
   - constant-velocity ground-truth obstacles (ground-truth masking, which ego does not get);
   - PX4 tracking lag of about 0.8 s;
   - no closed-loop stability proof;
   - VO side switching (R3) and corridor flipping (R5) measured, not removed;
   - one simulated world;
   - the configuration was fixed on g6 only (§7).

## 11. Open points requiring confirmation before the campaign

| ID | Question | Proposed default |
|---|---|---|
| Q-A | I-3 hover band [1.35, 1.65] m. The R5 validation run hovered at 1.21–1.25 m: COMMAND holds the position at the switch time. Under I-3 such a run is invalid and is rerun. | keep I-3; the batch driver waits until z ≥ 1.40 m before the switch is final, else it reruns |
| Q-B | `RECORD=0` for the batch (disk: 32 GB free, ≈ 200 MB per dynamic bag) | RECORD=0; bags only for the illustration runs |
| Q-C | State in the thesis that the engineering configuration was fixed on g6 runs only | yes |
| Q-D | Smoke run of g1–g5 (E-5) before the campaign | yes, 5 runs on S1_east seed 1, not counted in the results |
| Q-E | Ego baseline: run on all 6 variants × 20 seeds, or a subset | all, as a separate table |
