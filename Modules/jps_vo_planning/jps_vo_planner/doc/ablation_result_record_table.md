# ablation_result_record_table.md

**[File name]** `doc/ablation_result_record_table.md`

**[Purpose]** A blank template for recording the Subtask-4 ablation results by hand, after the simulations. It covers
all 6 scenario variants (`config/scenarios/scenario_ablation_all.yaml`) × all 6 groups g1–g6.

Recorded per (group, variant):
- single-step planning time (M1);
- total path length (M2);
- maximum turning angle (M3);
- dynamic-obstacle avoidance success rate (M4);
- deviation from the global path (M5);
- path-switching oscillation count (M6);
- `stall_breaks` and `stall_hold_cycles` totals.

Metric definitions: `doc/subtask4_ablation_experiment_run_spec.md` §4.
Value sources: `verify_run.py` → `metrics.yaml`, and the planner's `checks.yaml`. Aggregation rules:
`doc/experiment_batch_run_notes.md` §6–7.

**This file is a template. Every empty cell (`—` or blank) is filled only from real run logs.** It replaces the earlier
`doc/ablation_record_table.md` (written before the stall-breaker fields existed), which is kept for history.

Conventions:
- mean ± std (n) and median [Q1, Q3] over **valid** runs; n = number of valid runs (target 20).
- M4 as successes/valid, with the Wilson 95 % CI.
- `N/A` where a metric is undefined: M4 in static variants; M6 for g2.
- Flag a value with `†` if a run lies beyond Q3 + 3·IQR (it stays in the mean; see T2).

---

## T0. Campaign record

| Item | Value |
|---|---|
| Campaign start / end (date, time) | |
| Git commit (Prometheus repo) | |
| Core source timestamps verified (E-4) | yes / no |
| Binary build time (`devel/lib/jps_vo_planner/jps_vo_planner_node`) | |
| `check_scenarios.py scenario_ablation_all.yaml --seeds 1-20` exit code | |
| 4 offline test suites | ALL CHECKS PASSED: yes / no |
| Smoke runs g1–g5 (E-5): result per group | 2026-10-26 07:26–07:39, S1_east seed 1, `~/.ros/ablation_smoke/` (not in the formal data): g1 goal, g2 goal, g3 goal, **g4 timeout (F-2, valid failure)**, g5 goal; no crash, all logs complete. Details: `doc/smoke_test_report.md` |
| Post-processing script name + commit | `scripts/post_process_ablation.py` (+ `scripts/verify_run.py`), not yet committed: record the commit here before the campaign |
| Machine (CPU, RAM), OS, ROS, Gazebo, PX4 versions | |
| Mean Gazebo real-time factor over the campaign | |
| RECORD setting | 0 for the batch (Q-B); 1 only for manually selected illustration runs, listed in T9 |

### T0.1 Confirmed experiment decisions (Q-A … Q-E, confirmed by the author 2026-10-26)

| ID | Decision | Implementation |
|---|---|---|
| Q-A | **I-3 hover rule in effect.** Before t₀ the hover altitude must be within z ∈ [1.35, 1.65] m. A run outside it is invalid (I-3, infrastructure) and is rerun with the same seed, at most 3 attempts per seed, then the cell is "excluded" (T6). The low hover seen in validation run R5 (z 1.21–1.25 m after the COMMAND switch) is such an infrastructure case. | `run_ablation_batch.sh` step 7 samples odom z and \|v\| for 5 s after a 10 s settle; outside the band, or \|v\| ≥ 0.1 m/s, gives I-3 |
| Q-B | **`RECORD=0`** (no rosbag) for every batch run. `RECORD=1` only for manually selected illustration runs, to avoid disk exhaustion (≈ 200 MB per dynamic bag). | driver default `RECORD=0`; any `RECORD=1` run is listed in T9 with its reason |
| Q-C | The thesis **Limitations** section states that the engineering safeguards were debugged and validated **only on g6**: yaw pre-rotation, raised `map/z_min` 0.8 m, and the 6-cycle stall hold. g1–g5 received no targeted tuning, so g6 has an engineering-adaptation advantage. | text item for the thesis; no experiment change |
| Q-D | **Smoke test** before the formal campaign: g1–g5 on `S1_east`, seed 1. It only detects build, configuration or node-crash problems. Smoke runs are **not** part of the formal statistics. | separate output root `~/.ros/ablation_smoke/`, `TAG=smoke`; post-processing excludes tag ≠ `formal`; report `doc/smoke_test_report.md` |
| Q-E | **Ego-planner baseline**: all 6 variants × 20 seeds, as an independent external reference (T8). It is **excluded** from the g1–g6 significance tests (T5). | separate launch (`~/Prometheus_baseline`, `ground_height` 0.8); not covered by `run_ablation_batch.sh` yet (no `cycles.csv`/`checks.yaml` from ego) |
| Total runs started / valid / invalid / excluded cells | |

## T1. Frozen configuration (fill before the first run; never change afterwards)

| Item | Value |
|---|---|
| r_q, e_cut, v_c, Δt, τ_h, m, L_min | 0.30 m, 0.10 m, 0.80 m/s, 0.10 s, 4.0 s, 1.20 m, 2.0 m |
| ψ_max, θ_max, v_min, w_θ, w_a, ρ_g, ε_w | 60°, 60°, 0.10 m/s, 1.0, 1.0, 10.0, 0.05 |
| T_c, T_g (declared), Δv* | 0.80 s, 1.50 s, 1.60 m/s |
| v_max, a_max, d_min | 1.0 m/s, 0.8 m/s², 0.2 m |
| r_infl, τ₁, τ₂, ℓ_w (derived) | 0.60 m, 2.214 s, 12.043 s, 0.838 m |
| pre_rotate_yaw, map z band | true, [0.8, 3.0] m |
| tie_rule, search margin, async global, replan period | strict, 5 m, true, 1.0 s |
| Stall breaker: trigger cycles, goal distance, hold cycles | 12, 0.5 m, 6 |
| Tuning protocol | P0 (zero tuning) / P1 (state budget + selection score here) |
| P1 selection score (only if P1) | |
| Measured worst global step over the campaign (ms) | |

## T2. Per-run raw sheet (one row per run attempt; export as `ablation_runs.csv`, 720 valid rows + invalid attempts)

Column header = the first 31 columns of `results/ablation_runs.csv` written by `scripts/post_process_ablation.py` (same names, same order; the CSV appends `tag, goal_success, trigger_any, success_wo_trigger, invalid_reason, log_dir`):

| variant | group | seed | attempt | valid | invalid_code | end_reason | duration_s | M1_mean_ms | M1_p95_ms | M1_max_ms | M2_path_length_m | M3_max_turn_deg_1s | M4_success | M5_mean_m | M5_max_m | M6_flip_backs | stall_breaks | stall_hold_cycles | override_cycles | override_share | fallback_cycles | max_global_ms | cycles_over_dt | min_static_surface_dist_odom_m | min_dyn_surface_dist_odom_m | diag_path_switches | c2_researches_before_cpa | hover_z_before_t0 | outlier_flag | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |

---

## T3. Aggregate results per variant (valid runs)

One block per variant. Fill from `ablation_runs.csv` (valid rows only).

### T3.1 `S1_east` (static, timeout 70 s)

| Group | n valid | Success (goal) k/n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 path length [m] | M3 max turn [deg] | M4 avoidance success k/n (Wilson 95 % CI) | M5 mean [m] | M5 max [m] | M6 flip-backs |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | | N/A | | |  |
| g2 Pure VO | | | | | | | | N/A | | | N/A |
| g3 One-way JPS→VO | | | | | | | | N/A | | |  |
| g4 Const λ 0.5 | | | | | | | | N/A | | |  |
| g5 No Γ | | | | | | | | N/A | | |  |
| g6 Full (proposed) | | | | | | | | N/A | | |  |

Medians [Q1, Q3] for M2, M3, M5 mean, M6:

| Group | M2 median [IQR] | M3 median [IQR] | M5 mean median [IQR] | M6 median [IQR] |
|---|---|---|---|---|
| g1 Pure 3D-JPS | | | |  |
| g2 Pure VO | | | | N/A |
| g3 One-way JPS→VO | | | |  |
| g4 Const λ 0.5 | | | |  |
| g5 No Γ | | | |  |
| g6 Full (proposed) | | | |  |

### T3.2 `S2_north` (static, timeout 70 s)

| Group | n valid | Success (goal) k/n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 path length [m] | M3 max turn [deg] | M4 avoidance success k/n (Wilson 95 % CI) | M5 mean [m] | M5 max [m] | M6 flip-backs |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | | N/A | | |  |
| g2 Pure VO | | | | | | | | N/A | | | N/A |
| g3 One-way JPS→VO | | | | | | | | N/A | | |  |
| g4 Const λ 0.5 | | | | | | | | N/A | | |  |
| g5 No Γ | | | | | | | | N/A | | |  |
| g6 Full (proposed) | | | | | | | | N/A | | |  |

Medians [Q1, Q3] for M2, M3, M5 mean, M6:

| Group | M2 median [IQR] | M3 median [IQR] | M5 mean median [IQR] | M6 median [IQR] |
|---|---|---|---|---|
| g1 Pure 3D-JPS | | | |  |
| g2 Pure VO | | | | N/A |
| g3 One-way JPS→VO | | | |  |
| g4 Const λ 0.5 | | | |  |
| g5 No Γ | | | |  |
| g6 Full (proposed) | | | |  |

### T3.3 `S3_climb_ne` (static, timeout 70 s)

| Group | n valid | Success (goal) k/n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 path length [m] | M3 max turn [deg] | M4 avoidance success k/n (Wilson 95 % CI) | M5 mean [m] | M5 max [m] | M6 flip-backs |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | | N/A | | |  |
| g2 Pure VO | | | | | | | | N/A | | | N/A |
| g3 One-way JPS→VO | | | | | | | | N/A | | |  |
| g4 Const λ 0.5 | | | | | | | | N/A | | |  |
| g5 No Γ | | | | | | | | N/A | | |  |
| g6 Full (proposed) | | | | | | | | N/A | | |  |

Medians [Q1, Q3] for M2, M3, M5 mean, M6:

| Group | M2 median [IQR] | M3 median [IQR] | M5 mean median [IQR] | M6 median [IQR] |
|---|---|---|---|---|
| g1 Pure 3D-JPS | | | |  |
| g2 Pure VO | | | | N/A |
| g3 One-way JPS→VO | | | |  |
| g4 Const λ 0.5 | | | |  |
| g5 No Γ | | | |  |
| g6 Full (proposed) | | | |  |

### T3.4 `head_on` (single_dynamic, timeout 70 s)

| Group | n valid | Success (goal) k/n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 path length [m] | M3 max turn [deg] | M4 avoidance success k/n (Wilson 95 % CI) | M5 mean [m] | M5 max [m] | M6 flip-backs |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | |  | | |  |
| g2 Pure VO | | | | | | | |  | | | N/A |
| g3 One-way JPS→VO | | | | | | | |  | | |  |
| g4 Const λ 0.5 | | | | | | | |  | | |  |
| g5 No Γ | | | | | | | |  | | |  |
| g6 Full (proposed) | | | | | | | |  | | |  |

Medians [Q1, Q3] for M2, M3, M5 mean, M6:

| Group | M2 median [IQR] | M3 median [IQR] | M5 mean median [IQR] | M6 median [IQR] |
|---|---|---|---|---|
| g1 Pure 3D-JPS | | | |  |
| g2 Pure VO | | | | N/A |
| g3 One-way JPS→VO | | | |  |
| g4 Const λ 0.5 | | | |  |
| g5 No Γ | | | |  |
| g6 Full (proposed) | | | |  |

### T3.5 `crossing_90` (single_dynamic, timeout 70 s)

| Group | n valid | Success (goal) k/n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 path length [m] | M3 max turn [deg] | M4 avoidance success k/n (Wilson 95 % CI) | M5 mean [m] | M5 max [m] | M6 flip-backs |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | |  | | |  |
| g2 Pure VO | | | | | | | |  | | | N/A |
| g3 One-way JPS→VO | | | | | | | |  | | |  |
| g4 Const λ 0.5 | | | | | | | |  | | |  |
| g5 No Γ | | | | | | | |  | | |  |
| g6 Full (proposed) | | | | | | | |  | | |  |

Medians [Q1, Q3] for M2, M3, M5 mean, M6:

| Group | M2 median [IQR] | M3 median [IQR] | M5 mean median [IQR] | M6 median [IQR] |
|---|---|---|---|---|
| g1 Pure 3D-JPS | | | |  |
| g2 Pure VO | | | | N/A |
| g3 One-way JPS→VO | | | |  |
| g4 Const λ 0.5 | | | |  |
| g5 No Γ | | | |  |
| g6 Full (proposed) | | | |  |

### T3.6 `M1_three_way` (multi_dynamic, timeout 80 s)

| Group | n valid | Success (goal) k/n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 path length [m] | M3 max turn [deg] | M4 avoidance success k/n (Wilson 95 % CI) | M5 mean [m] | M5 max [m] | M6 flip-backs |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | |  | | |  |
| g2 Pure VO | | | | | | | |  | | | N/A |
| g3 One-way JPS→VO | | | | | | | |  | | |  |
| g4 Const λ 0.5 | | | | | | | |  | | |  |
| g5 No Γ | | | | | | | |  | | |  |
| g6 Full (proposed) | | | | | | | |  | | |  |

Medians [Q1, Q3] for M2, M3, M5 mean, M6:

| Group | M2 median [IQR] | M3 median [IQR] | M5 mean median [IQR] | M6 median [IQR] |
|---|---|---|---|---|
| g1 Pure 3D-JPS | | | |  |
| g2 Pure VO | | | | N/A |
| g3 One-way JPS→VO | | | |  |
| g4 Const λ 0.5 | | | |  |
| g5 No Γ | | | |  |
| g6 Full (proposed) | | | |  |

---

## T4. Stall-breaker / hold override per variant and group (mandatory, spec §8)

`override_cycles = stall_breaks + stall_hold_cycles`; `override_share = override_cycles / cycles`.
"Success w/o trigger" = success with `stall_breaks = 0`.

### T4.1 `S1_east`

| Group | n valid | Σ stall_breaks | stall_breaks mean ± std | Σ stall_hold_cycles | stall_hold_cycles mean ± std | runs with ≥ 1 trigger (k/n) | mean override_share [%] | successes that needed ≥ 1 trigger (k / successes) | success w/o trigger k/n | end F-2 / F-3 with trigger |
|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | | | | |
| g2 Pure VO | | | | | | | | | | |
| g3 One-way JPS→VO | | | | | | | | | | |
| g4 Const λ 0.5 | | | | | | | | | | |
| g5 No Γ | | | | | | | | | | |
| g6 Full (proposed) | | | | | | | | | | |

### T4.2 `S2_north`

| Group | n valid | Σ stall_breaks | stall_breaks mean ± std | Σ stall_hold_cycles | stall_hold_cycles mean ± std | runs with ≥ 1 trigger (k/n) | mean override_share [%] | successes that needed ≥ 1 trigger (k / successes) | success w/o trigger k/n | end F-2 / F-3 with trigger |
|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | | | | |
| g2 Pure VO | | | | | | | | | | |
| g3 One-way JPS→VO | | | | | | | | | | |
| g4 Const λ 0.5 | | | | | | | | | | |
| g5 No Γ | | | | | | | | | | |
| g6 Full (proposed) | | | | | | | | | | |

### T4.3 `S3_climb_ne`

| Group | n valid | Σ stall_breaks | stall_breaks mean ± std | Σ stall_hold_cycles | stall_hold_cycles mean ± std | runs with ≥ 1 trigger (k/n) | mean override_share [%] | successes that needed ≥ 1 trigger (k / successes) | success w/o trigger k/n | end F-2 / F-3 with trigger |
|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | | | | |
| g2 Pure VO | | | | | | | | | | |
| g3 One-way JPS→VO | | | | | | | | | | |
| g4 Const λ 0.5 | | | | | | | | | | |
| g5 No Γ | | | | | | | | | | |
| g6 Full (proposed) | | | | | | | | | | |

### T4.4 `head_on`

| Group | n valid | Σ stall_breaks | stall_breaks mean ± std | Σ stall_hold_cycles | stall_hold_cycles mean ± std | runs with ≥ 1 trigger (k/n) | mean override_share [%] | successes that needed ≥ 1 trigger (k / successes) | success w/o trigger k/n | end F-2 / F-3 with trigger |
|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | | | | |
| g2 Pure VO | | | | | | | | | | |
| g3 One-way JPS→VO | | | | | | | | | | |
| g4 Const λ 0.5 | | | | | | | | | | |
| g5 No Γ | | | | | | | | | | |
| g6 Full (proposed) | | | | | | | | | | |

### T4.5 `crossing_90`

| Group | n valid | Σ stall_breaks | stall_breaks mean ± std | Σ stall_hold_cycles | stall_hold_cycles mean ± std | runs with ≥ 1 trigger (k/n) | mean override_share [%] | successes that needed ≥ 1 trigger (k / successes) | success w/o trigger k/n | end F-2 / F-3 with trigger |
|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | | | | |
| g2 Pure VO | | | | | | | | | | |
| g3 One-way JPS→VO | | | | | | | | | | |
| g4 Const λ 0.5 | | | | | | | | | | |
| g5 No Γ | | | | | | | | | | |
| g6 Full (proposed) | | | | | | | | | | |

### T4.6 `M1_three_way`

| Group | n valid | Σ stall_breaks | stall_breaks mean ± std | Σ stall_hold_cycles | stall_hold_cycles mean ± std | runs with ≥ 1 trigger (k/n) | mean override_share [%] | successes that needed ≥ 1 trigger (k / successes) | success w/o trigger k/n | end F-2 / F-3 with trigger |
|---|---|---|---|---|---|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | | | | | | | |
| g2 Pure VO | | | | | | | | | | |
| g3 One-way JPS→VO | | | | | | | | | | |
| g4 Const λ 0.5 | | | | | | | | | | |
| g5 No Γ | | | | | | | | | | |
| g6 Full (proposed) | | | | | | | | | | |

### T4.7 Campaign totals per group (all variants)

| Group | Σ stall_breaks | Σ stall_hold_cycles | runs with ≥ 1 trigger (k / valid runs) | mean override_share [%] |
|---|---|---|---|---|
| g1 Pure 3D-JPS | | | | |
| g2 Pure VO | | | | |
| g3 One-way JPS→VO | | | | |
| g4 Const λ 0.5 | | | | |
| g5 No Γ | | | | |
| g6 Full (proposed) | | | | |

---

## T5. Paired comparisons g6 vs gX (seeds paired; Holm–Bonferroni over the 5 comparisons per metric and variant)

Wilcoxon signed-rank for M1 mean, M2, M3, M5 mean, M6; exact McNemar for M4. Effect = median paired difference
(g6 − gX) with a bootstrap 95 % CI. n pairs = pairs with both sides valid.

### T5.1 `S1_east`

| Metric | vs g1: n pairs / effect [CI] / p_adj | vs g2 | vs g3 | vs g4 | vs g5 |
|---|---|---|---|---|---|
| M1 mean [ms] | |  | | | |
| M2 [m] | |  | | | |
| M3 [deg] | |  | | | |
| M5 mean [m] | |  | | | |
| M6 | | N/A | | | |

### T5.2 `S2_north`

| Metric | vs g1: n pairs / effect [CI] / p_adj | vs g2 | vs g3 | vs g4 | vs g5 |
|---|---|---|---|---|---|
| M1 mean [ms] | |  | | | |
| M2 [m] | |  | | | |
| M3 [deg] | |  | | | |
| M5 mean [m] | |  | | | |
| M6 | | N/A | | | |

### T5.3 `S3_climb_ne`

| Metric | vs g1: n pairs / effect [CI] / p_adj | vs g2 | vs g3 | vs g4 | vs g5 |
|---|---|---|---|---|---|
| M1 mean [ms] | |  | | | |
| M2 [m] | |  | | | |
| M3 [deg] | |  | | | |
| M5 mean [m] | |  | | | |
| M6 | | N/A | | | |

### T5.4 `head_on`

| Metric | vs g1: n pairs / effect [CI] / p_adj | vs g2 | vs g3 | vs g4 | vs g5 |
|---|---|---|---|---|---|
| M1 mean [ms] | |  | | | |
| M2 [m] | |  | | | |
| M3 [deg] | |  | | | |
| M4 (McNemar) | |  | | | |
| M4 w/o trigger (sensitivity, spec §8.5) | |  | | | |
| M5 mean [m] | |  | | | |
| M6 | | N/A | | | |

### T5.5 `crossing_90`

| Metric | vs g1: n pairs / effect [CI] / p_adj | vs g2 | vs g3 | vs g4 | vs g5 |
|---|---|---|---|---|---|
| M1 mean [ms] | |  | | | |
| M2 [m] | |  | | | |
| M3 [deg] | |  | | | |
| M4 (McNemar) | |  | | | |
| M4 w/o trigger (sensitivity, spec §8.5) | |  | | | |
| M5 mean [m] | |  | | | |
| M6 | | N/A | | | |

### T5.6 `M1_three_way`

| Metric | vs g1: n pairs / effect [CI] / p_adj | vs g2 | vs g3 | vs g4 | vs g5 |
|---|---|---|---|---|---|
| M1 mean [ms] | |  | | | |
| M2 [m] | |  | | | |
| M3 [deg] | |  | | | |
| M4 (McNemar) | |  | | | |
| M4 w/o trigger (sensitivity, spec §8.5) | |  | | | |
| M5 mean [m] | |  | | | |
| M6 | | N/A | | | |

---

## T6. Invalid runs and exclusions

| variant | group | seed | attempt | invalid code (I-0 … I-11) | evidence (file / line / value) | rerun attempt # | final status (valid / excluded) |
|---|---|---|---|---|---|---|---|
| | | | | | | | |

Per-group counts:

| Group | invalid attempts | excluded cells |
|---|---|---|
| g1 Pure 3D-JPS | | |
| g2 Pure VO | | |
| g3 One-way JPS→VO | | |
| g4 Const λ 0.5 | | |
| g5 No Γ | | |
| g6 Full (proposed) | | |

## T7. Mechanism evidence (dynamic variants; spec §10)

| variant | group | runs with C2 re-search (`risk_Dk`/`kappa`) before CPA (k/n) | median lead time before CPA [s] | share of cycles with s_g* > 0 [%] | median diag_path_switches |
|---|---|---|---|---|---|
| head_on | g1 | | | | |
| head_on | g2 | | | | |
| head_on | g3 | | | | |
| head_on | g4 | | | | |
| head_on | g5 | | | | |
| head_on | g6 | | | | |
| crossing_90 | g1 | | | | |
| crossing_90 | g2 | | | | |
| crossing_90 | g3 | | | | |
| crossing_90 | g4 | | | | |
| crossing_90 | g5 | | | | |
| crossing_90 | g6 | | | | |
| M1_three_way | g1 | | | | |
| M1_three_way | g2 | | | | |
| M1_three_way | g3 | | | | |
| M1_three_way | g4 | | | | |
| M1_three_way | g5 | | | | |
| M1_three_way | g6 | | | | |

## T8. External reference: ego-planner baseline (`~/Prometheus_baseline`, ground_height 0.8; not part of T5)

| variant | n valid | success k/n | M2 [m] | M3 [deg] | M4 k/n | min dyn surface dist [m] | notes |
|---|---|---|---|---|---|---|---|
| S1_east | | | | | N/A | N/A | |
| S2_north | | | | | N/A | N/A | |
| S3_climb_ne | | | | | N/A | N/A | |
| head_on | | | | |  |  | |
| crossing_90 | | | | |  |  | |
| M1_three_way | | | | |  |  | |

## T9. Anomalies and notes

| date/time | variant | group | seed | observation | action (none / rerun I-x / note in limitations) |
|---|---|---|---|---|---|
| | | | | | |
