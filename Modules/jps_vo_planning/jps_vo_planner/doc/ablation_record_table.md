# ablation_record_table.md

**[File name]** `doc/ablation_record_table.md`
**[File purpose]** Blank record tables for the ablation experiment, filled in by hand with measured values
after the Prometheus simulation runs. Every cell is intentionally empty. Metric definitions (M1–M6), validity
rules and aggregation follow `subtask4_ablation_experiment_spec.md` §4–6 and `experiment_run_notes.md`.

Notation:
- Groups: ① pure 3D-JPS · ② pure VO · ③ one-way series JPS→VO · ④ coupling without adaptive λ(ξ) ·
  ⑤ coupling without Γ · ⑥ full C1/C2/C3.
- "mean ± std (median)" over valid runs. n = number of valid runs (target 20).

---

## T0 Environment record (one per experiment campaign)

| Item | Value |
|---|---|
| Date / operator | |
| Machine (CPU, RAM, GPU) | |
| OS / ROS / Gazebo / PX4 version | |
| Prometheus commit (jps_vo package) | |
| Baseline worktree commit (if Q5) | |
| Stash / uncommitted changes present? | |
| Mean Gazebo real-time factor (campaign) | |
| Notes | |

## T1 Constants record (fill before the first run; frozen afterwards)

| Constant | Value | Constant | Value |
|---|---|---|---|
| v_max | | ρ_g | |
| a_max | | ε_w | |
| d_min | | T_c | |
| τ₁ (eq. 32) | | T_g (declared) | |
| τ₂ (eq. 32) | | T_g (measured worst case, ③-i) | |
| r_q | | Δv* | |
| e_cut | | ψ_max | |
| r_infl | | θ_max | |
| v_c | | v_min | |
| Δt | | w_θ | |
| τ_h | | w_a | |
| m | | L_min | |
| G_θ (numerical estimate, ③-e) | | ρ_g > G_θ ? | |
| Tuning protocol (P0 / P1) | | P1 score definition (if P1) | |

## T2 Per-run raw record (copy one block per group × scenario variant: 6 × 6 = 36 blocks)

Group: ___ Scenario file: ___ Variant: ___ d_min: ___

| Seed | Valid? | Invalid reason (I-x) / attempt | End reason (goal / F-x) | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 [m] | M3 [deg] | M4 success (1/0) | Min dyn. surface dist, odom [m] | Min static surface dist, odom [m] | M5 mean [m] | M5 max [m] | M6 [count] | Diag: path changes (by reason) | Diag: VO side flips | Diag: fallback cycles | Diag: encounter_flag | Flagged extreme? | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | | | | | | | | | | | | | | | | | | | | |
| 2 | | | | | | | | | | | | | | | | | | | | |
| 3 | | | | | | | | | | | | | | | | | | | | |
| 4 | | | | | | | | | | | | | | | | | | | | |
| 5 | | | | | | | | | | | | | | | | | | | | |
| 6 | | | | | | | | | | | | | | | | | | | | |
| 7 | | | | | | | | | | | | | | | | | | | | |
| 8 | | | | | | | | | | | | | | | | | | | | |
| 9 | | | | | | | | | | | | | | | | | | | | |
| 10 | | | | | | | | | | | | | | | | | | | | |
| 11 | | | | | | | | | | | | | | | | | | | | |
| 12 | | | | | | | | | | | | | | | | | | | | |
| 13 | | | | | | | | | | | | | | | | | | | | |
| 14 | | | | | | | | | | | | | | | | | | | | |
| 15 | | | | | | | | | | | | | | | | | | | | |
| 16 | | | | | | | | | | | | | | | | | | | | |
| 17 | | | | | | | | | | | | | | | | | | | | |
| 18 | | | | | | | | | | | | | | | | | | | | |
| 19 | | | | | | | | | | | | | | | | | | | | |
| 20 | | | | | | | | | | | | | | | | | | | | |

M4 is "N/A" in static_only; static contact still makes the run a failure (F-1). M6 is "N/A" for group ②.

---

## T3 Summary per scenario variant (6 tables, one per variant)

### T3-S1 static_only / S1_east

| Group | n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 [m] | M3 [deg] | M4 [%] | M5 mean [m] | M5 max [m] | M6 [count] | Goal reached [x/n] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ① pure 3D-JPS | | | | | | | N/A | | | | |
| ② pure VO | | | | | | | N/A | | | N/A | |
| ③ series JPS→VO | | | | | | | N/A | | | | |
| ④ no adaptive λ | | | | | | | N/A | | | | |
| ⑤ no Γ | | | | | | | N/A | | | | |
| ⑥ full C1/C2/C3 | | | | | | | N/A | | | | |

### T3-S2 static_only / S2_north

| Group | n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 [m] | M3 [deg] | M4 [%] | M5 mean [m] | M5 max [m] | M6 [count] | Goal reached [x/n] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ① pure 3D-JPS | | | | | | | N/A | | | | |
| ② pure VO | | | | | | | N/A | | | N/A | |
| ③ series JPS→VO | | | | | | | N/A | | | | |
| ④ no adaptive λ | | | | | | | N/A | | | | |
| ⑤ no Γ | | | | | | | N/A | | | | |
| ⑥ full C1/C2/C3 | | | | | | | N/A | | | | |

### T3-S3 static_only / S3_climb_ne

| Group | n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 [m] | M3 [deg] | M4 [%] | M5 mean [m] | M5 max [m] | M6 [count] | Goal reached [x/n] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ① pure 3D-JPS | | | | | | | N/A | | | | |
| ② pure VO | | | | | | | N/A | | | N/A | |
| ③ series JPS→VO | | | | | | | N/A | | | | |
| ④ no adaptive λ | | | | | | | N/A | | | | |
| ⑤ no Γ | | | | | | | N/A | | | | |
| ⑥ full C1/C2/C3 | | | | | | | N/A | | | | |

### T3-D1 single_dynamic / head_on

| Group | n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 [m] | M3 [deg] | M4 [%] (Wilson 95 % CI) | M5 mean [m] | M5 max [m] | M6 [count] | Min dyn. surface dist, odom [m] (median) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ① pure 3D-JPS | | | | | | | | | | | |
| ② pure VO | | | | | | | | | | N/A | |
| ③ series JPS→VO | | | | | | | | | | | |
| ④ no adaptive λ | | | | | | | | | | | |
| ⑤ no Γ | | | | | | | | | | | |
| ⑥ full C1/C2/C3 | | | | | | | | | | | |

### T3-D2 single_dynamic / crossing_90

| Group | n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 [m] | M3 [deg] | M4 [%] (Wilson 95 % CI) | M5 mean [m] | M5 max [m] | M6 [count] | Min dyn. surface dist, odom [m] (median) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ① pure 3D-JPS | | | | | | | | | | | |
| ② pure VO | | | | | | | | | | N/A | |
| ③ series JPS→VO | | | | | | | | | | | |
| ④ no adaptive λ | | | | | | | | | | | |
| ⑤ no Γ | | | | | | | | | | | |
| ⑥ full C1/C2/C3 | | | | | | | | | | | |

### T3-D3 multi_dynamic / M1_three_way

| Group | n | M1 mean [ms] | M1 P95 [ms] | M1 max [ms] | M2 [m] | M3 [deg] | M4 [%] (Wilson 95 % CI) | M5 mean [m] | M5 max [m] | M6 [count] | Min dyn. surface dist, odom [m] (median) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ① pure 3D-JPS | | | | | | | | | | | |
| ② pure VO | | | | | | | | | | N/A | |
| ③ series JPS→VO | | | | | | | | | | | |
| ④ no adaptive λ | | | | | | | | | | | |
| ⑤ no Γ | | | | | | | | | | | |
| ⑥ full C1/C2/C3 | | | | | | | | | | | |

---

## T4 Paired tests: ⑥ vs each group (one table per scenario variant)

Variant: ___

| Comparison | M1 Δmedian [ms] (95 % CI) / p_Holm | M2 Δmedian [m] / p_Holm | M3 Δmedian [deg] / p_Holm | M4 Δsuccess [pp] / McNemar p_Holm | M5 Δmedian [m] / p_Holm | M6 Δmedian / p_Holm | n pairs |
|---|---|---|---|---|---|---|---|
| ⑥ vs ① | | | | | | | |
| ⑥ vs ② | | | | | | N/A | |
| ⑥ vs ③ | | | | | | | |
| ⑥ vs ④ | | | | | | | |
| ⑥ vs ⑤ | | | | | | | |

## T5 Mechanism evidence (dynamic variants, groups ③ and ⑥; IP2/IP3)

| Group | Variant | Seed | Global path changes before closest approach (reason) | Time of switch − time of CPA [s] | Cycles with s_g* > 0 [%] | s_g* > 0 only when cone ∩ F = ∅? (③-e) | κ max | VO side flips | Fallback cycles |
|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | |
| | | | | | | | | | |
| | | | | | | | | | |

## T6 Invalid-run and rerun log

| Group | Variant | Seed | Attempt | Invalid code (I-x) | Evidence (log line / file) | Rerun result |
|---|---|---|---|---|---|---|
| | | | | | | |
| | | | | | | |
| | | | | | | |

Summary: invalid runs per group ① __ ② __ ③ __ ④ __ ⑤ __ ⑥ __ (report in the thesis).

## T7 d_min sensitivity (optional, identical for all groups; one table per metric)

Metric: ___ Variant: ___

| Group | d_min = 0.1 | d_min = 0.2 | d_min = 0.3 |
|---|---|---|---|
| ① | | | |
| ② | | | |
| ③ | | | |
| ④ | | | |
| ⑤ | | | |
| ⑥ | | | |

## T8 Observed defects for the limitations section (no tuning response)

| Observation | Group(s) | Variant / seed | Frequency (runs) | Related Stage-2 risk (R1–R12) | Evidence file | Text for the limitations section |
|---|---|---|---|---|---|---|
| | | | | | | |
| | | | | | | |
