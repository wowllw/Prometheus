# subtask4_ablation_experiment_spec.md

**[File name]** `doc/subtask4_ablation_experiment_spec.md`
**[File purpose]** The complete ablation experiment specification. It defines:
- the 6 control groups and the parameter switches for each;
- the 6 fixed evaluation metrics;
- the execution constraints: equal tuning budget, 20 repetitions, means, run validity;
- the log CSV contract, which reuses the Subtask-3 fields;
- how the experiment shows the gain comes from the coupling structure;
- the writing points for the 3 innovation points.

The document is a template. It contains no results.

Related files:
- `doc/ablation_record_table.md`: blank record tables.
- `doc/experiment_run_notes.md`: procedure, failure and invalid-run rules, preprocessing.
- `config/scenarios/scenario_{static_only,single_dynamic,multi_dynamic}.yaml`: scenario configurations.
- `scripts/check_scenarios.py`: offline geometric check of the scenarios.

---

## 1. Prerequisites (gate before the first ablation run)

1. Subtask-3 steps ①②③ must be implemented and every verification row in `doc/subtask3_porting_plan.md` must
   pass. In particular, ③-a and ③-b (degeneracy) prove that groups ① and ② below really are the λ ≡ 0 and
   λ ≡ 1 limits of the same implementation.
   **At the time of writing, JPS, VO and the coupling are still pseudocode. No ablation run can start yet.**
2. `check_scenarios.py` returns exit 0 for all three scenario files at the chosen d_min.
3. The constants in §2 are entered into `ablation_record_table.md` table T1 before the first run and are not
   changed afterwards.

## 2. Constants (frozen for the whole experiment)

Fixed by the user, identical for all 6 groups:

| r_q | e_cut | v_c | Δt | τ_h | m | L_min | ψ_max | θ_max | v_min | w_θ | w_a | ρ_g | ε_w | T_c | T_g | Δv* |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.30 m | 0.10 m | 0.80 m/s | 0.10 s | 4.0 s | 1.20 m | 2.0 m | 60° | 60° | 0.10 m/s | 1.0 | 1.0 | 10.0 | 0.05 | 0.80 s | 1.50 s | 1.60 m/s |

Calibratable, identical for all groups:
- v_max = 1.0 m/s (confirmed 2026-10-26, = PX4 COMMAND cap) with cruise speed v_c = 0.8 m/s, so the
  planned speed is bounded by v_c (C-v); a_max = 0.8 m/s² (ego baseline value, under the 2.0 m/s² cap).
- d_min = **0.2 m** (Q1, confirmed 2026-10-26): r_infl 0.60 m, τ₁ 2.214 s, τ₂ 12.043 s.

Derived (eq. 1, 27, 32; r_o = 0.3 m for every dynamic obstacle; computed, not tuned):

| d_min | r_infl = r_q + d_min + e_cut | R_o = R_max = r_q + r_o + d_min | τ₁ = T_c + √(2R_max/a_max) | τ₁ + T_g | τ₁·exp(15Δv*/(8 a_max τ₁)) | **τ₂** |
|---|---|---|---|---|---|---|
| 0.1 | 0.50 m | 0.70 m | 2.123 s | 3.623 s | 12.419 s | 12.419 s |
| 0.2 | 0.60 m | 0.80 m | 2.214 s | 3.714 s | 12.043 s | 12.043 s |
| 0.3 | 0.70 m | 0.90 m | 2.300 s | 3.800 s | 11.744 s | 11.744 s |

- ℓ_w = ψ_max·v_c²/a_max = 0.838 m.
- L_s = max(‖p_g − p₀‖, 2.0).
- The implicit per-step turn bound from (C-a) at cruise speed is asin(a_max·Δt/v_c) = 5.74°, so θ_max = 60°
  only binds at low speed (Stage-2 §4.1 remark).
- `m` is the length margin of eq. (20), in metres.
- ρ_g = 10 must satisfy ρ_g > G_θ. This is checked numerically by verification ③-e and is recorded in T1.
  If ③-e fails, stop and report; do not raise ρ_g silently.

## 3. Control groups and switch configuration

All groups run **the same node binary** with `planner_mode: coupled`. They differ **only** in the switches
below (switch names from `subtask3_porting_plan.md` §5). Each group therefore differs from group ⑥ in exactly
the listed component, which is the nested-model principle of Stage-2 §5.2.

| # | Group | Stage-2 ID | Switches (everything not listed = group ⑥ default) | Component removed relative to ⑥ |
|---|---|---|---|---|
| ① | Pure 3D-JPS | A1 | `lambda_mode: zero` | VO entirely (λ ≡ 0 ⇒ w_e = ℓ_e/L_s, Γ = 0, κ = 0, θ_c = 0, dynamic part of (C-s) off; §4.2(a)) |
| ② | Pure VO | A2 | `lambda_mode: one`, `use_rho_in_pure_vo: false` | global layer in the local decision (λ ≡ 1 ⇒ J_A drops out, θ_c = π, Γ = 0; eq. (16)) |
| ③ | One-way series JPS → VO | A10 | `enable_c2: false`, `enable_c3: false` | both coupling channels: no risk/κ in the edge weights, no D_k re-expansion, no guidance cone |
| ④ | Coupling without adaptive λ(ξ) | A3 | `lambda_mode: const:0.5` | adaptivity of λ (Γ factor λ(1−λ) then constant 0.25) |
| ⑤ | Coupling without Γ | A7 | `enable_gamma: false` | coupling penalty Γ (34)(35) |
| ⑥ | Full C1/C2/C3 (proposed) | — | `lambda_mode: adaptive`, `enable_c2: true`, `enable_c3: true`, `enable_gamma: true`, `lambda_shape: quintic_log`, `theta_c_mode: linear`, `xi_source: reference`, `c3_soft_tracking: 0` | — |

Group ⑥ defaults as a launch-parameter block (the other groups override only their row above):

```yaml
planner_mode: coupled
lambda_mode: adaptive          # zero | one | const:<c> | adaptive
use_rho_in_pure_vo: false
enable_c2: true
enable_c3: true
c3_soft_tracking: 0.0          # 0 = guidance cone (C-g); > 0 = A6 soft tracking (not one of the 6 groups)
enable_gamma: true
lambda_shape: quintic_log
theta_c_mode: linear
xi_source: reference
```

Notes on the definitions (to be stated in the thesis):
- ② uses eq. (16), the classic VO: argmin over F of ‖v − v_pref‖, with v_pref pointing at the goal. The
  smooth-TTC variant (17), "② with ρ", is not one of the 6 groups.
- ③ is defined as Stage-2 A10: the local layer still minimises J_total with J_A = V_Π(q(v)), so the global path
  enters only through the cost-to-go. No information flows from local to global, and the global tangent is not
  a constraint. This is the "one-way series" of Stage-1 table C.
- ④ uses the constant λ = 0.5, the value at which Γ is largest (eq. 33).
- With 6 groups, **C2 and C3 cannot be separated** (③ removes both). Single-channel evidence needs A4 (no C2)
  and A5 (no C3); see §10, Q2.

## 4. Fixed evaluation metrics (definitions are final and are not changed during the experiment)

**Window:** from the goal publish time t₀ to t_end, where t_end is goal reached, failure or timeout
(`experiment_run_notes.md` P-2).
**Signals:** following the Subtask-2 decision, metrics use the **trajectory reference** (p_ref, v_ref), except
safety distances, which must describe the real vehicle and use odom together with the Gazebo ground truth.

| ID | Metric | Definition | Source fields | Per-run value | Group aggregate |
|---|---|---|---|---|---|
| M1 | Single-step planning time [ms] | wall time of one planning cycle k: `plan_ms_local(k) + plan_ms_global(k)` (the global term is 0 in cycles without a global search) | `plan_ms_local`, `plan_ms_global` | mean, P95 and max over cycles in the window | mean of per-run means; mean of per-run P95; max of per-run max |
| M2 | Total path length [m] | Σ_k ‖p_ref(k) − p_ref(k−1)‖ over the window | `p_ref_*` | one value | mean ± std, median [IQR] |
| M3 | Maximum turning angle [deg] | max over k of ∠(v_ref(t_k), v_ref(t_k − 1.0 s)), only where both speeds ≥ v_min (1 s window, §10 Q4) | `v_ref_*` | one value | mean ± std, median [IQR] |
| M4 | Dynamic-obstacle avoidance success rate [%] | success = (goal reached within timeout) ∧ (min over window of the surface distance UAV–dynamic obstacle, odom + ground truth, > 0) ∧ (no static contact: surface distance UAV–cylinder > 0) | `min_surface_dist_odom`, offline static check (run notes P-6) | 1 / 0 | successes / valid runs, Wilson 95 % CI. Only dynamic scenarios; N/A in static_only |
| M5 | Deviation from the global path [m] | lateral distance from p_ref(k) to the initial global path Π₀ (the path planned at t₀, identical start/goal/map for all groups) | `lateral_dev_ref_to_Pi0` | mean and max over the window | mean ± std of per-run means; mean of per-run max |
| M6 | Path-switching oscillation count | number of flip-backs: a global path change at cycle k (not caused by a goal change) whose new path matches (Hausdorff distance < 0.3 m) a different path that was active within the previous 10 s, i.e. an A → B → A return | `path_hash`, `replan_reason`, path files | integer | mean ± std, median [IQR]. N/A for ② (the global path does not influence the decision when 1 − λ = 0) |

Logged diagnostics. These are not metrics and must not be reported as additional results; they serve only to
explain the metrics:
- total global path changes by reason;
- VO side-flip count (R3: sign changes of the lateral component of v* relative to t_g within 3 s);
- fallback (40) cycles;
- minimum surface distance to dynamic obstacles on the reference;
- realised-encounter flag.

## 5. Run matrix

| Scenario file | Variants | Goal | Dynamic obstacles | Timeout |
|---|---|---|---|---|
| `scenario_static_only.yaml` | S1_east, S2_north, S3_climb_ne | (13,0,1.5), (0,14,1.5), (12,5,2.2) | 0 | 70 s |
| `scenario_single_dynamic.yaml` | head_on, crossing_90 | (0,14,1.5) | 1 sphere r_o = 0.3 m, ‖v_o‖ = 0.5 m/s | 70 s |
| `scenario_multi_dynamic.yaml` | M1_three_way | (0,14,1.5) | 3 spheres (patrol at the field exit, right crossing, head-on) | 80 s |

- 6 groups × 6 variants × 20 seeds = **720 valid runs**, plus reruns of invalid runs.
- Evaluation seeds: **1–20** (the same seeds for every group, so the comparison is paired).
- Tuning seeds, used only if protocol P1 is invoked: **101–110**. They are disjoint from the evaluation seeds.
- Rough duration: about 3 min per run including the full simulation restart, so about 36 h of simulation
  (not counting reruns).

## 6. Execution constraints

### 6.1 Equal tuning budget
- **P0 (main experiment): zero tuning for every group.** All groups use the identical constants of §2 and the
  τ₁, τ₂ derived by eq. (32). No group, including ⑥, gets any parameter search. This is the strongest guard
  against tuning effects, and it is the protocol for every number reported in the thesis results tables.
- **P1 (only if a calibratable parameter must be changed, e.g. d_min):**
  - every group receives the same budget: N = 6 configurations, drawn with the same method (fixed list or
    Latin hypercube with a fixed seed) from the same range;
  - evaluation uses tuning seeds 101–110 only;
  - the selection score is declared in T1 **before** any P1 run;
  - each group is then evaluated on seeds 1–20 with its own selected configuration;
  - the log of all P1 trials (every configuration, every score) goes into the appendix.
- **Sensitivity (A14-style, optional):** a d_min sweep {0.1, 0.2, 0.3} applied identically to all groups and
  reported as curves. A gain that appears at only one d_min value is reported as such.
- After the first evaluation run, the constants of §2 are never changed. If a defect appears, it is written up
  as a limitation; no retuning.

### 6.2 Repetitions and aggregation
- 20 valid runs per (group, variant). Report mean ± std and median [IQR] for M1–M3, M5 and M6, and the success
  rate with its Wilson 95 % CI for M4. Always also report n_valid.
- Paired tests between ⑥ and each other group on the same seeds:
  - Wilcoxon signed-rank for M1–M3, M5 and M6;
  - exact McNemar for M4;
  - Holm–Bonferroni correction over the 5 comparisons per metric and scenario;
  - effect size: median paired difference with a bootstrap 95 % CI.
- 20 runs give limited power. Non-significant differences are reported as "no evidence of difference", not as
  "equal".

### 6.3 Invalid runs and outliers
- Runs are removed **only** for infrastructure reasons (run notes §3: I-1 … I-9). Removed runs are rerun with
  the same seed, up to 3 attempts. Every removal and its reason is recorded in T6, and removal counts are
  reported per group.
- A collision, timeout, stall, fallback or crash caused by the planner is a **failure (valid run)**, never an
  outlier.
- **No metric-based outlier removal.** Extreme values beyond Q3 + 3·IQR are flagged in the raw table and
  discussed. They stay in the mean, and the median is reported next to the mean so the reader can see how much
  they matter.

## 7. How the experiment shows the gain comes from the coupling structure

1. **Nested models:** every comparison ⑥ vs ③/④/⑤ changes exactly one coupling component, with identical
   binary, constants, scenarios and seeds. A difference can only come from that component.
2. **Degenerate ends of one cost function:** ① and ② are the λ ≡ 0 and λ ≡ 1 limits of the same J_total.
   Verification ③-a/③-b proves this numerically before the experiment. ⑥ vs ①/② therefore compares the
   adaptive interior of one model with its own boundaries, not with foreign implementations.
3. **Zero tuning (P0) for all groups:** the proposed method gets no advantage from parameter search.
4. **Paired seeds and identical scenario realisations** (logged seed files): each comparison is made on the
   same encounter.
5. **Mechanism evidence, not only outcomes:** for C2, log that the global path changes for reason `risk` or
   `kappa` **before** the encounter (time of the switch vs time of closest approach). For C3, log s_g* > 0 only
   in cycles where the guidance cone and F do not intersect (③-e). An outcome difference without the matching
   mechanism signature is reported as unexplained.
6. **Robustness:** the d_min sensitivity (6.1), and results per scenario rather than pooled.
7. **Negative results are reported** in the same tables. Example: ⑥ not better than ③ in single_dynamic but
   better in multi_dynamic.

## 8. Log CSV contract (reuses the Subtask-3 fields, `subtask3_porting_plan.md` §4)

One file per run, `cycles.csv`, one row per local cycle (Δt = 0.1 s). Columns in this order:

```
t, mode, plan_ms_global, plan_ms_local,
p_ref_x, p_ref_y, p_ref_z, v_ref_x, v_ref_y, v_ref_z,
p_odom_x, p_odom_y, p_odom_z,
v_star_x, v_star_y, v_star_z,
lambda, xi, s_g_star, kappa,
path_id, path_hash, n_jump_pts,
min_surface_dist_ref, min_surface_dist_odom,
turn_angle, lateral_dev_ref_to_Pi
```

These are the Subtask-3 fields. Vectors are split into _x/_y/_z, and `turn_angle` is the per-cycle
∠(v*, v_prev).

**Extension fields, appended after the Subtask-3 columns** (needed for M5, M6 and the validity rules; §10 Q3):

```
lateral_dev_ref_to_Pi0, replan_reason, fallback_active, nearest_dyn_obs, encounter_flag
```

- `replan_reason` ∈ {none, goal, map, periodic, risk_Dk, kappa}.
- `encounter_flag` = 1 once the realised conflict prediction at trigger time has d_cpa < R_o.

Other files per run:
- `run_meta.yaml`: group, switches, scenario, variant, seed, git commit, constants hash, t₀, t_end, end reason.
- `pi0.csv`: Π₀ jump points.
- `paths.csv`: every new global path, as path_hash plus its jump points.
- `dyn_obstacles_seed<k>.yaml`: written by `dyn_obstacles.py`.
- `odom_gt.csv`: 20 Hz odom and dynamic-obstacle ground truth, for safety distances.
- Optional: `run.bag`.

## 9. Thesis results-analysis writing points (3 innovation points)

The bullets below state **hypotheses** to test, not results. Each point lists which comparison supports it and
how to write it up if the data do not support it.

**IP1: unified-cost mathematical coupling (C1: one J_total, shared ρ, λ(ξ), Γ).**
- *Evidence:* ⑥ vs ① and ② in all scenarios (one cost function, interior vs boundaries); ⑥ vs ④
  (adaptive λ); ⑥ vs ⑤ (Γ).
- *Hypotheses:*
  - H1a: ① fails M4 in head_on and multi (no dynamic avoidance).
  - H1b: ② has larger M2/M5 and possibly fails static S1–S3 (local minima, goal-directed v_pref).
  - H1c: ⑥ reaches the M4 of the better boundary while keeping M2/M5 near ①.
  - H1d: ④ has larger M3/M5 than ⑥ in the static and early-dynamic phases (λ ≠ 0 when no threat).
  - H1e: ⑤ has larger M3 than ⑥.
- *Write-up:* show M4 and M5 together (safety vs path fidelity), plus a time plot of λ(t) for one
  representative run. The plot must be marked as an illustration and the run chosen by a pre-declared rule,
  e.g. the run with median M2.
- *If unsupported:* report it. For example, "④ ≈ ⑥" means adaptivity gave no measurable gain in these
  scenarios. Discuss scenario coverage as a limitation.

**IP2: local risk written back into the global planner (C2: ρ_e in w_e, κ from s_g*, D_k re-expansion).**
- *Evidence:* ⑥ vs ③ in multi_dynamic (the patrol obstacle at the field exit). The mechanism signature
  (switch reason `risk_Dk`/`kappa` before the encounter) is required. **Single-channel attribution requires
  A4; without it, write "C2 and C3 jointly".**
- *Hypotheses:*
  - H2a: ⑥ re-plans away from the threatened corridor before the encounter; ③ does not.
  - H2b: ⑥ has M4 ≥ ③.
  - H2c: M6 of ⑥ stays low. This is the oscillation risk R5, and it must be reported even if it is high.
- *Write-up:* table of switch times relative to closest approach; M6 next to M4.
- *If unsupported:* if M6 is high, write it into limitations (R5). Do not add a low-pass filter, which would be
  a model change.

**IP3: global tangent as a hard constraint trimming the VO feasible region (C3: guidance cone (28), exact
penalty).**
- *Evidence:* ⑥ vs ③ in single_dynamic and multi_dynamic (with the same caveat: A5 is needed to isolate C3).
- *Hypotheses:*
  - H3a: lower M5 than ③ and ②.
  - H3b: fewer VO side flips (diagnostic) than ②.
  - H3c: s_g* > 0 only where the cone and F are disjoint (③-e), giving κ feedback.
- *Write-up:* M5 distribution; the share of cycles with s_g* > 0 and its timing relative to encounters.
- *If unsupported:* report it. R3 (side switching) is a known VO property that the model does not remove (user
  decision 3).

**Limitations to write in all cases** (agreed with the user):
- JPS optimality holds only on G_J (Prop. 1).
- Constant-velocity, ground-truth obstacles (D2, decision Q-VO).
- **Ground-truth masking:** the static grid of this method removes dynamic obstacles and their octomap trails
  using `/gazebo/model_states`, whereas the ego-planner reference relies only on D435i/octomap.
- PX4 tracking lag of about 0.8 s: the planner works on the reference, and safety is measured on odom.
- No closed-loop stability proof (Stage-2 §4.2(d)).
- VO side switching (R3) and corridor flipping (R5) are measured, not removed.
- 20 runs per cell; a single simulated world.

## 10. Decisions (confirmed 2026-10-26)
- **Q1** d_min = 0.2 m.
- **Q2** No A4/A5 groups. The thesis states that C2 and C3 act jointly and are not isolated by the ablation.
- **Q3** The 5 extension CSV fields are approved (implemented in `planner_node_coupled.cpp`).
- **Q4** M3 = maximum turning angle over a 1 s sliding window (implemented in `verify_run.py`).
- **Q5** The clean ego-planner (`~/Prometheus_baseline`, pre-DASC) runs as an external reference, outside the 6
  groups.
- Implementation choices (confirmed 2026-10-26): v_max 1.0 / v_c 0.8; gate true at 10 Hz only in COMMAND with an active
  trajectory; eq. (7) tie rule `strict`; `pre_rotate_yaw` false.
