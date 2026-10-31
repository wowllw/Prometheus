# fix_coupling_artifacts — pipeline report (supplementary analysis, thesis §4.5)

> Chapter 4 reports the frozen baseline `thesis-frozen-v1` and is unchanged. Everything here is a post-campaign engineering patch evaluated as supplementary analysis. g6 vs g3 is the joint C2 + C3 effect only. V0 log diagnostics are correlational clues, not evidence. Unit tests are not closed-loop evidence; only V1–V3 simulations are.

## 1 Git baseline protection

| Item | Value |
|---|---|
| Tag | `thesis-frozen-v1` (annotated, object `b755fb87d0f7`) → commit `130163686c45` |
| Protected base branch | `main`, commit unchanged: `130163686c45` |
| Development branch | `fix_coupling_artifacts` |
| Read-only assets | Chapter-4 doc, frozen 720-cell dataset, frozen analysis outputs: hashes identical at every stage (Gate-I) |
| Uncommitted user files left untouched | `Modules/jps_vo_planning/jps_vo_planner/doc/ablation_result_record_table.md`, `Modules/jps_vo_planning/jps_vo_planner/scripts/__pycache__/post_process_ablation.cpython-38.pyc`, `Modules/jps_vo_planning/jps_vo_planner/scripts/post_process_ablation.py` |

Commits on `fix_coupling_artifacts`:

```
62243d0 Pipeline update (analysis.py)
3f1f042 Stop report: pipeline ended at a protocol gate (supplementary analysis, section 4.5)
70ee6b8 Pipeline update (analysis.py, pipeline.py, report.py)
3742d38 V1: preset v1_r4_r4m (V1 step v1_r4 with R4-m fallback)
328fae4 Pipeline update (pipeline.py, report.py)
4636085 Pipeline update (pipeline.py)
227ffe5 Pipeline update (fixlib.py, pipeline.py, report.py)
066418e Pipeline update (discover.py, pipeline.py)
160e7e8 Add FX1-FX6 coupling repairs behind default-false flags, presets, tests, pipeline
```

Gate stops during the pipeline run, in order (full reports in `/home/amov/.ros/ablation_fix/pipeline`). A stop caused by a defect of the pipeline itself was fixed and the pipeline resumed; a stop by a protocol gate ends the run:

- `GATE_FAILURE_stage0.md` time: 2026-10-31 00:49:38 gate: Gate-0 discovery: role check_scenarios: expected exactly one match, found 3: ['/home/amov/Prometheus/Modules/jps_vo_planning/jps_vo_planner/scripts/check_scenarios.py', '/home/amov/Prometheus/Modules/jps_vo_planning/jps_vo_planner/scripts/fix_pipeline/discover.py', '/home/amov/Prometheus/Modules/jps_vo_planning/jps_v
- `GATE_FAILURE_stage2.md` time: 2026-10-31 01:02:57 gate: Gate-2 existing tests: test_fixes failed:
- `GATE_FAILURE_v1_20261031_011803.md` time: 2026-10-31 01:18:03 gate: Gate-I output: refusing to write outside /home/amov/.ros/ablation_fix: /home/amov/.ros/ablation_fix/v1/diag_only
- `GATE_FAILURE_v1_20261031_014322.md` time: 2026-10-31 01:43:22 gate: unexpected error: Traceback (most recent call last):
- `GATE_FAILURE_v1_20261031_035301.md` time: 2026-10-31 03:53:01 gate: Gate-V1 safety: unsafe after the R4-m fallback at v1_r4

## 2 Build and offline tests

- Existing `test_occupancy_grid`: exit 0, 10 PASS, 0 FAIL.
- Existing `test_jps`: exit 0, 10 PASS, 0 FAIL.
- Existing `test_vo`: exit 0, 3 PASS, 0 FAIL.
- Existing `test_coupling`: exit 0, 18 PASS, 0 FAIL.
- Frozen reference built from `thesis-frozen-v1` with the package flags `-O3 -Wall -g -std=c++14`: 2578995 bytes, sha256 `dfcd7292a6db5ccc`.
- Branch `gen_golden` with every flag at its default (false): identical to the frozen reference.
- Log-format identity (source level): the frozen lines of `openLogs`, `writeCycleRow`, `writePathEvents`, `writeChecks` are unchanged; additions are guarded by the fix flags and never write to `cycles.csv` / `paths.csv`.
- `test_fixes`: 43 PASS, 0 FAIL. Gate-T0: [PASS] T0 all fixes off: dump identical to the frozen reference (2578995 bytes)
  - `[INFO] T3 lambda = 0, R1a+R1b on: v* == frozen ray closed form in 276/276 cases with |v_prev| >= 0.4 and 324/324 with |v_prev| < 0.4`
  - `[INFO] T5 risk source at lateral 2.2 m: route side changes over 200 replans: R4 off 199, R4 on 66`
  - `[INFO] T5 risk source at lateral 5.5 m: route side changes over 200 replans: R4 off 0, R4 on 0`
  - `[INFO] T5b route side changes over 200 replans: R4 33, R4 + R4-m 33; R4-m judged switches: 1 under threat (1 accepted inside the dwell), 116 threat-free`
  - `[INFO] T6 300 random obstacles: frozen-only fires 40, both 49, R3-only 1, neither 210`
  - `[INFO] T6b 61 cases with a remaining-edge increase > eps_w (3 with simultaneous decreases); the signed sum of the first patch version stays <= eps_w in 0 of them`
  - `[INFO] T8 S1: 453 non-fallback problems, risk budget active in 304`
  - `[INFO] T10 R1d: 880 problems with a feasible moving candidate; v* = 0 chosen in 188 of them`
  - `[INFO] T9 offline kinematic loop on planning_easy (60 s cap):`
  - `[INFO] scenario     arm           reached  length_m stop_cycles max_stop_run  switches  fallbacks  occupied`
  - `[INFO] static_east  g3 frozen           1     13.03           1            1         3          2         1`
  - `[INFO] static_east  g6 frozen           1     13.44           1            1         0          2         2`
  - `[INFO] static_east  g6 + R1-R5          1     13.18           1            1         0          6         3`
  - `[INFO] static_ne    g3 frozen           1     10.11           1            1         1          0         0`
  - `[INFO] static_ne    g6 frozen           1     10.55           1            1         0          1         1`
  - `[INFO] static_ne    g6 + R1-R5          1     10.23           1            1         0          0         0`
  - `[INFO] head_on      g3 frozen           1     13.55          42           10         4         15        14`
  - `[INFO] head_on      g6 frozen           0      5.49         438          437         6          0         0`
  - `[INFO] head_on      g6 + R1-R5          1     13.54          10            9         1          4         4`
  - `[INFO] three_way    g3 frozen           1     13.56          14           11         4          7         6`
  - `[INFO] three_way    g6 frozen           1     14.27           1            1         4          2         1`
  - `[INFO] three_way    g6 + R1-R5          1     13.79           1            1         1          2         2`

## 3 V1 smoke ladder (g6; S3_climb_ne, head_on, M1_three_way; development seeds 101–103)

Descriptive only (n = 3 development seeds per cell; no inference). One row per preset that ran, in run order; medians over the three seeds, sums for counts.

| Preset | Variant | ok | M2 [m] | M3 [°] | M6 | Σ stall breaks | low-speed share | path switches | min dyn. clearance [m] | replans (Σ) |
|---|---|---|---|---|---|---|---|---|---|---|
| diag_only | M1_three_way | 3/3 | 16.12 | 80.8 | 2.0 | 0 | 0.003 | 15.0 | 0.785 | goal:3 kappa:10 map:31 periodic:28 risk_Dk:156 |
| diag_only | S3_climb_ne | 3/3 | 15.54 | 135.0 | 0.0 | 8 | 0.006 | 6.0 | — | goal:3 kappa:14 map:18 periodic:89 |
| diag_only | head_on | 3/3 | 15.07 | 69.8 | 2.0 | 0 | 0.008 | 8.0 | 0.644 | goal:3 kappa:15 map:3 periodic:42 risk_Dk:135 |
| v1_r1 | M1_three_way | 3/3 | 15.82 | 90.0 | 2.0 | 1 | 0.160 | 13.0 | 0.664 | goal:3 kappa:4 map:24 periodic:45 risk_Dk:208 |
| v1_r1 | S3_climb_ne | 3/3 | 13.76 | 28.2 | 0.0 | 0 | 0.003 | 4.0 | — | goal:3 kappa:14 map:20 periodic:87 |
| v1_r1 | head_on | 3/3 | 15.00 | 66.9 | 0.0 | 0 | 0.038 | 6.0 | 0.619 | goal:3 kappa:11 map:2 periodic:45 risk_Dk:112 |
| v1_r2 | M1_three_way | 3/3 | 15.80 | 68.7 | 2.0 | 1 | 0.061 | 13.0 | 0.782 | goal:3 kappa:11 map:27 periodic:38 risk_Dk:182 |
| v1_r2 | S3_climb_ne | 3/3 | 13.63 | 27.4 | 0.0 | 0 | 0.016 | 4.0 | — | goal:3 kappa:14 map:21 periodic:74 |
| v1_r2 | head_on | 3/3 | 15.12 | 118.9 | 0.0 | 0 | 0.038 | 5.0 | 0.593 | goal:3 kappa:13 map:7 periodic:40 risk_Dk:137 |
| v1_r3 | M1_three_way | 3/3 | 15.99 | 72.7 | 1.0 | 0 | 0.003 | 13.0 | 0.632 | goal:3 kappa:6 map:26 periodic:28 risk_Dk:197 |
| v1_r3 | S3_climb_ne | 3/3 | 13.71 | 31.5 | 0.0 | 0 | 0.028 | 4.0 | — | goal:3 kappa:9 map:19 periodic:93 |
| v1_r3 | head_on | 3/3 | 15.19 | 62.8 | 1.0 | 0 | 0.004 | 6.0 | 0.837 | goal:3 kappa:19 map:3 periodic:43 risk_Dk:125 |
| v1_r4 | M1_three_way | 3/3 | 15.91 | 65.1 | 0.0 | 1 | 0.013 | 9.0 | 0.800 | goal:3 kappa:9 map:27 periodic:34 risk_Dk:222 |
| v1_r4 | S3_climb_ne | 3/3 | 13.72 | 33.7 | 0.0 | 0 | 0.057 | 3.0 | — | goal:3 kappa:18 map:16 periodic:81 |
| v1_r4 | head_on | 3/3 | 15.11 | 54.3 | 0.0 | 0 | 0.004 | 4.0 | 0.536 | goal:3 kappa:11 map:5 periodic:42 risk_Dk:148 |
| v1_r4_r4m | M1_three_way | 2/3 | 16.05 | 59.9 | 0.0 | 16 | 0.184 | 8.0 | 0.332 | goal:3 kappa:2 map:16 periodic:42 risk_Dk:207 |
| v1_r4_r4m | S3_climb_ne | 3/3 | 13.58 | 34.8 | 0.0 | 0 | 0.004 | 1.0 | — | goal:3 kappa:3 map:6 periodic:82 |
| v1_r4_r4m | head_on | 3/3 | 15.41 | 63.0 | 0.0 | 0 | 0.004 | 2.0 | 0.635 | goal:3 kappa:14 map:4 periodic:44 risk_Dk:110 |

Gate outcome per ladder step:

- diag_only (diag_only, units —): unsafe runs 0, avoidance delays 0.
- v1_r1 (v1_r1, units FX1): unsafe runs 0, avoidance delays 0.
- v1_r2 (v1_r2, units FX1+FX2): unsafe runs 0, avoidance delays 0.
- v1_r3 (v1_r3, units FX1+FX2+FX3): unsafe runs 0, avoidance delays 0.
- Switch adjustments made by the V1 gates: step v1_r4: avoidance delayed by the dwell -> r4m_dwell_free_only enabled

### Gate stop in V1

`GATE_FAILURE_v1_20261031_035301.md` — Gate-V1 safety: unsafe after the R4-m fallback at v1_r4

r4_delays:

- `{'cell': ['head_on', 'g6', '101'], 't': 1793444696.86, 'window_min_dyn': 0.555, 'reference_min_dyn': 0.936}`
- `{'cell': ['head_on', 'g6', '101'], 't': 1793444696.96, 'window_min_dyn': 0.569, 'reference_min_dyn': 0.936}`
- `{'cell': ['head_on', 'g6', '101'], 't': 1793444697.56, 'window_min_dyn': 0.726, 'reference_min_dyn': 0.936}`
- `{'cell': ['head_on', 'g6', '101'], 't': 1793444697.66, 'window_min_dyn': 0.761, 'reference_min_dyn': 0.936}`
- `{'cell': ['head_on', 'g6', '102'], 't': 1793445200.021, 'window_min_dyn': 0.592, 'reference_min_dyn': 0.698}`
- `{'cell': ['head_on', 'g6', '102'], 't': 1793445200.121, 'window_min_dyn': 0.601, 'reference_min_dyn': 0.698}`
- `{'cell': ['head_on', 'g6', '102'], 't': 1793445200.221, 'window_min_dyn': 0.615, 'reference_min_dyn': 0.698}`

unsafe_after_r4m:

- `{'cell': ['M1_three_way', 'g6', '102'], 'end_reason': 'timeout (F-2)', 'M4': '0', 'log_dir': '/home/amov/.ros/ablation_fix/v1/v1_r4_r4m/M1_three_way/g6/seed102_a1'}`

Timeline of `/home/amov/.ros/ablation_fix/v1/v1_r4_r4m/M1_three_way/g6/seed102_a1` (generated from cycles.csv / cycles_ext.csv):

- goal [0, 14, 1.5]; 822 cycles from t0 + 3.1 s to t0 + 86.4 s
- cycles with lambda > 0: 474 of 822; with lambda >= 0.5: 165, between t0 + 20.5 and 43.0 s, reference height 1.04 .. 2.25 m in those cycles
- 14 stall breaks between t0 + 58.0 and 85.0 s at height 1.08 .. 1.39 m, 0.81 .. 1.36 m from the goal; lambda at the breaks 0.00 .. 0.02; v = 0 was the optimum in 11 .. 12 of the 12 cycles before each break
- slowest global searches: 11123 ms (risk_Dk, t0 + 84.7 s), 10495 ms (risk_Dk, t0 + 73.4 s), 10107 ms (risk_Dk, t0 + 63.0 s), 9421 ms (risk_Dk, t0 + 52.9 s)
- end: reference 0.77 m from the goal at height 1.41 m

## 4 V2 screening (seeds 1–5; g3, g5, g6 × all_off / all_on × 6 variants)

Not executed: the pipeline stopped before V2 (gate stop above).

## 5 V3 confirmation (fresh seeds 21–40; g3, g6 × all_off / all_on × 6 variants)

Not executed: the pipeline stopped before V3 (gate stop above).





## 6 Thesis material

### §4.2.4 (addition to the Chapter-4 discussion; frozen numbers only)

In the frozen baseline (`{tag}`), g6 and the one-way series g3 both succeeded in all 120 runs, but g6 produced worse
paths (§4.2.2): M2 was significantly longer in all six variants (paired median differences +0.241 m to +1.711 m), M3
significantly larger in S3_climb_ne, head_on, crossing_90 and M1_three_way, and M6 significantly higher in
S3_climb_ne, crossing_90 and M1_three_way. The degradation also appears in the three static variants, which contain no
moving obstacle; there the urgency ξ is infinite and λ ≡ 0, so neither the VO risk term nor Γ is active, and the static
degradation cannot be explained as the price of dynamic obstacle avoidance. All effects belong to the C2 + C3
combination jointly. Source inspection identified coupling defects able to produce this behaviour (a speed-independent
guidance-cone slack with a zero-width cone at λ = 0, single-cycle κ feedback, a replanning trigger over all cached
edges, no switching hysteresis); the post-hoc diagnostics of Appendix B are consistent with them but are associations,
not causal evidence. Chapter 4 reports the frozen baseline; the repair is evaluated in §4.5.

### §4.5 Supplementary extended analysis: repair of the coupling defects

**Scope.** This section reports a defect repair implemented in this work and the evaluation run on it. It does not
replace §4.1–4.3, whose conclusions are derived only from the frozen baseline `thesis-frozen-v1`. The repaired planner is the
frozen code plus repair items FX1–FX6, each behind a Boolean flag that defaults to false; with every flag off the
planner output is byte-identical to the frozen reference (offline golden test T0) and the run-log writers are
unchanged.

**Protocol.** The evaluation was pre-registered as a gated sequence: V1 incremental smoke ladder (development seeds
101–103), V2 screening (seeds 1–5) and V3 confirmation (fresh seeds 21–40). A failed gate stops the sequence.

**Outcome.** The evaluation stopped at the V1 safety gate: Gate-V1 safety (unsafe after the R4-m fallback at v1_r4). Switch adjustments made before the stop:
step v1_r4: avoidance delayed by the dwell -> r4m_dwell_free_only enabled. The later stages were not run. Consequently this work makes **no claim** that the repair mitigates the
path-quality degradation of g6 relative to g3; the V1 observations (Appendix C) are descriptive clues from three
development seeds per cell, not evidence. The repair remains available behind default-false flags, and the frozen
baseline of Chapter 4 is unaffected.

### Limitations (additions)

- The frozen baseline contains coupling defects that degrade path quality even in static scenes; Chapter 4 reports it
  with these defects.
- The repair evaluation stopped at the V1 safety gate, so whether the repair mitigates the degradation is not
  established.
- The run that stopped the evaluation shows the reference held at v = 0 near the floor layer after a dynamic
  avoidance manoeuvre, advancing only through the stall breaker; whether a repair item contributes to this cannot be
  identified from a single run (the same configuration's repeated attempt reached the goal).

### Defence Q&A

- *Why did g6 degrade relative to g3 in the baseline?* The coupling introduced defects that act even without moving
  obstacles; statistically the effect is attributed only to C2 + C3 jointly.
- *What does the thesis do about it?* Chapter 4 reports the frozen results unchanged; §4.2.4 explains the mechanisms
  and labels the log evidence as associative; §4.5 documents the repair and its gated evaluation.
- *What was repaired, and was it verified?* FX1–FX6 behind default-false flags, verified offline (golden identity, unit
  tests). The closed-loop evaluation stopped at the V1 safety gate; no mitigation claim is made.
- *Why not modify the baseline directly?* Changing the model after the campaign and re-reporting would turn the
  evaluation into tuning on the test results; the tag keeps Chapter 4 reproducible and the flags keep the repair
  auditable and reversible.
- *Why was the evaluation not simply continued?* The gates were fixed before any simulation; continuing past a failed
  safety gate, or relaxing it after seeing the outcome, would invalidate the protocol.

