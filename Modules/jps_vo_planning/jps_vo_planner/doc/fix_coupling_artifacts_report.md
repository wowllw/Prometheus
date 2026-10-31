# fix_coupling_artifacts — pipeline report (iteration 2; supplementary analysis, thesis §4.5)

> Chapter 4 reports the frozen baseline `thesis-frozen-v1` and is unchanged. Everything here is a post-campaign engineering patch evaluated as supplementary analysis. Evidence classes: **[code]** fact read from the source, **[log]** clue from run logs (association only), **[sim]** result of the ablation simulations — the only class accepted as evidence for or against mitigation. g6 vs g3 is the joint C2 + C3 effect only. Unit tests and the dry run are not closed-loop evidence.

## 1 Git baseline protection

| Item | Value |
|---|---|
| Tag | `thesis-frozen-v1` (annotated, object `b755fb87d0f7`) → commit `130163686c45` |
| Protected base branch | `main`, commit unchanged: `130163686c45` |
| Development branch | `fix_coupling_artifacts` |
| Read-only assets | Chapter-4 doc, frozen 720-cell dataset, frozen analysis outputs: SHA-256 identical at every gate (Gate-I) |
| Uncommitted user files left untouched | `Modules/jps_vo_planning/jps_vo_planner/doc/ablation_result_record_table.md`, `Modules/jps_vo_planning/jps_vo_planner/scripts/__pycache__/post_process_ablation.cpython-38.pyc`, `Modules/jps_vo_planning/jps_vo_planner/scripts/post_process_ablation.py` |

Commits on `fix_coupling_artifacts`:

```
7b6b9d7 Pipeline update (report.py)
6608469 Iteration 2: apply R1e low-speed angle penalty (iteration 2), R4-new leaky evidence + safety override (iteration 2); regenerate tests and presets
d813b5b Stop report: pipeline ended at a protocol gate (supplementary analysis, section 4.5)
0a96a17 Pipeline update (report.py)
dd4fc3a Stop report: pipeline ended at a protocol gate (supplementary analysis, section 4.5)
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

## 2 Iterations

- **Iteration 1** (closed 2026-10-31 05:13:58): V1 stopped at the safety gate (M1_three_way seed 102 timeout under v1_r4_r4m). Log clues: R1b runs crawl in the narrow gap (y 0.7-3.2) at ~0.06-0.10 m/s vs ~0.5 m/s without; the late arrival led to an untriggered obstacle parked at the goal and a creeping approach under the frozen tau2-VO. Change: FX1 uses R1e (frozen angle penalty above v_min, linear fade below) instead of R1b; V1/V2 rules of the new task (3 seeds, no auto-repair, V3 manual).
  - V1 steps completed: diag_only, v1_r1, v1_r2, v1_r3; adjustments: step v1_r4: avoidance delayed by the dwell -> r4m_dwell_free_only enabled
  - gate stop `GATE_FAILURE_stage0.md`: Gate-0 discovery — role check_scenarios: expected exactly one match, found 3: ['/home/amov/Prometheus/Modules/jps_vo_planning/jps_vo_planner/scripts/check_scenarios.py', '/home/amov/Prometheus/Modules/jps_vo_planning/jp
  - gate stop `GATE_FAILURE_stage2.md`: Gate-2 existing tests — test_fixes failed:
  - gate stop `GATE_FAILURE_v1_20261031_011803.md`: Gate-I output — refusing to write outside /home/amov/.ros/ablation_fix: /home/amov/.ros/ablation_fix/v1/diag_only
  - gate stop `GATE_FAILURE_v1_20261031_014322.md`: unexpected error — Traceback (most recent call last):
  - gate stop `GATE_FAILURE_v1_20261031_035301.md`: Gate-V1 safety — unsafe after the R4-m fallback at v1_r4
- **Iteration 2** (this report).

## 3 Dry run (sandbox copy of the tag; repository untouched)

- Edit layers applied on the frozen sources (every anchor matched exactly once): base FX1-FX6 (iteration 1); R3 positive part + R4-m (iteration 1); R1e low-speed angle penalty (iteration 2); R4-new leaky evidence + safety override (iteration 2); experiment drivers: fixes preset pass-through.
- Full diff: 27 files, +1854 / −13 lines (sha256 `d5e74d2ad3f493e9`), kept in `/home/amov/.ros/ablation_fix/pipeline/dryrun_iter2/full.diff`.
- Compilation with the package flags: no errors, no new warnings (core library, node translation units with their ROS flags, all test programs).
- Frozen `gen_golden` vs sandbox `gen_golden` (all flags false): identical (2578995 bytes).
- Existing tests in the sandbox: test_coupling 18 PASS, test_jps 10 PASS, test_occupancy_grid 10 PASS, test_vo 3 PASS.
- Sandbox `test_fixes`: 62 PASS, 0 FAIL. Log writers of cycles.csv / paths.csv unchanged: openLogs, writeCycleRow, writePathEvents, writeChecks.

**Phase A check** of `/home/amov/.ros/ablation_fix/pipeline/dryrun_iter2_console.log` and the sandbox test output: layers 5 of 5; problems: none.

| Implementation checklist | Status |
|---|---|
| FX1 (R1a, R1c, R1d, R1e) | declared, default false |
| FX2 (R2a, R2b) | declared, default false |
| FX3 (R3 positive increments) | declared, default false |
| FX4 (R4, R4-m, R4-new evidence, R4-s override) | declared, default false |
| FX5 (R5) | declared, default false |
| FX6 (S1) | declared, default false |
| R1b retained for comparison | declared, default false |

| Test | Result (sandbox) |
|---|---|
| T0 | 2 PASS / 0 FAIL |
| T0b | 1 PASS / 0 FAIL |
| T1 | 3 PASS / 0 FAIL |
| T10 | 2 PASS / 0 FAIL |
| T11 | 6 PASS / 0 FAIL |
| T2 | 4 PASS / 0 FAIL |
| T2e | 4 PASS / 0 FAIL |
| T3 | 1 PASS / 0 FAIL |
| T4 | 4 PASS / 0 FAIL |
| T5 | 4 PASS / 0 FAIL |
| T5b | 5 PASS / 0 FAIL |
| T6 | 2 PASS / 0 FAIL |
| T6b | 1 PASS / 0 FAIL |
| T7 | 2 PASS / 0 FAIL |
| T8 | 1 PASS / 0 FAIL |
| T9 | 20 PASS / 0 FAIL |

- Superseded dry run (edit sources changed afterwards; fingerprint `None`): 56 PASS / 0 FAIL.

## 4 Stage 0–2 (deployment)

- Existing `test_occupancy_grid`: exit 0, 10 PASS, 0 FAIL.
- Existing `test_jps`: exit 0, 10 PASS, 0 FAIL.
- Existing `test_vo`: exit 0, 3 PASS, 0 FAIL.
- Existing `test_coupling`: exit 0, 18 PASS, 0 FAIL.
- Frozen reference built from `thesis-frozen-v1` with `-O3 -Wall -g -std=c++14`: 2578995 bytes; branch `gen_golden` (flags false): identical.
- `test_fixes` (catkin build): 62 PASS, 0 FAIL. Gate-T0: [PASS] T0 all fixes off: dump identical to the frozen reference (2578995 bytes)
  - `[INFO] T3 lambda = 0, R1a+R1b on: v* == frozen ray closed form in 303/303 cases with |v_prev| >= 0.4 and 297/297 with |v_prev| < 0.4`
  - `[INFO] T5 risk source at lateral 2.2 m: route side changes over 200 replans: R4 off 199, R4 on 66`
  - `[INFO] T5 risk source at lateral 5.5 m: route side changes over 200 replans: R4 off 0, R4 on 0`
  - `[INFO] T5b route side changes over 200 replans: R4 33, R4 + R4-m 33; R4-m judged switches: 1 under threat (1 accepted inside the dwell), 116 threat-free`
  - `[INFO] T6 300 random obstacles: frozen-only fires 38, both 62, R3-only 2, neither 198`
  - `[INFO] T6b 59 cases with a remaining-edge increase > eps_w (12 with simultaneous decreases); the signed sum of the first patch version stays <= eps_w in 0 of them`
  - `[INFO] T11 route side changes over 200 replans with an alternating risk source: no R4 199, R4 (it. 1) 66, R4-new 199`
  - `[INFO] T11 safety override: predicted clearance old route -0.73 m, new route 1.75 m`
  - `[INFO] T8 S1: 466 non-fallback problems, risk budget active in 333`
  - `[INFO] T10 R1d+R1b: 879 problems with a feasible moving candidate; v* = 0 chosen in 182 of them`
  - `[INFO] T10 R1d+R1e: 896 problems with a feasible moving candidate; v* = 0 chosen in 187 of them`
  - `[INFO] T9 offline kinematic loop on planning_easy (60 s cap):`
  - `[INFO] scenario     arm           reached  length_m stop_cycles max_stop_run  switches  fallbacks  occupied`
  - `[INFO] static_east  g3 frozen           1     13.03           1            1         3          2         1`
  - `[INFO] static_east  g6 frozen           1     13.44           1            1         0          2         2`
  - `[INFO] static_east  g6 fix it1          1     13.18           1            1         0          6         3`
  - `[INFO] static_east  g6 fix it2          1     13.60           1            1         0          3         2`
  - `[INFO] static_ne    g3 frozen           1     10.11           1            1         1          0         0`
  - `[INFO] static_ne    g6 frozen           1     10.55           1            1         0          1         1`
  - `[INFO] static_ne    g6 fix it1          1     10.23           1            1         0          0         0`
  - `[INFO] static_ne    g6 fix it2          1     10.52           1            1         0          1         1`
  - `[INFO] head_on      g3 frozen           1     13.55          42           10         4         15        14`
  - `[INFO] head_on      g6 frozen           0      5.49         438          437         6          0         0`
  - `[INFO] head_on      g6 fix it1          1     13.54          10            9         1          4         4`
  - `[INFO] head_on      g6 fix it2          1     14.03          37           35         1          6         6`
  - `[INFO] three_way    g3 frozen           1     13.56          14           11         4          7         6`
  - `[INFO] three_way    g6 frozen           1     14.27           1            1         4          2         1`
  - `[INFO] three_way    g6 fix it1          1     13.79           1            1         1          2         2`
  - `[INFO] three_way    g6 fix it2          1     14.32           1            1         1          3         2`
  - `[INFO] gap_north    g3 frozen           1     13.89           1            1         3          2         2`
  - `[INFO] gap_north    g6 frozen           1     14.01           1            1         0          3         3`
  - `[INFO] gap_north    g6 fix it1          1     13.95           1            1         0          0         0`
  - `[INFO] gap_north    g6 fix it2          1     14.04           1            1         0          5         3`

## 5 V1 pre-screening (g6; S3_climb_ne, head_on, M1_three_way; seeds 101–103) [sim, n = 3 per cell, descriptive]

| Preset | Variant | ok | M2 [m] | M3 [°] | M6 | Σ stall breaks | path switches | min dyn. clearance [m] | duration [s] | gap time [s] | gap median speed [m/s] | gap low-speed share | Σ R4 overrides |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| diag_only | M1_three_way | 3/3 | 15.82 | 83.5 | 3.0 | 16 | 18.0 | 0.810 | 50.5 | 21.5 | 0.05 | 0.787 | 0 |
| diag_only | S3_climb_ne | 3/3 | 14.47 | 116.0 | 0.0 | 16 | 6.0 | — | 44.4 | — | — | — | 0 |
| diag_only | head_on | 3/3 | 15.37 | 82.7 | 0.0 | 0 | 4.0 | 0.428 | 31.2 | — | — | — | 0 |
| v1_r1_e | M1_three_way | 3/3 | 15.99 | 80.7 | 4.0 | 1 | 15.0 | 0.729 | 38.7 | 10.0 | 0.11 | 0.482 | 0 |
| v1_r1_e | S3_climb_ne | 3/3 | 15.12 | 106.2 | 0.0 | 0 | 6.0 | — | 33.7 | — | — | — | 0 |
| v1_r1_e | head_on | 3/3 | 15.18 | 56.7 | 0.0 | 0 | 5.0 | 0.562 | 31.0 | — | — | — | 0 |

Pre-registered V1 gates: no collision and every goal reached; minimum dynamic clearance not more than 0.05 m below diag_only (same variant and seed); M1_three_way narrow-gap traversal at most 2 × the diag_only time of the same seed.

- diag_only (units —): gate failures 0; dwell-held switches under threat followed by a clearance drop > 0.05 m: 0.
- v1_r1_e (units FX1): gate failures 4 — [{'cell': ['head_on', 'g6', '101'], 'diag_only': 0.655691806163464, 'gate': 'dynamic clearance', 'min_dyn': 0.562242648594661}, {'cell': ['M1_three_way', 'g6', '101'], 'diag_only': 0.9347901330553774, 'gate': 'dynamic clearance', 'min_dyn': 0.7916088374555401}, {'cell': ['head_on', 'g6', '102'], 'diag_only': 0.7987957555126419, 'gate': 'dynamic clearance', 'min_dyn': 0.7185855573964162}, {'cell': ['M1_three_way', 'g6', '102'], 'diag_only': 0.8182478787645902, 'gate': 'dynamic clearance', 'min_dyn': 0.7289702900262229}]; dwell-held switches under threat followed by a clearance drop > 0.05 m: 0.

### Repeatability: identical configuration (`diag_only`, frozen behaviour) and seed in two iterations [sim]

| Variant | Seed | min dyn. clearance it. 1 [m] | min dyn. clearance it. 2 [m] |
|---|---|---||---|---|---|
| head_on | 101 | 0.936 | 0.656 |
| head_on | 102 | 0.698 | 0.799 |
| head_on | 103 | 0.644 | 0.428 |
| M1_three_way | 101 | 0.785 | 0.935 |
| M1_three_way | 102 | 0.821 | 0.818 |
| M1_three_way | 103 | 0.839 | 0.810 |

### M1_three_way narrow gap (y 0.7–3.2 m) in every V1 run [sim]

| Iteration | Preset | Seed | traversal [s] | median speed [m/s] | low-speed share |
|---|---|---|---|---|---|
| 1 | diag_only | 101 | 7.2 | 0.378 | 0.00 |
| 1 | diag_only | 102 | 5.5 | 0.513 | 0.00 |
| 1 | diag_only | 103 | 4.1 | 0.668 | 0.00 |
| 1 | v1_r1 | 101 | 9.9 | 0.098 | 0.51 |
| 1 | v1_r1 | 102 | 15.4 | 0.097 | 0.53 |
| 1 | v1_r1 | 103 | 6.4 | 0.295 | 0.00 |
| 1 | v1_r2 | 101 | 13.6 | 0.080 | 0.53 |
| 1 | v1_r2 | 102 | 6.6 | 0.387 | 0.00 |
| 1 | v1_r2 | 103 | 7.7 | 0.311 | 0.24 |
| 1 | v1_r3 | 101 | 5.3 | 0.483 | 0.00 |
| 1 | v1_r3 | 102 | 5.3 | 0.490 | 0.00 |
| 1 | v1_r3 | 103 | 5.8 | 0.465 | 0.00 |
| 1 | v1_r4 | 101 | 6.3 | 0.347 | 0.00 |
| 1 | v1_r4 | 102 | 12.2 | 0.085 | 0.50 |
| 1 | v1_r4 | 103 | 7.0 | 0.334 | 0.04 |
| 1 | v1_r4_r4m | 101 | 14.7 | 0.080 | 0.57 |
| 1 | v1_r4_r4m | 102 | 15.2 | 0.058 | 0.65 |
| 1 | v1_r4_r4m | 103 | 7.7 | 0.304 | 0.00 |
| 2 | diag_only | 101 | 22.8 | 0.053 | 0.79 |
| 2 | diag_only | 102 | 7.2 | 0.351 | 0.00 |
| 2 | diag_only | 103 | 21.5 | 0.027 | 0.79 |
| 2 | v1_r1_e | 101 | 10.0 | 0.085 | 0.52 |
| 2 | v1_r1_e | 102 | 6.3 | 0.450 | 0.00 |
| 2 | v1_r1_e | 103 | 14.0 | 0.109 | 0.48 |

## 6 V2 pre-screening ablation (seeds 1–3; g3, g5, g6 × all_off / all_on × 6 variants) [sim, n = 3, descriptive]

Not executed in this iteration (stopped before V2).

## 7 V3 formal evaluation

**Not started — requires explicit manual confirmation.** Specification:

- Seeds 21–40 (fresh, never used in V1/V2); all six variants; groups g3 and g6 (g5 optional) × `all_off` / `all_on` (the V1-decided configuration); arms interleaved per seed; 480 runs (720 with g5), about 3 min each.
- Per arm, the project's `post_process_ablation.py --index` (validity rules, M1–M6, Wilcoxon–Pratt, Holm, bootstrap, exact McNemar). Cross-arm contrasts with the same functions, one Holm family per variant × metric: A = g6 fixed vs g6 frozen, B = g6 fixed vs g3 frozen, C = g3 fixed vs g3 frozen, D = g6 frozen vs g3 frozen; bootstrap 10 000 resamples, seed 20261026.
- Start command after confirmation: `pipeline.py run-v3 --confirm-v3 [--with-g5]`.

## 8 Gate stops in this iteration

- `GATE_FAILURE_v1_iter2_20261031_070501.md` (2026-10-31 07:05:01) — Gate-V1: 4 V1 gate failures at v1_r1_e
  - {'gate': 'dynamic clearance', 'cell': ['head_on', 'g6', '101'], 'min_dyn': 0.562, 'diag_only': 0.656}
  - {'gate': 'dynamic clearance', 'cell': ['M1_three_way', 'g6', '101'], 'min_dyn': 0.792, 'diag_only': 0.935}
  - {'gate': 'dynamic clearance', 'cell': ['head_on', 'g6', '102'], 'min_dyn': 0.719, 'diag_only': 0.799}
  - {'gate': 'dynamic clearance', 'cell': ['M1_three_way', 'g6', '102'], 'min_dyn': 0.729, 'diag_only': 0.818}

## 9 Thesis material

### §4.2.4 (addition to the Chapter-4 discussion; frozen numbers only)

In the frozen baseline (`thesis-frozen-v1`), g6 and the one-way series g3 both succeeded in all 120 runs, but g6 produced worse
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

**Scope.** This section reports a defect repair implemented in this work and its evaluation. It does not replace
§4.1–4.3, whose conclusions are derived only from the frozen baseline `thesis-frozen-v1`. The repaired planner is the frozen code
plus repair items FX1–FX6, each behind a Boolean flag that defaults to false; with every flag off the planner output is
byte-identical to the frozen reference (offline golden test T0) and the run-log writers are unchanged.

**Protocol.** Pre-registered gated sequence: dry run in a sandbox; V1 incremental smoke ladder (development seeds
101–103); V2 screening of {g3, g5, g6} × {off, on} × six variants (seeds 1–3); V3 confirmation on fresh seeds 21–40
only after manual confirmation. A failed gate stops the sequence; the next iteration starts with a changed design and a
new dry run, and every iteration is reported.

**Outcome.** The evaluation stopped at the V1 gate: Gate-V1 (4 V1 gate failures at v1_r1_e). Switch adjustments made before the stop: no repair switch had to be disabled or downgraded.
The later stages were not run, so this work makes **no claim** that the repair mitigates the path-quality degradation
of g6 relative to g3; the V1/V2 observations are descriptive screening data, not evidence.


### Limitations (additions)

- The frozen baseline contains coupling defects that degrade path quality even in static scenes; Chapter 4 reports it
  with these defects.
- The repair evaluation uses one simulated world; V1/V2 are screening with three seeds per cell.
- FX3 guarantees decision consistency only for routes through cached edges; FX4 can delay a voluntary switch by at most
  T_g plus one replanning period in threat-free mode (none under a threat with R4-m).
- The frozen dynamic constraint (time-to-collision > τ₂ = 12 s) also applies to a stationary obstacle; near a goal that
  lies on such an obstacle's safety sphere it admits only approach speeds ≤ (d − R)/τ₂ (a creeping approach). This is a
  property of the frozen model, not of the repair, and it is not changed here.
- Simulator repeatability limits the screening: the same configuration (frozen behaviour) on the same seed gave
  minimum dynamic clearances differing by up to 0.28 m between two runs, so a per-run 0.05 m clearance gate cannot
  separate configuration effects from simulator nondeterminism; the M1_three_way narrow-gap low-speed passage also
  occurred with the frozen behaviour in two of three seeds in one iteration and in none in the other.
- The repaired version is a post-campaign patch; the conclusions of Chapter 4 rest on the frozen version.

### Defence Q&A

- *Why did g6 degrade relative to g3 in the baseline?* The coupling introduced defects that act even without moving
  obstacles; statistically the effect is attributed only to C2 + C3 jointly.
- *What does the thesis do about it?* Chapter 4 reports the frozen results unchanged; §4.2.4 explains the mechanisms
  and labels the log evidence as associative; §4.5 documents the repair and its gated evaluation.
- *What was repaired, and how was it verified?* FX1–FX6 behind default-false flags; a sandbox dry run and offline
  golden identity before deployment; closed-loop V1/V2 screening with safety gates; V3 on fresh seeds only after
  manual confirmation. Only the simulations count as evidence.
- *Why not modify the baseline directly?* Changing the model after the campaign and re-reporting would turn the
  evaluation into tuning on the test results; the tag keeps Chapter 4 reproducible and the flags keep the repair
  auditable and reversible.
- *Did a repair item fail?* Iteration 1 stopped at the V1 safety gate (a mission failure). Its runs with the
  speed-proportional cone slack (R1b) showed long low-speed phases in a narrow gap, and R1b was replaced by R1e in
  iteration 2. That association did not replicate: in iteration 2 the frozen behaviour itself showed the same
  low-speed passage in two of three seeds, and R1e showed it in two of three. Iteration 2 stopped at the V1
  dynamic-clearance gate. Every iteration and gate stop is listed in this report.

