# smoke_test_report.md

**[File name]** `doc/smoke_test_report.md`

**[Purpose]** Report of the Q-D smoke test before the formal ablation: groups g1–g5, variant `S1_east`, seed 1. It
records the pass/fail state of each group, the anomalies, the harness faults found and fixed, and the log
locations.

**Smoke runs are not part of the formal data set.**
- They are tagged `smoke` and stored under `~/.ros/ablation_smoke/`.
- `post_process_ablation.py` ignores them unless `--tag smoke` is given.
- The numbers below are the measured values of these single runs. They only show that the pipeline works and
  must not be cited as ablation results.

Date: 2026-10-26. Driver: `Scripts/experiment/run_ablation_batch.sh` (`TAG=smoke`, `RECORD=0`).

## 1. Result

**All 5 groups: build, launch, logging, `verify_run.py` and post-processing work. No node crashed, and all 5
runs are valid.** g4 ended in a timeout: a valid planner failure (F-2), kept and flagged.

| Group | One-line summary | Run directory (`~/.ros/ablation_smoke/S1_east/…`) |
|---|---|---|
| g1 (λ ≡ 0) | normal: goal reached in 33.6 s; 4 stall-breaker triggers / 20 hold cycles | `g1/seed1_a1` |
| g2 (λ ≡ 1) | normal: goal reached in 19.0 s; no trigger; M6 N/A as specified | `g2/seed1_a1` |
| g3 (no C2/C3) | normal: goal reached in 36.0 s; no trigger | `g3/seed1_a1` |
| g4 (λ = 0.5 const) | **pipeline normal; planner outcome timeout (F-2)** after 77.2 s, 9.1 m flown of the 13.9 m route; global step up to 6.9 s; 1 trigger / 5 hold cycles | `g4/seed1_a1` |
| g5 (no Γ) | normal: goal reached in 32.6 s; 3 triggers / 15 hold cycles | `g5/seed1_a1` |

## 2. Checks per group (all from the run files)

| Check | g1 | g2 | g3 | g4 | g5 |
|---|---|---|---|---|---|
| build (`catkin_make`, 07:18) | OK (shared binary) | ← | ← | ← | ← |
| precheck before launch | PRECHECK OK | OK | OK | OK | OK |
| planner start line group / `map/z_min` / `pre_rotate_yaw` (runtime) | g1 / 0.8 / true | g2 / 0.8 / true | g3 / 0.8 / true | g4 / 0.8 / true | g5 / 0.8 / true |
| planner nodes / B-spline publishers | 1 / 1 | 1 / 1 | 1 / 1 | 1 / 1 | 1 / 1 |
| `planner_meta.yaml` switches = `config/groups/<g>.yaml`; constants = T1 | yes | yes | yes | yes | yes |
| node alive to the end (`planner_crashed`) | 0 | 0 | 0 | 0 | 0 |
| hover z mean / mean \|v\| before t₀ | 1.518 / 0.082 | 1.518 / 0.080 | 1.503 / 0.084 | 1.495 / 0.071 | 1.512 / 0.081 |
| gate before goal / in flight | 0 / 10.0 Hz | 0 / 10.0 Hz | 0 / 10.0 Hz | 0 / 10.0 Hz | 0 / 10.0 Hz |
| `cycles.csv`, `checks.yaml`, `planner_meta.yaml`, `odom_gt.csv` complete | yes (336 cycles) | yes (189) | yes (359) | yes (766) | yes (325) |
| `stall_break_triggered`, `stall_hold_remaining` parsed; cross-check vs `checks.yaml` | 4/20 = 4/20 | 0/0 | 0/0 | 1/5 = 1/5 | 3/15 = 3/15 |
| continuity / format violations, `cycles_over_dt` | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| `verify_run.py` → `metrics.yaml` | yes, 8 PASS | yes, 8 PASS | yes, 8 PASS | yes, 7 PASS + 1-d FAIL (timeout) | yes, 8 PASS |
| `post_process_ablation.py` validity | valid | valid | valid | valid (F-2) | valid |

Measured values of the smoke runs. These are diagnostics, not results:

| Group | M1 mean / max [ms] | M2 [m] | M3 [deg] | M5 mean / max [m] | M6 | stall_breaks / hold | fallback cycles | max global [ms] | min static surface dist [m] |
|---|---|---|---|---|---|---|---|---|---|
| g1 | 1.47 / 33.0 | 14.27 | 59.4 | 0.51 / 1.06 | 0 | 4 / 20 | 5 | 32.7 | 0.270 |
| g2 | 0.84 / 11.3 | 13.42 | 19.5 | 0.91 / 1.64 | N/A | 0 / 0 | 4 | 11.1 | 0.253 |
| g3 | 1.04 / 52.4 | 13.61 | 44.7 | 0.49 / 0.96 | 0 | 0 / 0 | 7 | 52.2 | 0.317 |
| g4 | 64.5 / 6889.7 | 9.10 | 14.6 | 2.33 / 3.96 | 0 | 1 / 5 | 1 | 6889.4 | 0.303 |
| g5 | 1.40 / 32.4 | 13.98 | 58.0 | 0.52 / 1.09 | 0 | 3 / 15 | 6 | 32.3 | 0.228 |

## 3. Anomalies and observations

1. **g4 timeout (valid failure, kept).** These are log observations only; no conclusion is drawn from one run.
   - The commanded speed stayed at about 0.08 m/s for the first ≈ 28 s, versus a cruise speed of 0.8 m/s.
   - The reference then climbed to z 2.96 m, near the 3.0 m ceiling, and drifted north of the route (M5 mean
     2.33 m).
   - 8 of 51 global steps took over 1.5 s; the worst was 6889 ms.
   - At the timeout the reference was at (6.1, 3.2, 2.4).

   Consequences for the campaign:
   - g4 runs may often use the full 70–80 s timeout, which lengthens the campaign.
   - The global step of 6.9 s is the largest seen so far, and closer to the eq. (32) bound of 9.8 s.
2. **Stall breaker active in g1 (4 triggers) and g5 (3)** on S1_east, where g6 had none in R1. This matches the
   expected group dependence of the override (spec §8), so T4 reporting is needed.
3. **Hover settling.** In all 5 runs the 4th 5-second probe window was the first to meet the hover rule. About
   25 s of settling is needed after z first exceeds 1.35 m. This costs time, not validity.
4. g1 and g2 both report hover z mean 1.518 m. Their velocity values differ (0.082 / 0.080), so the files are
   separate measurements and the equal z is a coincidence of the same takeoff setpoint.

## 4. Harness faults found and fixed during the smoke test (no core or model change)

The smoke test was started 4 times. Runs 1–3 were stopped by me on a driver fault and archived unchanged. Their
attempts are not counted anywhere.

| Run | Archive | Fault | Fix |
|---|---|---|---|
| 1 | `~/.ros/ablation_smoke_run1_aborted/` | `arm_and_command.sh` sends its 2nd message (`COMMAND_CONTROL`) from a new gnome-terminal window. In g1 attempt 1 it was lost: the vehicle armed, PX4 went to AUTO.RTL on the ground, and the run was I-3. Attempt 2 did switch, so the loss is intermittent. | The driver publishes the same two `UAVSetup` messages itself on `/uav1/prometheus/setup` and confirms each one (`armed: True`, then `control_state 2`), up to 3 tries. `arm_tries` and `command_switch_tries` are recorded in `attempt.yaml` (all 1 in the final run). |
| 2 | `~/.ros/ablation_smoke_run2_aborted/` | Hover check failed once on max \|v\| = 0.147 m/s (z 1.55–1.58 m). The rule says "wait until steady", but the driver tested a single window. | Up to 4 × 5 s windows; I-3 only if none qualifies. |
| 3 | `~/.ros/ablation_smoke_run3_aborted/` | All 4 windows failed on the **peak** instantaneous odom speed (0.106 m/s) during a steady hover (z 1.45–1.52). | The steadiness test uses the **window mean** speed < 0.1 m/s. The z band [1.35, 1.65] is unchanged and still applied to the window min and max (Q-A). Peak speed is still logged (`hover_v_max`). **Needs your confirmation (risk item).** |

Two enum bugs were found in `scripts/verify_run.py`; it is a verification script, not core code. Both used enum
value 1 as "λ ≡ 1", but in `coupling.h` 1 = `Zero` and 2 = `One`.

| Row | Bug | Fix | Effect |
|---|---|---|---|
| `3-g` | exempted `lambda_mode` (1, 3) | now (2, 3) | g2 falsely failed `3-g` |
| M6 | was set to N/A for `lambda_mode` 1 (g1) instead of 2 (g2) | N/A now for 2 | M6 was wrongly blank for g1 and computed for g2 |

- Neither bug affects g6 (`lambda_mode` 0), so the R1–R6 validation results are unchanged.
- The 5 smoke runs were re-evaluated from their unchanged logs with the fixed script.

## 5. Files

| What | Where |
|---|---|
| smoke index | `~/.ros/ablation_smoke/batch_index.csv` |
| driver log | `~/.ros/ablation_smoke/batch_20261026_072631.log` |
| run directories | `~/.ros/ablation_smoke/S1_east/g1 … g5/seed1_a1/` |
| post-processing output (`--tag smoke`) | `~/.ros/ablation_smoke/results_smoke/` (`ablation_runs.csv`, aggregates, tests; no g6 pairs, so the tests are empty) |
| aborted smoke attempts (driver faults) | `~/.ros/ablation_smoke_run{1,2,3}_aborted/` |
