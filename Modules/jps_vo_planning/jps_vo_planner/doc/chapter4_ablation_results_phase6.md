# Chapter 4 — Ablation results (Phase 6, full dataset)

> **Draft for the thesis, generated 2026-10-30.** Every number is copied from the outputs of
> `scripts/post_process_ablation.py`. Nothing was recomputed by hand. Sources:
> - `out_phase6_full/`: `--index ~/.ros/ablation/batch_index.csv --tag formal`. This is the primary run: 728 index
>   rows, 720 valid, one valid row per (variant, group, seed).
> - `out_phase6_full_success/`: the same script in `--dirs` mode over an explicit list of the **550 successful**
>   runs taken from `out_phase6_full/ablation_runs.csv`. It is used only for success-only descriptive statistics
>   and for the continuous-metric tests. It is not a directory scan.
>
> Group switches (`config/groups/*.yaml`):
> - g1: λ ≡ 0, JPS only.
> - g2: λ ≡ 1, classic VO.
> - g3: adaptive λ with C2 and C3 off, i.e. one-way JPS → VO.
> - g4: λ ≡ 0.5 with C2 and C3 on.
> - g5: full coupling without Γ.
> - g6: full C1/C2/C3 (proposed).

## 4.0 Data validity

The success counts were checked against the frozen prior, revised on 2026-10-30. The revision added three
retried seeds that were missing when the prior was first frozen: S3_climb_ne/g1/seed5, S1_east/g2/seed1 and
S1_east/g3/seed1. All 12 entries match exactly.

| Group | Static (prior = observed) | Dynamic (prior = observed) |
|---|---|---|
| g1 | 59/60 | 0/60 |
| g2 | 60/60 | 57/60 |
| g3 | 60/60 | 60/60 |
| g4 | 12/60 | 2/60 |
| g5 | 60/60 | 60/60 |
| g6 | 60/60 | 60/60 |

Ends of type "goal + collision (F-1)" and "last_cycle + collision (F-1)" count as failures:
- 60 runs in g1 (all of its dynamic runs);
- 1 run in g2 (M1_three_way, seed 16).

Eight index rows are invalid or superseded (see `out_phase6_full/invalid_runs.csv`):
- five driver I-8 "log dir exists already";
- one I-7 (odom not valid);
- one I-3 (hover band);
- head_on/g4/seed15 attempt 2, which is SUPERSEDED. Attempt 1 passes the current checks, and both attempts ended in
  a timeout.

## 4.1 Table 4-1 — Ablation comparison, g1–g6 × six scenario variants

The columns are the six variants:
- **static:** S1_east, S2_north, S3_climb_ne;
- **single dynamic:** head_on, crossing_90;
- **multi dynamic:** M1_three_way.

Each cell has 20 seeds. Panel A gives success over all valid runs.

**Panels B–F use successful runs only.** g3, g5 and g6 succeeded in all 120 runs, so their values contain no
survivorship effect. The values for g1, g2 and g4 come only from the episodes those groups completed. They are
biased towards the easier episodes (survivorship bias), so they are descriptive and are not compared. A cell is
marked "(n = k)" when fewer than 20 runs succeeded, and "—" when no run succeeded.

The aggregates follow spec §4:
- **M1:** mean of the run means, mean of the run P95 values, and the maximum of the run maxima.
- **M2, M3, M5, M6:** median [IQR] over runs.
- **M4:** the dynamic success column of Panel A.
- **M6:** N/A for g2, because the global path does not enter the decision when 1 − λ = 0.

**Panel A — Success OK/n [Wilson 95 % CI]** (static: goal reached without contact; dynamic: M4)

| Group | S1_east | S2_north | S3_climb_ne | head_on | crossing_90 | M1_three_way |
|---|---|---|---|---|---|---|
| g1 | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 19/20 [0.76, 0.99] | 0/20 [0.00, 0.16] | 0/20 [0.00, 0.16] | 0/20 [0.00, 0.16] |
| g2 | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 17/20 [0.64, 0.95] |
| g3 | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] |
| g4 | 1/20 [0.01, 0.24] | 11/20 [0.34, 0.74] | 0/20 [0.00, 0.16] | 0/20 [0.00, 0.16] | 2/20 [0.03, 0.30] | 0/20 [0.00, 0.16] |
| g5 | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] |
| g6 | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] | 20/20 [0.84, 1.00] |

**Panel A′ — Pooled per scenario class (supplementary; Wilson 95 % CI via the same function)**

| Group | Static (S1, S2, S3) | Dynamic (head_on, crossing_90, M1_three_way) |
|---|---|---|
| g1 | 59/60 [0.91, 1.00] | 0/60 [0.00, 0.06] |
| g2 | 60/60 [0.94, 1.00] | 57/60 [0.86, 0.98] |
| g3 | 60/60 [0.94, 1.00] | 60/60 [0.94, 1.00] |
| g4 | 12/60 [0.12, 0.32] | 2/60 [0.01, 0.11] |
| g5 | 60/60 [0.94, 1.00] | 60/60 [0.94, 1.00] |
| g6 | 60/60 [0.94, 1.00] | 60/60 [0.94, 1.00] |

**Panel B — M1 planning time per cycle [ms]: mean of run means / mean of run P95 / max of run max**

| Group | S1_east | S2_north | S3_climb_ne | head_on | crossing_90 | M1_three_way |
|---|---|---|---|---|---|---|
| g1 | 1.40 / 11.84 / 51.7 | 0.43 / 1.61 / 11.5 | 0.83 / 5.74 / 18.8 (n=19) | — (0 succ.) | — (0 succ.) | — (0 succ.) |
| g2 | 1.27 / 8.58 / 40.2 | 0.46 / 1.75 / 11.4 | 0.60 / 3.30 / 8.2 | 26.96 / 150.62 / 943.6 | 30.31 / 191.63 / 999.3 | 35.92 / 235.13 / 763.7 (n=17) |
| g3 | 1.02 / 5.57 / 54.1 | 0.35 / 1.39 / 11.5 | 0.68 / 5.10 / 21.9 | 0.39 / 1.94 / 11.8 | 0.36 / 1.64 / 11.0 | 0.47 / 2.41 / 10.7 |
| g4 | 55.20 / 63.17 / 5088.5 (n=1) | 39.22 / 46.86 / 4629.2 (n=11) | — (0 succ.) | — (0 succ.) | 67.57 / 155.99 / 3807.8 (n=2) | — (0 succ.) |
| g5 | 1.39 / 11.69 / 47.6 | 0.43 / 1.58 / 11.8 | 0.81 / 5.64 / 17.7 | 13.47 / 63.92 / 434.3 | 13.38 / 81.17 / 444.2 | 23.32 / 147.19 / 432.8 |
| g6 | 2.76 / 22.28 / 134.5 | 0.46 / 1.59 / 18.7 | 0.88 / 5.97 / 35.0 | 27.16 / 33.86 / 3398.4 | 28.77 / 48.32 / 3499.2 | 31.67 / 112.05 / 2168.0 |

**Panel C — M2 path length [m]: median [IQR]**

| Group | S1_east | S2_north | S3_climb_ne | head_on | crossing_90 | M1_three_way |
|---|---|---|---|---|---|---|
| g1 | 14.02 [13.92, 14.18] | 14.10 [14.05, 14.18] | 15.11 [14.36, 15.59] (n=19) | — (0 succ.) | — (0 succ.) | — (0 succ.) |
| g2 | 13.44 [13.39, 13.97] | 14.22 [14.21, 14.24] | 13.35 [13.33, 13.36] | 14.43 [14.38, 14.47] | 14.30 [14.28, 14.35] | 14.71 [14.50, 15.07] (n=17) |
| g3 | 13.60 [13.57, 13.62] | 13.91 [13.90, 13.92] | 13.41 [13.38, 13.43] | 14.14 [14.09, 14.66] | 14.57 [14.48, 14.82] | 14.65 [14.56, 14.77] |
| g4 | 16.65 [16.65, 16.65] (n=1) | 15.49 [15.32, 15.81] (n=11) | — (0 succ.) | — (0 succ.) | 14.94 [14.89, 15.00] (n=2) | — (0 succ.) |
| g5 | 14.06 [13.97, 14.16] | 14.11 [14.05, 14.19] | 15.00 [14.54, 15.57] | 15.45 [15.22, 15.73] | 15.43 [15.31, 15.63] | 15.82 [15.73, 16.09] |
| g6 | 14.20 [14.09, 14.22] | 14.15 [14.06, 14.23] | 15.09 [14.60, 15.69] | 15.32 [15.23, 15.42] | 15.63 [15.06, 15.86] | 15.81 [15.72, 16.24] |

**Panel D — M3 max turning angle in 1 s [deg]: median [IQR]**

| Group | S1_east | S2_north | S3_climb_ne | head_on | crossing_90 | M1_three_way |
|---|---|---|---|---|---|---|
| g1 | 55.8 [44.4, 61.3] | 44.0 [39.0, 50.4] | 108.5 [97.9, 132.5] (n=19) | — (0 succ.) | — (0 succ.) | — (0 succ.) |
| g2 | 39.2 [26.0, 50.8] | 24.4 [23.4, 29.7] | 23.7 [20.8, 29.4] | 29.2 [26.5, 32.5] | 27.6 [24.1, 31.3] | 48.9 [30.9, 68.7] (n=17) |
| g3 | 46.9 [39.2, 55.2] | 31.3 [27.7, 42.9] | 37.6 [34.6, 46.8] | 57.9 [56.1, 64.7] | 43.0 [39.6, 49.2] | 41.2 [35.6, 48.1] |
| g4 | 22.6 [22.6, 22.6] (n=1) | 24.0 [17.4, 28.1] (n=11) | — (0 succ.) | — (0 succ.) | 28.5 [22.2, 34.8] (n=2) | — (0 succ.) |
| g5 | 54.8 [45.3, 59.5] | 40.1 [36.6, 46.5] | 130.4 [108.9, 145.8] | 75.8 [69.2, 94.1] | 54.6 [52.9, 90.0] | 66.6 [59.0, 90.0] |
| g6 | 52.8 [43.1, 57.8] | 42.8 [37.1, 48.0] | 128.7 [114.4, 139.5] | 63.9 [57.0, 116.3] | 65.9 [59.0, 84.8] | 77.7 [73.1, 99.0] |

**Panel E — M5 mean lateral deviation from Π₀ [m]: median [IQR]**

| Group | S1_east | S2_north | S3_climb_ne | head_on | crossing_90 | M1_three_way |
|---|---|---|---|---|---|---|
| g1 | 0.521 [0.487, 0.543] | 0.099 [0.094, 0.110] | 1.338 [1.214, 1.423] (n=19) | — (0 succ.) | — (0 succ.) | — (0 succ.) |
| g2 | 0.819 [0.539, 0.904] | 0.259 [0.255, 0.270] | 0.498 [0.490, 0.505] | 0.324 [0.315, 0.386] | 0.274 [0.250, 0.326] | 0.519 [0.482, 0.543] (n=17) |
| g3 | 0.492 [0.489, 0.502] | 0.111 [0.110, 0.113] | 1.392 [1.368, 1.407] | 0.283 [0.236, 0.408] | 0.393 [0.358, 0.430] | 0.472 [0.433, 0.493] |
| g4 | 1.129 [1.129, 1.129] (n=1) | 0.587 [0.539, 0.658] (n=11) | — (0 succ.) | — (0 succ.) | 0.739 [0.677, 0.801] (n=2) | — (0 succ.) |
| g5 | 0.526 [0.517, 0.552] | 0.104 [0.100, 0.111] | 1.490 [1.293, 1.636] | 0.391 [0.371, 0.420] | 0.438 [0.412, 0.477] | 0.699 [0.546, 0.821] |
| g6 | 0.521 [0.499, 0.550] | 0.104 [0.099, 0.111] | 1.362 [1.254, 1.567] | 0.426 [0.410, 0.462] | 0.477 [0.254, 0.559] | 0.712 [0.593, 0.813] |

**Panel F — M6 flip-backs [count]: median [IQR]**

| Group | S1_east | S2_north | S3_climb_ne | head_on | crossing_90 | M1_three_way |
|---|---|---|---|---|---|---|
| g1 | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] (n=19) | — (0 succ.) | — (0 succ.) | — (0 succ.) |
| g2 | N/A | N/A | N/A | N/A | N/A | N/A (n=17) |
| g3 | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 1.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] |
| g4 | 1.00 [1.00, 1.00] (n=1) | 1.00 [0.00, 1.00] (n=11) | — (0 succ.) | — (0 succ.) | 1.00 [0.50, 1.50] (n=2) | — (0 succ.) |
| g5 | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 6.50 [2.75, 8.00] | 1.00 [0.00, 5.25] | 7.00 [4.00, 9.25] |
| g6 | 0.00 [0.00, 0.00] | 0.00 [0.00, 0.00] | 0.00 [0.00, 1.00] | 1.00 [0.00, 2.00] | 1.50 [0.00, 3.00] | 2.50 [1.00, 6.25] |

## 4.2 Discussion

**Conventions.**
- All effects are paired median differences g6 − gX, with a percentile bootstrap 95 % CI (10 000 resamples, seed
  20261026).
- The test is a two-sided Wilcoxon signed-rank test with Pratt zero handling.
- Holm correction runs over the five comparisons g6 vs g1…g5 for each (variant, metric) pair.
- A lower value is better for M1, M2, M3 and M6. M5 is reported as larger or smaller deviation from the initial
  global path Π₀.
- p-values below 0.0001 are written "< 0.0001".

### 4.2.1 Success-rate contrasts (evidence boundaries 1 and 2)

**g1 vs g6: necessity of local VO, success rate only.**
- Dynamic scenarios: g1 succeeded in 0/60 runs, g6 in 60/60. Every one of the 20 seeds per dynamic variant is
  discordant in favour of g6 (exact McNemar, Holm-corrected p < 0.0001 in head_on, crossing_90 and M1_three_way).
- Static scenarios: 59/60 vs 60/60, with no evidence of a significant difference in any static variant (Holm p =
  1.0000).
- The data therefore support the necessity of the local VO layer for dynamic obstacles. They say nothing about
  static scenarios.

**g4 vs g6: necessity of λ adaptation, success rate only.**
- Constant λ = 0.5 (g4) succeeded in 12/60 static and 2/60 dynamic runs; g6 succeeded in 60/60 and 60/60.
- The per-variant exact McNemar tests are significant in all six variants:

  | Variant | Discordant seeds favouring g6 | Holm p |
  |---|---|---|
  | S1_east | 19 | < 0.0001 |
  | S2_north | 9 | 0.0195 |
  | S3_climb_ne | 20 | < 0.0001 |
  | head_on | 20 | < 0.0001 |
  | crossing_90 | 18 | < 0.0001 |
  | M1_three_way | 20 | < 0.0001 |

- No seed favoured g4.
- This supports the necessity of an adaptive λ(ξ). g4 and g6 have the same C2/C3 switches, so this contrast is not
  evidence about cross-layer sharing, and no such claim is made.

### 4.2.2 g6 vs g3: overall path quality of the C2 + C3 combination (evidence boundary 4)

g3 differs from g6 only by `enable_c2: false` and `enable_c3: false`, so this contrast measures the two
mechanisms **jointly**. Nothing below is attributed to C2 or C3 individually.

Both groups succeeded in all runs (120/120; 20/20 in every variant), so the success rates are identical and every
path-quality comparison uses all 20 pairs. M1 differences are listed in Table 4-2 but are outside the path-quality
scope of this contrast.

**Summary.** In this campaign the C2 + C3 combination does **not** improve overall path quality compared with the
one-way series g3. Where a difference is significant, it nearly always goes against g6.

- **M2 path length:** significantly longer for g6 in all six variants. The paired median difference ranges from
  +0.241 m (S2_north) to +1.711 m (S3_climb_ne).
- **M3 maximum 1 s turning angle:**
  - significantly larger for g6 in S3_climb_ne (+80.0°), head_on (+7.1°), crossing_90 (+28.4°) and M1_three_way
    (+43.5°);
  - no evidence of a significant difference in S1_east and S2_north. S2_north has raw p = 0.0441, which is not
    significant after Holm correction.
- **M6 flip-backs:**
  - significantly more for g6 in S3_climb_ne, crossing_90 (+1.5) and M1_three_way (+2.5);
  - no evidence of a significant difference in head_on (Holm p = 0.0570);
  - not testable in S1_east and S2_north, where every paired difference is 0.
  - The higher M6 is consistent with the known corridor-flipping risk R5, which is measured but not removed. It is
    attributed to the combination.
- **M5 deviation from Π₀:** mixed.
  - Larger for g6 in head_on and M1_three_way (mean and maximum).
  - Smaller for g6 in S2_north. The mean is −0.007 m, a negligible magnitude; the maximum is −0.195 m.
  - No evidence of a significant difference in S1_east, S3_climb_ne and crossing_90.

**Robustness notes.**
- head_on, M5 maximum: Holm p = 0.0054, but the bootstrap CI of the median difference, [−0.041, 0.593] m,
  includes 0. The rank test and the median estimate disagree, so this result should be read with caution.
- S1_east, M5 mean: the result depends on the Holm family. It has Holm p = 0.0545 in the primary (success-only)
  family and 0.0408 in the all-valid family, so it is reported as no evidence of a significant difference.
- S3_climb_ne, M6: significant with a paired median difference of 0. The shift lies in the upper quartile: g6 IQR
  [0, 1] vs g3 [0, 0].

The exact template sentences follow.


#### Template sentences, g6 vs g3

- *S1_east:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 0.595 m; 95% bootstrap CI = [0.548, 0.648] m; n = 20 pairs).
- *S2_north:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0004; effect size: paired median difference = 0.241 m; 95% bootstrap CI = [0.151, 0.310] m; n = 20 pairs).
- *S3_climb_ne:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 1.711 m; 95% bootstrap CI = [1.281, 2.199] m; n = 20 pairs).
- *head_on:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 1.034 m; 95% bootstrap CI = [0.743, 1.235] m; n = 20 pairs).
- *crossing_90:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 0.757 m; 95% bootstrap CI = [0.693, 1.089] m; n = 20 pairs).
- *M1_three_way:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 1.222 m; 95% bootstrap CI = [1.090, 1.498] m; n = 20 pairs).
- *S1_east:* Given an identical overall success rate, compared to g3, g6 shows no evidence of a significant difference in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 1.0000; effect size: paired median difference = 4.954°; 95% bootstrap CI = [-2.950, 13.125]°; n = 20 pairs).
- *S2_north:* Given an identical overall success rate, compared to g3, g6 shows no evidence of a significant difference in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.1322; effect size: paired median difference = 6.673°; 95% bootstrap CI = [1.016, 11.008]°; n = 20 pairs).
- *S3_climb_ne:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 80.004°; 95% bootstrap CI = [77.677, 98.826]°; n = 20 pairs).
- *head_on:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0344; effect size: paired median difference = 7.118°; 95% bootstrap CI = [0.740, 36.984]°; n = 20 pairs).
- *crossing_90:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0021; effect size: paired median difference = 28.394°; 95% bootstrap CI = [15.309, 40.404]°; n = 20 pairs).
- *M1_three_way:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 43.549°; 95% bootstrap CI = [29.849, 52.974]°; n = 20 pairs).
- *S1_east:* Given an identical overall success rate, compared to g3, g6 shows no evidence of a significant difference in the M5 mean deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0545; effect size: paired median difference = 0.031 m; 95% bootstrap CI = [0.005, 0.055] m; n = 20 pairs).
- *S2_north:* Given an identical overall success rate, compared to g3, g6 shows a significantly smaller deviation in the M5 mean deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0249; effect size: paired median difference = -0.007 m; 95% bootstrap CI = [-0.013, -0.001] m; n = 20 pairs).
- *S3_climb_ne:* Given an identical overall success rate, compared to g3, g6 shows no evidence of a significant difference in the M5 mean deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p = 1.0000; effect size: paired median difference = -0.010 m; 95% bootstrap CI = [-0.120, 0.196] m; n = 20 pairs).
- *head_on:* Given an identical overall success rate, compared to g3, g6 shows a significantly larger deviation in the M5 mean deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0012; effect size: paired median difference = 0.174 m; 95% bootstrap CI = [0.043, 0.192] m; n = 20 pairs).
- *crossing_90:* Given an identical overall success rate, compared to g3, g6 shows no evidence of a significant difference in the M5 mean deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p = 1.0000; effect size: paired median difference = 0.050 m; 95% bootstrap CI = [-0.062, 0.127] m; n = 20 pairs).
- *M1_three_way:* Given an identical overall success rate, compared to g3, g6 shows a significantly larger deviation in the M5 mean deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 0.239 m; 95% bootstrap CI = [0.150, 0.313] m; n = 20 pairs).
- *S1_east:* Given an identical overall success rate, compared to g3, g6 shows no evidence of a significant difference in the M5 maximum deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.2127; effect size: paired median difference = 0.026 m; 95% bootstrap CI = [0.006, 0.075] m; n = 20 pairs).
- *S2_north:* Given an identical overall success rate, compared to g3, g6 shows a significantly smaller deviation in the M5 maximum deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = -0.195 m; 95% bootstrap CI = [-0.200, -0.191] m; n = 20 pairs).
- *S3_climb_ne:* Given an identical overall success rate, compared to g3, g6 shows no evidence of a significant difference in the M5 maximum deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.5547; effect size: paired median difference = -0.020 m; 95% bootstrap CI = [-0.048, 0.005] m; n = 20 pairs).
- *head_on:* Given an identical overall success rate, compared to g3, g6 shows a significantly larger deviation in the M5 maximum deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0054; effect size: paired median difference = 0.520 m; 95% bootstrap CI = [-0.041, 0.593] m; n = 20 pairs).
- *crossing_90:* Given an identical overall success rate, compared to g3, g6 shows no evidence of a significant difference in the M5 maximum deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.4609; effect size: paired median difference = 0.247 m; 95% bootstrap CI = [-0.029, 0.538] m; n = 20 pairs).
- *M1_three_way:* Given an identical overall success rate, compared to g3, g6 shows a significantly larger deviation in the M5 maximum deviation from Π₀ (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0001; effect size: paired median difference = 0.302 m; 95% bootstrap CI = [0.186, 0.452] m; n = 20 pairs).
- *S1_east, M6 flip-back count:* not testable (all differences 0; paired median difference 0.000); reported descriptively only.
- *S2_north, M6 flip-back count:* not testable (all differences 0; paired median difference 0.000); reported descriptively only.
- *S3_climb_ne:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M6 flip-back count (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0271; effect size: paired median difference = 0.000; 95% bootstrap CI = [0.000, 0.500]; n = 20 pairs) (the paired median difference is 0; the shift lies in the upper quartile, see Table 4-1 Panel F).
- *head_on:* Given an identical overall success rate, compared to g3, g6 shows no evidence of a significant difference in the M6 flip-back count (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0570; effect size: paired median difference = 0.500; 95% bootstrap CI = [0.000, 2.000]; n = 20 pairs).
- *crossing_90:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M6 flip-back count (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0014; effect size: paired median difference = 1.500; 95% bootstrap CI = [0.000, 3.000]; n = 20 pairs).
- *M1_three_way:* Given an identical overall success rate, compared to g3, g6 shows a significant deterioration (higher value) in the M6 flip-back count (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0006; effect size: paired median difference = 2.500; 95% bootstrap CI = [1.000, 5.500]; n = 20 pairs).


### 4.2.3 g6 vs g5: a clean comparison of Γ (evidence boundary 3; M1, M2, M3 and M6 only)

g5 differs from g6 only by `enable_gamma: false`. Success rates are not compared for this pair. Both groups have
120/120 valid runs, so the successful set is the full set of 20 pairs per variant.

- **M2 path length and M3 turning angle:** no evidence of a significant difference in any of the six variants.
  - The smallest Holm p for M2 is 0.0515 (S1_east), whose raw p of 0.0172 does not survive correction.
  - The smallest Holm p for M3 is 0.2462 (M1_three_way).
- **M6 flip-backs:** significantly fewer with Γ in the two scenarios where g5 oscillates most.
  - head_on: −6.0 (CI [−7.0, −2.5]), Holm p = 0.0014. The g5 median is 6.5, the g6 median 1.0.
  - M1_three_way: −5.0 (CI [−6.0, −1.5]), Holm p = 0.0166. The g5 median is 7.0, the g6 median 2.5.
  - No evidence of a significant difference in crossing_90 and S3_climb_ne.
  - Not testable in S1_east and S2_north, where the differences are all or almost all zero.
- **M1 planning time:** the cost profile changes shape rather than simply rising or falling.
  - The **mean** is significantly higher with Γ in S1_east (+1.4 ms), head_on (+13.3 ms), crossing_90 (+14.7 ms)
    and M1_three_way (+7.9 ms). There is no evidence of a significant difference in S2_north and S3_climb_ne.
  - The **P95** is significantly lower with Γ in all three dynamic variants: −28.7 ms, −37.8 ms and −29.2 ms. It is
    higher in S1_east (+10.0 ms), with no evidence of a significant difference in S2_north and S3_climb_ne.
  - The per-variant **maximum**, which is not tested, is 2168–3499 ms for g6 against 433–444 ms for g5 in the
    dynamic variants. These are global-search spikes above the declared T_g = 1.5 s, still within the eq. (32)
    bound of 9.8 s (limitation L9).
  - The metrics do not isolate the mechanism behind this mean / P95 / max pattern, so it is reported without a
    causal interpretation.

The exact template sentences follow.

#### Template sentences, g6 vs g5

- *S1_east:* Given an identical overall success rate, compared to g5, g6 shows a significant deterioration (higher value) in the M1 mean planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 1.361 ms; 95% bootstrap CI = [1.171, 1.605] ms; n = 20 pairs).
- *S2_north:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M1 mean planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.1063; effect size: paired median difference = 0.019 ms; 95% bootstrap CI = [0.001, 0.051] ms; n = 20 pairs).
- *S3_climb_ne:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M1 mean planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.1517; effect size: paired median difference = 0.108 ms; 95% bootstrap CI = [0.009, 0.169] ms; n = 20 pairs).
- *head_on:* Given an identical overall success rate, compared to g5, g6 shows a significant deterioration (higher value) in the M1 mean planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 13.332 ms; 95% bootstrap CI = [12.951, 14.926] ms; n = 20 pairs).
- *crossing_90:* Given an identical overall success rate, compared to g5, g6 shows a significant deterioration (higher value) in the M1 mean planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p < 0.0001; effect size: paired median difference = 14.694 ms; 95% bootstrap CI = [12.726, 17.374] ms; n = 20 pairs).
- *M1_three_way:* Given an identical overall success rate, compared to g5, g6 shows a significant deterioration (higher value) in the M1 mean planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0017; effect size: paired median difference = 7.904 ms; 95% bootstrap CI = [3.392, 12.469] ms; n = 20 pairs).
- *S1_east:* Given an identical overall success rate, compared to g5, g6 shows a significant deterioration (higher value) in the M1 P95 planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0003; effect size: paired median difference = 10.030 ms; 95% bootstrap CI = [3.918, 16.742] ms; n = 20 pairs).
- *S2_north:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M1 P95 planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.8182; effect size: paired median difference = 0.107 ms; 95% bootstrap CI = [0.003, 0.146] ms; n = 20 pairs).
- *S3_climb_ne:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M1 P95 planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.2280; effect size: paired median difference = 0.362 ms; 95% bootstrap CI = [-0.059, 0.665] ms; n = 20 pairs).
- *head_on:* Given an identical overall success rate, compared to g5, g6 shows a significant improvement (lower value) in the M1 P95 planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0001; effect size: paired median difference = -28.714 ms; 95% bootstrap CI = [-35.363, -17.874] ms; n = 20 pairs).
- *crossing_90:* Given an identical overall success rate, compared to g5, g6 shows a significant improvement (lower value) in the M1 P95 planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0189; effect size: paired median difference = -37.763 ms; 95% bootstrap CI = [-58.602, -16.513] ms; n = 20 pairs).
- *M1_three_way:* Given an identical overall success rate, compared to g5, g6 shows a significant improvement (lower value) in the M1 P95 planning time per cycle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0153; effect size: paired median difference = -29.242 ms; 95% bootstrap CI = [-78.070, -3.761] ms; n = 20 pairs).
- *S1_east:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0515; effect size: paired median difference = 0.164 m; 95% bootstrap CI = [0.018, 0.212] m; n = 20 pairs).
- *S2_north:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.7285; effect size: paired median difference = 0.011 m; 95% bootstrap CI = [-0.079, 0.083] m; n = 20 pairs).
- *S3_climb_ne:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.9799; effect size: paired median difference = 0.083 m; 95% bootstrap CI = [-0.701, 0.584] m; n = 20 pairs).
- *head_on:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0759; effect size: paired median difference = -0.146 m; 95% bootstrap CI = [-0.242, 0.099] m; n = 20 pairs).
- *crossing_90:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p = 1.0000; effect size: paired median difference = 0.023 m; 95% bootstrap CI = [-0.351, 0.440] m; n = 20 pairs).
- *M1_three_way:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M2 path length (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.5217; effect size: paired median difference = -0.001 m; 95% bootstrap CI = [-0.156, 0.371] m; n = 20 pairs).
- *S1_east:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 1.0000; effect size: paired median difference = 1.103°; 95% bootstrap CI = [-4.985, 8.070]°; n = 20 pairs).
- *S2_north:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 1.0000; effect size: paired median difference = 2.174°; 95% bootstrap CI = [-6.263, 9.173]°; n = 20 pairs).
- *S3_climb_ne:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.8983; effect size: paired median difference = -5.217°; 95% bootstrap CI = [-23.944, 23.269]°; n = 20 pairs).
- *head_on:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.2774; effect size: paired median difference = -16.019°; 95% bootstrap CI = [-27.605, 11.863]°; n = 20 pairs).
- *crossing_90:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 1.0000; effect size: paired median difference = 6.726°; 95% bootstrap CI = [-14.755, 25.600]°; n = 20 pairs).
- *M1_three_way:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M3 maximum 1 s turning angle (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.2462; effect size: paired median difference = 16.709°; 95% bootstrap CI = [-6.790, 28.178]°; n = 20 pairs).
- *S1_east, M6 flip-back count:* not testable (degenerate; paired median difference 0.000); reported descriptively only.
- *S2_north, M6 flip-back count:* not testable (all differences 0; paired median difference 0.000); reported descriptively only.
- *S3_climb_ne:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M6 flip-back count (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.5808; effect size: paired median difference = 0.000; 95% bootstrap CI = [0.000, 0.500]; n = 20 pairs).
- *head_on:* Given an identical overall success rate, compared to g5, g6 shows a significant improvement (lower value) in the M6 flip-back count (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0014; effect size: paired median difference = -6.000; 95% bootstrap CI = [-7.000, -2.500]; n = 20 pairs).
- *crossing_90:* Given an identical overall success rate, compared to g5, g6 shows no evidence of a significant difference in the M6 flip-back count (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.5843; effect size: paired median difference = 0.000; 95% bootstrap CI = [-3.000, 1.000]; n = 20 pairs).
- *M1_three_way:* Given an identical overall success rate, compared to g5, g6 shows a significant improvement (lower value) in the M6 flip-back count (Wilcoxon signed-rank Pratt test; Holm-corrected p = 0.0166; effect size: paired median difference = -5.000; 95% bootstrap CI = [-6.000, -1.500]; n = 20 pairs).

## 4.3 Table 4-2 — Complete list of paired significance tests (g6 vs g1…g5, every variant)

The list has 240 rows: 6 variants × 8 metrics × 5 comparisons. The diagnostic binary `success_wo_trigger` is in
Appendix A.

**Continuous metrics** (M1 mean, M1 P95, M2, M3, M5 mean, M5 max, M6):
- The primary values come from `out_phase6_full_success`, i.e. pairs in which both runs succeeded.
- The column "p Holm (all-valid family)" gives the value from `out_phase6_full` for comparison. There the Holm
  family includes the failed g1, g2 and g4 runs.
- For g3 and g5 the raw p, the effect and the CI are identical in both outputs; only the Holm adjustment can differ.

**Success metrics** (goal_success for static variants, M4_success for dynamic ones):
- These come from `out_phase6_full`, using the exact McNemar test on all valid pairs.
- The effect is the difference in success rate.

**"Use" column.** It states whether a row falls inside an evidence boundary. Rows marked "not interpreted" are
listed for completeness only. They are mostly continuous metrics against g1, g2 or g4, where the pairs exist only
for surviving episodes or n is very small (g4: n = 1, 11, 2).

| # | Variant | Metric | Comparison | n pairs | Test | Effect g6−gX [boot 95 % CI] | p raw | p Holm (primary) | p Holm (all-valid family) | Sig. 0.05 | Use |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | S1_east | M1_mean_ms | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.369 [1.019, 1.571] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 2 | S1_east | M1_mean_ms | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.645 [1.144, 1.918] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 3 | S1_east | M1_mean_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.774 [1.527, 1.925] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 4 | S1_east | M1_mean_ms | g6 vs g4 | 1 | Wilcoxon signed-rank (Pratt, exact) | -52.371 [-52.371, -52.371] | 1.0000 | 1.0000 | <0.0001 | no | not interpreted |
| 5 | S1_east | M1_mean_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.361 [1.171, 1.605] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 3 (Γ) |
| 6 | S1_east | M1_p95_ms | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | 8.791 [4.360, 16.320] | <0.0001 | 0.0001 | 0.0001 | yes | not interpreted |
| 7 | S1_east | M1_p95_ms | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 12.545 [7.658, 21.084] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 8 | S1_east | M1_p95_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 15.060 [10.603, 21.908] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 9 | S1_east | M1_p95_ms | g6 vs g4 | 1 | Wilcoxon signed-rank (Pratt, exact) | -37.203 [-37.203, -37.203] | 1.0000 | 1.0000 | <0.0001 | no | not interpreted |
| 10 | S1_east | M1_p95_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 10.030 [3.918, 16.742] | 0.0001 | 0.0003 | 0.0001 | yes | boundary 3 (Γ) |
| 11 | S1_east | M2_path_length_m | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.117 [0.016, 0.250] | 0.0240 | 0.0515 | 0.0515 | no | not interpreted |
| 12 | S1_east | M2_path_length_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.385 [0.208, 0.824] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 13 | S1_east | M2_path_length_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.595 [0.548, 0.648] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 14 | S1_east | M2_path_length_m | g6 vs g4 | 1 | Wilcoxon signed-rank (Pratt, exact) | -2.738 [-2.738, -2.738] | 1.0000 | 1.0000 | 0.1429 | no | not interpreted |
| 15 | S1_east | M2_path_length_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.164 [0.018, 0.212] | 0.0172 | 0.0515 | 0.0515 | no | boundary 3 (Γ) |
| 16 | S1_east | M3_max_turn_deg_1s | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | -2.344 [-6.498, 4.580] | 0.4524 | 1.0000 | 0.9048 | no | not interpreted |
| 17 | S1_east | M3_max_turn_deg_1s | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 15.068 [2.555, 26.374] | 0.0583 | 0.2913 | 0.2330 | no | not interpreted |
| 18 | S1_east | M3_max_turn_deg_1s | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 4.954 [-2.950, 13.125] | 0.2611 | 1.0000 | 0.7833 | no | boundary 4 (C2+C3 jointly) |
| 19 | S1_east | M3_max_turn_deg_1s | g6 vs g4 | 1 | Wilcoxon signed-rank (Pratt, exact) | 20.432 [20.432, 20.432] | 1.0000 | 1.0000 | 0.0024 | no | not interpreted |
| 20 | S1_east | M3_max_turn_deg_1s | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.103 [-4.985, 8.070] | 0.9563 | 1.0000 | 0.9563 | no | boundary 3 (Γ) |
| 21 | S1_east | M5_mean_m | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.018 [-0.002, 0.029] | 0.0973 | 0.2919 | 0.1946 | no | not interpreted |
| 22 | S1_east | M5_mean_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.271 [-0.378, -0.035] | 0.0004 | 0.0020 | 0.0016 | yes | not interpreted |
| 23 | S1_east | M5_mean_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.031 [0.005, 0.055] | 0.0136 | 0.0545 | 0.0408 | no | boundary 4 (C2+C3 jointly) |
| 24 | S1_east | M5_mean_m | g6 vs g4 | 1 | Wilcoxon signed-rank (Pratt, exact) | -0.580 [-0.580, -0.580] | 1.0000 | 1.0000 | <0.0001 | no | not interpreted |
| 25 | S1_east | M5_mean_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.004 [-0.038, 0.029] | 0.5706 | 1.0000 | 0.5706 | no | outside boundary 3 |
| 26 | S1_east | M5_max_m | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.030 [-0.095, -0.008] | 0.0532 | 0.2127 | 0.1595 | no | not interpreted |
| 27 | S1_east | M5_max_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.461 [-0.583, -0.077] | 0.0003 | 0.0016 | 0.0013 | yes | not interpreted |
| 28 | S1_east | M5_max_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.026 [0.006, 0.075] | 0.0583 | 0.2127 | 0.1595 | no | boundary 4 (C2+C3 jointly) |
| 29 | S1_east | M5_max_m | g6 vs g4 | 1 | Wilcoxon signed-rank (Pratt, exact) | -1.531 [-1.531, -1.531] | 1.0000 | 1.0000 | <0.0001 | no | not interpreted |
| 30 | S1_east | M5_max_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.002 [-0.031, 0.014] | 0.4524 | 0.9048 | 0.4524 | no | outside boundary 3 |
| 31 | S1_east | M6_flip_backs | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, degenerate) | 0.000 [0.000, 0.000] | — | — (degenerate) | — | — | not interpreted |
| 32 | S1_east | M6_flip_backs | g6 vs g2 | — | — | N/A (M6 undefined for g2) | — | — | — | — | — |
| 33 | S1_east | M6_flip_backs | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, all differences 0) | 0.000 [0.000, 0.000] | — | — (all differences 0) | — | — | boundary 4 (C2+C3 jointly) |
| 34 | S1_east | M6_flip_backs | g6 vs g4 | 1 | Wilcoxon signed-rank (Pratt, exact) | -1.000 [-1.000, -1.000] | 1.0000 | 1.0000 | 0.0004 | no | not interpreted |
| 35 | S1_east | M6_flip_backs | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, degenerate) | 0.000 [0.000, 0.000] | — | — (degenerate) | — | — | boundary 3 (Γ) |
| 36 | S1_east | goal_success | g6 vs g1 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | boundary 1 (local VO) |
| 37 | S1_east | goal_success | g6 vs g2 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 38 | S1_east | goal_success | g6 vs g3 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 39 | S1_east | goal_success | g6 vs g4 | 20 | exact McNemar (g6-only 19, gX-only 0) | 0.950 [0.850, 1.000] | <0.0001 | <0.0001 | (same) | yes | boundary 2 (λ adaptation) |
| 40 | S1_east | goal_success | g6 vs g5 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | not reported (boundary 3) |
| 41 | S2_north | M1_mean_ms | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.027 [0.009, 0.045] | 0.0083 | 0.0249 | 0.0249 | yes | not interpreted |
| 42 | S2_north | M1_mean_ms | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.005 [-0.029, 0.004] | 0.2162 | 0.2162 | 0.2162 | no | not interpreted |
| 43 | S2_north | M1_mean_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.091 [0.078, 0.110] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 44 | S2_north | M1_mean_ms | g6 vs g4 | 11 | Wilcoxon signed-rank (Pratt, exact) | -38.398 [-40.953, -33.985] | 0.0010 | 0.0039 | <0.0001 | yes | not interpreted |
| 45 | S2_north | M1_mean_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.019 [0.001, 0.051] | 0.0532 | 0.1063 | 0.1063 | no | boundary 3 (Γ) |
| 46 | S2_north | M1_p95_ms | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.029 [-0.036, 0.121] | 0.7841 | 0.8182 | 0.8182 | no | not interpreted |
| 47 | S2_north | M1_p95_ms | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.120 [-0.394, -0.058] | 0.0094 | 0.0377 | 0.0377 | yes | not interpreted |
| 48 | S2_north | M1_p95_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.142 [0.044, 0.287] | 0.0107 | 0.0377 | 0.0377 | yes | boundary 4 (C2+C3 jointly) |
| 49 | S2_north | M1_p95_ms | g6 vs g4 | 11 | Wilcoxon signed-rank (Pratt, exact) | -36.394 [-66.735, -24.164] | 0.0010 | 0.0049 | <0.0001 | yes | not interpreted |
| 50 | S2_north | M1_p95_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.107 [0.003, 0.146] | 0.4091 | 0.8182 | 0.8182 | no | boundary 3 (Γ) |
| 51 | S2_north | M2_path_length_m | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.071 [-0.069, 0.171] | 0.2774 | 0.5547 | 0.5547 | no | not interpreted |
| 52 | S2_north | M2_path_length_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.079 [-0.175, 0.010] | 0.0296 | 0.0887 | 0.0887 | no | not interpreted |
| 53 | S2_north | M2_path_length_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.241 [0.151, 0.310] | 0.0001 | 0.0004 | 0.0003 | yes | boundary 4 (C2+C3 jointly) |
| 54 | S2_north | M2_path_length_m | g6 vs g4 | 11 | Wilcoxon signed-rank (Pratt, exact) | -1.352 [-1.714, -1.116] | 0.0010 | 0.0039 | <0.0001 | yes | not interpreted |
| 55 | S2_north | M2_path_length_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.011 [-0.079, 0.083] | 0.7285 | 0.7285 | 0.7285 | no | boundary 3 (Γ) |
| 56 | S2_north | M3_max_turn_deg_1s | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.551 [-9.789, 6.691] | 0.8983 | 1.0000 | 1.0000 | no | not interpreted |
| 57 | S2_north | M3_max_turn_deg_1s | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 16.734 [9.661, 21.773] | 0.0006 | 0.0029 | 0.0023 | yes | not interpreted |
| 58 | S2_north | M3_max_turn_deg_1s | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 6.673 [1.016, 11.008] | 0.0441 | 0.1322 | 0.1322 | no | boundary 4 (C2+C3 jointly) |
| 59 | S2_north | M3_max_turn_deg_1s | g6 vs g4 | 11 | Wilcoxon signed-rank (Pratt, exact) | 15.601 [12.493, 23.821] | 0.0020 | 0.0078 | <0.0001 | yes | not interpreted |
| 60 | S2_north | M3_max_turn_deg_1s | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 2.174 [-6.263, 9.173] | 0.6477 | 1.0000 | 1.0000 | no | boundary 3 (Γ) |
| 61 | S2_north | M5_mean_m | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.002 [-0.002, 0.012] | 0.5459 | 1.0000 | 1.0000 | no | not interpreted |
| 62 | S2_north | M5_mean_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.155 [-0.169, -0.146] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 63 | S2_north | M5_mean_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.007 [-0.013, -0.001] | 0.0083 | 0.0249 | 0.0249 | yes | boundary 4 (C2+C3 jointly) |
| 64 | S2_north | M5_mean_m | g6 vs g4 | 11 | Wilcoxon signed-rank (Pratt, exact) | -0.498 [-0.556, -0.441] | 0.0010 | 0.0039 | <0.0001 | yes | not interpreted |
| 65 | S2_north | M5_mean_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.003 [-0.009, 0.005] | 0.5706 | 1.0000 | 1.0000 | no | outside boundary 3 |
| 66 | S2_north | M5_max_m | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.005 [-0.003, 0.015] | 0.0484 | 0.0969 | 0.0969 | no | not interpreted |
| 67 | S2_north | M5_max_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.365 [-0.382, -0.350] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 68 | S2_north | M5_max_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.195 [-0.200, -0.191] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 69 | S2_north | M5_max_m | g6 vs g4 | 11 | Wilcoxon signed-rank (Pratt, exact) | -0.725 [-0.738, -0.530] | 0.0010 | 0.0029 | <0.0001 | yes | not interpreted |
| 70 | S2_north | M5_max_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.001 [-0.009, 0.010] | 0.8695 | 0.8695 | 0.8695 | no | outside boundary 3 |
| 71 | S2_north | M6_flip_backs | g6 vs g1 | 20 | Wilcoxon signed-rank (Pratt, all differences 0) | 0.000 [0.000, 0.000] | — | — (all differences 0) | — | — | not interpreted |
| 72 | S2_north | M6_flip_backs | g6 vs g2 | — | — | N/A (M6 undefined for g2) | — | — | — | — | — |
| 73 | S2_north | M6_flip_backs | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, all differences 0) | 0.000 [0.000, 0.000] | — | — (all differences 0) | — | — | boundary 4 (C2+C3 jointly) |
| 74 | S2_north | M6_flip_backs | g6 vs g4 | 11 | Wilcoxon signed-rank (Pratt, normal) | -1.000 [-1.000, 0.000] | 0.0091 | 0.0091 | 0.0004 | yes | not interpreted |
| 75 | S2_north | M6_flip_backs | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, all differences 0) | 0.000 [0.000, 0.000] | — | — (all differences 0) | — | — | boundary 3 (Γ) |
| 76 | S2_north | goal_success | g6 vs g1 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | boundary 1 (local VO) |
| 77 | S2_north | goal_success | g6 vs g2 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 78 | S2_north | goal_success | g6 vs g3 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 79 | S2_north | goal_success | g6 vs g4 | 20 | exact McNemar (g6-only 9, gX-only 0) | 0.450 [0.250, 0.650] | 0.0039 | 0.0195 | (same) | yes | boundary 2 (λ adaptation) |
| 80 | S2_north | goal_success | g6 vs g5 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | not reported (boundary 3) |
| 81 | S3_climb_ne | M1_mean_ms | g6 vs g1 | 19 | Wilcoxon signed-rank (Pratt, exact) | 0.061 [-0.048, 0.132] | 0.0799 | 0.1517 | 0.1517 | no | not interpreted |
| 82 | S3_climb_ne | M1_mean_ms | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.302 [0.233, 0.347] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 83 | S3_climb_ne | M1_mean_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.200 [0.161, 0.231] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 84 | S3_climb_ne | M1_mean_ms | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 85 | S3_climb_ne | M1_mean_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.108 [0.009, 0.169] | 0.0759 | 0.1517 | 0.1517 | no | boundary 3 (Γ) |
| 86 | S3_climb_ne | M1_p95_ms | g6 vs g1 | 19 | Wilcoxon signed-rank (Pratt, exact) | -0.053 [-0.252, 0.841] | 0.5949 | 0.5949 | 0.8124 | no | not interpreted |
| 87 | S3_climb_ne | M1_p95_ms | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 2.603 [2.373, 2.903] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 88 | S3_climb_ne | M1_p95_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.631 [0.288, 1.127] | 0.0002 | 0.0006 | 0.0006 | yes | boundary 4 (C2+C3 jointly) |
| 89 | S3_climb_ne | M1_p95_ms | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 90 | S3_climb_ne | M1_p95_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.362 [-0.059, 0.665] | 0.1140 | 0.2280 | 0.2280 | no | boundary 3 (Γ) |
| 91 | S3_climb_ne | M2_path_length_m | g6 vs g1 | 19 | Wilcoxon signed-rank (Pratt, exact) | 0.351 [-0.321, 0.683] | 0.4900 | 0.9799 | 0.6236 | no | not interpreted |
| 92 | S3_climb_ne | M2_path_length_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.641 [1.179, 2.274] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 93 | S3_climb_ne | M2_path_length_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.711 [1.281, 2.199] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 94 | S3_climb_ne | M2_path_length_m | g6 vs g4 | 0 | — | — | — | — (no pairs) | 0.0002 | — | not interpreted |
| 95 | S3_climb_ne | M2_path_length_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.083 [-0.701, 0.584] | 0.8983 | 0.9799 | 0.8983 | no | boundary 3 (Γ) |
| 96 | S3_climb_ne | M3_max_turn_deg_1s | g6 vs g1 | 19 | Wilcoxon signed-rank (Pratt, exact) | 10.619 [-16.060, 26.477] | 0.3124 | 0.6248 | 0.3787 | no | not interpreted |
| 97 | S3_climb_ne | M3_max_turn_deg_1s | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 99.129 [77.163, 111.895] | <0.0001 | 0.0001 | 0.0001 | yes | not interpreted |
| 98 | S3_climb_ne | M3_max_turn_deg_1s | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 80.004 [77.677, 98.826] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 99 | S3_climb_ne | M3_max_turn_deg_1s | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 100 | S3_climb_ne | M3_max_turn_deg_1s | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -5.217 [-23.944, 23.269] | 0.8983 | 0.8983 | 0.8983 | no | boundary 3 (Γ) |
| 101 | S3_climb_ne | M5_mean_m | g6 vs g1 | 19 | Wilcoxon signed-rank (Pratt, exact) | 0.149 [-0.160, 0.213] | 0.4413 | 1.0000 | 1.0000 | no | not interpreted |
| 102 | S3_climb_ne | M5_mean_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.855 [0.771, 1.037] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 103 | S3_climb_ne | M5_mean_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.010 [-0.120, 0.196] | 0.6477 | 1.0000 | 1.0000 | no | boundary 4 (C2+C3 jointly) |
| 104 | S3_climb_ne | M5_mean_m | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 105 | S3_climb_ne | M5_mean_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.003 [-0.258, 0.058] | 0.3488 | 1.0000 | 1.0000 | no | outside boundary 3 |
| 106 | S3_climb_ne | M5_max_m | g6 vs g1 | 19 | Wilcoxon signed-rank (Pratt, exact) | -0.006 [-0.038, 0.015] | 0.4900 | 0.5547 | 0.7285 | no | not interpreted |
| 107 | S3_climb_ne | M5_max_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 2.006 [1.969, 2.038] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 108 | S3_climb_ne | M5_max_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.020 [-0.048, 0.005] | 0.2774 | 0.5547 | 0.5547 | no | boundary 4 (C2+C3 jointly) |
| 109 | S3_climb_ne | M5_max_m | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 110 | S3_climb_ne | M5_max_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.019 [-0.073, -0.003] | 0.0240 | 0.0719 | 0.0719 | no | outside boundary 3 |
| 111 | S3_climb_ne | M6_flip_backs | g6 vs g1 | 19 | Wilcoxon signed-rank (Pratt, normal) | 0.000 [0.000, 0.000] | 0.3879 | 0.5808 | 0.5808 | no | not interpreted |
| 112 | S3_climb_ne | M6_flip_backs | g6 vs g2 | — | — | N/A (M6 undefined for g2) | — | — | — | — | — |
| 113 | S3_climb_ne | M6_flip_backs | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, normal) | 0.000 [0.000, 0.500] | 0.0090 | 0.0271 | 0.0271 | yes | boundary 4 (C2+C3 jointly) |
| 114 | S3_climb_ne | M6_flip_backs | g6 vs g4 | 0 | — | — | — | — (no pairs) | 0.0060 | — | not interpreted |
| 115 | S3_climb_ne | M6_flip_backs | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, normal) | 0.000 [0.000, 0.500] | 0.2904 | 0.5808 | 0.5808 | no | boundary 3 (Γ) |
| 116 | S3_climb_ne | goal_success | g6 vs g1 | 20 | exact McNemar (g6-only 1, gX-only 0) | 0.050 [0.000, 0.150] | 1.0000 | 1.0000 | (same) | no | boundary 1 (local VO) |
| 117 | S3_climb_ne | goal_success | g6 vs g2 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 118 | S3_climb_ne | goal_success | g6 vs g3 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 119 | S3_climb_ne | goal_success | g6 vs g4 | 20 | exact McNemar (g6-only 20, gX-only 0) | 1.000 [1.000, 1.000] | <0.0001 | <0.0001 | (same) | yes | boundary 2 (λ adaptation) |
| 120 | S3_climb_ne | goal_success | g6 vs g5 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | not reported (boundary 3) |
| 121 | head_on | M1_mean_ms | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 122 | head_on | M1_mean_ms | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.275 [-0.615, 1.343] | 0.6742 | 0.6742 | 0.6742 | no | not interpreted |
| 123 | head_on | M1_mean_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 27.192 [25.443, 27.713] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 124 | head_on | M1_mean_ms | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 125 | head_on | M1_mean_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 13.332 [12.951, 14.926] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 3 (Γ) |
| 126 | head_on | M1_p95_ms | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 127 | head_on | M1_p95_ms | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | -107.836 [-136.521, -92.560] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 128 | head_on | M1_p95_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 26.425 [22.515, 32.455] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 129 | head_on | M1_p95_ms | g6 vs g4 | 0 | — | — | — | — (no pairs) | 0.0017 | — | not interpreted |
| 130 | head_on | M1_p95_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -28.714 [-35.363, -17.874] | 0.0001 | 0.0001 | 0.0002 | yes | boundary 3 (Γ) |
| 131 | head_on | M2_path_length_m | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 132 | head_on | M2_path_length_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.904 [0.864, 0.947] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 133 | head_on | M2_path_length_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.034 [0.743, 1.235] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 134 | head_on | M2_path_length_m | g6 vs g4 | 0 | — | — | — | — (no pairs) | 0.3955 | — | not interpreted |
| 135 | head_on | M2_path_length_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.146 [-0.242, 0.099] | 0.0759 | 0.0759 | 0.1517 | no | boundary 3 (Γ) |
| 136 | head_on | M3_max_turn_deg_1s | g6 vs g1 | 0 | — | — | — | — (no pairs) | 0.0362 | — | not interpreted |
| 137 | head_on | M3_max_turn_deg_1s | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 35.489 [26.875, 74.254] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 138 | head_on | M3_max_turn_deg_1s | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 7.118 [0.740, 36.984] | 0.0172 | 0.0344 | 0.0362 | yes | boundary 4 (C2+C3 jointly) |
| 139 | head_on | M3_max_turn_deg_1s | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 140 | head_on | M3_max_turn_deg_1s | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -16.019 [-27.605, 11.863] | 0.2774 | 0.2774 | 0.2774 | no | boundary 3 (Γ) |
| 141 | head_on | M5_mean_m | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 142 | head_on | M5_mean_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.098 [0.076, 0.119] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 143 | head_on | M5_mean_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.174 [0.043, 0.192] | 0.0006 | 0.0012 | 0.0012 | yes | boundary 4 (C2+C3 jointly) |
| 144 | head_on | M5_mean_m | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 145 | head_on | M5_mean_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.040 [-0.000, 0.061] | 0.0121 | 0.0121 | 0.0121 | yes | outside boundary 3 |
| 146 | head_on | M5_max_m | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 147 | head_on | M5_max_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.530 [0.487, 0.596] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 148 | head_on | M5_max_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.520 [-0.041, 0.593] | 0.0027 | 0.0054 | 0.0054 | yes | boundary 4 (C2+C3 jointly) |
| 149 | head_on | M5_max_m | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 150 | head_on | M5_max_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.161 [0.085, 0.254] | 0.0049 | 0.0054 | 0.0054 | yes | outside boundary 3 |
| 151 | head_on | M6_flip_backs | g6 vs g1 | 0 | — | — | — | — (no pairs) | 0.0282 | — | not interpreted |
| 152 | head_on | M6_flip_backs | g6 vs g2 | — | — | N/A (M6 undefined for g2) | — | — | — | — | — |
| 153 | head_on | M6_flip_backs | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, normal) | 0.500 [0.000, 2.000] | 0.0570 | 0.0570 | 0.1140 | no | boundary 4 (C2+C3 jointly) |
| 154 | head_on | M6_flip_backs | g6 vs g4 | 0 | — | — | — | — (no pairs) | 0.6807 | — | not interpreted |
| 155 | head_on | M6_flip_backs | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, normal) | -6.000 [-7.000, -2.500] | 0.0007 | 0.0014 | 0.0028 | yes | boundary 3 (Γ) |
| 156 | head_on | M4_success | g6 vs g1 | 20 | exact McNemar (g6-only 20, gX-only 0) | 1.000 [1.000, 1.000] | <0.0001 | <0.0001 | (same) | yes | boundary 1 (local VO) |
| 157 | head_on | M4_success | g6 vs g2 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 158 | head_on | M4_success | g6 vs g3 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 159 | head_on | M4_success | g6 vs g4 | 20 | exact McNemar (g6-only 20, gX-only 0) | 1.000 [1.000, 1.000] | <0.0001 | <0.0001 | (same) | yes | boundary 2 (λ adaptation) |
| 160 | head_on | M4_success | g6 vs g5 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | not reported (boundary 3) |
| 161 | crossing_90 | M1_mean_ms | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 162 | crossing_90 | M1_mean_ms | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | -1.310 [-4.915, 1.234] | 0.2024 | 0.4049 | 0.2024 | no | not interpreted |
| 163 | crossing_90 | M1_mean_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 29.172 [26.762, 30.010] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 164 | crossing_90 | M1_mean_ms | g6 vs g4 | 2 | Wilcoxon signed-rank (Pratt, exact) | -40.363 [-47.887, -32.838] | 0.5000 | 0.5000 | <0.0001 | no | not interpreted |
| 165 | crossing_90 | M1_mean_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 14.694 [12.726, 17.374] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 3 (Γ) |
| 166 | crossing_90 | M1_p95_ms | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 167 | crossing_90 | M1_p95_ms | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | -163.397 [-173.076, -140.108] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 168 | crossing_90 | M1_p95_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 40.389 [28.104, 50.605] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 169 | crossing_90 | M1_p95_ms | g6 vs g4 | 2 | Wilcoxon signed-rank (Pratt, exact) | -128.422 [-131.402, -125.442] | 0.5000 | 0.5000 | 0.0047 | no | not interpreted |
| 170 | crossing_90 | M1_p95_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -37.763 [-58.602, -16.513] | 0.0094 | 0.0189 | 0.0094 | yes | boundary 3 (Γ) |
| 171 | crossing_90 | M2_path_length_m | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 172 | crossing_90 | M2_path_length_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.349 [0.740, 1.575] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 173 | crossing_90 | M2_path_length_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.757 [0.693, 1.089] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 174 | crossing_90 | M2_path_length_m | g6 vs g4 | 2 | Wilcoxon signed-rank (Pratt, exact) | -0.090 [-0.142, -0.038] | 0.5000 | 1.0000 | 0.1063 | no | not interpreted |
| 175 | crossing_90 | M2_path_length_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.023 [-0.351, 0.440] | 0.7285 | 1.0000 | 0.7285 | no | boundary 3 (Γ) |
| 176 | crossing_90 | M3_max_turn_deg_1s | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 177 | crossing_90 | M3_max_turn_deg_1s | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 38.059 [33.180, 53.591] | 0.0003 | 0.0013 | 0.0010 | yes | not interpreted |
| 178 | crossing_90 | M3_max_turn_deg_1s | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 28.394 [15.309, 40.404] | 0.0007 | 0.0021 | 0.0014 | yes | boundary 4 (C2+C3 jointly) |
| 179 | crossing_90 | M3_max_turn_deg_1s | g6 vs g4 | 2 | Wilcoxon signed-rank (Pratt, exact) | 27.300 [11.473, 43.128] | 0.5000 | 1.0000 | <0.0001 | no | not interpreted |
| 180 | crossing_90 | M3_max_turn_deg_1s | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 6.726 [-14.755, 25.600] | 0.5217 | 1.0000 | 0.5217 | no | boundary 3 (Γ) |
| 181 | crossing_90 | M5_mean_m | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 182 | crossing_90 | M5_mean_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.225 [-0.050, 0.288] | 0.0215 | 0.0859 | 0.0645 | no | not interpreted |
| 183 | crossing_90 | M5_mean_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.050 [-0.062, 0.127] | 0.4091 | 1.0000 | 0.8182 | no | boundary 4 (C2+C3 jointly) |
| 184 | crossing_90 | M5_mean_m | g6 vs g4 | 2 | Wilcoxon signed-rank (Pratt, exact) | -0.503 [-0.657, -0.350] | 0.5000 | 1.0000 | <0.0001 | no | not interpreted |
| 185 | crossing_90 | M5_mean_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.007 [-0.156, 0.092] | 0.4980 | 1.0000 | 0.8182 | no | outside boundary 3 |
| 186 | crossing_90 | M5_max_m | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 187 | crossing_90 | M5_max_m | g6 vs g2 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.122 [0.254, 1.266] | <0.0001 | 0.0001 | 0.0001 | yes | not interpreted |
| 188 | crossing_90 | M5_max_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.247 [-0.029, 0.538] | 0.1536 | 0.4609 | 0.3073 | no | boundary 4 (C2+C3 jointly) |
| 189 | crossing_90 | M5_max_m | g6 vs g4 | 2 | Wilcoxon signed-rank (Pratt, exact) | -0.725 [-1.372, -0.079] | 0.5000 | 1.0000 | 0.1912 | no | not interpreted |
| 190 | crossing_90 | M5_max_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.034 [-0.641, 0.289] | 0.5706 | 1.0000 | 0.5706 | no | outside boundary 3 |
| 191 | crossing_90 | M6_flip_backs | g6 vs g1 | 0 | — | — | — | — (no pairs) | 0.0241 | — | not interpreted |
| 192 | crossing_90 | M6_flip_backs | g6 vs g2 | — | — | N/A (M6 undefined for g2) | — | — | — | — | — |
| 193 | crossing_90 | M6_flip_backs | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, normal) | 1.500 [0.000, 3.000] | 0.0005 | 0.0014 | 0.0018 | yes | boundary 4 (C2+C3 jointly) |
| 194 | crossing_90 | M6_flip_backs | g6 vs g4 | 2 | Wilcoxon signed-rank (Pratt, exact) | -0.500 [-2.000, 1.000] | 1.0000 | 1.0000 | 0.5320 | no | not interpreted |
| 195 | crossing_90 | M6_flip_backs | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, normal) | 0.000 [-3.000, 1.000] | 0.2921 | 0.5843 | 0.5320 | no | boundary 3 (Γ) |
| 196 | crossing_90 | M4_success | g6 vs g1 | 20 | exact McNemar (g6-only 20, gX-only 0) | 1.000 [1.000, 1.000] | <0.0001 | <0.0001 | (same) | yes | boundary 1 (local VO) |
| 197 | crossing_90 | M4_success | g6 vs g2 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 198 | crossing_90 | M4_success | g6 vs g3 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 199 | crossing_90 | M4_success | g6 vs g4 | 20 | exact McNemar (g6-only 18, gX-only 0) | 0.900 [0.750, 1.000] | <0.0001 | <0.0001 | (same) | yes | boundary 2 (λ adaptation) |
| 200 | crossing_90 | M4_success | g6 vs g5 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | not reported (boundary 3) |
| 201 | M1_three_way | M1_mean_ms | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 202 | M1_three_way | M1_mean_ms | g6 vs g2 | 17 | Wilcoxon signed-rank (Pratt, exact) | -3.585 [-9.718, 1.464] | 0.0348 | 0.0348 | 0.0441 | yes | not interpreted |
| 203 | M1_three_way | M1_mean_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 32.336 [28.357, 33.878] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 204 | M1_three_way | M1_mean_ms | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 205 | M1_three_way | M1_mean_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 7.904 [3.392, 12.469] | 0.0009 | 0.0017 | 0.0017 | yes | boundary 3 (Γ) |
| 206 | M1_three_way | M1_p95_ms | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 207 | M1_three_way | M1_p95_ms | g6 vs g2 | 17 | Wilcoxon signed-rank (Pratt, exact) | -124.855 [-146.889, -87.627] | <0.0001 | 0.0001 | <0.0001 | yes | not interpreted |
| 208 | M1_three_way | M1_p95_ms | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 115.924 [98.994, 137.169] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 209 | M1_three_way | M1_p95_ms | g6 vs g4 | 0 | — | — | — | — (no pairs) | 0.1769 | — | not interpreted |
| 210 | M1_three_way | M1_p95_ms | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -29.242 [-78.070, -3.761] | 0.0153 | 0.0153 | 0.0306 | yes | boundary 3 (Γ) |
| 211 | M1_three_way | M2_path_length_m | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 212 | M1_three_way | M2_path_length_m | g6 vs g2 | 17 | Wilcoxon signed-rank (Pratt, exact) | 1.327 [0.909, 1.722] | 0.0011 | 0.0021 | 0.0005 | yes | not interpreted |
| 213 | M1_three_way | M2_path_length_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 1.222 [1.090, 1.498] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 214 | M1_three_way | M2_path_length_m | g6 vs g4 | 0 | — | — | — | — (no pairs) | 0.5885 | — | not interpreted |
| 215 | M1_three_way | M2_path_length_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.001 [-0.156, 0.371] | 0.5217 | 0.5217 | 0.5885 | no | boundary 3 (Γ) |
| 216 | M1_three_way | M3_max_turn_deg_1s | g6 vs g1 | 0 | — | — | — | — (no pairs) | 0.6742 | — | not interpreted |
| 217 | M1_three_way | M3_max_turn_deg_1s | g6 vs g2 | 17 | Wilcoxon signed-rank (Pratt, exact) | 21.234 [-7.832, 57.416] | 0.1594 | 0.2462 | 0.6600 | no | not interpreted |
| 218 | M1_three_way | M3_max_turn_deg_1s | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 43.549 [29.849, 52.974] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 219 | M1_three_way | M3_max_turn_deg_1s | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 220 | M1_three_way | M3_max_turn_deg_1s | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 16.709 [-6.790, 28.178] | 0.1231 | 0.2462 | 0.3693 | no | boundary 3 (Γ) |
| 221 | M1_three_way | M5_mean_m | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 222 | M1_three_way | M5_mean_m | g6 vs g2 | 17 | Wilcoxon signed-rank (Pratt, exact) | 0.167 [0.102, 0.260] | 0.0008 | 0.0017 | 0.0006 | yes | not interpreted |
| 223 | M1_three_way | M5_mean_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.239 [0.150, 0.313] | <0.0001 | <0.0001 | <0.0001 | yes | boundary 4 (C2+C3 jointly) |
| 224 | M1_three_way | M5_mean_m | g6 vs g4 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 225 | M1_three_way | M5_mean_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.048 [-0.094, 0.228] | 0.5706 | 0.5706 | 0.5706 | no | outside boundary 3 |
| 226 | M1_three_way | M5_max_m | g6 vs g1 | 0 | — | — | — | — (no pairs) | <0.0001 | — | not interpreted |
| 227 | M1_three_way | M5_max_m | g6 vs g2 | 17 | Wilcoxon signed-rank (Pratt, exact) | 0.619 [0.590, 0.997] | <0.0001 | <0.0001 | <0.0001 | yes | not interpreted |
| 228 | M1_three_way | M5_max_m | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, exact) | 0.302 [0.186, 0.452] | 0.0001 | 0.0001 | 0.0002 | yes | boundary 4 (C2+C3 jointly) |
| 229 | M1_three_way | M5_max_m | g6 vs g4 | 0 | — | — | — | — (no pairs) | 0.0017 | — | not interpreted |
| 230 | M1_three_way | M5_max_m | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, exact) | -0.168 [-0.296, 0.164] | 0.2024 | 0.2024 | 0.2024 | no | outside boundary 3 |
| 231 | M1_three_way | M6_flip_backs | g6 vs g1 | 0 | — | — | — | — (no pairs) | 0.0112 | — | not interpreted |
| 232 | M1_three_way | M6_flip_backs | g6 vs g2 | — | — | N/A (M6 undefined for g2) | — | — | — | — | — |
| 233 | M1_three_way | M6_flip_backs | g6 vs g3 | 20 | Wilcoxon signed-rank (Pratt, normal) | 2.500 [1.000, 5.500] | 0.0003 | 0.0006 | 0.0013 | yes | boundary 4 (C2+C3 jointly) |
| 234 | M1_three_way | M6_flip_backs | g6 vs g4 | 0 | — | — | — | — (no pairs) | 0.0097 | — | not interpreted |
| 235 | M1_three_way | M6_flip_backs | g6 vs g5 | 20 | Wilcoxon signed-rank (Pratt, normal) | -5.000 [-6.000, -1.500] | 0.0166 | 0.0166 | 0.0166 | yes | boundary 3 (Γ) |
| 236 | M1_three_way | M4_success | g6 vs g1 | 20 | exact McNemar (g6-only 20, gX-only 0) | 1.000 [1.000, 1.000] | <0.0001 | <0.0001 | (same) | yes | boundary 1 (local VO) |
| 237 | M1_three_way | M4_success | g6 vs g2 | 20 | exact McNemar (g6-only 3, gX-only 0) | 0.150 [0.000, 0.300] | 0.2500 | 0.7500 | (same) | no | context only |
| 238 | M1_three_way | M4_success | g6 vs g3 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | context only |
| 239 | M1_three_way | M4_success | g6 vs g4 | 20 | exact McNemar (g6-only 20, gX-only 0) | 1.000 [1.000, 1.000] | <0.0001 | <0.0001 | (same) | yes | boundary 2 (λ adaptation) |
| 240 | M1_three_way | M4_success | g6 vs g5 | 20 | exact McNemar (g6-only 0, gX-only 0) | 0.000 [0.000, 0.000] | 1.0000 | 1.0000 | (same) | no | not reported (boundary 3) |

## 4.4 Evidence gaps and limitations

**Attribution gaps (design of the ablation)**
- **L1. The components of the coupling cannot be isolated.**
  - g3 removes C2 and C3 together. There are no A4 (no C2) or A5 (no C3) groups, by decision Q2, so the g3 vs g6
    contrast is a joint effect and no gain or loss is attributed to C2 or C3 individually.
  - No group removes C1 (the unified J_total) as a whole. Only two of its elements are probed: λ adaptation (g4)
    and Γ (g5).
  - Consequently, neither C1 nor C3 has an individually identified contribution.
- **L2. There is no dedicated control group for C2 (cross-layer sharing).** In the g4 vs g6 contrast C2 is
  switched on in both groups, so it is evidence about λ adaptation only. The thesis makes no claim of superiority
  for cross-layer sharing.
- **L3. The hypothesised path-quality benefit of the C2 + C3 combination is not supported in this campaign.**
  - g6 has significantly longer paths than g3 in all six variants.
  - g6 has significantly larger turning angles in four variants and more flip-backs in three (§4.2.2).
  - This is reported as found. It was not retuned.

**Ceiling and floor effects (scenario design)**
- **L4. Success rates hit a ceiling for the strong groups.**
  - g3, g5 and g6 reach 20/20 in every variant. The Wilson 95 % lower bound is 0.84 per variant and 0.94 pooled,
    so success rate cannot separate these groups.
  - The static variants are also near the ceiling for g1 and g2.
  - Harder scenarios would be needed to test whether C2, C3 or Γ affect success. No such claim is made.
- **L5. Some groups hit a floor.**
  - g1 has 0/20 in every dynamic variant.
  - g4 has 0/20 in S3_climb_ne, head_on and M1_three_way, and only 1, 11 and 2 successes in S1_east, S2_north and
    crossing_90.
  - These cells have no or very few success-only metric values.

**Statistical limitations**
- **L6. Survivorship bias.**
  - The efficiency and smoothness metrics (M1–M3, M5, M6) are computed on successful runs only.
  - For g1, g2 and g4 they describe the surviving, and presumably easier, episodes, and are not compared with g6.
  - For g3, g5 and g6, with 100 % success, there is no survivorship selection.
- **L7. Multiple-testing scope.**
  - Holm correction runs over the five comparisons per (variant, metric) pair, not across metrics or variants.
  - The family composition affects the adjusted p. One result changes significance between families: S1_east, M5
    mean, g6 vs g3.
  - The Wilcoxon test (a rank location shift) and the bootstrap CI of the median difference can disagree, as in
    head_on, M5 max, g6 vs g3.
- **L8. Static variants have no seeded input** (run spec, limitation 3).
  - Their 20 runs are repetitions under simulator nondeterminism, not independent scenario draws.
  - Seed pairing in the static variants is nominal.

**Engineering-layer limitations**
- **L9. Global search time can exceed the declared T_g = 1.5 s.** The M1 maximum reaches 3499 ms for g6
  (crossing_90) and 5088.5 ms for g4 (S1_east). This is within the eq. (32) bound T_g ≤ 9.8 s.
- **L10. Stall breaker and hold.**
  - The 6-cycle hold overrides the J_total-optimal velocity. It is the same mechanism for all groups, but it fires
    with different frequency.
  - Override share: g6 2.41 %, with 37/120 runs triggering at least once; g5 2.64 %; g3 0.20 %.
  - Static-corner escape can be slow (v_esc ≈ 0.08 m/s).
  - Per-group figures are in Appendix A. They are diagnostics, not results.
- **L11. The engineering safeguards were debugged and validated only on g6** (decision Q-C): yaw pre-rotation,
  `map/z_min` 0.8 m and the 6-cycle hold. g1–g5 received no targeted tuning, which gives g6 an
  engineering-adaptation advantage.

**Model and setup limitations (agreed list, spec §10 and run spec)**
- **L12.** JPS optimality holds only on G_J (Prop. 1). There is no closed-loop stability proof (Stage 2 §4.2(d)).
- **L13. Obstacle input.**
  - Obstacles are constant-velocity and taken from ground truth.
  - **Ground-truth masking** removes dynamic obstacles and their octomap trails using `/gazebo/model_states`. The
    ego-planner reference does not get this.
- **L14.** PX4 tracking lags by about 0.8 s. The planner works on the reference, so M2, M3 and M5 describe the
  reference; safety is measured on odom.
- **L15. VO side switching (R3) and corridor flipping (R5) are measured, not removed.** g6 still has median M6 =
  1.0–2.5 in the dynamic variants.
- **L16.** There is one simulated world, 6 variants and 20 runs per cell.
- **L17.** The Gazebo real-time factor was not collected, so invalid-run rule I-1 could not be applied.
- **L18.** The external ego-planner baseline (Q-E) is not part of this chapter's results.

**Provenance notes (for the methods section)**
- **P1.** The frozen success prior was revised once, on 2026-10-30, to include three retried seeds:
  S3_climb_ne/g1/seed5, S1_east/g2/seed1 and S1_east/g3/seed1. Their earlier attempts were infrastructure-invalid
  (I-7, I-8, I-8). Only the static counts of g1, g2 and g3 changed, each by +1.
- **P2.** The post-processing used the working-tree `post_process_ablation.py`, which had uncommitted changes. That
  version should be committed together with these results.
- **P3.** Table 4-1 reports M1 as mean / mean P95 / max and the other metrics as median [IQR], following the frozen
  spec §4. The script does not produce a cross-run P95 for M2–M6.

## Appendix A — Diagnostics (not part of the results comparison)

These values come from `out_phase6_full/stall_totals_by_group.csv` and `aggregates_*.csv` (all valid runs) and
from `tests_*.csv` (`success_wo_trigger`). They explain the engineering layer and are **not** used as evidence for
or against any component.

| Group | Valid runs | Runs with ≥1 stall-break | Stall breaks (sum) | Hold cycles (sum) | Mean override share [%] |
|---|---|---|---|---|---|
| g1 | 120 | 25 | 142 | 710 | 1.69 |
| g2 | 120 | 2 | 29 | 143 | 0.17 |
| g3 | 120 | 13 | 14 | 70 | 0.20 |
| g4 | 120 | 120 | 139 | 695 | 0.96 |
| g5 | 120 | 38 | 218 | 1090 | 2.64 |
| g6 | 120 | 37 | 200 | 1000 | 2.41 |

| Variant | Group | Successes needing a trigger | Success without trigger k/n |
|---|---|---|---|
| S1_east | g1 | 14/20 | 6/20 |
| S1_east | g2 | 0/20 | 20/20 |
| S1_east | g3 | 0/20 | 20/20 |
| S1_east | g4 | 1/1 | 0/20 |
| S1_east | g5 | 17/20 | 3/20 |
| S1_east | g6 | 17/20 | 3/20 |
| S2_north | g1 | 0/20 | 20/20 |
| S2_north | g2 | 0/20 | 20/20 |
| S2_north | g3 | 0/20 | 20/20 |
| S2_north | g4 | 11/11 | 0/20 |
| S2_north | g5 | 0/20 | 20/20 |
| S2_north | g6 | 0/20 | 20/20 |
| S3_climb_ne | g1 | 8/19 | 11/20 |
| S3_climb_ne | g2 | 0/20 | 20/20 |
| S3_climb_ne | g3 | 0/20 | 20/20 |
| S3_climb_ne | g4 | 0/0 | 0/20 |
| S3_climb_ne | g5 | 13/20 | 7/20 |
| S3_climb_ne | g6 | 11/20 | 9/20 |
| head_on | g1 | 0/0 | 0/20 |
| head_on | g2 | 0/20 | 20/20 |
| head_on | g3 | 7/20 | 13/20 |
| head_on | g4 | 0/0 | 0/20 |
| head_on | g5 | 1/20 | 19/20 |
| head_on | g6 | 0/20 | 20/20 |
| crossing_90 | g1 | 0/0 | 0/20 |
| crossing_90 | g2 | 0/20 | 20/20 |
| crossing_90 | g3 | 6/20 | 14/20 |
| crossing_90 | g4 | 2/2 | 0/20 |
| crossing_90 | g5 | 1/20 | 19/20 |
| crossing_90 | g6 | 0/20 | 20/20 |
| M1_three_way | g1 | 0/0 | 0/20 |
| M1_three_way | g2 | 0/17 | 17/20 |
| M1_three_way | g3 | 0/20 | 20/20 |
| M1_three_way | g4 | 0/0 | 0/20 |
| M1_three_way | g5 | 6/20 | 14/20 |
| M1_three_way | g6 | 9/20 | 11/20 |

| Variant | Comparison | Effect g6−gX [CI] | p Holm | Sig. |
|---|---|---|---|---|
| S1_east | g6 vs g1 | -0.150 [-0.350, 0.050] | 0.7500 | no |
| S1_east | g6 vs g2 | -0.850 [-1.000, -0.700] | 0.0001 | yes |
| S1_east | g6 vs g3 | -0.850 [-1.000, -0.700] | 0.0001 | yes |
| S1_east | g6 vs g4 | 0.150 [0.000, 0.300] | 0.7500 | no |
| S1_east | g6 vs g5 | 0.000 [-0.250, 0.250] | 1.0000 | no |
| S2_north | g6 vs g1 | 0.000 [0.000, 0.000] | 1.0000 | no |
| S2_north | g6 vs g2 | 0.000 [0.000, 0.000] | 1.0000 | no |
| S2_north | g6 vs g3 | 0.000 [0.000, 0.000] | 1.0000 | no |
| S2_north | g6 vs g4 | 1.000 [1.000, 1.000] | <0.0001 | yes |
| S2_north | g6 vs g5 | 0.000 [0.000, 0.000] | 1.0000 | no |
| S3_climb_ne | g6 vs g1 | -0.100 [-0.350, 0.150] | 1.0000 | no |
| S3_climb_ne | g6 vs g2 | -0.550 [-0.750, -0.350] | 0.0049 | yes |
| S3_climb_ne | g6 vs g3 | -0.550 [-0.750, -0.350] | 0.0049 | yes |
| S3_climb_ne | g6 vs g4 | 0.450 [0.250, 0.650] | 0.0117 | yes |
| S3_climb_ne | g6 vs g5 | 0.100 [-0.150, 0.350] | 1.0000 | no |
| head_on | g6 vs g1 | 1.000 [1.000, 1.000] | <0.0001 | yes |
| head_on | g6 vs g2 | 0.000 [0.000, 0.000] | 1.0000 | no |
| head_on | g6 vs g3 | 0.350 [0.150, 0.550] | 0.0469 | yes |
| head_on | g6 vs g4 | 1.000 [1.000, 1.000] | <0.0001 | yes |
| head_on | g6 vs g5 | 0.050 [0.000, 0.150] | 1.0000 | no |
| crossing_90 | g6 vs g1 | 1.000 [1.000, 1.000] | <0.0001 | yes |
| crossing_90 | g6 vs g2 | 0.000 [0.000, 0.000] | 1.0000 | no |
| crossing_90 | g6 vs g3 | 0.300 [0.100, 0.500] | 0.0938 | no |
| crossing_90 | g6 vs g4 | 1.000 [1.000, 1.000] | <0.0001 | yes |
| crossing_90 | g6 vs g5 | 0.050 [0.000, 0.150] | 1.0000 | no |
| M1_three_way | g6 vs g1 | 0.550 [0.350, 0.750] | 0.0049 | yes |
| M1_three_way | g6 vs g2 | -0.300 [-0.550, 0.000] | 0.2188 | no |
| M1_three_way | g6 vs g3 | -0.450 [-0.650, -0.250] | 0.0117 | yes |
| M1_three_way | g6 vs g4 | 0.550 [0.350, 0.750] | 0.0049 | yes |
| M1_three_way | g6 vs g5 | -0.150 [-0.450, 0.150] | 0.5488 | no |
