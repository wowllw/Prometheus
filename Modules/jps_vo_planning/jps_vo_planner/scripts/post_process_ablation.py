#!/usr/bin/env python3
"""
[File name] post_process_ablation.py
[Purpose] Post-processing of the Subtask-4 ablation (doc/experiment_batch_run_notes.md sections 5-7,
doc/subtask4_ablation_experiment_run_spec.md sections 4, 6, 8):
  - per-run parsing of metrics.yaml (verify_run.py), checks.yaml, planner_meta.yaml, attempt.yaml, cycles.csv, odom_gt.csv
  - validity assessment: invalid codes I-0 .. I-11 (driver codes from the index + file-based checks here)
  - final end reason incl. F-4 (planner died / NaN) and F-5 (reference outside the map box)
  - cross-check of the stall fields (cycles.csv vs checks.yaml); a mismatch is a logging fault (I-8)
  - derived stall fields: override_cycles, override_share, trigger_any, success_wo_trigger
  - ablation_runs.csv (T2 columns), aggregates per variant (T3/T4), stall totals per group (T4.7),
    paired tests g6 vs gX (T5), invalid-run list (T6)
It never modifies a run directory and never invents values: a missing value stays empty.

Usage:
  post_process_ablation.py --check-run <run_dir>
      prints 'valid' or the invalid code (last line); reasons on stderr. Used by run_ablation_batch.sh.
  post_process_ablation.py --index ~/.ros/ablation/batch_index.csv [--tag formal] [--out <dir>]
      full pipeline over the index (default out: <index dir>/results)
  post_process_ablation.py --dirs <variant>:<group>:<seed>:<run_dir> ... [--out <dir>] [--tag adhoc]
      same per-run evaluation for directories without an index (e.g. the R1-R6 validation runs)
Only numpy and PyYAML are needed (no scipy on the target machine): the tests are implemented here.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GROUPS = ['g1', 'g2', 'g3', 'g4', 'g5', 'g6']
VARIANTS = ['S1_east', 'S2_north', 'S3_climb_ne', 'head_on', 'crossing_90', 'M1_three_way']
N_OBS = {'S1_east': 0, 'S2_north': 0, 'S3_climb_ne': 0, 'head_on': 1, 'crossing_90': 1, 'M1_three_way': 3}
STATIC = {v for v, n in N_OBS.items() if n == 0}
REQUIRED = ['cycles.csv', 'checks.yaml', 'planner_meta.yaml', 'odom_gt.csv', 'metrics.yaml']

# T1 constants (spec section 2); compared with planner_meta.yaml
T1 = {'r_q': 0.3, 'e_cut': 0.1, 'v_c': 0.8, 'dt': 0.1, 'tau_h': 4, 'm': 1.2, 'L_min': 2, 'psi_max_deg': 60,
      'theta_max_deg': 60, 'v_min': 0.1, 'w_theta': 1, 'w_a': 1, 'rho_g': 10, 'eps_w': 0.05, 'T_c': 0.8, 'T_g': 1.5,
      'dv_star': 1.6, 'v_max': 1, 'a_max': 0.8, 'd_min': 0.2, 'r_o_max': 0.3, 'tau1': 2.21421356, 'tau2': 12.0432082,
      'r_infl': 0.6, 'ell_w': 0.837758041}
ENGINEERING = {'tie_rule': 'strict', 'search_margin': 5, 'async_global': 1, 'replan_period': 1,
               'switch_threshold': 0.3, 'pre_rotate_yaw': 1}
# enum order in include/jps_vo_planner/coupling.h
LAMBDA_MODE = {'adaptive': 0, 'zero': 1, 'one': 2, 'const': 3}
LAMBDA_SHAPE = {'quintic_log': 0, 'linear': 1, 'logistic': 2}
THETA_C_MODE = {'linear': 0, 'fixed': 1, 'sqrt': 2}
XI_SOURCE = {'reference': 0, 'executed': 1}

T2_COLUMNS = ['variant', 'group', 'seed', 'attempt', 'valid', 'invalid_code', 'end_reason', 'duration_s',
              'M1_mean_ms', 'M1_p95_ms', 'M1_max_ms', 'M2_path_length_m', 'M3_max_turn_deg_1s', 'M4_success',
              'M5_mean_m', 'M5_max_m', 'M6_flip_backs', 'stall_breaks', 'stall_hold_cycles', 'override_cycles',
              'override_share', 'fallback_cycles', 'max_global_ms', 'cycles_over_dt',
              'min_static_surface_dist_odom_m', 'min_dyn_surface_dist_odom_m', 'diag_path_switches',
              'c2_researches_before_cpa', 'hover_z_before_t0', 'outlier_flag', 'notes',
              # extra columns (after the T2 block)
              'tag', 'goal_success', 'trigger_any', 'success_wo_trigger', 'invalid_reason', 'log_dir']
OUTLIER_METRICS = ['M1_mean_ms', 'M2_path_length_m', 'M3_max_turn_deg_1s', 'M5_mean_m', 'M5_max_m', 'M6_flip_backs']
CONT_TESTS = ['M1_mean_ms', 'M1_p95_ms', 'M2_path_length_m', 'M3_max_turn_deg_1s', 'M5_mean_m', 'M5_max_m',
              'M6_flip_backs']
BOOT_SEED, BOOT_N = 20261026, 10000


def eprint(*a):
    print(*a, file=sys.stderr)


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float('nan')


def load_yaml(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return None


def group_switches(group):
    """Expected numeric switch block of planner_meta.yaml for config/groups/<group>.yaml."""
    sw = load_yaml(os.path.join(PKG, 'config', 'groups', group + '.yaml'))['switches']
    lm = str(sw['lambda_mode'])
    const = 0.5
    if lm.startswith('const:'):
        const, lm = float(lm.split(':', 1)[1]), 'const'
    return {'lambda_mode': LAMBDA_MODE[lm], 'lambda_const': const, 'use_rho_in_pure_vo': int(bool(sw['use_rho_in_pure_vo'])),
            'enable_c2': int(bool(sw['enable_c2'])), 'enable_c3': int(bool(sw['enable_c3'])),
            'c3_soft_tracking': float(sw['c3_soft_tracking']), 'enable_gamma': int(bool(sw['enable_gamma'])),
            'lambda_shape': LAMBDA_SHAPE[sw['lambda_shape']], 'theta_c_mode': THETA_C_MODE[sw['theta_c_mode']],
            'xi_source': XI_SOURCE[sw['xi_source']]}


def close(a, b, tol=1e-3):
    if isinstance(b, str):
        return str(a) == b
    try:
        return abs(float(a) - float(b)) <= tol * max(1.0, abs(float(b)))
    except (TypeError, ValueError):
        return False


# ---------------------------------------------------------------- per-run evaluation
def evaluate_run(run_dir, variant=None, group=None, seed=None, attempt=1, tag='', driver_code='', driver_notes=''):
    """Returns (row dict with T2 columns, invalid code or '', list of reasons)."""
    att = load_yaml(os.path.join(run_dir, 'attempt.yaml')) or {}
    variant = variant or att.get('variant')
    group = group or att.get('group')
    seed = seed if seed not in (None, '') else att.get('seed')
    attempt = att.get('attempt', attempt)
    tag = tag or att.get('tag', '')
    row = {c: '' for c in T2_COLUMNS}
    row.update(variant=variant, group=group, seed=seed, attempt=attempt, tag=tag, log_dir=run_dir,
               hover_z_before_t0=att.get('hover_z_mean', ''), notes=driver_notes)
    reasons = []
    code = driver_code or ''
    if code:
        reasons.append('driver: %s %s' % (code, driver_notes))

    def invalid(c, why):
        nonlocal code
        reasons.append('%s: %s' % (c, why))
        if not code:
            code = c

    missing = [f for f in REQUIRED if not os.path.isfile(os.path.join(run_dir, f))]
    if missing:
        invalid('I-8', 'missing ' + ', '.join(missing))
        return finish(row, code, reasons)

    met_all = load_yaml(os.path.join(run_dir, 'metrics.yaml'))
    checks = load_yaml(os.path.join(run_dir, 'checks.yaml'))
    meta = load_yaml(os.path.join(run_dir, 'planner_meta.yaml'))
    if not met_all or not checks or not meta or 'metrics' not in met_all:
        invalid('I-8', 'unreadable metrics/checks/meta yaml')
        return finish(row, code, reasons)
    met = met_all['metrics']
    for k in ['M1_mean_ms', 'M1_p95_ms', 'M1_max_ms', 'M2_path_length_m', 'M3_max_turn_deg_1s', 'M4_success',
              'M5_mean_m', 'M5_max_m', 'M6_flip_backs', 'min_static_surface_dist_odom_m',
              'min_dyn_surface_dist_odom_m', 'diag_path_switches', 'duration_s', 'end_reason']:
        row[k] = met.get(k, '')
    for k in ['stall_breaks', 'stall_hold_cycles', 'fallback_cycles', 'max_global_ms', 'cycles_over_dt']:
        row[k] = checks.get(k, '')
    for v in met_all.get('verification', []) or []:
        if v.get('row') == '3-h':
            txt = v.get('text', '')
            try:
                row['c2_researches_before_cpa'] = int(txt.split('closest approach:')[1].split()[0])
            except (IndexError, ValueError):
                pass

    # ---- I-10 identity: group, switches, constants, engineering, runtime z_min ----
    if group and meta.get('group') != group:
        invalid('I-10', 'planner_meta group %s != %s' % (meta.get('group'), group))
    if group in GROUPS:
        exp = group_switches(group)
        got = meta.get('switches', {})
        bad = [k for k, v in exp.items() if not close(got.get(k), v)]
        if bad:
            invalid('I-10', 'switches differ from config/groups/%s.yaml: %s' % (group, bad))
    badc = [k for k, v in T1.items() if not close(meta.get('constants', {}).get(k), v)]
    if badc:
        invalid('I-10', 'constants differ from T1: %s' % badc)
    bade = [k for k, v in ENGINEERING.items() if not close(meta.get('engineering', {}).get(k), v)]
    if bade:
        invalid('I-10', 'engineering block differs: %s' % bade)
    if 'runtime_map_z_min' in att and not close(att['runtime_map_z_min'], 0.8):
        invalid('I-10', 'runtime map/z_min %s' % att['runtime_map_z_min'])

    # ---- I-4 obstacle input ----
    if int(checks.get('stale_obstacle_drops', 0) or 0) > 0:
        invalid('I-4', 'stale_obstacle_drops %s' % checks['stale_obstacle_drops'])
    t0 = float(meta['t0'])
    duration = fnum(met.get('duration_s'))
    with open(os.path.join(run_dir, 'odom_gt.csv')) as f:
        gt = list(csv.DictReader(f))
    if variant in N_OBS and N_OBS[variant] > 0:
        names = {r['obs_name'] for r in gt if r.get('obs_name') not in ('-', '', None)}
        if len(names) != N_OBS[variant]:
            invalid('I-4', 'ground truth has obstacles %s, expected %d' % (sorted(names), N_OBS[variant]))

    # ---- I-9 octomap ----
    if int(checks.get('no_cloud_cycles', 0) or 0) > 20:
        invalid('I-9', 'no_cloud_cycles %s' % checks['no_cloud_cycles'])

    # ---- cycles.csv: coverage (I-8), stall cross-check (I-8), NaN (F-4), map box (F-5) ----
    with open(os.path.join(run_dir, 'cycles.csv')) as f:
        cyc = [r for r in csv.DictReader(f) if r.get('stall_hold_remaining') not in (None, '')]
    end_reason = str(met.get('end_reason', ''))
    if not cyc:
        invalid('I-8', 'cycles.csv has no complete row')
        return finish(row, code, reasons)
    ts = [fnum(r['t']) for r in cyc]
    # completeness = every cycle the node executed is in the file. A late first row is NOT a log gap: the node
    # starts local cycles only after pre-rotation and its first global search (up to several s, e.g. g4).
    if 'cycles' in checks and len(cyc) != int(checks['cycles']):
        invalid('I-8', 'cycles.csv has %d complete rows, node counted %s cycles' % (len(cyc), checks['cycles']))
    row['notes'] = (row['notes'] + '; ' if row['notes'] else '') + 'first cycle %.1f s after t0' % (ts[0] - t0) \
        if ts[0] - t0 > 5.0 else row['notes']
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    if gaps and max(gaps) > 1.0:
        invalid('I-8', 'cycle gap %.2f s' % max(gaps))
    if not math.isnan(duration) and ts[-1] < t0 + duration - 0.5:
        invalid('I-8', 'cycles end %.2f s before t_end' % (t0 + duration - ts[-1]))

    goal = meta.get('goal', [0, 0, 0])
    n_break = sum(int(fnum(r['stall_break_triggered']) or 0) for r in cyc)
    n_hold = 0
    for a, b in zip(cyc, cyc[1:]):
        ha, hb = int(fnum(a['stall_hold_remaining'])), int(fnum(b['stall_hold_remaining']))
        near_goal = math.dist([fnum(b['p_ref_x']), fnum(b['p_ref_y']), fnum(b['p_ref_z'])], goal) <= 0.5
        if b['stall_break_triggered'] == '0' and ha > 0 and hb == ha - 1 and not near_goal:
            n_hold += 1
    if n_break != int(checks.get('stall_breaks', -1)) or n_hold != int(checks.get('stall_hold_cycles', -1)):
        invalid('I-8', 'stall fields: cycles.csv %d/%d vs checks.yaml %s/%s' %
                (n_break, n_hold, checks.get('stall_breaks'), checks.get('stall_hold_cycles')))

    win = [r for r in cyc if t0 <= fnum(r['t']) <= t0 + (duration if not math.isnan(duration) else 1e9)]
    keys = ['p_ref_x', 'p_ref_y', 'p_ref_z', 'v_ref_x', 'v_ref_y', 'v_ref_z']
    if any(math.isnan(fnum(r[k])) for r in win for k in keys):
        end_reason += ' + fault (F-4: NaN in reference)'
    if str(att.get('planner_crashed', 0)) == '1':
        end_reason += ' + fault (F-4: planner node died)'
    if any(abs(fnum(r['p_ref_x'])) > 25 or abs(fnum(r['p_ref_y'])) > 25 or not (0.8 - 1e-6 <= fnum(r['p_ref_z']) <= 3.0 + 1e-6)
           for r in win):
        end_reason += ' + band (F-5)'
    row['end_reason'] = end_reason

    # ---- I-2 odom jump > 1 m without contact ----
    pts, last_t = [], None
    for r in gt:
        t = fnum(r['t'])
        if t0 <= t <= t0 + (duration if not math.isnan(duration) else 1e9) and r['t'] != last_t:
            pts.append((fnum(r['odom_x']), fnum(r['odom_y']), fnum(r['odom_z'])))
            last_t = r['t']
    jump = max((math.dist(a, b) for a, b in zip(pts, pts[1:])), default=0.0)
    contact = fnum(met.get('min_static_surface_dist_odom_m')) <= 0 or fnum(met.get('min_dyn_surface_dist_odom_m')) <= 0
    if jump > 1.0 and not contact:
        invalid('I-2', 'odom jump %.2f m without contact' % jump)

    # ---- derived stall fields ----
    sb, sh = int(checks.get('stall_breaks', 0)), int(checks.get('stall_hold_cycles', 0))
    ncyc = int(checks.get('cycles', len(cyc)) or len(cyc))
    row['override_cycles'] = sb + sh
    row['override_share'] = round((sb + sh) / ncyc, 6) if ncyc else ''
    goal_ok = int(end_reason == 'goal')
    row['goal_success'] = goal_ok
    row['trigger_any'] = int(sb > 0)
    succ = goal_ok if variant in STATIC else (int(met.get('M4_success')) if met.get('M4_success') in (0, 1) else goal_ok)
    row['success_wo_trigger'] = int(succ == 1 and sb == 0)
    return finish(row, code, reasons)


def finish(row, code, reasons):
    row['invalid_code'] = code
    row['valid'] = 0 if code else 1
    row['invalid_reason'] = '; '.join(reasons)
    return row, code, reasons


# ---------------------------------------------------------------- statistics (no scipy)
def wilson(k, n, z=1.959964):
    if n == 0:
        return float('nan'), float('nan')
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def norm_sf(z):
    return 0.5 * math.erfc(z / math.sqrt(2))


def rank_avg(a):
    order = sorted(range(len(a)), key=lambda i: a[i])
    r = [0.0] * len(a)
    i = 0
    while i < len(a):
        j = i
        while j + 1 < len(a) and a[order[j + 1]] == a[order[i]]:
            j += 1
        for k in range(i, j + 1):
            r[order[k]] = (i + j) / 2 + 1
        i = j + 1
    return r


def wilcoxon_pratt(d):
    """Two-sided Wilcoxon signed-rank with Pratt zero handling. Exact null distribution when there are no zeros
    and no ties (n <= 25), normal approximation with zero/tie correction otherwise. Returns (p, method)."""
    d = [x for x in d if not math.isnan(x)]
    n = len(d)
    if n == 0:
        return None, 'no pairs'
    if all(x == 0 for x in d):
        return None, 'all differences 0'
    ad = [abs(x) for x in d]
    r = rank_avg(ad)
    r_plus = sum(ri for ri, x in zip(r, d) if x > 0)
    n0 = sum(1 for x in d if x == 0)
    nz = [x for x in ad if x != 0]
    ties = len(set(nz)) != len(nz)
    if n0 == 0 and not ties and n <= 25:
        tmax = n * (n + 1) // 2
        cnt = [0] * (tmax + 1)
        cnt[0] = 1
        for k in range(1, n + 1):
            for s in range(tmax, k - 1, -1):
                cnt[s] += cnt[s - k]
        t = min(r_plus, tmax - r_plus)
        p = 2 * sum(cnt[:int(round(t)) + 1]) / 2 ** n
        return min(1.0, p), 'exact'
    mn = n * (n + 1) / 4 - n0 * (n0 + 1) / 4
    var = n * (n + 1) * (2 * n + 1) / 24 - n0 * (n0 + 1) * (2 * n0 + 1) / 24
    _, tc = np.unique(np.array(ad), return_counts=True)
    var -= float(np.sum(tc ** 3 - tc)) / 48
    if var <= 0:
        return None, 'degenerate'
    z = (r_plus - mn) / math.sqrt(var)
    return min(1.0, 2 * norm_sf(abs(z))), 'normal'


def mcnemar_exact(a, b):
    """a, b: paired 0/1 lists (g6, gX). Two-sided exact binomial test on discordant pairs."""
    n10 = sum(1 for x, y in zip(a, b) if x == 1 and y == 0)
    n01 = sum(1 for x, y in zip(a, b) if x == 0 and y == 1)
    n = n10 + n01
    if n == 0:
        return 1.0, n10, n01
    k = min(n10, n01)
    p = 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return min(1.0, p), n10, n01


def holm(ps):
    """ps: dict key -> p or None. Returns key -> adjusted p (None stays None)."""
    items = sorted([(p, k) for k, p in ps.items() if p is not None])
    m, out, run = len(items), {k: None for k in ps}, 0.0
    for i, (p, k) in enumerate(items):
        run = max(run, min(1.0, (m - i) * p))
        out[k] = run
    return out


def boot_ci(diffs, stat):
    d = np.array([x for x in diffs if not math.isnan(x)], dtype=float)
    if d.size == 0:
        return float('nan'), float('nan'), float('nan')
    rng = np.random.default_rng(BOOT_SEED)
    idx = rng.integers(0, d.size, size=(BOOT_N, d.size))
    bs = stat(d[idx], axis=1)
    return float(stat(d)), float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))


def desc(vals):
    v = np.array([x for x in vals if not math.isnan(x)], dtype=float)
    if v.size == 0:
        return {}
    q1, med, q3 = np.percentile(v, [25, 50, 75])
    return {'mean': v.mean(), 'std': v.std(ddof=1) if v.size > 1 else 0.0, 'median': med, 'q1': q1, 'q3': q3,
            'max': v.max(), 'n': int(v.size)}


def fmt(x, nd=3):
    return '' if x is None or (isinstance(x, float) and math.isnan(x)) else ('%.*f' % (nd, x))


def ms(d, nd=3):
    return '%s ± %s (%d)' % (fmt(d['mean'], nd), fmt(d['std'], nd), d['n']) if d else ''


def miqr(d, nd=3):
    return '%s [%s, %s]' % (fmt(d['median'], nd), fmt(d['q1'], nd), fmt(d['q3'], nd)) if d else ''


# ---------------------------------------------------------------- pipeline
def mark_outliers(rows):
    for v in VARIANTS:
        for g in GROUPS:
            cell = [r for r in rows if r['variant'] == v and r['group'] == g and r['valid'] == 1]
            for m in OUTLIER_METRICS:
                vals = np.array([fnum(r[m]) for r in cell if not math.isnan(fnum(r[m]))])
                if vals.size < 4:
                    continue
                q1, q3 = np.percentile(vals, [25, 75])
                lim = q3 + 3 * (q3 - q1)
                for r in cell:
                    if fnum(r[m]) > lim:
                        r['outlier_flag'] = (r['outlier_flag'] + ' ' if r['outlier_flag'] else '') + m


def aggregate(rows, out):
    stall_tot = {g: [] for g in GROUPS}
    for v in VARIANTS:
        lines = []
        for g in GROUPS:
            cell = [r for r in rows if r['variant'] == v and r['group'] == g and r['valid'] == 1]
            stall_tot[g] += cell
            n = len(cell)
            col = lambda k: [fnum(r[k]) for r in cell]
            agg = {'variant': v, 'group': g, 'n_valid': n}
            if n == 0:
                lines.append(agg)
                continue
            k_goal = sum(int(r['goal_success']) for r in cell)
            agg['goal_success_k_n'] = '%d/%d' % (k_goal, n)
            agg['M1_mean_ms'] = fmt(np.mean(col('M1_mean_ms')), 2)
            agg['M1_p95_ms'] = fmt(np.mean(col('M1_p95_ms')), 2)
            agg['M1_max_ms'] = fmt(np.max(col('M1_max_ms')), 1)
            for m, nd in [('M2_path_length_m', 2), ('M3_max_turn_deg_1s', 1), ('M5_mean_m', 3), ('M5_max_m', 3),
                          ('M6_flip_backs', 2)]:
                if m == 'M6_flip_backs' and g == 'g2':
                    agg[m + '_mean_std'] = agg[m + '_median_iqr'] = 'N/A'
                    continue
                d = desc(col(m))
                agg[m + '_mean_std'], agg[m + '_median_iqr'] = ms(d, nd), miqr(d, nd)
            if v in STATIC:
                agg['M4_k_n_wilson'] = 'N/A'
            else:
                k4 = sum(1 for r in cell if str(r['M4_success']) == '1')
                lo, hi = wilson(k4, n)
                agg['M4_k_n_wilson'] = '%d/%d [%.2f, %.2f]' % (k4, n, lo, hi)
            sb, sh = col('stall_breaks'), col('stall_hold_cycles')
            succ = [r for r in cell if (int(r['goal_success']) if v in STATIC else str(r['M4_success']) == '1')]
            agg.update({
                'stall_breaks_sum': int(np.nansum(sb)), 'stall_breaks_mean_std': ms(desc(sb), 2),
                'stall_hold_cycles_sum': int(np.nansum(sh)), 'stall_hold_cycles_mean_std': ms(desc(sh), 2),
                'runs_with_trigger_k_n': '%d/%d' % (sum(int(r['trigger_any']) for r in cell), n),
                'mean_override_share_pct': fmt(100 * np.nanmean(col('override_share')), 2),
                'successes_needing_trigger': '%d/%d' % (sum(int(r['trigger_any']) for r in succ), len(succ)),
                'success_wo_trigger_k_n': '%d/%d' % (sum(int(r['success_wo_trigger']) for r in cell), n),
                'F2_F3_with_trigger': sum(1 for r in cell if int(r['trigger_any']) and
                                          ('F-2' in r['end_reason'] or 'F-3' in r['end_reason'])),
                'outlier_runs': sum(1 for r in cell if r['outlier_flag']),
            })
            lines.append(agg)
        write_csv(os.path.join(out, 'aggregates_%s.csv' % v), lines)
    tot = []
    for g in GROUPS:
        cell = stall_tot[g]
        tot.append({'group': g, 'valid_runs': len(cell),
                    'stall_breaks_sum': int(sum(fnum(r['stall_breaks']) for r in cell)),
                    'stall_hold_cycles_sum': int(sum(fnum(r['stall_hold_cycles']) for r in cell)),
                    'runs_with_trigger': sum(int(r['trigger_any']) for r in cell),
                    'mean_override_share_pct': fmt(100 * np.mean([fnum(r['override_share']) for r in cell]), 2) if cell else ''})
    write_csv(os.path.join(out, 'stall_totals_by_group.csv'), tot)


def tests(rows, out):
    for v in VARIANTS:
        res = []
        by = {(r['group'], str(r['seed'])): r for r in rows if r['variant'] == v and r['valid'] == 1}
        binaries = (['goal_success'] if v in STATIC else ['M4_success']) + ['success_wo_trigger']
        for m in CONT_TESTS + binaries:
            ps, recs = {}, {}
            for gx in GROUPS[:-1]:
                if m == 'M6_flip_backs' and gx == 'g2':
                    recs[gx] = {'variant': v, 'metric': m, 'comparison': 'g6 vs ' + gx, 'note': 'N/A (M6 undefined for g2)'}
                    ps[gx] = None
                    continue
                seeds = sorted({s for (g, s) in by if g == 'g6'} & {s for (g, s) in by if g == gx}, key=int)
                a = [fnum(by[('g6', s)][m]) for s in seeds]
                b = [fnum(by[(gx, s)][m]) for s in seeds]
                rec = {'variant': v, 'metric': m, 'comparison': 'g6 vs ' + gx, 'n_pairs': len(seeds)}
                if not seeds:
                    rec['note'] = 'no pairs'
                    ps[gx] = None
                elif m in binaries:
                    p, n10, n01 = mcnemar_exact([int(x) for x in a], [int(x) for x in b])
                    eff, lo, hi = boot_ci([x - y for x, y in zip(a, b)], np.mean)
                    rec.update(test='exact McNemar', g6_only=n10, gx_only=n01, effect='rate diff g6-gX', effect_value=fmt(eff),
                               ci_low=fmt(lo), ci_high=fmt(hi), p_raw=fmt(p, 4))
                    ps[gx] = p
                else:
                    d = [x - y for x, y in zip(a, b)]
                    p, meth = wilcoxon_pratt(d)
                    eff, lo, hi = boot_ci(d, np.median)
                    rec.update(test='Wilcoxon signed-rank (Pratt, %s)' % meth, effect='median paired diff g6-gX',
                               effect_value=fmt(eff), ci_low=fmt(lo), ci_high=fmt(hi), p_raw=fmt(p, 4) if p is not None else '')
                    if p is None:
                        rec['note'] = meth
                    ps[gx] = p
                recs[gx] = rec
            adj = holm(ps)
            for gx in GROUPS[:-1]:
                if adj.get(gx) is not None:
                    recs[gx]['p_holm'] = fmt(adj[gx], 4)
                    recs[gx]['significant_0.05'] = int(adj[gx] < 0.05)
                res.append(recs[gx])
        write_csv(os.path.join(out, 'tests_%s.csv' % v), res)


def write_csv(path, rows, cols=None):
    cols = cols or sorted({k for r in rows for k in r}, key=lambda k: (k not in ('variant', 'group', 'metric', 'comparison'), k))
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check-run')
    ap.add_argument('--index')
    ap.add_argument('--dirs', nargs='+')
    ap.add_argument('--tag', default=None, help='index rows with this tag only (default: formal; --dirs: adhoc)')
    ap.add_argument('--out')
    a = ap.parse_args()

    if a.check_run:
        _, code, reasons = evaluate_run(os.path.abspath(a.check_run))
        for r in reasons:
            eprint(r)
        print(code or 'valid')
        return

    rows = []
    if a.index:
        tag = a.tag or 'formal'
        with open(a.index) as f:
            idx = [r for r in csv.DictReader(f) if r['tag'] == tag]
        for r in idx:
            if r['invalid_code'] and not os.path.isfile(os.path.join(r['log_dir'], 'metrics.yaml')):
                row, _, _ = finish({c: '' for c in T2_COLUMNS}, r['invalid_code'], ['driver: %s %s' % (r['invalid_code'], r['notes'])])
                row.update(variant=r['variant'], group=r['group'], seed=r['seed'], attempt=r['attempt'], tag=tag,
                           log_dir=r['log_dir'], hover_z_before_t0=r['hover_z'], notes=r['notes'])
            else:
                # the run reached evaluation: its index code (if any) came from --check-run, i.e. from this file's
                # checks, so it is re-derived here instead of carried over (checks may have been corrected)
                row, _, _ = evaluate_run(r['log_dir'], r['variant'], r['group'], r['seed'], r['attempt'], tag,
                                         '', r['notes'])
            rows.append(row)
        # only the FIRST valid attempt of a cell counts (a valid run is never rerun). A later attempt can exist
        # only if an earlier one was classified invalid by a check that was corrected afterwards; it is kept in
        # ablation_runs.csv but marked and excluded from all statistics.
        first_valid = {}
        for row in sorted(rows, key=lambda r: int(r['attempt'])):
            key = (row['variant'], row['group'], str(row['seed']))
            if row['valid'] == 1:
                if key in first_valid:
                    row['valid'] = 0
                    row['invalid_code'] = 'SUPERSEDED'
                    row['invalid_reason'] = 'rerun after an earlier valid attempt (attempt %s)' % first_valid[key]
                else:
                    first_valid[key] = row['attempt']
        out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.index)), 'results')
    elif a.dirs:
        tag = a.tag or 'adhoc'
        for spec in a.dirs:
            v, g, s, d = spec.split(':', 3)
            row, _, _ = evaluate_run(d, v, g, s, 1, tag)
            rows.append(row)
        out = a.out or os.path.join(os.getcwd(), 'post_process_out')
    else:
        ap.error('one of --check-run, --index, --dirs is required')
    os.makedirs(out, exist_ok=True)
    mark_outliers(rows)
    write_csv(os.path.join(out, 'ablation_runs.csv'), rows, T2_COLUMNS)
    write_csv(os.path.join(out, 'invalid_runs.csv'),
              [r for r in rows if r['valid'] == 0], ['variant', 'group', 'seed', 'attempt', 'invalid_code', 'invalid_reason', 'log_dir'])
    aggregate(rows, out)
    tests(rows, out)
    nv = sum(r['valid'] for r in rows)
    print('runs %d, valid %d, invalid %d -> %s' % (len(rows), nv, len(rows) - nv, out))


if __name__ == '__main__':
    main()
