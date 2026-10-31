"""Run evaluation for V1/V2/V3: standard rows from the project's post-processing script (run per arm on its own
index), extra per-run diagnostics from the run logs, safety gates, and the V3 cross-arm statistics computed with
the statistics functions of the same post-processing script (Wilcoxon-Pratt, Holm, bootstrap CI, exact McNemar)."""
import csv
import importlib.util
import math
import os

import numpy as np
import yaml

from fixlib import sh, log, GateFailure

PP = None  # the project's post-processing module (imported from its discovered path)


def load_pp(R):
    global PP
    if PP is None:
        spec = importlib.util.spec_from_file_location('post_process_ablation', R['post_process'])
        PP = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(PP)
    return PP


def fnum(x):
    try:
        v = float(x)
        return v
    except (TypeError, ValueError):
        return float('nan')


def _csv(path):
    if not os.path.isfile(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def _yaml(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return {}


# ---------------------------------------------------------------- per arm: the project's own post-processing
def post_process_arm(R, out_dir, tag):
    """Runs post_process_ablation.py --index on one arm; returns its ablation_runs.csv rows (valid first attempts)."""
    res = os.path.join(out_dir, 'results')
    sh(['python3', R['post_process'], '--index', os.path.join(out_dir, 'batch_index.csv'), '--tag', tag, '--out', res])
    return _csv(os.path.join(res, 'ablation_runs.csv'))


def success(row):
    """Success as in Chapter 4: goal reached without contact (static), M4 (dynamic)."""
    pp = PP
    if row['variant'] in pp.STATIC:
        return int(fnum(row['goal_success']) == 1)
    m4 = row.get('M4_success', '')
    return int(fnum(m4) == 1) if m4 not in ('', None) else int(fnum(row['goal_success']) == 1)


def collided(row):
    return 'collision' in str(row.get('end_reason', ''))


# ---------------------------------------------------------------- per run: extra diagnostics from the logs
def run_extras(log_dir, eps_w=0.05, v_min=0.1, window=2.5):
    meta = _yaml(os.path.join(log_dir, 'planner_meta.yaml'))
    const = meta.get('constants', {}) or {}
    r_q = fnum(const.get('r_q', 0.3))
    eps_w = fnum(const.get('eps_w', eps_w))
    v_min = fnum(const.get('v_min', v_min))
    goal = meta.get('goal', [0, 0, 0])
    out = {'fixes': meta.get('fixes')}
    cyc = [r for r in _csv(os.path.join(log_dir, 'cycles.csv')) if r.get('stall_hold_remaining') not in (None, '')]
    reasons = {}
    low = n = 0
    lam_t = []
    for r in cyc:
        rr = r['replan_reason']
        if rr and rr != 'none':
            reasons[rr] = reasons.get(rr, 0) + 1
        p = [fnum(r['p_ref_x']), fnum(r['p_ref_y']), fnum(r['p_ref_z'])]
        if r['stall_hold_remaining'] == '0' and r['stall_break_triggered'] == '0' and math.dist(p, goal) > 0.5:
            n += 1
            low += math.sqrt(sum(fnum(r[k]) ** 2 for k in ('v_star_x', 'v_star_y', 'v_star_z'))) < v_min
        lam_t.append((fnum(r['t']), fnum(r['lambda'])))
    out['replans'] = reasons
    out['low_speed_share'] = low / n if n else float('nan')
    sw = {}
    for r in _csv(os.path.join(log_dir, 'paths.csv')):
        if r['is_switch'] == '1' and r['replan_reason'] != 'goal':
            sw[r['t']] = r['replan_reason']
    out['path_switches'] = len(sw)
    # dynamic surface distance time series on odometry (ground truth), min over obstacles per sample
    series = {}
    for r in _csv(os.path.join(log_dir, 'odom_gt.csv')):
        if r.get('obs_name') in ('-', '', None):
            continue
        d = math.dist([fnum(r['odom_x']), fnum(r['odom_y']), fnum(r['odom_z'])],
                      [fnum(r['obs_x']), fnum(r['obs_y']), fnum(r['obs_z'])]) - r_q - fnum(r['obs_r'])
        t = fnum(r['t'])
        series[t] = min(series.get(t, float('inf')), d)
    ts = sorted(series)
    out['dyn_series'] = [(t, series[t]) for t in ts]
    out['dyn_min'] = min(series.values()) if series else float('nan')
    # R4: voluntary switches that met the eps_w margin but were held by the dwell while a threat was present
    held = []
    for r in _csv(os.path.join(log_dir, 'switches_ext.csv')):
        if r['is_switch'] == '1' and r['accepted'] == '0' and r['forced'] == '0' and fnum(r['dwell_left']) > 0.0 and \
                fnum(r['C_old']) - fnum(r['C_new']) > eps_w:
            t = fnum(r['t'])
            prev = [l for (tc, l) in lam_t if tc <= t]
            if prev and prev[-1] > 0.0:
                win = [d for (tt, d) in out['dyn_series'] if t <= tt <= t + window]
                held.append({'t': t, 'margin': fnum(r['C_old']) - fnum(r['C_new']), 'win_min_dyn': min(win) if win else float('nan')})
    out['r4_held_under_threat'] = held
    return out


def run_timeline(log_dir, top=4):
    """Code-generated failure timeline: height before/during/after threats, zero-speed optimum share, stall breaks,
    the slowest global searches, and the end state."""
    meta = _yaml(os.path.join(log_dir, 'planner_meta.yaml'))
    t0, goal = fnum(meta.get('t0')), meta.get('goal', [0, 0, 0])
    cyc = [r for r in _csv(os.path.join(log_dir, 'cycles.csv')) if r.get('stall_hold_remaining') not in (None, '')]
    ext = {r['t']: r for r in _csv(os.path.join(log_dir, 'cycles_ext.csv'))}
    if not cyc:
        return ['no cycles']
    p = lambda r: [fnum(r['p_ref_x']), fnum(r['p_ref_y']), fnum(r['p_ref_z'])]
    thr = [r for r in cyc if fnum(r['lambda']) > 0]
    out = ['goal %s; %d cycles from t0 + %.1f s to t0 + %.1f s' % (goal, len(cyc), fnum(cyc[0]['t']) - t0, fnum(cyc[-1]['t']) - t0)]
    if thr:
        out.append('threat phase (lambda > 0): t0 + %.1f .. %.1f s, reference height %.2f .. %.2f m' % (
            fnum(thr[0]['t']) - t0, fnum(thr[-1]['t']) - t0, min(p(r)[2] for r in thr), max(p(r)[2] for r in thr)))
    br = [i for i, r in enumerate(cyc) if r['stall_break_triggered'] == '1']
    if br:
        zo = [sum(int(fnum(ext.get(c['t'], {}).get('zero_is_opt', 0)) or 0) for c in cyc[max(0, i - 12):i]) for i in br]
        out.append('%d stall breaks between t0 + %.1f and %.1f s at height %.2f .. %.2f m, %.2f .. %.2f m from the goal; '
                   'lambda at the breaks %.2f .. %.2f; v = 0 was the optimum in %d .. %d of the 12 cycles before each break' % (
                       len(br), fnum(cyc[br[0]]['t']) - t0, fnum(cyc[br[-1]]['t']) - t0, min(p(cyc[i])[2] for i in br),
                       max(p(cyc[i])[2] for i in br), min(math.dist(p(cyc[i]), goal) for i in br),
                       max(math.dist(p(cyc[i]), goal) for i in br), min(fnum(cyc[i]['lambda']) for i in br),
                       max(fnum(cyc[i]['lambda']) for i in br), min(zo), max(zo)))
    slow = sorted(((fnum(r['plan_ms_global']), fnum(r['t']) - t0, r['replan_reason']) for r in cyc if r['replan_reason'] != 'none'),
                  reverse=True)[:top]
    out.append('slowest global searches: %s' % ', '.join('%.0f ms (%s, t0 + %.1f s)' % (a, c, b) for a, b, c in slow))
    out.append('end: reference %.2f m from the goal at height %.2f m' % (math.dist(p(cyc[-1]), goal), p(cyc[-1])[2]))
    return out


def fixes_match(meta_fixes, flags):
    """planner_meta.yaml 'fixes' block (written only when a flag is on) against the preset flags."""
    if not any(flags.values()):
        return meta_fixes is None
    if not isinstance(meta_fixes, dict):
        return False
    return all(int(meta_fixes.get(k, -1)) == int(v) for k, v in flags.items())


# ---------------------------------------------------------------- V3 statistics (post-processing functions)
CONTRASTS = [('A', 'g6_on', 'g6_off', 'g6 fixed vs g6 frozen'), ('B', 'g6_on', 'g3_off', 'g6 fixed vs g3 frozen'),
             ('C', 'g3_on', 'g3_off', 'g3 fixed vs g3 frozen'), ('D', 'g6_off', 'g3_off', 'g6 frozen vs g3 frozen (C2+C3 jointly)')]


def v3_tests(arms):
    """arms: dict arm -> list of ablation_runs rows. Returns test records (one Holm family per variant x metric)."""
    pp = PP
    recs = []
    for v in pp.VARIANTS:
        by = {}
        for arm, rows in arms.items():
            for r in rows:
                if r['variant'] == v and str(r['valid']) == '1':
                    by[('%s_%s' % (r['group'], arm), str(r['seed']))] = r
        binm = 'goal_success' if v in pp.STATIC else 'M4_success'
        for m in pp.CONT_TESTS + ['stall_breaks', binm]:
            fam, tmp = {}, {}
            for cid, a_key, b_key, label in CONTRASTS:
                seeds = sorted({s for (k, s) in by if k == a_key} & {s for (k, s) in by if k == b_key}, key=int)
                a = [fnum(by[(a_key, s)][m]) for s in seeds]
                b = [fnum(by[(b_key, s)][m]) for s in seeds]
                rec = {'variant': v, 'metric': m, 'contrast': cid, 'label': label, 'n_pairs': len(seeds)}
                if not seeds:
                    rec['note'] = 'no pairs'
                    fam[cid] = None
                elif m == binm:
                    p, n10, n01 = pp.mcnemar_exact([int(x) for x in a], [int(x) for x in b])
                    eff, lo, hi = pp.boot_ci([x - y for x, y in zip(a, b)], np.mean)
                    rec.update(test='exact McNemar', a_only=n10, b_only=n01, effect=eff, ci_low=lo, ci_high=hi, p_raw=p,
                               rate_a=float(np.mean(a)), rate_b=float(np.mean(b)))
                    fam[cid] = p
                else:
                    d = [x - y for x, y in zip(a, b)]
                    p, meth = pp.wilcoxon_pratt(d)
                    eff, lo, hi = pp.boot_ci(d, np.median)
                    rec.update(test='Wilcoxon signed-rank (Pratt, %s)' % meth, effect=eff, ci_low=lo, ci_high=hi, p_raw=p,
                               med_a=float(np.nanmedian(a)), med_b=float(np.nanmedian(b)))
                    if p is None:
                        rec['note'] = meth
                    fam[cid] = p
                tmp[cid] = rec
            adj = pp.holm(fam)
            for cid in tmp:
                tmp[cid]['p_holm'] = adj.get(cid)
                tmp[cid]['significant'] = int(adj.get(cid) is not None and adj[cid] < 0.05)
                recs.append(tmp[cid])
    return recs


def write_rows(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cols = []
    for r in rows:
        for k in r:
            if k not in cols:
                cols.append(k)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow(r)
