#!/usr/bin/env python3
"""Evaluate one run folder written by jps_vo_planner_node (planner_mode coupled, log_dir set).

Prints the online verification rows of subtask3_porting_plan.md section 3 (PASS / FAIL / INFO) and the fixed
metrics M1-M6 of subtask4_ablation_experiment_spec.md section 4, computed with the preprocessing rules P-2 ... P-8
of experiment_run_notes.md. Writes <run>/metrics.yaml. Does not invent anything: every value comes from the logs.

usage: verify_run.py <run_dir> [--timeout 70] [--cylinders <planning_easy_cylinders.yaml>]
"""
import argparse
import csv
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')


def vec(row, prefix):
    return tuple(fnum(row[prefix + a]) for a in ('_x', '_y', '_z'))


def seg_point_dist(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    L2 = sum(x * x for x in ab)
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, sum((p[i] - a[i]) * ab[i] for i in range(3)) / L2))
    return math.dist(p, [a[i] + t * ab[i] for i in range(3)])


def poly_dist(p, pts):
    return min(seg_point_dist(p, pts[i], pts[i + 1]) for i in range(len(pts) - 1)) if len(pts) > 1 else math.dist(p, pts[0])


def sample_poly(pts, step=0.1):
    out = []
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.ceil(math.dist(a, b) / step)))
        out += [[a[i] + (b[i] - a[i]) * k / n for i in range(3)] for k in range(n)]
    return out + [pts[-1]]


def hausdorff(a, b):
    return max(max(poly_dist(p, b) for p in sample_poly(a)), max(poly_dist(p, a) for p in sample_poly(b)))


def suffix_from(pts, p):
    """part of polyline pts after the projection of p (route still ahead)"""
    best, bi, foot = 1e18, 0, pts[0]
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        ab = [b[k] - a[k] for k in range(3)]
        L2 = sum(x * x for x in ab) or 1e-12
        t = max(0.0, min(1.0, sum((p[k] - a[k]) * ab[k] for k in range(3)) / L2))
        f = [a[k] + t * ab[k] for k in range(3)]
        d = math.dist(p, f)
        if d < best:
            best, bi, foot = d, i, f
    return [foot] + pts[bi + 1:]


def angle(a, b):
    na, nb = math.hypot(*a), math.hypot(*b)
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b)) / (na * nb)))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--timeout', type=float, default=70.0)
    ap.add_argument('--cylinders', default=os.path.join(HERE, '..', 'config', 'scenarios', 'planning_easy_cylinders.yaml'))
    a = ap.parse_args()
    d = a.run_dir
    meta = yaml.safe_load(open(os.path.join(d, 'planner_meta.yaml')))
    checks = yaml.safe_load(open(os.path.join(d, 'checks.yaml'))) if os.path.exists(os.path.join(d, 'checks.yaml')) else {}
    cyc = read_csv(os.path.join(d, 'cycles.csv'))
    paths = read_csv(os.path.join(d, 'paths.csv'))
    gt = read_csv(os.path.join(d, 'odom_gt.csv'))
    cyl = yaml.safe_load(open(a.cylinders))['cylinders']
    c = meta['constants']
    goal, t0 = meta['goal'], float(meta['t0'])
    v_min, d_min, r_q, dt = c['v_min'], c['d_min'], c['r_q'], c['dt']
    res, metrics = [], {}

    def row(rid, ok, text):
        res.append((rid, 'INFO' if ok is None else ('PASS' if ok else 'FAIL'), text))

    # ---- P-2 window ----
    rows = [r for r in cyc if fnum(r['t']) >= t0]
    if not rows:
        print('no cycles after t0'); sys.exit(1)
    t_end, end_reason = fnum(rows[-1]['t']), 'last_cycle'
    for r in rows:
        if math.dist(vec(r, 'p_ref'), goal) <= 0.3 and math.hypot(*vec(r, 'v_ref')) <= 0.1:
            t_end, end_reason = fnum(r['t']), 'goal'
            break
    if end_reason != 'goal' and t_end - t0 >= a.timeout:
        end_reason = 'timeout (F-2)'
    rows = [r for r in rows if fnum(r['t']) <= t_end]

    # ---- P-6 safety distances from odom + ground truth ----
    gt_rows = [r for r in gt if t0 <= fnum(r['t']) <= t_end]
    min_dyn, min_static, t_cpa = float('inf'), float('inf'), None
    seen_t = set()
    for r in gt_rows:
        p = (fnum(r['odom_x']), fnum(r['odom_y']), fnum(r['odom_z']))
        if r['obs_name'] not in ('-', ''):
            dd = math.dist(p, (fnum(r['obs_x']), fnum(r['obs_y']), fnum(r['obs_z']))) - r_q - fnum(r['obs_r'])
            if dd < min_dyn:
                min_dyn, t_cpa = dd, fnum(r['t'])
        if r['t'] in seen_t:
            continue
        seen_t.add(r['t'])
        for cy in cyl:
            if p[2] + r_q > cy['z_min'] and p[2] - r_q < cy['z_max']:
                min_static = min(min_static, math.hypot(p[0] - cy['x'], p[1] - cy['y']) - cy['r'] - r_q)
    dynamic = any(r['obs_name'] not in ('-', '') for r in gt_rows)

    # ---- F-3 stall: reference moved < 0.2 m in 15 s while not at the goal ----
    stall = False
    j = 0
    for i, r in enumerate(rows):
        while fnum(r['t']) - fnum(rows[j]['t']) > 15.0:
            j += 1
        if fnum(r['t']) - fnum(rows[j]['t']) >= 15.0 - dt and math.dist(vec(r, 'p_ref'), vec(rows[j], 'p_ref')) < 0.2 \
                and math.dist(vec(r, 'p_ref'), goal) > 0.3:
            stall = True
    reached = end_reason == 'goal'
    contact = min_static <= 0.0 or (dynamic and min_dyn <= 0.0)
    if stall and not reached:
        end_reason = 'stall (F-3)'
    if contact:
        end_reason += ' + collision (F-1)'

    # ---- verification rows ----
    row('1-d', reached, 'goal reached within 0.3 m / 0.1 m/s (reference): %s, end reason: %s, %.1f s' % (reached, end_reason, t_end - t0))
    row('1-e', min_static > 0.0, 'min static surface distance (odom vs cylinders) %.3f m; >= d_min - 0.087 = %.3f: %s'
        % (min_static, d_min - 0.087, min_static >= d_min - 0.087))
    row('1-f', checks.get('continuity_violations', 1) == 0, 'continuity violations %s (max start jump %.4f m < 0.01)'
        % (checks.get('continuity_violations'), checks.get('max_start_jump_m', float('nan'))))
    row('B', checks.get('format_violations', 1) == 0, 'B-spline format violations %s' % checks.get('format_violations'))
    loc = sorted(fnum(r['plan_ms_local']) for r in rows)
    p99 = loc[min(len(loc) - 1, int(0.99 * len(loc)))]
    row('2-g/3-i', p99 <= dt * 1000.0, 'local cycle P99 %.2f ms (<= %.0f ms), max %.2f ms' % (p99, dt * 1000.0, loc[-1]))
    glob = [fnum(r['plan_ms_global']) for r in rows if r['replan_reason'] != 'none']
    row('3-i', None, 'global steps %d, max %.1f ms (declared T_g %.0f ms; tau2 >= tau1 + T_g needs T_g <= %.0f ms)'
        % (len(glob), max(glob) if glob else 0.0, c['T_g'] * 1000, (c['tau2'] - c['tau1']) * 1000))
    row('2-d', checks.get('stale_obstacle_drops', 1) == 0, 'stale obstacle drops %s' % checks.get('stale_obstacle_drops'))
    if dynamic:
        row('2-e', min_dyn > 0.0, 'min dynamic surface distance (odom vs ground truth) %.3f m at t = %.1f s' % (min_dyn, (t_cpa or t0) - t0))
    inv = all(0 <= fnum(r['lambda']) <= 1 and 0 <= fnum(r['kappa']) <= 1 and (r['fallback_active'] == '1' or fnum(r['kappa']) <= fnum(r['lambda']) + 1e-9) for r in rows)
    row('3-f', inv, 'lambda, kappa in [0,1], kappa <= lambda (non-fallback) on all %d cycles' % len(rows))
    if not dynamic:
        row('3-g', max(fnum(r['lambda']) for r in rows) == 0.0 or meta['switches']['lambda_mode'] in (2, 3),  # lambda_mode enum: 2 = one (g2), 3 = const (g4)
            'static run: max lambda = %.3f' % max(fnum(r['lambda']) for r in rows))
    if dynamic and meta['switches']['enable_c2'] and t_cpa is not None:
        sw = [r for r in rows if r['replan_reason'] in ('risk_Dk', 'kappa') and fnum(r['t']) < t_cpa]
        row('3-h', None, 'C2 re-searches (risk_Dk / kappa) before the closest approach: %d (first at CPA - %.1f s)'
            % (len(sw), (t_cpa - fnum(sw[0]['t'])) if sw else float('nan')))

    # ---- metrics M1..M6 ----
    tot = [fnum(r['plan_ms_local']) + fnum(r['plan_ms_global']) for r in rows]
    tot_s = sorted(tot)
    metrics['M1_mean_ms'] = sum(tot) / len(tot)
    metrics['M1_p95_ms'] = tot_s[min(len(tot_s) - 1, int(0.95 * len(tot_s)))]
    metrics['M1_max_ms'] = tot_s[-1]
    metrics['M2_path_length_m'] = sum(math.dist(vec(a1, 'p_ref'), vec(a0, 'p_ref')) for a0, a1 in zip(rows, rows[1:]))
    m3, k = 0.0, 0
    for r in rows:  # P-5: angle between v_ref(t) and v_ref(t - 1 s)
        while fnum(r['t']) - fnum(rows[k]['t']) > 1.0:
            k += 1
        va, vb = vec(r, 'v_ref'), vec(rows[k], 'v_ref')
        if fnum(r['t']) - fnum(rows[k]['t']) >= 1.0 - dt / 2 and math.hypot(*va) >= v_min and math.hypot(*vb) >= v_min:
            m3 = max(m3, angle(va, vb))
    metrics['M3_max_turn_deg_1s'] = m3
    metrics['M4_success'] = int(reached and not contact) if dynamic else 'N/A'
    dev = [fnum(r['lateral_dev_ref_to_Pi0']) for r in rows]
    metrics['M5_mean_m'] = sum(dev) / len(dev)
    metrics['M5_max_m'] = max(dev)
    # M6 (P-8): flip-backs among global paths
    events = {}
    for r in paths:
        key = (fnum(r['t']), int(r['path_id']), r['replan_reason'], int(r['is_switch']))
        events.setdefault(key, []).append((fnum(r['x']), fnum(r['y']), fnum(r['z'])))
    ev = sorted(events.items())
    flips, history = 0, []
    for (t, pid, reason, is_switch), pts in ev:
        if reason != 'goal' and is_switch and history:
            for (th, ph) in history[:-1]:
                if t - th <= 10.0 and len(ph) > 1 and len(pts) > 1 and hausdorff(suffix_from(ph, pts[0]), pts) < 0.3:
                    flips += 1
                    break
        if is_switch or not history:
            history.append((t, pts))
    metrics['M6_flip_backs'] = flips if meta['switches']['lambda_mode'] != 2 else 'N/A'  # N/A for lambda == 1 (enum 2 = one, g2)
    metrics['diag_path_switches'] = sum(1 for (t, pid, reason, s), _ in ev if s and reason != 'goal')
    metrics['diag_fallback_cycles'] = sum(r['fallback_active'] == '1' for r in rows)
    metrics['diag_encounter_flag'] = int(any(r['encounter_flag'] == '1' for r in rows))
    metrics['min_dyn_surface_dist_odom_m'] = min_dyn if dynamic else 'N/A'
    metrics['min_static_surface_dist_odom_m'] = min_static
    metrics['end_reason'] = end_reason
    metrics['duration_s'] = t_end - t0

    for rid, st, text in res:
        print('[%s] %-8s %s' % (st, rid, text))
    print('--- metrics (spec section 4) ---')
    for kk, vv in metrics.items():
        print('%-32s %s' % (kk, ('%.4f' % vv) if isinstance(vv, float) else vv))
    with open(os.path.join(d, 'metrics.yaml'), 'w') as f:
        yaml.safe_dump({'verification': [{'row': r, 'status': s, 'text': t} for r, s, t in res], 'metrics': metrics}, f)


if __name__ == '__main__':
    main()
