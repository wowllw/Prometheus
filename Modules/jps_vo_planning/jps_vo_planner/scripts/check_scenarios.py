#!/usr/bin/env python3
"""Offline geometric check of the experiment scenario files (no ROS master, no Gazebo).

For every variant and every seed it reproduces exactly what dyn_obstacles.py would spawn (same Obstacle class,
same seed jitter) and checks:
  C1 obstacle vs static cylinders : surface clearance >= 0.05 m over the whole motion (when z ranges overlap)
  C2 obstacle vs obstacle         : surface clearance >= 0.10 m at all times
  C3 bounds                       : |x|,|y| <= 15 (textured ground), z in [0.5 + r_o, 3.0 - r_o]
  C4 nominal conflict             : every obstacle in expected_encounters comes closer than R_o = r_q + r_o + d_min
                                    to a nominal UAV that flies the static shortest route at v_c and never avoids,
                                    i.e. the encounter really happens if the planner does nothing
The nominal UAV (route on a 0.1 m grid, cylinders inflated by r + r_infl) also triggers the obstacles.

Usage: check_scenarios.py <scenario.yaml> [<scenario.yaml> ...] [--seeds 1-20] [--d_min 0.2]
"""
import argparse
import heapq
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from dyn_obstacles import Obstacle, load_scenario  # noqa: E402  same spawn logic as the real script
import random  # noqa: E402

R_Q, E_CUT, V_C = 0.30, 0.10, 0.80


def load_cylinders():
    path = os.path.join(HERE, '..', 'config', 'scenarios', 'planning_easy_cylinders.yaml')
    return yaml.safe_load(open(path))['cylinders']


def static_route(cyl, start, goal, r_infl, res=0.1, lo=-15.0, n=300):
    """8-connected shortest route (no corner cutting) in the xy plane; returns list of (x, y)"""
    occ = [[False] * n for _ in range(n)]
    for c in cyl:
        rr = c['r'] + r_infl
        i0, i1 = int((c['x'] - rr - lo) / res) - 1, int((c['x'] + rr - lo) / res) + 1
        j0, j1 = int((c['y'] - rr - lo) / res) - 1, int((c['y'] + rr - lo) / res) + 1
        for i in range(max(0, i0), min(n, i1 + 1)):
            for j in range(max(0, j0), min(n, j1 + 1)):
                if (lo + (i + .5) * res - c['x']) ** 2 + (lo + (j + .5) * res - c['y']) ** 2 <= rr * rr:
                    occ[i][j] = True
    idx = lambda p: (int((p[0] - lo) / res), int((p[1] - lo) / res))
    s, g = idx(start), idx(goal)
    dist, par, pq = {s: 0.0}, {}, [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == g:
            break
        if d > dist[u]:
            continue
        for a in (-1, 0, 1):
            for b in (-1, 0, 1):
                if a == b == 0:
                    continue
                v = (u[0] + a, u[1] + b)
                if not (0 <= v[0] < n and 0 <= v[1] < n) or occ[v[0]][v[1]]:
                    continue
                if a and b and (occ[u[0] + a][u[1]] or occ[u[0]][u[1] + b]):
                    continue
                nd = d + math.hypot(a, b)
                if nd < dist.get(v, 1e18):
                    dist[v], par[v] = nd, u
                    heapq.heappush(pq, (nd, v))
    if g not in dist:
        return None
    cells = [g]
    while cells[-1] != s:
        cells.append(par[cells[-1]])
    return [(lo + (i + .5) * res, lo + (j + .5) * res) for i, j in reversed(cells)]


def nominal_uav(route, z0, z1):
    """position function p(t) for constant speed V_C along the route, z interpolated by arc length"""
    seg = [math.dist(a, b) for a, b in zip(route, route[1:])]
    total = sum(seg)

    def p(t):
        s = min(V_C * t, total)
        acc = 0.0
        for (a, b), l in zip(zip(route, route[1:]), seg):
            if acc + l >= s and l > 0:
                f = (s - acc) / l
                return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), z0 + (z1 - z0) * s / total)
            acc += l
        return (route[-1][0], route[-1][1], z1)
    return p, total / V_C


def check_variant(cfg, cyl, seeds, d_min):
    r_infl = R_Q + d_min + E_CUT
    hover, goal = cfg['uav']['hover'], cfg['uav']['goal']
    route = static_route(cyl, hover, goal, r_infl)
    if route is None:
        return ['route blocked'], {}
    p_uav, t_route = nominal_uav(route, hover[2], goal[2])
    expected = set(cfg.get('expected_encounters', []))
    problems, stats = [], {'route_len': t_route * V_C, 'dcpa': {}}
    for seed in seeds:
        rng = random.Random(seed)
        obs = [Obstacle(i, c, cfg.get('prefix', 'dyn_obs_'), rng) for i, c in enumerate(cfg['obstacles'])]
        dmin_uav = {o.name: (1e9, None) for o in obs}
        t, dt = 0.0, 0.05
        while t <= t_route + 25.0:
            pu = p_uav(t)
            states = []
            for o in obs:
                if o.t_start is None:
                    if o.trigger_point is None:
                        o.t_start = o.delay
                    elif math.dist(pu, o.trigger_point) <= o.trigger_radius:
                        o.t_start = t + o.delay
                po, _ = o.state(t)
                states.append((o, po))
                # C3
                if not (abs(po[0]) <= 15 and abs(po[1]) <= 15 and 0.5 + o.r <= po[2] <= 3.0 - o.r):
                    problems.append('seed %d %s out of bounds at t=%.1f %s' % (seed, o.name, t, tuple(round(x, 2) for x in po)))
                # C1
                for c in cyl:
                    if po[2] + o.r > c['z_min'] and po[2] - o.r < c['z_max']:
                        if math.hypot(po[0] - c['x'], po[1] - c['y']) - c['r'] - o.r < 0.05:
                            problems.append('seed %d %s touches %s at t=%.1f' % (seed, o.name, c['name'], t))
                d = math.dist(pu, po)
                if d < dmin_uav[o.name][0] and t <= t_route:
                    dmin_uav[o.name] = (d, t)
            # C2
            for i in range(len(states)):
                for j in range(i + 1, len(states)):
                    (oi, pi), (oj, pj) = states[i], states[j]
                    if math.dist(pi, pj) - oi.r - oj.r < 0.10:
                        problems.append('seed %d %s meets %s at t=%.1f' % (seed, oi.name, oj.name, t))
            t += dt
        # C4
        for o in obs:
            d, tc = dmin_uav[o.name]
            R_o = R_Q + o.r + d_min
            stats['dcpa'].setdefault(o.name, []).append(d)
            if o.name in expected and d >= R_o:
                problems.append('seed %d %s: no nominal conflict (d_cpa %.2f >= R_o %.2f)' % (seed, o.name, d, R_o))
            if o.t_start is None:
                problems.append('seed %d %s never triggered' % (seed, o.name))
    return sorted(set(problems)), stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('--seeds', default='1-20')
    ap.add_argument('--d_min', type=float, default=0.2)
    a = ap.parse_args()
    lo, hi = (int(x) for x in a.seeds.split('-'))
    seeds = list(range(lo, hi + 1))
    cyl = load_cylinders()
    bad = 0
    for f in a.files:
        raw = yaml.safe_load(open(f))
        for v in (raw.get('variants') or {'': None}):
            cfg = load_scenario(f, v)
            problems, stats = check_variant(cfg, cyl, seeds, a.d_min)
            dc = '  '.join('%s d_cpa[min/max]=%.2f/%.2f' % (k, min(x), max(x)) for k, x in stats.get('dcpa', {}).items())
            print('%-28s %-14s route %.1f m  %s  %s' % (os.path.basename(f), v, stats.get('route_len', float('nan')),
                                                         'OK' if not problems else 'PROBLEMS (%d)' % len(problems), dc))
            for p in problems[:8]:
                print('    -', p)
            bad += bool(problems)
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
