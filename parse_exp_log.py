#!/usr/bin/env python3
"""parse_exp_log.py - turn run_experiment.sh output into the Chapter 5 metric tables.

Usage:
    python3 parse_exp_log.py <results_root | run_dir> [...] [--out-prefix PREFIX]

Writes <PREFIX>_runs.csv (one row per run) and <PREFIX>_summary.csv (one row per scenario/drift/group);
PREFIX defaults to <first input>/exp_metrics.

Inputs per run directory (created by run_experiment.sh): run.bag, planner.log (wall-clock time-stamped
planner stdout), result.json, meta.json.

Metric definitions (all wall-clock; the simulation runs with use_sim_time=false):
  planning_time_ms     per replan attempt: from the "[drone no.1, replan N times]" header in planner.log
                       to the start_time stamp of the first Bspline with a new traj_id published before
                       the next header (start_time is set by updateTrajInfo() at the end of reboundReplan).
  feasibility_rate     replans that produced a new trajectory / replan attempts. Fallback cycles
                       (tail of the previous trajectory re-published) are not counted as feasible.
  fallback_count       number of "[DASC] optimization failed" warnings in /rosout_agg (cross-checked with
                       Bsplines that re-use an already published traj_id).
  snap_cost            EGO-Planner trajectories are cubic uniform B-splines, whose snap is identically zero
                       inside every span; the reported cost is the exact jerk-squared integral
                       sum ||q[i+3]-3q[i+2]+3q[i+1]-q[i]||^2 / ts^5 of each new trajectory (SNAP-cost proxy).
  margin_rho_mean      mean planning safety margin rho from "[DASC] rho=..." messages; groups built without
                       DASC use the fixed inflation (0.2 m), marked rho_source=config.
  clearance_mean/min   measured clearance of the flown path: distance from each true odometry sample after
                       the goal was sent to the nearest octomap obstacle point (last recorded cloud).
  outcome, flight_time, path_length   from result.json (exp_monitor.py).
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys

import numpy as np
import rosbag

PLANNER_NODE = "/uav1_ego_planner_node"
T_BSPLINE = "/uav1/planning/bspline"
T_ROSOUT = "/rosout_agg"
T_ODOM = "/uav1/mavros/local_position/odom"
T_CLOUD = "/uav1/octomap_point_cloud_centers"
BASE_MARGIN = 0.2

RE_HEADER = re.compile(r"^(\d+\.\d+) .*replan (\d+) times\]")
RE_RHO = re.compile(r"\[DASC\] rho=([-\d.eE+naif]+), kappa=([-\d.eE+naif]+), v_dyn=([-\d.eE+naif]+)")
RE_FALLBACK = re.compile(r"\[DASC\] optimization failed")


def nanmean(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return float(np.mean(xs)) if xs else float("nan")


def jerk_cost(msg):
    """exact integral of ||jerk||^2 of a uniform cubic B-spline (piecewise-constant jerk)"""
    q = np.array([[p.x, p.y, p.z] for p in msg.pos_pts])
    k = np.array(msg.knots)
    if len(q) < 4 or len(k) < 5:
        return float("nan")
    ts = k[4] - k[3]
    if not np.isfinite(ts) or ts <= 1e-9:
        return float("nan")
    d3 = q[3:] - 3.0 * q[2:-1] + 3.0 * q[1:-2] - q[:-3]
    return float(np.sum(d3 * d3) / ts ** 5)


def read_headers(planner_log):
    heads = []
    if not os.path.exists(planner_log):
        return heads
    with open(planner_log, errors="replace") as f:
        for line in f:
            m = RE_HEADER.search(line)
            if m:
                heads.append((float(m.group(1)), int(m.group(2))))
    return heads


def read_cloud(msg, max_points=20000):
    from sensor_msgs import point_cloud2
    pts = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=float)
    if len(pts) > max_points:
        pts = pts[np.linspace(0, len(pts) - 1, max_points).astype(int)]
    return pts


def nearest_dist(points, cloud, chunk=256):
    out = np.empty(len(points))
    for i in range(0, len(points), chunk):
        p = points[i:i + chunk]
        d2 = ((p[:, None, :] - cloud[None, :, :]) ** 2).sum(-1)
        out[i:i + chunk] = np.sqrt(d2.min(1))
    return out


def parse_run(run_dir):
    row = {"run_dir": run_dir}
    meta = json.load(open(os.path.join(run_dir, "meta.json"))) if os.path.exists(os.path.join(run_dir, "meta.json")) else {}
    res = json.load(open(os.path.join(run_dir, "result.json"))) if os.path.exists(os.path.join(run_dir, "result.json")) else {}
    for k in ("scenario", "drift_level", "group", "rep"):
        row[k] = meta.get(k, "")
    row["outcome"] = res.get("outcome", "missing")
    row["flight_time"] = res.get("flight_time", float("nan"))
    row["path_length"] = res.get("path_length", float("nan"))
    row["final_dist"] = res.get("final_dist", float("nan"))

    bag_path = os.path.join(run_dir, "run.bag")
    if not os.path.exists(bag_path):
        active = glob.glob(os.path.join(run_dir, "run.bag.active"))
        row["bag"] = "unclosed (.active)" if active else "missing"
        return row
    row["bag"] = "ok"
    heads = read_headers(os.path.join(run_dir, "planner.log"))

    bsplines, rhos, kappas, fallbacks, odom, cloud = [], [], [], 0, [], None
    with rosbag.Bag(bag_path) as bag:
        info = bag.get_type_and_topic_info().topics
        row["bag_duration"] = bag.get_end_time() - bag.get_start_time()
        row["n_bspline_msgs"] = info[T_BSPLINE].message_count if T_BSPLINE in info else 0
        for topic, msg, t in bag.read_messages(topics=[T_BSPLINE, T_ROSOUT, T_ODOM]):
            if topic == T_BSPLINE:
                bsplines.append((t.to_sec(), msg.traj_id, msg.start_time.to_sec(), msg))
            elif topic == T_ROSOUT:
                if msg.name != PLANNER_NODE:
                    continue
                m = RE_RHO.search(msg.msg)
                if m:
                    rhos.append(float(m.group(1)))
                    kappas.append(float(m.group(2)))
                if RE_FALLBACK.search(msg.msg):
                    fallbacks += 1
            elif topic == T_ODOM:
                p = msg.pose.pose.position
                odom.append((t.to_sec(), p.x, p.y, p.z))
        last_cloud = None
        for _, msg, _ in bag.read_messages(topics=[T_CLOUD]):
            last_cloud = msg
        if last_cloud is not None:
            cloud = read_cloud(last_cloud)

    t_goal = res.get("t_goal", None)
    t_lo = t_goal if t_goal else (odom[0][0] if odom else -1e18)
    heads = [h for h in heads if h[0] >= t_lo - 1.0]

    # pair replan headers with new trajectories
    seen_ids, plan_ms, costs, repeated = set(), [], [], 0
    new_traj = []  # (start_time, traj_id, msg) of first occurrence of each traj_id
    for t, tid, st, msg in bsplines:
        if tid in seen_ids:
            repeated += 1
            continue
        seen_ids.add(tid)
        new_traj.append((st, tid, msg))
    used = set()
    n_success = 0
    for i, (t0, _) in enumerate(heads):
        t1 = heads[i + 1][0] if i + 1 < len(heads) else float("inf")
        for j, (st, tid, msg) in enumerate(new_traj):
            if j in used or st < t0 or st >= t1:
                continue
            used.add(j)
            n_success += 1
            plan_ms.append((st - t0) * 1e3)
            costs.append(jerk_cost(msg))
            break
    row["n_replan"] = len(heads)
    row["n_success"] = n_success
    row["feasibility_rate"] = n_success / len(heads) if heads else float("nan")
    row["planning_time_ms_mean"] = nanmean(plan_ms)
    row["planning_time_ms_median"] = float(np.median(plan_ms)) if plan_ms else float("nan")
    row["planning_time_ms_max"] = float(np.max(plan_ms)) if plan_ms else float("nan")
    row["snap_cost_mean"] = nanmean(costs)
    row["fallback_count"] = fallbacks
    row["repeated_traj_msgs"] = repeated
    if rhos:
        row["margin_rho_mean"], row["kappa_mean"], row["rho_source"] = nanmean(rhos), nanmean(kappas), "dasc"
    else:
        row["margin_rho_mean"], row["kappa_mean"], row["rho_source"] = BASE_MARGIN, 1.0, "config"

    # measured clearance of the flown path
    fl = np.array([o[1:] for o in odom if o[0] >= t_lo and o[3] > 0.5]) if odom else np.zeros((0, 3))
    if cloud is not None and len(cloud) and len(fl):
        fl = fl[:: max(1, len(fl) // 2000)]
        d = nearest_dist(fl, cloud)
        row["clearance_mean"], row["clearance_min"] = float(d.mean()), float(d.min())
    else:
        row["clearance_mean"] = row["clearance_min"] = float("nan")
    return row


RUN_COLS = ["scenario", "drift_level", "group", "rep", "outcome", "flight_time", "path_length", "final_dist",
            "n_replan", "n_success", "feasibility_rate", "planning_time_ms_mean", "planning_time_ms_median",
            "planning_time_ms_max", "snap_cost_mean", "fallback_count", "repeated_traj_msgs", "margin_rho_mean",
            "kappa_mean", "rho_source", "clearance_mean", "clearance_min", "bag", "bag_duration",
            "n_bspline_msgs", "run_dir"]
SUM_COLS = ["scenario", "drift_level", "group", "n_runs", "mission_success_rate", "flight_time_mean",
            "planning_time_ms_mean", "snap_cost_mean", "fallback_count_mean", "fallback_count_total",
            "margin_rho_mean", "clearance_mean", "clearance_min", "feasibility_rate"]


def summarize(rows):
    groups = {}
    for r in rows:
        groups.setdefault((r.get("scenario"), r.get("drift_level"), r.get("group")), []).append(r)
    out = []
    for (s, d, g), rs in sorted(groups.items(), key=lambda kv: tuple(str(x) for x in kv[0])):
        tot_rep = sum(r.get("n_replan", 0) or 0 for r in rs)
        tot_ok = sum(r.get("n_success", 0) or 0 for r in rs)
        out.append({
            "scenario": s, "drift_level": d, "group": g, "n_runs": len(rs),
            "mission_success_rate": sum(r["outcome"] == "reached" for r in rs) / len(rs),
            "flight_time_mean": nanmean([r.get("flight_time") for r in rs if r["outcome"] == "reached"]),
            "planning_time_ms_mean": nanmean([r.get("planning_time_ms_mean") for r in rs]),
            "snap_cost_mean": nanmean([r.get("snap_cost_mean") for r in rs]),
            "fallback_count_mean": nanmean([r.get("fallback_count") for r in rs]),
            "fallback_count_total": sum(r.get("fallback_count", 0) or 0 for r in rs),
            "margin_rho_mean": nanmean([r.get("margin_rho_mean") for r in rs]),
            "clearance_mean": nanmean([r.get("clearance_mean") for r in rs]),
            "clearance_min": min([r.get("clearance_min", float("inf")) for r in rs
                                  if not math.isnan(r.get("clearance_min", float("nan")))] or [float("nan")]),
            "feasibility_rate": tot_ok / tot_rep if tot_rep else float("nan"),
        })
    return out


def write_csv(path, cols, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({c: (("%.6g" % r[c]) if isinstance(r.get(c), float) else r.get(c, "")) for c in cols})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+")
    ap.add_argument("--out-prefix", default=None)
    a = ap.parse_args()
    run_dirs = []
    for p in a.inputs:
        if os.path.exists(os.path.join(p, "meta.json")) or os.path.exists(os.path.join(p, "run.bag")):
            run_dirs.append(p)
        else:
            run_dirs += sorted(os.path.dirname(m) for m in glob.glob(os.path.join(p, "**", "meta.json"), recursive=True))
    if not run_dirs:
        sys.exit("no run directories found under %s" % a.inputs)
    rows = []
    for d in run_dirs:
        try:
            rows.append(parse_run(d))
        except Exception as e:  # keep going: one broken bag must not stop a 540-run analysis
            rows.append({"run_dir": d, "outcome": "parse_error: %s" % e})
        r = rows[-1]
        print("%-60s %-10s replans=%s plan_ms=%s fallback=%s rho=%s clearance=%s" % (
            d, r.get("outcome"), r.get("n_replan"), "%.1f" % r["planning_time_ms_mean"] if isinstance(r.get("planning_time_ms_mean"), float) else "-",
            r.get("fallback_count"), r.get("margin_rho_mean"), "%.2f" % r["clearance_mean"] if isinstance(r.get("clearance_mean"), float) else "-"))
    prefix = a.out_prefix or os.path.join(a.inputs[0] if os.path.isdir(a.inputs[0]) else ".", "exp_metrics")
    write_csv(prefix + "_runs.csv", RUN_COLS, rows)
    write_csv(prefix + "_summary.csv", SUM_COLS, summarize([r for r in rows if "group" in r]))
    print("wrote %s_runs.csv and %s_summary.csv" % (prefix, prefix))


if __name__ == "__main__":
    main()
