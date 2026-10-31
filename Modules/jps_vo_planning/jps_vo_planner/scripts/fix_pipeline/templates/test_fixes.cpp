// Offline tests of the engineering fixes after thesis-frozen-v1 (FX1 R1a-R1d, FX2 R2a-R2b, FX3 R3, FX4 R4/R4-m, FX5 R5,
// FX6 S1).   test_fixes <golden_file>   (the reference dump made by gen_golden built from the thesis-frozen-v1 tag)
// T0 is the baseline-identity gate: with every Fixes flag false the dump must equal the frozen reference byte for byte.
#include <fstream>
#include <map>
#include <set>
#include <sstream>

#include "golden_scenarios.h"

using namespace jps_vo;
using test_common::check;

namespace
{
  std::mt19937 rng(77);
  std::uniform_real_distribution<double> U(-1.0, 1.0), U01(0.0, 1.0);
  Eigen::Vector3d unit3()
  {
    Eigen::Vector3d d(U(rng), U(rng), U(rng));
    return d.normalized();
  }

  OccupancyGrid openGrid(double r_infl)
  {
    OccupancyGrid g;
    OccupancyGrid::Params p;
    p.inflation = r_infl;
    g.init(p);
    g.rebuild({}, {});
    return g;
  }

  OccupancyGrid pillarGrid(double r_infl, double x, double y, double r)
  {
    OccupancyGrid g;
    OccupancyGrid::Params p;
    p.inflation = r_infl;
    g.init(p);
    g.rebuild(test_common::cylinderVoxels({test_common::Cylinder{x, y, r, 0.0, 3.5}}), {});
    return g;
  }

  CoreConfig cfgFor(int group, const std::function<void(Fixes &)> &set)
  {
    CoreConfig cfg;
    cfg.c.deriveTaus();
    cfg.sw = golden::groupSwitches(group);
    set(cfg.sw.fx);
    return cfg;
  }

  ObstacleState obstacle(const std::string &n, const Eigen::Vector3d &p, const Eigen::Vector3d &v)
  {
    ObstacleState o;
    o.name = n;
    o.p = p;
    o.v = v;
    o.r = 0.3;
    return o;
  }

  std::string g3(double x)
  {
    char b[32];
    std::snprintf(b, sizeof(b), "%.3g", x);
    return b;
  }

  double sideOf(const GlobalPath &path)
  {
    double s = 0.0;
    for (const auto &q : path.points())
      s += q.y();
    return s;
  }
} // namespace

int main(int argc, char **argv)
{
  ModelConstants c;
  c.deriveTaus();

  // ---- T0 baseline identity: every flag false == thesis-frozen-v1 reference, byte for byte ----
  {
    const std::string gold_path = argc > 1 ? argv[1] : std::string();
    std::ifstream gf(gold_path);
    std::stringstream gold, now;
    gold << gf.rdbuf();
    golden::dumpAll(now, [](Switches &sw) { sw.fx = Fixes(); });
    const std::string a = gold.str(), b = now.str();
    size_t first = 0;
    while (first < a.size() && first < b.size() && a[first] == b[first])
      ++first;
    check(gf.good() && !a.empty(), "T0 golden reference readable: " + gold_path);
    check(a == b, "T0 all fixes off: dump identical to the frozen reference (" + std::to_string(b.size()) + " bytes" +
                      (a == b ? "" : ", first difference at byte " + std::to_string(first)) + ")");
  }

  // ---- T0b diag_log only: the extended diagnostics change no planner output ----
  {
    std::ifstream gf(argc > 1 ? argv[1] : "");
    std::stringstream gold, now;
    gold << gf.rdbuf();
    golden::dumpAll(now, [](Switches &sw) {
      sw.fx = Fixes();
      sw.fx.diag_log = true;
    });
    check(gf.good() && gold.str() == now.str(), "T0b diag_log only: dump identical to the frozen reference");
  }

  // ---- T1 R1a reachability: v_rot (v_prev turned towards t_g by min(reach, angle)) satisfies (C-a), (C-theta) and
  //      has zero angle excess and zero slack w.r.t. theta_eff ----
  {
    bool acc_ok = true, turn_ok = true, slack_ok = true;
    double worst_ang = 0.0, worst_slack = 0.0;
    int n = 0;
    for (int k = 0; k < 20000; ++k)
    {
      const double sp = c.v_min + (c.v_c - c.v_min) * U01(rng);
      const Eigen::Vector3d v_prev = unit3() * sp, t_g = unit3();
      const double lam = (k % 3 == 0) ? 0.0 : U01(rng);
      const double reach = reachTurn(sp, c);
      const double ang = angleBetween(v_prev, t_g);
      const double phi = std::min(reach, ang);
      const Eigen::Vector3d axis = v_prev.cross(t_g);
      if (axis.norm() < 1e-9)
        continue;
      const Eigen::Vector3d v_rot = Eigen::AngleAxisd(phi, axis.normalized()) * v_prev;
      const double theta_eff = std::max(M_PI * lam, ang - reach);
      ++n;
      acc_ok = acc_ok && (v_rot - v_prev).norm() <= c.a_max * c.dt + 1e-12;
      turn_ok = turn_ok && angleBetween(v_rot, v_prev) <= c.theta_max + 1e-12;
      worst_ang = std::max(worst_ang, angleBetween(v_rot, t_g) - theta_eff);
      worst_slack = std::max(worst_slack, coneSlack(v_rot, t_g, theta_eff, c));
      // acos near 0 has an error of order sqrt(machine eps) = 1.5e-8 rad, hence 1e-7
      slack_ok = slack_ok && angleBetween(v_rot, t_g) <= theta_eff + 1e-7 && coneSlack(v_rot, t_g, theta_eff, c) <= 1e-7;
    }
    check(n > 19000 && acc_ok, "T1 R1a: v_rot within a_max dt of v_prev (C-a) (" + std::to_string(n) + " cases)");
    check(turn_ok, "T1 R1a: turn of v_rot <= theta_max for |v_prev| >= v_min (reachTurn(v_min) = " +
                       golden::f(reachTurn(c.v_min, c) * 180 / M_PI).substr(0, 6) + " deg)");
    check(slack_ok, "T1 R1a: v_rot inside K(theta_eff) up to acos round-off (worst angle excess " + g3(worst_ang) +
                        " rad, worst slack " + g3(worst_slack) + ")");
  }

  // ---- T2 R1b slack: exact distance to the cone, continuous at v = 0, 1/v_c-Lipschitz ----
  {
    bool zero_ok = coneSlack(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 0.0, c) == 0.0;
    bool cont_ok = true, lip_ok = true, exact_ok = true;
    double worst_exact = 0.0;
    for (int k = 0; k < 20000; ++k)
    {
      const Eigen::Vector3d t = unit3(), u = unit3();
      const double th = M_PI * U01(rng);
      for (double e = 1e-1; e >= 1e-9; e *= 0.1)
        cont_ok = cont_ok && coneSlack(e * u, t, th, c) <= e / c.v_c + 1e-15;
      const Eigen::Vector3d a = unit3() * c.v_c * U01(rng), b = a + unit3() * 0.2 * U01(rng);
      lip_ok = lip_ok && std::fabs(coneSlack(a, t, th, c) - coneSlack(b, t, th, c)) <= (a - b).norm() / c.v_c + 1e-12;
      if (k < 400)
      {
        // brute force: distance to sampled points of K (generators at angle th, radii 0..|a|+1) and the apex
        Eigen::Vector3d e1 = t.unitOrthogonal(), e2 = t.cross(e1);
        double best = a.norm();
        if (angleBetween(a, t) <= th)
          best = 0.0;
        for (int i = 0; i < 720 && best > 0.0; ++i)
        {
          const double phi = 2.0 * M_PI * i / 720.0;
          const Eigen::Vector3d g = std::cos(th) * t + std::sin(th) * (std::cos(phi) * e1 + std::sin(phi) * e2);
          const double r = std::max(0.0, a.dot(g)); // nearest point on this generator ray
          best = std::min(best, (a - r * g).norm());
        }
        const double d = std::fabs(best / c.v_c - coneSlack(a, t, th, c));
        worst_exact = std::max(worst_exact, d);
        exact_ok = exact_ok && d < 2e-4;
      }
    }
    check(zero_ok, "T2 R1b: slack(0) == 0");
    check(cont_ok, "T2 R1b: slack(e u) <= e / v_c for e = 1e-1 ... 1e-9 (no jump at v = 0)");
    check(lip_ok, "T2 R1b: |slack(a) - slack(b)| <= |a - b| / v_c on 20000 random pairs");
    check(exact_ok, "T2 R1b: slack == brute-force distance to K / v_c (720 generators; worst " + g3(worst_exact) + ")");
  }

  // ---- T2e R1e slack: zero at v = 0, continuous, equal to the frozen angle excess for |v| >= v_min, non-decreasing in
  //      speed along a fixed direction ----
  {
    const OccupancyGrid open = openGrid(c.rInfl());
    Switches sw_e, sw_f;
    sw_e.fx.r1e_lowspeed_angle = true;
    const LocalPlanner lp_e(c, sw_e), lp_f(c, sw_f);
    bool zero_ok = true, cont_ok = true, equal_ok = true, mono_ok = true;
    for (int k = 0; k < 4000; ++k)
    {
      LocalProblem P;
      P.p_ref = Eigen::Vector3d(0, 0, 1.5);
      P.goal = Eigen::Vector3d(10, 0, 1.5);
      P.t_g = unit3();
      P.lambda = (k % 2) ? 0.0 : U01(rng);
      P.L_s = 10.0;
      P.grid = &open;
      const Eigen::Vector3d u = unit3();
      zero_ok = zero_ok && lp_e.evaluate(P, Eigen::Vector3d::Zero()).s_g == 0.0;
      const double excess = std::max(0.0, angleBetween(u, P.t_g) - thetaC(P.lambda, sw_e));
      for (double e = 1e-1; e >= 1e-9; e *= 0.1)
        cont_ok = cont_ok && lp_e.evaluate(P, e * u).s_g <= e / c.v_min * M_PI + 1e-15;
      for (double s : {c.v_min, 0.3, c.v_c})
        equal_ok = equal_ok && std::fabs(lp_e.evaluate(P, s * u).s_g - lp_f.evaluate(P, s * u).s_g) <= 1e-12 &&
                   std::fabs(lp_e.evaluate(P, s * u).s_g - excess) <= 1e-9;
      double last = -1.0;
      for (double s = 0.0; s <= c.v_c + 1e-12; s += 0.01)
      {
        const double sg = lp_e.evaluate(P, s * u).s_g;
        mono_ok = mono_ok && sg >= last - 1e-12;
        last = sg;
      }
    }
    check(zero_ok, "T2e R1e: slack(0) == 0");
    check(cont_ok, "T2e R1e: slack(e u) <= (e / v_min) pi for e = 1e-1 ... 1e-9 (no jump at v = 0)");
    check(equal_ok, "T2e R1e: slack == frozen angle excess for |v| >= v_min (4000 random problems)");
    check(mono_ok, "T2e R1e: slack non-decreasing in speed along a fixed direction");
  }

  // ---- T3 lambda = 0 steady state: R1a + R1b keep the frozen ray solution of section 4.2(a)? (reported honestly) ----
  {
    const OccupancyGrid open = openGrid(c.rInfl());
    int same_hi = 0, n_hi = 0, same_lo = 0, n_lo = 0;
    for (int k = 0; k < 600; ++k)
    {
      Eigen::Vector3i d;
      do
        d = Eigen::Vector3i(std::uniform_int_distribution<int>(-1, 1)(rng), std::uniform_int_distribution<int>(-1, 1)(rng), 0);
      while (d == Eigen::Vector3i::Zero());
      const Eigen::Vector3d p0 = open.indexToPos(open.posToIndex(Eigen::Vector3d(0, 0, 1.5)));
      const Eigen::Vector3d goal = p0 + d.cast<double>() * 0.1 * (60 + k % 40);
      PlannerCore core;
      core.configure(cfgFor(1, [](Fixes &f) { f.r1a_reachable_cone = f.r1b_continuous_slack = true; }));
      core.setGoal(goal, p0);
      const Eigen::Vector3d dir = d.cast<double>().normalized();
      const Eigen::Vector3d p_ref = p0 + dir * 0.3;
      const double sp = 0.8 * std::fabs(U(rng));
      const Eigen::Vector3d v_prev = dir * sp + Eigen::Vector3d(U(rng), U(rng), U(rng)) * 0.03;
      const CycleRecord r = core.cycle(0.0, open, p_ref, v_prev, p_ref, {});
      Eigen::Vector3d v_cf;
      if (!LocalPlanner(c, Switches()).rayClosedForm(core.lastLocalProblem(), v_cf))
        continue;
      const bool same = (r.v_star - v_cf).norm() < 1e-9;
      if (v_prev.norm() >= 0.4)
        ++n_hi, same_hi += same;
      else
        ++n_lo, same_lo += same;
    }
    std::printf("[INFO] T3 lambda = 0, R1a+R1b on: v* == frozen ray closed form in %d/%d cases with |v_prev| >= 0.4 and "
                "%d/%d with |v_prev| < 0.4\n", same_hi, n_hi, same_lo, n_lo);
    check(n_hi > 100 && same_hi == n_hi, "T3 lambda = 0 steady state at |v_prev| >= 0.4: ray solution unchanged (" +
                                            std::to_string(same_hi) + "/" + std::to_string(n_hi) + ")");
  }

  // ---- T4 R2a kappa memory: after a fallback, kappa on that edge == gamma^k, erased below 1e-3, kappa <= bound ----
  {
    const OccupancyGrid open = openGrid(c.rInfl());
    PlannerCore core;
    core.configure(cfgFor(6, [](Fixes &f) { f.r2a_kappa_decay = true; }));
    const Eigen::Vector3d p0(0, 0, 1.5), goal(12, 0, 1.5);
    core.setGoal(goal, p0);
    Eigen::Vector3d p = p0, v(0.5, 0, 0);
    core.cycle(0.0, open, p, v, p, {});
    // one cycle with an obstacle on top of the reference: every candidate is in the VO -> fallback -> kappa = 1
    const CycleRecord rf = core.cycle(0.1, open, p, v, p, {obstacle("x", p + Eigen::Vector3d(0.2, 0, 0), {-0.5, 0, 0})});
    const GlobalPath path = core.path();
    const auto pr = path.project(p);
    const PathEdge ek = path.edges[pr.edge];
    const double base = ek.ell / core.Ls();
    const double gam = std::exp(-c.dt / c.T_g);
    bool decay_ok = rf.fallback_active && std::fabs(rf.kappa - 1.0) < 1e-12, bound_ok = true, erased_ok = false;
    double worst = 0.0;
    int k_erase = -1;
    for (int k = 1; k < 200; ++k)
    {
      const CycleRecord r = core.cycle(0.1 + 0.1 * k, open, p, v, p, {});
      bound_ok = bound_ok && r.kappa <= r.kappa_bound + 1e-12;
      const double kap = core.edgeWeight(open, ek.from_cell, ek.to_cell, ek.dir_in, p, v, {}) - base;
      const double want = std::pow(gam, k);
      if (want >= 1e-3)
      {
        const double rel = std::fabs(kap - want) / want;
        worst = std::max(worst, rel);
        decay_ok = decay_ok && rel < 1e-9;
      }
      else if (k_erase < 0)
      {
        k_erase = k;
        erased_ok = std::fabs(kap) < 1e-15;
      }
    }
    check(decay_ok, "T4 R2a: kappa = 1 after the fallback, then gamma^k with gamma = exp(-dt/T_g) (worst rel. error " +
                        g3(worst) + ")");
    check(erased_ok, "T4 R2a: kappa erased at the first k with gamma^k < 1e-3 (k = " + std::to_string(k_erase) + ")");
    check(bound_ok, "T4 R2a: kappa <= kappa_bound on every cycle");
    check(1.0 - gam <= 0.0645 + 1e-4, "T4 R2a: per-cycle decay 1 - gamma = " + g3(1.0 - gam) +
                                          " (< eps_w = 0.05 relative change below kappa = 0.775)");
  }

  // ---- T5 R4 hysteresis: two symmetric routes around a pillar, a risk source alternating sides every 0.5 s ----
  {
    const OccupancyGrid g = pillarGrid(c.rInfl(), 6.0, 0.0, 0.8);
    const Eigen::Vector3d p0(0, 0, 1.5), goal(12, 0, 1.5);
    for (double lateral : {2.2, 5.5})
    {
      int sides_off = 0, sides_on = 0;
      bool margin_ok = true;
      for (int on = 0; on < 2; ++on)
      {
        PlannerCore core;
        core.configure(cfgFor(6, [&](Fixes &f) { f.r4_switch_hysteresis = on == 1; f.diag_log = true; }));
        core.setGoal(goal, p0);
        double last_side = 0.0;
        int changes = 0;
        for (int k = 0; k < 200; ++k)
        {
          const double side = (k % 2 == 0) ? 1.0 : -1.0;
          const std::vector<ObstacleState> obs = {obstacle("s", Eigen::Vector3d(6.0, side * lateral, 1.5), {0, 0, 0})};
          core.replanGlobal(0.5 * k, g, p0, Eigen::Vector3d::Zero(), obs, k == 0 ? "goal" : "periodic");
          const double s = sideOf(core.path());
          if (k > 0 && s * last_side < 0.0)
            ++changes;
          last_side = s;
        }
        for (const auto &d : core.takeSwitchDiags())
          if (on == 1 && d.is_switch && d.accepted && !d.forced) // switches the R4 rule actually judged
            margin_ok = margin_ok && d.C_new <= d.C_old - c.eps_w + 1e-12 && d.dwell_left <= 0.0;
        (on ? sides_on : sides_off) = changes;
      }
      std::printf("[INFO] T5 risk source at lateral %.1f m: route side changes over 200 replans: R4 off %d, R4 on %d\n",
                  lateral, sides_off, sides_on);
      check(margin_ok, "T5 R4: every accepted voluntary switch had C_new <= C_old - eps_w and dwell >= T_g (lateral " +
                           golden::f(lateral).substr(0, 3) + " m)");
      check(sides_on <= sides_off, "T5 R4: no more route changes than without hysteresis (lateral " +
                                       golden::f(lateral).substr(0, 3) + " m)");
    }
  }

  // ---- T5b R4-m: with r4m_dwell_free_only the dwell applies exactly in threat-free mode (lambda == 0 for >= T_c);
  //      under a threat a voluntary switch needs the eps_w margin only ----
  {
    const OccupancyGrid g = pillarGrid(c.rInfl(), 6.0, 0.0, 0.8);
    const Eigen::Vector3d p0(0, 0, 1.5), goal(12, 0, 1.5);
    int changes[2] = {0, 0}, waived = 0, judged_free = 0, judged_threat = 0;
    bool margin_ok = true, mode_ok = true, plain_ok = true;
    for (int m = 0; m < 2; ++m)
    {
      PlannerCore core;
      core.configure(cfgFor(6, [&](Fixes &f) {
        f.r4_switch_hysteresis = true;
        f.r4m_dwell_free_only = m == 1;
        f.diag_log = true;
      }));
      core.setGoal(goal, p0);
      double last_side = 0.0, t_threat = -1e18;
      bool free = true;
      std::map<int, bool> free_at;
      for (int k = 0; k < 200; ++k)
      {
        const double t = 0.5 * k;
        std::vector<ObstacleState> obs = {obstacle("s", Eigen::Vector3d(6.0, (k % 2 == 0 ? 1.0 : -1.0) * 2.2, 1.5), {0, 0, 0})};
        if (k < 100) // first half: an obstacle approaching the reference keeps lambda > 0
          obs.push_back(obstacle("t", p0 + Eigen::Vector3d(2.5, 0, 0), {-0.6, 0, 0}));
        const CycleRecord r = core.localStep(t, g, p0, Eigen::Vector3d::Zero(), p0, obs, GlobalStepResult());
        if (r.lambda > 0.0) // same arithmetic as the core
          free = false, t_threat = t;
        else if (t - t_threat >= c.T_c)
          free = true;
        free_at[k] = free;
        core.replanGlobal(t + 0.01, g, p0, Eigen::Vector3d::Zero(), obs, k == 0 ? "goal" : "periodic");
        const double s = sideOf(core.path());
        if (k > 0 && s * last_side < 0.0)
          ++changes[m];
        last_side = s;
      }
      for (const auto &d : core.takeSwitchDiags())
      {
        if (!d.is_switch || d.forced)
          continue;
        const int k = static_cast<int>(std::lround((d.t - 0.01) / 0.5));
        if (d.accepted)
          margin_ok = margin_ok && d.C_new <= d.C_old - c.eps_w + 1e-12;
        if (m == 0)
        {
          plain_ok = plain_ok && d.dwell_applied == 1 && (!d.accepted || d.dwell_left <= 0.0);
          continue;
        }
        const bool fr = free_at.count(k) ? free_at[k] : true;
        (fr ? judged_free : judged_threat) += 1;
        mode_ok = mode_ok && d.dwell_applied == (fr ? 1 : 0) && (!d.accepted || !fr || d.dwell_left <= 0.0);
        waived += !fr && d.accepted && d.dwell_left > 0.0;
      }
    }
    std::printf("[INFO] T5b route side changes over 200 replans: R4 %d, R4 + R4-m %d; R4-m judged switches: %d under threat "
                "(%d accepted inside the dwell), %d threat-free\n",
                changes[0], changes[1], judged_threat, waived, judged_free);
    check(margin_ok, "T5b R4/R4-m: every accepted voluntary switch had C_new <= C_old - eps_w");
    check(plain_ok, "T5b R4 alone: the dwell applies to every voluntary switch");
    check(mode_ok, "T5b R4-m: dwell applied iff threat-free (lambda == 0 for >= T_c), checked on every judged switch");
    check(waived > 0, "T5b R4-m: under a threat, switches inside the dwell are accepted on the margin alone");
    check(changes[1] >= changes[0], "T5b R4-m: no fewer route changes than R4 alone (the waiver only relaxes R4)");
  }

  // ---- T6 R3 trigger: off-path increases ignored; on-path increases fire; soundness vs a fresh search ----
  {
    const OccupancyGrid open = golden::planningEasy(c.rInfl()); // cluttered: JPS caches many off-path edges
    const Eigen::Vector3d p0(0, 0, 1.5), goal(13, 0, 1.5);
    int frozen_only = 0, both = 0, relevant_only = 0, none = 0, unsound = 0, checked = 0;
    double worst_gap = -1e9;
    for (int k = 0; k < 300; ++k)
    {
      PlannerCore core;
      core.configure(cfgFor(6, [](Fixes &f) { f.r3_relevant_trigger = true; }));
      core.setGoal(goal, p0);
      core.replanGlobal(0.0, open, p0, Eigen::Vector3d::Zero(), {}, "goal");
      const GlobalPath path = core.path();
      const Eigen::Vector3d op(13.0 * U01(rng), 5.0 * U(rng), 1.5);
      const std::vector<ObstacleState> obs = {obstacle("o", op, Eigen::Vector3d(U(rng), U(rng), 0) * 0.5)};
      const bool fz = core.changedEdges(open, p0, Eigen::Vector3d::Zero(), obs) != "none";
      const bool rl = core.changedEdgesRelevant(open, path, p0, Eigen::Vector3d::Zero(), obs) != "none";
      frozen_only += fz && !rl;
      both += fz && rl;
      relevant_only += !fz && rl;
      none += !fz && !rl;
      if (!rl)
      {
        // soundness: the kept route is within eps_w + delta_sw (= 2 eps_w) of a fresh search under the new weights
        double c_old = 0.0;
        for (const auto &e : path.edges)
          c_old += core.edgeWeight(open, e.from_cell, e.to_cell, e.dir_in, p0, Eigen::Vector3d::Zero(), obs);
        PlannerCore fresh;
        fresh.configure(cfgFor(6, [](Fixes &) {}));
        fresh.setGoal(goal, p0);
        fresh.replanGlobal(0.0, open, p0, Eigen::Vector3d::Zero(), obs, "goal");
        double c_new = 0.0;
        for (const auto &e : fresh.path().edges)
          c_new += e.w;
        ++checked;
        worst_gap = std::max(worst_gap, c_old - c_new);
        unsound += c_old - c_new > 2.0 * c.eps_w + 1e-9;
      }
    }
    std::printf("[INFO] T6 300 random obstacles: frozen-only fires %d, both %d, R3-only %d, neither %d\n", frozen_only,
                both, relevant_only, none);
    check(frozen_only > 0 && both > 0, "T6 R3: some frozen D_k triggers are filtered, on-path changes still fire");
    check(unsound == 0, "T6 R3: when R3 does not fire, the kept route is within 2 eps_w of a fresh search (" +
                            std::to_string(checked) + " cases, worst gap " + g3(worst_gap) + ")");
  }

  // ---- T6b R3 positive part: any remaining-path edge whose weight rose by more than eps_w fires the trigger, also
  //      when other remaining edges fell (the signed sum of the first patch version let decreases cancel it) ----
  {
    const OccupancyGrid grid = golden::planningEasy(c.rInfl());
    const Eigen::Vector3d p0(0, 0, 1.5), goal(13, 0, 1.5);
    int n_inc = 0, fired = 0, mixed = 0, masked = 0;
    for (int k = 0; k < 400; ++k)
    {
      PlannerCore core;
      core.configure(cfgFor(6, [](Fixes &f) { f.r3_relevant_trigger = true; }));
      core.setGoal(goal, p0);
      const Eigen::Vector3d o1(2.0 + 9.0 * U01(rng), 2.0 * U(rng), 1.5), vel = Eigen::Vector3d(U(rng), U(rng), 0) * 0.6;
      core.replanGlobal(0.0, grid, p0, Eigen::Vector3d::Zero(), {obstacle("o", o1, vel)}, "goal");
      const GlobalPath path = core.path();
      // the obstacle moves: the risk rises on some remaining edges and falls on others
      const std::vector<ObstacleState> after = {
          obstacle("o", o1 + Eigen::Vector3d(U(rng), U(rng), 0).normalized() * (0.5 + 2.5 * U01(rng)), vel)};
      double max_inc = 0.0, signed_sum = 0.0;
      bool any_dec = false;
      for (size_t i = path.project(p0).edge; i < path.edges.size(); ++i)
      {
        const PathEdge &e = path.edges[i];
        const double d = core.edgeWeight(grid, e.from_cell, e.to_cell, e.dir_in, p0, Eigen::Vector3d::Zero(), after) - e.w;
        max_inc = std::max(max_inc, d);
        signed_sum += d;
        any_dec = any_dec || d < -1e-12;
      }
      if (max_inc <= c.eps_w)
        continue;
      ++n_inc;
      mixed += any_dec;
      masked += signed_sum <= c.eps_w;
      fired += core.changedEdgesRelevant(grid, path, p0, Eigen::Vector3d::Zero(), after) != "none";
    }
    std::printf("[INFO] T6b %d cases with a remaining-edge increase > eps_w (%d with simultaneous decreases); the signed "
                "sum of the first patch version stays <= eps_w in %d of them\n", n_inc, mixed, masked);
    check(n_inc >= 10 && fired == n_inc, "T6b R3: every remaining-edge increase > eps_w fires the trigger (" +
                                             std::to_string(fired) + "/" + std::to_string(n_inc) + ")");
  }

  // ---- T11 R4-new: rolling-window predicted clearance, leaky CUSUM evidence, safety override ----
  {
    const OccupancyGrid g = pillarGrid(c.rInfl(), 6.0, 0.0, 0.8);
    const Eigen::Vector3d p0(0, 0, 1.5), goal(12, 0, 1.5);
    // (a) predictedClearance == brute-force rolling-window prediction (UAV at v_c along the route, constant-velocity obstacle)
    {
      PlannerCore core;
      core.configure(cfgFor(6, [](Fixes &) {}));
      core.setGoal(goal, p0);
      core.replanGlobal(0.0, g, p0, Eigen::Vector3d::Zero(), {}, "goal");
      const GlobalPath route = core.path();
      double worst = 0.0;
      for (int k = 0; k < 200; ++k)
      {
        const std::vector<ObstacleState> obs = {
            obstacle("o", Eigen::Vector3d(12.0 * U01(rng), 4.0 * U(rng), 1.5), Eigen::Vector3d(U(rng), U(rng), 0) * 0.6)};
        double best = kInf, arc = 0.0;
        for (const auto &e : route.edges)
        {
          for (double s = 0.0; s <= e.ell + 1e-12; s += 0.002)
          {
            const double t = (arc + s) / c.v_c;
            if (t > c.tau2)
              break;
            best = std::min(best, (e.a + e.t * s - (obs[0].p + obs[0].v * t)).norm() - c.safetyRadius(obs[0].r));
          }
          arc += e.ell;
        }
        worst = std::max(worst, std::fabs(best - core.predictedClearance(route, obs)));
      }
      check(worst < 0.01, "T11 R4-s: predicted clearance == brute-force rolling-window prediction (worst |diff| " + g3(worst) +
                              " m, 200 cases)");
    }
    // (b) CUSUM recursion and acceptance rule, (c) A-B-A suppression: risk source alternating sides every 0.5 s
    {
      int changes[3] = {0, 0, 0};
      bool rec_ok = true, accept_ok = true;
      double worst = 0.0;
      for (int m = 0; m < 3; ++m) // 0: no R4, 1: R4 iteration 1, 2: R4-new (evidence, R4-m, override)
      {
        PlannerCore core;
        core.configure(cfgFor(6, [&](Fixes &f) {
          f.diag_log = true;
          f.r4_switch_hysteresis = m >= 1;
          f.r4n_leaky_evidence = f.r4m_dwell_free_only = f.r4s_safety_override = m == 2;
        }));
        core.setGoal(goal, p0);
        double last_side = 0.0;
        for (int k = 0; k < 200; ++k)
        {
          const double side = (k % 2 == 0) ? 1.0 : -1.0;
          core.replanGlobal(0.5 * k, g, p0, Eigen::Vector3d::Zero(), {obstacle("s", Eigen::Vector3d(6.0, side * 2.2, 1.5), {0, 0, 0})},
                            k == 0 ? "goal" : "periodic");
          const double s = sideOf(core.path());
          if (k > 0 && s * last_side < 0.0)
            ++changes[m];
          last_side = s;
        }
        if (m != 2)
          continue;
        double E_prev = 0.0, t_prev = -1e18;
        for (const auto &d : core.takeSwitchDiags())
        {
          const double gam = std::exp(-std::max(0.0, d.t - t_prev) / c.T_g);
          const bool judged = d.is_switch && d.reason != "goal";
          const double want = judged ? std::max(0.0, gam * E_prev + (d.C_old - d.C_new - c.eps_w)) : std::max(0.0, gam * E_prev - c.eps_w);
          if (d.reason != "goal")
          {
            worst = std::max(worst, std::fabs(want - d.evidence));
            rec_ok = rec_ok && std::fabs(want - d.evidence) < 1e-9;
          }
          if (judged && d.accepted && !d.forced && !d.safety_override)
            accept_ok = accept_ok && d.evidence >= c.eps_w - 1e-12 && (!d.dwell_applied || d.dwell_left <= 0.0);
          if (judged && !d.accepted)
            accept_ok = accept_ok && (d.evidence < c.eps_w || (d.dwell_applied && d.dwell_left > 0.0));
          E_prev = (d.is_switch && d.accepted) ? 0.0 : d.evidence;
          t_prev = d.t;
        }
      }
      std::printf("[INFO] T11 route side changes over 200 replans with an alternating risk source: no R4 %d, R4 (it. 1) %d, "
                  "R4-new %d\n", changes[0], changes[1], changes[2]);
      check(rec_ok, "T11 R4-new: evidence follows E = max(0, exp(-dt/T_g) E + (C_old - C_new - eps_w)) on every search (worst " +
                        g3(worst) + ")");
      check(accept_ok, "T11 R4-new: a voluntary switch is accepted iff E >= eps_w and the applicable dwell has elapsed");
      check(changes[2] <= changes[0], "T11 R4-new: no more route changes than without hysteresis");
    }
    // (d) safety override: a stationary obstacle on the current route side inside the dwell window
    {
      int held = -1, forced = -1;
      double d_old = 0.0, d_new = 0.0;
      for (int m = 0; m < 2; ++m)
      {
        PlannerCore core;
        core.configure(cfgFor(6, [&](Fixes &f) {
          f.diag_log = f.r4_switch_hysteresis = f.r4n_leaky_evidence = true;
          f.r4s_safety_override = m == 1;
        }));
        core.setGoal(goal, p0);
        core.replanGlobal(0.0, g, p0, Eigen::Vector3d::Zero(), {}, "goal");
        const double side = sideOf(core.path()) >= 0.0 ? 1.0 : -1.0;
        core.takeSwitchDiags();
        core.replanGlobal(0.5, g, p0, Eigen::Vector3d::Zero(), {obstacle("x", Eigen::Vector3d(6.0, side * 1.5, 1.5), {0, 0, 0})},
                          "periodic");
        for (const auto &d : core.takeSwitchDiags())
          if (d.is_switch)
          {
            (m == 0 ? held : forced) = d.accepted;
            if (m == 1)
              d_old = d.d_old_pred, d_new = d.d_new_pred, forced = forced && d.safety_override;
          }
      }
      std::printf("[INFO] T11 safety override: predicted clearance old route %.2f m, new route %.2f m\n", d_old, d_new);
      check(held == 0, "T11 R4-s off: the dwell holds the switch away from an obstacle on the current route");
      check(forced == 1 && d_old < 0.0 && d_new > d_old,
            "T11 R4-s on: the old route's predicted safety-sphere entry within tau2 forces the switch past the dwell");
    }
  }

  // ---- T7 R5 gating: threat-free mode only after T_c without lambda > 0 ----
  {
    const OccupancyGrid open = openGrid(c.rInfl());
    PlannerCore core;
    core.configure(cfgFor(6, [](Fixes &f) { f.r5_mode_gating = true; }));
    const Eigen::Vector3d p0(0, 0, 1.5), goal(12, 0, 1.5);
    core.setGoal(goal, p0);
    Eigen::Vector3d p = p0, v(0.6, 0, 0);
    double t_last = -1.0;
    bool timing_ok = true, initial_free = false;
    for (int k = 0; k < 60; ++k)
    {
      const double t = 0.1 * k;
      std::vector<ObstacleState> obs;
      if (k >= 5 && k < 15)
        obs.push_back(obstacle("h", p + Eigen::Vector3d(3.0, 0, 0), {-0.6, 0, 0}));
      const CycleRecord r = core.cycle(t, open, p, v, p, obs);
      if (k == 0)
        initial_free = r.free_mode == 1;
      if (r.lambda > 0.0)
        t_last = t;
      const bool want = r.lambda <= 0.0 && (t_last < 0.0 || t - t_last >= c.T_c); // same arithmetic as the core
      timing_ok = timing_ok && (r.free_mode == 1) == want;
    }
    check(initial_free, "T7 R5: free mode at the start (no threat seen)");
    check(t_last > 0.0 && timing_ok, "T7 R5: free mode iff lambda == 0 and >= T_c since the last lambda > 0");
  }

  // ---- T8 S1 budget: chosen v meets rho <= 1 - lambda whenever a feasible candidate does ----
  {
    const OccupancyGrid grid = golden::planningEasy(c.rInfl());
    int n = 0, active = 0, viol = 0;
    for (int k = 0; k < 1500; ++k)
    {
      Switches sw;
      sw.fx.s1_eps_constraint = true;
      LocalProblem P;
      P.p_ref = Eigen::Vector3d(3.0 + 8.0 * U(rng), 6.0 * U(rng), 1.5);
      P.goal = P.p_ref + Eigen::Vector3d(U(rng), U(rng), 0).normalized() * 8.0;
      P.v_prev = Eigen::Vector3d(U(rng), U(rng), 0) * 0.5;
      P.lambda = 0.05 + 0.95 * U01(rng);
      P.L_s = 8.0;
      P.grid = &grid;
      for (int i = 0; i < 1 + k % 3; ++i)
        P.obs.push_back(obstacle("o", P.p_ref + Eigen::Vector3d(U(rng), U(rng), 0).normalized() * (1.0 + 3.0 * U01(rng)),
                                 Eigen::Vector3d(U(rng), U(rng), 0) * 0.6));
      const LocalPlanner lp(c, sw);
      const LocalSolution s = lp.solve(P);
      bool any = false;
      for (const auto &v : lp.candidates(P))
        if (lp.turnOk(P, v) && lp.staticOk(P, v) && lp.dynamicOk(P, v) && lp.evaluate(P, v).rho <= 1.0 - P.lambda + 1e-12)
          any = true;
      if (s.fallback)
        continue;
      ++n;
      active += s.budget_active;
      viol += any && s.rho > 1.0 - P.lambda + 1e-12;
      viol += any != s.budget_active;
    }
    std::printf("[INFO] T8 S1: %d non-fallback problems, risk budget active in %d\n", n, active);
    check(n > 200 && viol == 0, "T8 S1: chosen v satisfies rho <= 1 - lambda whenever any feasible candidate does");
  }

  // ---- T10 R1d: whenever a feasible moving candidate exists, one of them has zero cone slack (R1b and R1e slack) ----
  for (int variant = 0; variant < 2; ++variant)
  {
    const OccupancyGrid grid = golden::planningEasy(c.rInfl());
    int n = 0, ok = 0, chose_zero = 0;
    const std::string tag = variant == 0 ? "R1d+R1b" : "R1d+R1e";
    for (int k = 0; k < 2000; ++k)
    {
      Switches sw;
      sw.fx.r1d_feasible_cone = true;
      (variant == 0 ? sw.fx.r1b_continuous_slack : sw.fx.r1e_lowspeed_angle) = true;
      LocalProblem P;
      P.p_ref = Eigen::Vector3d(3.0 + 8.0 * U(rng), 6.0 * U(rng), 1.5);
      P.goal = P.p_ref + Eigen::Vector3d(U(rng), U(rng), 0).normalized() * 8.0;
      P.v_prev = Eigen::Vector3d(U(rng), U(rng), 0) * (k % 4 == 0 ? 0.0 : 0.8 * U01(rng));
      P.t_g = Eigen::Vector3d(U(rng), U(rng), 0.2 * U(rng)).normalized();
      P.lambda = k % 2 == 0 ? 0.0 : U01(rng);
      P.L_s = 8.0;
      P.grid = &grid;
      const LocalPlanner lp(c, sw);
      LocalProblem Pf = P;
      // the floor computed by solve() is not exposed; recompute it the same way (closest feasible moving candidate)
      double amin = M_PI;
      bool any = false;
      for (const auto &v : lp.candidates(P))
        if (v.norm() > 1e-9 && lp.turnOk(P, v) && lp.staticOk(P, v) && (P.lambda <= 0.0 || lp.dynamicOk(P, v)))
          amin = std::min(amin, angleBetween(v, P.t_g)), any = true;
      if (!any)
        continue;
      Pf.theta_floor = amin;
      Pf.floor_set = true;
      ++n;
      bool zero_slack = false;
      for (const auto &v : lp.candidates(Pf))
        if (v.norm() > 1e-9 && lp.turnOk(Pf, v) && lp.staticOk(Pf, v) && (Pf.lambda <= 0.0 || lp.dynamicOk(Pf, v)))
          zero_slack = zero_slack || lp.evaluate(Pf, v).s_g == 0.0;
      ok += zero_slack;
      chose_zero += lp.solve(P).v.norm() < 1e-9;
    }
    std::printf("[INFO] T10 %s: %d problems with a feasible moving candidate; v* = 0 chosen in %d of them\n", tag.c_str(), n,
                chose_zero);
    check(n > 500 && ok == n, "T10 " + tag + ": a feasible moving candidate with zero cone slack exists in every such problem (" +
                                  std::to_string(ok) + "/" + std::to_string(n) + ")");
  }

  // ---- T9 offline kinematic comparison (INFO only: ideal tracking, no PX4, no stall breaker; NOT Gazebo evidence) ----
  {
    const OccupancyGrid grid = golden::planningEasy(c.rInfl());
    struct Arm
    {
      std::string name;
      int group;
      int fixes; // 0 none, 1 iteration-1 set (R1b), 2 iteration-2 set (R1e, R4-m)
    };
    const std::vector<Arm> arms = {{"g3 frozen", 3, 0}, {"g6 frozen", 6, 0}, {"g6 fix it1", 6, 1}, {"g6 fix it2", 6, 2}};
    auto scenarios = golden::loopScenarios();
    scenarios.push_back({"gap_north", {0, 0, 1.5}, {0, 14, 1.5}, {}, 600}); // M1 route through the narrow gap, no obstacle
    std::printf("[INFO] T9 offline kinematic loop on planning_easy (60 s cap):\n");
    std::printf("[INFO] %-12s %-12s %8s %9s %11s %12s %9s %10s %9s\n", "scenario", "arm", "reached", "length_m",
                "stop_cycles", "max_stop_run", "switches", "fallbacks", "occupied");
    for (const auto &sc : scenarios)
      for (const auto &arm : arms)
      {
        PlannerCore core;
        core.configure(cfgFor(arm.group, [&](Fixes &f) {
          if (arm.fixes > 0)
            f.r1a_reachable_cone = f.r1c_heading_memory = f.r1d_feasible_cone = f.r2a_kappa_decay =
                f.r2b_kappa_local_excluded = f.r3_relevant_trigger = f.r4_switch_hysteresis = true;
          if (arm.fixes == 1) // iteration-1 set (R1b, R4, R5)
            f.r1b_continuous_slack = f.r5_mode_gating = true;
          if (arm.fixes == 2) // iteration-2 all_on (R1e, R4-new with R4-m)
            f.r1e_lowspeed_angle = f.r4m_dwell_free_only = f.r4n_leaky_evidence = f.r4s_safety_override = true;
        }));
        Eigen::Vector3d p = sc.start, v = Eigen::Vector3d::Zero();
        core.setGoal(sc.goal, p);
        auto obs = sc.obs;
        double length = 0.0;
        int stops = 0, run = 0, max_run = 0, switches = 0, fallbacks = 0, occupied = 0;
        bool reached = false;
        bool inv_ok = true;
        for (int k = 0; k < 600 && !reached; ++k)
        {
          const CycleRecord r = core.cycle(0.1 * k, grid, p, v, p, obs);
          inv_ok = inv_ok && r.kappa >= 0.0 && r.kappa <= 1.0 &&
                   (arm.fixes ? r.kappa <= r.kappa_bound + 1e-12 : (r.fallback_active || r.kappa <= r.lambda + 1e-12));
          for (const auto &e : core.takePathEvents())
            switches += e.is_switch && e.reason != "goal";
          fallbacks += r.fallback_active;
          reached = r.reached;
          const bool stop = r.v_star.norm() < c.v_min && (sc.goal - p).norm() > 0.5;
          stops += stop;
          run = stop ? run + 1 : 0;
          max_run = std::max(max_run, run);
          const Eigen::Vector3d p_next = p + r.v_star * 0.1;
          occupied += !grid.isFree(p_next);
          length += (p_next - p).norm();
          v = r.v_star;
          p = p_next;
          for (auto &o : obs)
            o.p += o.v * 0.1;
        }
        std::printf("[INFO] %-12s %-12s %8d %9.2f %11d %12d %9d %10d %9d\n", sc.name.c_str(), arm.name.c_str(), reached,
                    length, stops, max_run, switches, fallbacks, occupied);
        check(inv_ok, "T9 invariants (kappa range and bound) on " + sc.name + " / " + arm.name);
      }
  }

  return test_common::finish();
}
