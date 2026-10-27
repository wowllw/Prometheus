// Offline verification of step 3 (subtask3_porting_plan.md section 3, rows 3-a ... 3-f) and of the B-spline
// bridge format (checklist B-1 ... B-5)
#include <chrono>
#include <cmath>
#include <map>
#include <queue>

#include "jps_vo_planner/bspline_utils.h"
#include "jps_vo_planner/planner_core.h"
#include "test_common.h"

using namespace jps_vo;
using test_common::check;

namespace
{
  std::mt19937 rng(23);
  std::uniform_real_distribution<double> U(-1.0, 1.0);
  Eigen::Vector3d rv(double s) { return Eigen::Vector3d(U(rng), U(rng), U(rng)) * s; }

  OccupancyGrid planningEasyGrid(int argc, char **argv, double r_infl)
  {
    OccupancyGrid g;
    OccupancyGrid::Params p;
    p.inflation = r_infl;
    g.init(p);
    g.rebuild(test_common::cylinderVoxels(test_common::loadCylinders(test_common::cylinderFile(argc, argv))), {});
    return g;
  }

  std::vector<ObstacleState> randomObstacles(const Eigen::Vector3d &around, int n)
  {
    std::vector<ObstacleState> obs;
    for (int i = 0; i < n; ++i)
    {
      ObstacleState o;
      o.name = "dyn_obs_" + std::to_string(i);
      o.p = around + Eigen::Vector3d(1.5 + 5.0 * std::fabs(U(rng)), 4.0 * U(rng), 0.4 * U(rng));
      o.v = Eigen::Vector3d(U(rng), U(rng), 0.0) * 0.5;
      o.r = 0.3;
      obs.push_back(o);
    }
    return obs;
  }

  CoreConfig config(const std::string &lambda_mode)
  {
    CoreConfig cfg;
    cfg.c.deriveTaus();
    cfg.sw.parseLambdaMode(lambda_mode);
    return cfg;
  }
} // namespace

int main(int argc, char **argv)
{
  ModelConstants c;
  c.deriveTaus();
  std::printf("[INFO] constants: d_min %.2f r_infl %.2f tau1 %.3f s tau2 %.3f s ell_w %.3f m\n", c.d_min, c.rInfl(), c.tau1,
              c.tau2, c.ellW());
  check(std::fabs(c.tau1 - 2.214) < 1e-3 && std::fabs(c.tau2 - 12.043) < 1e-3,
        "tau1, tau2 from eq. (32) match the spec table for d_min = 0.2");

  const OccupancyGrid grid = planningEasyGrid(argc, argv, c.rInfl());

  // ---- 3-a (i): lambda = 0 -> w_e = ell/L_s, D_k empty, kappa 0, path identical to geometric JPS
  {
    int same = 0, n = 0;
    bool weights_ok = true, dk_ok = true, kappa_ok = true;
    const std::vector<Eigen::Vector3d> goals{{13, 0, 1.5}, {0, 14, 1.5}, {12, 5, 2.2}};
    for (const auto &goal : goals)
      for (int k = 0; k < 5; ++k)
      {
        PlannerCore core;
        core.configure(config("zero"));
        const Eigen::Vector3d p(0.0, 0.0, 1.5);
        core.setGoal(goal, p);
        const auto obs = randomObstacles(Eigen::Vector3d(2, 2, 1.5), 3);
        const CycleRecord r = core.cycle(0.0, grid, p, Eigen::Vector3d::Zero(), p, obs);
        Eigen::Vector3i s, g;
        grid.nearestFree(p, 10, s);
        grid.nearestFree(goal, 10, g);
        const auto ref = Jps3D(&grid).searchGeometric(s, g);
        ++n;
        const GlobalPath path = core.path();
        bool cells_equal = ref.ok && path.edges.size() + 1 == ref.cells.size();
        for (size_t i = 0; cells_equal && i < path.edges.size(); ++i)
          cells_equal = path.edges[i].from_cell == ref.cells[i];
        same += cells_equal;
        for (const auto &e : path.edges)
          weights_ok &= std::fabs(e.w - e.ell / core.Ls()) < 1e-12;
        dk_ok &= core.changedEdges(grid, p, Eigen::Vector3d::Zero(), randomObstacles(Eigen::Vector3d(1, 1, 1.5), 3)) == "none";
        kappa_ok &= r.kappa == 0.0 && r.lambda == 0.0 && r.s_g_star >= 0.0;
      }
    check(same == n, "3-a(i) lambda == 0: global path identical to geometric JPS (" + std::to_string(same) + "/" +
                         std::to_string(n) + ")");
    check(weights_ok, "3-a(i) lambda == 0: every w_e == ell_e / L_s");
    check(dk_ok, "3-a(i) lambda == 0: D_k empty for any obstacle motion");
    check(kappa_ok, "3-a(i) lambda == 0: kappa == 0");
  }

  // ---- 3-a (ii): lambda = 0 local optimum == closed form of section 4.2(a) on straight path pieces
  {
    OccupancyGrid open;
    OccupancyGrid::Params p;
    p.inflation = c.rInfl();
    open.init(p);
    open.rebuild({}, {});
    int same = 0, n = 0;
    for (int k = 0; k < 400; ++k)
    {
      // goal along one of the 26 grid directions -> the path is a single straight edge
      Eigen::Vector3i d;
      do
        d = Eigen::Vector3i(std::uniform_int_distribution<int>(-1, 1)(rng), std::uniform_int_distribution<int>(-1, 1)(rng), 0);
      while (d == Eigen::Vector3i::Zero());
      const Eigen::Vector3d p0 = open.indexToPos(open.posToIndex(Eigen::Vector3d(0, 0, 1.5)));
      const Eigen::Vector3d goal = p0 + d.cast<double>() * 0.1 * (60 + k % 40);
      PlannerCore core;
      core.configure(config("zero"));
      core.setGoal(goal, p0);
      const Eigen::Vector3d dir = d.cast<double>().normalized();
      const Eigen::Vector3d p_ref = p0 + dir * 0.3;
      const Eigen::Vector3d v_prev = dir * (0.8 * std::fabs(U(rng))) + rv(0.03);
      const CycleRecord r = core.cycle(0.0, open, p_ref, v_prev, p_ref, {});
      Eigen::Vector3d v_cf;
      if (!LocalPlanner(c, Switches()).rayClosedForm(core.lastLocalProblem(), v_cf))
        continue; // ray outside V_adm: section 4.2(a) does not apply
      ++n;
      same += (r.v_star - v_cf).norm() < 1e-9;
    }
    check(n > 300 && same == n, "3-a(ii) lambda == 0: v* == s_max t_g closed form (" + std::to_string(same) + "/" +
                                    std::to_string(n) + ")");
  }

  // ---- 3-b: lambda = 1 (no rho) -> s_g* = 0, Gamma = 0, v* == classic VO (16)
  {
    CoreConfig cfg = config("one");
    cfg.sw.use_rho_in_pure_vo = false;
    LocalPlanner lp(cfg.c, cfg.sw);
    int same = 0, n = 0;
    bool sg_ok = true, gamma_ok = true;
    for (int k = 0; k < 2000; ++k)
    {
      LocalProblem P;
      P.p_ref = Eigen::Vector3d(4.0 * U(rng), 8.0 + 3.0 * U(rng), 1.5);
      P.goal = Eigen::Vector3d(0, 14, 1.5);
      P.v_prev = rv(0.45);
      P.t_g = rv(1.0).normalized();
      P.lambda = 1.0;
      P.L_s = 14.0;
      P.grid = &grid;
      P.obs = randomObstacles(P.p_ref, 1 + k % 3);
      const LocalSolution a = lp.solve(P), b = lp.classicVO(P);
      ++n;
      same += (a.v - b.v).norm() < 1e-12 && a.fallback == b.fallback;
      sg_ok &= a.s_g == 0.0;
      gamma_ok &= a.Gamma == 0.0;
    }
    check(same == n, "3-b lambda == 1: v* identical to classic VO (16) (" + std::to_string(same) + "/" + std::to_string(n) + ")");
    check(sg_ok && gamma_ok, "3-b lambda == 1: s_g* == 0 and Gamma == 0 on every scene");
  }

  // ---- 3-c: lambda(xi) properties (30)(31)
  {
    Switches sw;
    bool ends = true, mono = true;
    double prev = 1.0, max_sp = 0.0;
    for (double xi = 0.01; xi < 2.0 * c.tau2; xi += 0.001)
    {
      const double l = lambdaOf(xi, c, sw);
      if (xi >= c.tau2)
        ends &= l == 0.0;
      if (xi <= c.tau1)
        ends &= l == 1.0;
      mono &= l <= prev + 1e-15;
      prev = l;
    }
    for (double uu = 0.0; uu <= 1.0; uu += 1e-4)
      max_sp = std::max(max_sp, (smoothStep(uu + 1e-6) - smoothStep(uu)) / 1e-6);
    check(ends, "3-c lambda = 0 for xi >= tau2 and 1 for xi <= tau1");
    check(mono, "3-c lambda monotone non-increasing in xi");
    check(max_sp <= 15.0 / 8.0 + 1e-4, "3-c max S'(u) <= 15/8 (eq. 30)");
  }

  // ---- 3-d: Proposition 1 on a small grid: w_e >= ell/L_s, A* on G_J == Dijkstra on the enumerated G_J
  {
    int same = 0, n = 0;
    bool lower = true;
    for (int k = 0; k < 40; ++k)
    {
      OccupancyGrid g;
      OccupancyGrid::Params p;
      p.origin = Eigen::Vector3d(0, 0, 0.5);
      p.size = Eigen::Vector3d(3.0, 3.0, 0.6);
      p.inflation = 0.0;
      g.init(p);
      std::vector<Eigen::Vector3d> pts;
      for (int x = 0; x < 30; ++x)
        for (int y = 0; y < 30; ++y)
          for (int z = 0; z < 6; ++z)
            if (std::fabs(U(rng)) < 0.15)
              pts.push_back(g.indexToPos(Eigen::Vector3i(x, y, z)));
      g.rebuild(pts, {});
      Eigen::Vector3i s(1, 1, 2), e(27, 26, 3);
      if (!g.isFree(s) || !g.isFree(e))
        continue;
      PlannerCore core;
      CoreConfig cfg = config("adaptive");
      cfg.search_margin = 0.0; // compare against the full enumerated G_J
      core.configure(cfg);
      const Eigen::Vector3d p_ref = g.indexToPos(s), v_ref(0.5, 0.2, 0.0);
      core.setGoal(g.indexToPos(e), p_ref);
      auto obs = randomObstacles(p_ref, 2);
      for (auto &o : obs)
        o.p = p_ref + Eigen::Vector3d(1.5 * std::fabs(U(rng)) + 0.5, 1.5 * std::fabs(U(rng)) + 0.5, 0.0);
      const Jps3D jps(&g);
      // explicit G_J by expanding every reachable (cell, dir_in) state
      std::map<std::pair<long, int>, double> dist;
      using QE = std::pair<double, std::pair<long, int>>;
      std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
      auto ck = [&](const Eigen::Vector3i &c3) { return (static_cast<long>(c3.z()) * 30 + c3.y()) * 30 + c3.x(); };
      auto cell = [&](long k2) { return Eigen::Vector3i(k2 % 30, (k2 / 30) % 30, k2 / 900); };
      dist[{ck(s), Jps3D::kNoDir}] = 0.0;
      pq.push({0.0, {ck(s), Jps3D::kNoDir}});
      double best = 1e18;
      std::vector<std::pair<Eigen::Vector3i, int>> succ;
      while (!pq.empty())
      {
        const QE top = pq.top();
        pq.pop();
        if (top.first > dist[top.second] + 1e-12)
          continue;
        const Eigen::Vector3i x = cell(top.second.first);
        if (x == e)
        {
          best = std::min(best, top.first);
          continue;
        }
        jps.successors(x, top.second.second, e, succ);
        for (const auto &sj : succ)
        {
          const double we = core.edgeWeight(g, x, sj.first, top.second.second, p_ref, v_ref, obs);
          const double ell = (sj.first - x).cast<double>().norm() * 0.1;
          lower &= we >= ell / core.Ls() - 1e-12;
          const std::pair<long, int> key{ck(sj.first), sj.second};
          if (!dist.count(key) || top.first + we < dist[key] - 1e-12)
          {
            dist[key] = top.first + we;
            pq.push({dist[key], key});
          }
        }
      }
      if (best > 1e17)
        continue;
      ++n;
      core.replanGlobal(0.0, g, p_ref, v_ref, obs, "goal");
      double cost = 0.0;
      for (const auto &ed : core.path().edges)
        cost += ed.w;
      same += std::fabs(cost - best) < 1e-9;
    }
    check(lower, "3-d w_e >= ell_e / L_s on every enumerated G_J edge (Prop. 1 premise)");
    check(n >= 20 && same == n, "3-d A* on (jump point x dir_in) == Dijkstra on enumerated G_J (" + std::to_string(same) +
                                    "/" + std::to_string(n) + ")");
  }

  // ---- 3-e: exact penalty: s_g* = 0 whenever a feasible candidate inside the guidance cone exists; G_theta estimate
  {
    Switches sw;
    LocalPlanner lp(c, sw);
    int n = 0, ok = 0, n_pos = 0;
    double g_theta = 0.0;
    for (int k = 0; k < 3000; ++k)
    {
      LocalProblem P;
      P.p_ref = Eigen::Vector3d(4.0 * U(rng), 8.0 + 3.0 * U(rng), 1.5);
      P.goal = Eigen::Vector3d(0, 14, 1.5);
      P.v_prev = rv(0.5);
      P.t_g = (P.goal - P.p_ref).normalized();
      if (P.v_prev.norm() > 0.1)
        P.t_g = (P.t_g + rv(0.5)).normalized();
      P.lambda = std::fabs(U(rng));
      P.L_s = 14.0;
      P.grid = &grid;
      P.obs = randomObstacles(P.p_ref, 1 + k % 3);
      const LocalSolution s = lp.solve(P);
      if (s.fallback)
        continue;
      bool cone_feasible = false;
      for (const auto &v : lp.candidates(P))
      {
        if (!lp.turnOk(P, v) || !lp.staticOk(P, v) || (P.lambda > 0 && !lp.dynamicOk(P, v)))
          continue;
        if (lp.evaluate(P, v).s_g <= 1e-9)
          cone_feasible = true;
        // dJ_total / dtheta by rotating v by 1 mrad in the (v, t_g) plane, same speed
        const Eigen::Vector3d axis = v.cross(P.t_g);
        if (v.norm() > 0.1 && axis.norm() > 1e-6)
        {
          const Eigen::Vector3d v2 = Eigen::AngleAxisd(-1e-3, axis.normalized()) * v;
          const LocalSolution a = lp.evaluate(P, v), b = lp.evaluate(P, v2);
          g_theta = std::max(g_theta, std::fabs((b.J - c.rho_g * b.s_g) - (a.J - c.rho_g * a.s_g)) / 1e-3);
        }
      }
      if (!cone_feasible)
      {
        n_pos += s.s_g > 1e-9;
        continue;
      }
      ++n;
      ok += s.s_g <= 1e-9;
    }
    std::printf("[INFO] 3-e: estimated G_theta = sup |dJ_total/dtheta| = %.3f (rho_g = %.1f); s_g* > 0 in %d scenes where "
                "the cone and F are disjoint\n",
                g_theta, c.rho_g, n_pos);
    check(n > 500 && ok == n, "3-e s_g* == 0 whenever cone and F intersect (" + std::to_string(ok) + "/" + std::to_string(n) + ")");
    check(c.rho_g > g_theta, "3-e rho_g > estimated G_theta");
  }

  // ---- 3-f: invariants over random coupled cycles; global/local timing on planning_easy
  {
    bool ranges = true, kappa_le_lambda = true;
    double worst_local = 0.0, worst_global = 0.0, worst_dk = 0.0;
    for (int k = 0; k < 60; ++k)
    {
      PlannerCore core;
      core.configure(config("adaptive"));
      Eigen::Vector3d p(0, 0, 1.5), v(0, 0, 0);
      core.setGoal(Eigen::Vector3d(0, 14, 1.5), p);
      auto obs = randomObstacles(Eigen::Vector3d(0, 8, 1.5), 3);
      for (int step = 0; step < 40; ++step)
      {
        const auto t0 = std::chrono::steady_clock::now();
        const std::string dk = core.changedEdges(grid, p, v, obs);
        worst_dk = std::max(worst_dk, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        (void)dk;
        const CycleRecord r = core.cycle(step * 0.1, grid, p, v, p, obs);
        worst_local = std::max(worst_local, r.plan_ms_local);
        worst_global = std::max(worst_global, r.plan_ms_global);
        ranges &= r.lambda >= 0 && r.lambda <= 1 && r.kappa >= 0 && r.kappa <= 1;
        kappa_le_lambda &= r.fallback_active || r.kappa <= r.lambda + 1e-12;
        v = r.v_star;
        p += v * 0.1;
        for (auto &o : obs)
          o.p += o.v * 0.1;
      }
    }
    std::printf("[INFO] 3-f worst times over 2400 cycles: local %.2f ms, global %.2f ms, D_k check %.2f ms\n", worst_local,
                worst_global, worst_dk);
    check(ranges, "3-f lambda, kappa in [0, 1] on every cycle");
    check(kappa_le_lambda, "3-f kappa <= lambda_k (eq. 25) except fallback (kappa = 1)");
  }

  // ---- B-1 ... B-5: velocity bridge format
  {
    bool ok = true;
    for (int k = 0; k < 2000; ++k)
    {
      const Eigen::Vector3d p0 = rv(5.0), v0 = rv(0.45), vt = v0 + rv(0.046);
      const auto tr = buildVelocityBridge(p0, v0, vt, c.a_max, c.tau_h, 0.1);
      const auto knots = tr.knots();
      ok &= knots.size() == tr.ctrl_pts.size() + 4;                                    // B-1
      Eigen::Vector3d pp, vv;
      tr.evaluate(0.0, pp, vv);
      ok &= (pp - p0).norm() < 0.01 && (vv - v0).norm() < c.a_max * 0.1;              // B-2, B-3
      const size_t m = tr.ctrl_pts.size();
      ok &= (tr.ctrl_pts[m - 1] - tr.ctrl_pts[m - 3]).norm() < 1e-12;                  // B-4
      for (size_t i = 0; i + 1 < m; ++i)                                               // B-5
        ok &= (tr.ctrl_pts[i + 1] - tr.ctrl_pts[i]).norm() / 0.1 <= std::max(v0.norm(), vt.norm()) + 1e-9;
      for (size_t i = 0; i + 2 < m; ++i)
        ok &= (tr.ctrl_pts[i + 2] - 2.0 * tr.ctrl_pts[i + 1] + tr.ctrl_pts[i]).norm() / 0.01 <= c.a_max + 1e-9;
    }
    check(ok, "B-1..B-5 bridge: knots = n+4, start == (p0, v0), stop with zero velocity, |v| and |a| bounds");
  }
  return test_common::finish();
}
