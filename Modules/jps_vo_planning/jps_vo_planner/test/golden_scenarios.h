#ifndef JPS_VO_PLANNER_GOLDEN_SCENARIOS_H
#define JPS_VO_PLANNER_GOLDEN_SCENARIOS_H

// Deterministic dump of planner outputs for the baseline-identity test (fixes after thesis-frozen-v1).
// Uses ONLY the API of thesis-frozen-v1, so gen_golden.cpp compiles unchanged against the frozen sources and
// produces the reference file; test_fixes.cpp produces the same dump with every Fixes flag false and compares.
// Timing fields (plan_ms_*) are excluded: they are not deterministic.

#include <cstdio>
#include <functional>
#include <ostream>
#include <random>
#include <string>

#include "jps_vo_planner/planner_core.h"
#include "test_common.h"

namespace golden
{
  using namespace jps_vo;

  inline std::string f(double x)
  {
    char b[40];
    std::snprintf(b, sizeof(b), "%.17g", x);
    return b;
  }
  inline std::string f3(const Eigen::Vector3d &v) { return f(v.x()) + " " + f(v.y()) + " " + f(v.z()); }

  using Customize = std::function<void(Switches &)>;

  // group switch sets of config/groups/g1..g6.yaml
  inline Switches groupSwitches(int g)
  {
    Switches sw;
    switch (g)
    {
    case 1: sw.parseLambdaMode("zero"); break;
    case 2: sw.parseLambdaMode("one"); break;
    case 3: sw.enable_c2 = false; sw.enable_c3 = false; break;
    case 4: sw.parseLambdaMode("const:0.5"); break;
    case 5: sw.enable_gamma = false; break;
    default: break;
    }
    return sw;
  }

  inline OccupancyGrid planningEasy(double r_infl)
  {
    OccupancyGrid g;
    OccupancyGrid::Params p;
    p.inflation = r_infl;
    g.init(p);
    g.rebuild(test_common::cylinderVoxels(test_common::loadCylinders(test_common::cylinderFile(0, nullptr))), {});
    return g;
  }

  // (a) random local problems: solve() and evaluate() outputs
  inline void dumpLocal(std::ostream &os, const OccupancyGrid &grid, int n, unsigned seed, const Customize &cz)
  {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U(-1.0, 1.0), U01(0.0, 1.0);
    auto unit = [&]() {
      Eigen::Vector3d d(U(rng), U(rng), 0.3 * U(rng));
      return Eigen::Vector3d(d.normalized());
    };
    ModelConstants c;
    c.deriveTaus();
    for (int k = 0; k < n; ++k)
    {
      const int g = 1 + k % 6;
      Switches sw = groupSwitches(g);
      cz(sw);
      LocalProblem P;
      P.p_ref = Eigen::Vector3d(3.0 + 8.0 * U(rng), 6.0 * U(rng), 1.5 + 0.5 * U(rng));
      P.goal = P.p_ref + unit() * (2.0 + 10.0 * U01(rng));
      const int sc = k % 5;
      const double sp = sc == 0 ? 0.0 : (sc == 1 ? 0.1 * U01(rng) : 0.1 + 0.7 * U01(rng));
      P.v_prev = unit() * sp;
      const int lc = k % 3;
      P.lambda = g == 1 ? 0.0 : (g == 2 ? 1.0 : (g == 4 ? 0.5 : (lc == 0 ? 0.0 : (lc == 1 ? 1.0 : U01(rng)))));
      P.L_s = std::max((P.goal - P.p_ref).norm(), c.L_min);
      GlobalPath path;
      if (U01(rng) < 0.85)
      {
        Eigen::Vector3d a = P.p_ref - 0.5 * unit();
        Eigen::Vector3d h = unit();
        for (int i = 0; i < 3; ++i)
        {
          PathEdge e;
          e.a = a;
          e.b = a + h * (1.0 + 3.0 * U01(rng));
          e.ell = (e.b - e.a).norm();
          e.t = (e.b - e.a) / e.ell;
          e.from_cell = grid.posToIndex(e.a);
          e.to_cell = grid.posToIndex(e.b);
          e.w = e.ell / P.L_s + (k % 4 == 0 ? 0.5 * U01(rng) : 0.0);
          path.edges.push_back(e);
          a = e.b;
          const double turn = 1.5 * U(rng);
          h = Eigen::AngleAxisd(turn, Eigen::Vector3d::UnitZ()) * h;
        }
        path.finalize();
        P.t_g = path.guideTangent(path.project(P.p_ref).s, c.ellW());
        P.path = &path;
      }
      P.grid = &grid;
      for (int i = 0; i < k % 4; ++i)
      {
        ObstacleState o;
        o.name = "o" + std::to_string(i);
        o.p = P.p_ref + unit() * (1.0 + 5.0 * U01(rng));
        o.v = Eigen::Vector3d(U(rng), U(rng), 0.0) * 0.6;
        o.r = 0.3;
        P.obs.push_back(o);
      }
      const LocalPlanner lp(c, sw);
      const LocalSolution s = lp.solve(P);
      os << "L " << k << " " << f3(s.v) << " " << f(s.J) << " " << f(s.s_g) << " " << s.fallback << " " << s.n_feasible
         << " " << s.n_candidates << " " << f(s.J_A) << " " << f(s.J_B) << " " << f(s.Gamma) << " " << f(s.rho) << "\n";
      const auto cand = lp.candidates(P);
      for (size_t j : {size_t(0), cand.size() / 3, 2 * cand.size() / 3, cand.size() - 1})
      {
        const LocalSolution e = lp.evaluate(P, cand[j]);
        os << "E " << k << " " << j << " " << f(e.J) << " " << f(e.s_g) << " " << f(e.J_A) << " " << f(e.J_B) << " "
           << f(e.Gamma) << " " << f(e.rho) << "\n";
      }
    }
  }

  struct LoopScenario
  {
    std::string name;
    Eigen::Vector3d start, goal;
    std::vector<ObstacleState> obs;
    int cycles;
  };

  inline std::vector<LoopScenario> loopScenarios()
  {
    auto ob = [](const std::string &n, Eigen::Vector3d p, Eigen::Vector3d v) {
      ObstacleState o;
      o.name = n;
      o.p = p;
      o.v = v;
      o.r = 0.3;
      return o;
    };
    return {
        {"static_east", {0, 0, 1.5}, {13, 0, 1.5}, {}, 260},
        {"static_ne", {0, 0, 1.5}, {9, 5, 1.8}, {}, 220},
        {"head_on", {0, 0, 1.5}, {13, 0, 1.5}, {ob("h", {11, 0.1, 1.5}, {-0.6, 0, 0})}, 260},
        {"three_way",
         {0, 0, 1.5},
         {13, 0, 1.5},
         {ob("a", {10, 0.2, 1.5}, {-0.5, 0, 0}), ob("b", {6, -5, 1.5}, {0, 0.6, 0}), ob("c", {7, 5, 1.5}, {-0.1, -0.6, 0})},
         260},
    };
  }

  // (b) closed-loop PlannerCore::cycle (reference integrated from v*, ideal tracking), groups g1/g3/g5/g6
  inline void dumpClosedLoop(std::ostream &os, const OccupancyGrid &grid, const Customize &cz)
  {
    for (const auto &sc : loopScenarios())
      for (int g : {1, 3, 5, 6})
      {
        CoreConfig cfg;
        cfg.c.deriveTaus();
        cfg.sw = groupSwitches(g);
        cz(cfg.sw);
        PlannerCore core;
        core.configure(cfg);
        Eigen::Vector3d p = sc.start, v = Eigen::Vector3d::Zero();
        core.setGoal(sc.goal, p);
        auto obs = sc.obs;
        const double dt = cfg.c.dt;
        for (int k = 0; k < sc.cycles; ++k)
        {
          const double t = k * dt;
          const CycleRecord r = core.cycle(t, grid, p, v, p, obs);
          os << "C " << sc.name << " g" << g << " " << k << " " << f(r.lambda) << " " << f(r.xi) << " " << f(r.s_g_star)
             << " " << f(r.kappa) << " " << f3(r.v_star) << " " << r.path_id << " " << r.path_hash << " " << r.n_jump_pts
             << " " << f(r.min_surface_dist_ref) << " " << f(r.turn_angle) << " " << f(r.lateral_dev_ref_to_Pi) << " "
             << f(r.lateral_dev_ref_to_Pi0) << " " << r.replan_reason << " " << r.fallback_active << " " << r.encounter_flag
             << " " << r.reached << " " << r.path_switched << " " << r.n_feasible << " " << r.n_candidates << " " << f(r.J)
             << " " << f3(r.t_g) << "\n";
          for (const auto &e : core.takePathEvents())
            os << "P " << sc.name << " g" << g << " " << f(e.t) << " " << e.path_id << " " << e.hash << " " << e.reason << " "
               << e.is_switch << " " << e.pts.size() << " " << f3(e.pts.front()) << " " << f3(e.pts.back()) << "\n";
          if (k % 10 == 5)
          {
            os << "D " << sc.name << " g" << g << " " << k << " " << core.changedEdges(grid, p, v, obs) << "\n";
            const GlobalPath path = core.path();
            for (size_t i = 0; i < std::min<size_t>(3, path.edges.size()); ++i)
            {
              const auto &e = path.edges[i];
              os << "W " << sc.name << " g" << g << " " << k << " " << i << " "
                 << f(core.edgeWeight(grid, e.from_cell, e.to_cell, e.dir_in, p, v, obs)) << " " << f(e.w) << "\n";
            }
          }
          if (r.reached)
            break;
          v = r.v_star;
          p += v * dt;
          for (auto &o : obs)
            o.p += o.v * dt;
        }
      }
  }

  inline void dumpAll(std::ostream &os, const Customize &cz)
  {
    ModelConstants c;
    c.deriveTaus();
    const OccupancyGrid grid = planningEasy(c.rInfl());
    dumpLocal(os, grid, 3000, 20261030u, cz);
    dumpClosedLoop(os, grid, cz);
  }
} // namespace golden

#endif
