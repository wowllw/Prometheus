// Offline verification of step 1 (subtask3_porting_plan.md section 3, rows 1-a, 1-b, 1-c)
#include <chrono>
#include <cmath>

#include "jps_vo_planner/jps3d.h"
#include "test_common.h"

using namespace jps_vo;
using test_common::check;

namespace
{
  OccupancyGrid smallGrid(const Eigen::Vector3d &size)
  {
    OccupancyGrid g;
    OccupancyGrid::Params p;
    p.origin = Eigen::Vector3d(0, 0, 0.5);
    p.size = size;
    p.inflation = 0.0;
    g.init(p);
    return g;
  }

  // every segment is one of the 26 directions and all its cells are legal (rows 1-b)
  bool segmentsValid(const Jps3D &jps, const Jps3D::Result &r)
  {
    for (size_t i = 0; i + 1 < r.cells.size(); ++i)
    {
      const Eigen::Vector3i diff = r.cells[i + 1] - r.cells[i];
      const int steps = diff.cwiseAbs().maxCoeff();
      if (steps == 0)
        return false;
      const Eigen::Vector3i d = diff / steps;
      if (d * steps != diff || d.cwiseAbs().maxCoeff() != 1 || Jps3D::dirIdx(d) != r.dirs[i])
        return false;
      Eigen::Vector3i x = r.cells[i];
      for (int s = 0; s < steps; ++s, x += d)
        if (!jps.legal(x, d))
          return false;
    }
    return true;
  }
} // namespace

int main(int argc, char **argv)
{
  // ---- 1-a: JPS cost == 26-connected A* cost on random maps (10-30 % occupancy)
  {
    std::mt19937 rng(7);
    int agree = 0, total = 0, both_fail = 0, valid = 0, literal_agree = 0;
    size_t exp_jps = 0, exp_astar = 0;
    double worst = 0.0;
    for (int t = 0; t < 200; ++t)
    {
      OccupancyGrid g = smallGrid(Eigen::Vector3d(3.0, 3.0, 1.0)); // 30 x 30 x 10 cells
      const double occ = 0.10 + 0.20 * (t % 11) / 10.0;
      std::uniform_real_distribution<double> u01(0, 1);
      std::vector<Eigen::Vector3d> pts;
      for (int x = 0; x < 30; ++x)
        for (int y = 0; y < 30; ++y)
          for (int z = 0; z < 10; ++z)
            if (u01(rng) < occ)
              pts.push_back(g.indexToPos(Eigen::Vector3i(x, y, z)));
      g.rebuild(pts, {});
      Jps3D jps(&g);
      std::uniform_int_distribution<int> ux(0, 29), uz(0, 9);
      Eigen::Vector3i s, e;
      do
        s = Eigen::Vector3i(ux(rng), ux(rng), uz(rng));
      while (!g.isFree(s));
      do
        e = Eigen::Vector3i(ux(rng), ux(rng), uz(rng));
      while (!g.isFree(e) || e == s);
      const auto a = jps.astar26(s, e);
      const auto j = jps.searchGeometric(s, e);
      Jps3D lit(&g, Jps3D::TieRule::Literal);
      const auto jl = lit.searchGeometric(s, e);
      if (a.ok && jl.ok && std::fabs(a.cost - jl.cost) < 1e-9)
        ++literal_agree;
      ++total;
      if (!a.ok && !j.ok)
      {
        ++both_fail;
        ++agree;
        continue;
      }
      if (a.ok && j.ok && std::fabs(a.cost - j.cost) < 1e-9)
        ++agree;
      else if (a.ok && j.ok)
        worst = std::max(worst, j.cost - a.cost);
      if (j.ok && segmentsValid(jps, j))
        ++valid;
      exp_jps += j.expansions;
      exp_astar += a.expansions;
    }
    std::printf("[INFO] random maps: %d/%d agree (%d unreachable for both), worst excess %.4f m, expansions JPS %zu vs A* %zu\n",
                agree, total, both_fail, worst, exp_jps, exp_astar);
    std::printf("[INFO] tie rule 'literal' (eq. 7 '<=' also for diagonal arrivals): %d/%d agree with A*\n", literal_agree,
                total - both_fail);
    check(agree == total, "1-a JPS (tie rule 'strict') cost == 26-connected A* cost on 200 random maps (10-30 % occupancy)");
    check(valid == total - both_fail, "1-b every JPS segment is one of the 26 directions and all its cells are legal");
    std::printf("[INFO] expansions are informative only: JPS states (scan steps not counted) %zu, A* %zu\n", exp_jps, exp_astar);
  }

  // ---- 1-c: h_D consistency h(n) <= c(d) + h(n') over all cells and directions (small grid)
  {
    OccupancyGrid g = smallGrid(Eigen::Vector3d(2.0, 2.0, 1.0));
    g.rebuild({}, {});
    Jps3D jps(&g);
    const Eigen::Vector3i goal(13, 4, 7);
    bool ok = true;
    for (int x = 0; x < 20; ++x)
      for (int y = 0; y < 20; ++y)
        for (int z = 0; z < 10; ++z)
          for (int k = 0; k < 27; ++k)
          {
            if (k == Jps3D::kNoDir)
              continue;
            const Eigen::Vector3i n(x, y, z), d = Jps3D::dirVec(k);
            ok &= jps.hD(n, goal) <= d.cast<double>().norm() * 0.1 + jps.hD(n + d, goal) + 1e-12;
          }
    check(ok, "1-c h_D (eq. 5) consistent on all cells and 26 directions");
  }

  // ---- 1-a on the planning_easy grid (r_infl = 0.6 m, d_min = 0.2)
  {
    const auto cyl = test_common::loadCylinders(test_common::cylinderFile(argc, argv));
    check(cyl.size() == 39, "loaded 39 planning_easy cylinders");
    OccupancyGrid g;
    OccupancyGrid::Params p;
    p.inflation = 0.6;
    g.init(p);
    g.rebuild(test_common::cylinderVoxels(cyl), {});
    Jps3D jps(&g);
    const std::vector<Eigen::Vector3d> goals{{13, 0, 1.5}, {0, 14, 1.5}, {12, 5, 2.2}};
    for (const auto &gp : goals)
    {
      Eigen::Vector3i s, e;
      g.nearestFree(Eigen::Vector3d(0, 0, 1.5), 10, s);
      g.nearestFree(gp, 10, e);
      auto t0 = std::chrono::steady_clock::now();
      const auto j = jps.searchGeometric(s, e);
      const double ms_j = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      t0 = std::chrono::steady_clock::now();
      const auto a = jps.astar26(s, e);
      const double ms_a = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      std::printf("[INFO] goal (%.0f,%.0f,%.1f): JPS %.3f m %zu exp %.1f ms | A* %.3f m %zu exp %.1f ms | %zu jump points\n",
                  gp.x(), gp.y(), gp.z(), j.cost, j.expansions, ms_j, a.cost, a.expansions, ms_a, j.cells.size());
      check(j.ok && a.ok && std::fabs(j.cost - a.cost) < 1e-9 && segmentsValid(jps, j),
            "1-a/1-b planning_easy goal (" + std::to_string(int(gp.x())) + "," + std::to_string(int(gp.y())) + ")");
      // direction-augmented states (needed with Gamma_e): same optimum as per-cell closure
      t0 = std::chrono::steady_clock::now();
      const auto jd = jps.search(
          s, e, [](const Eigen::Vector3i &x, const Eigen::Vector3i &y, int, int) { return (y - x).cast<double>().norm() * 0.1; },
          1.0, true);
      const double ms_d = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      std::printf("[INFO]   direction-augmented states: JPS %.3f m %zu exp %.1f ms\n", jd.cost, jd.expansions, ms_d);
      check(jd.ok && std::fabs(jd.cost - a.cost) < 1e-9, "Prop. 1 (ii) state space (jump point x dir_in) keeps the optimum");
    }
  }
  return test_common::finish();
}
