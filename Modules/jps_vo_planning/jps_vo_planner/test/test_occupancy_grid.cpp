// Offline checks of the 0.1 m static grid (Subtask 3, deliverable 3). Prints PASS/FAIL per check.
#include "jps_vo_planner/occupancy_grid.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <random>

using jps_vo::OccupancyGrid;

static int failures = 0;
static void check(bool ok, const char *what)
{
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    ++failures;
}

static OccupancyGrid makeGrid(double inflation)
{
  OccupancyGrid g;
  OccupancyGrid::Params p;
  p.inflation = inflation;
  g.init(p);
  return g;
}

int main()
{
  // 1. octomap 0.05 voxel centres fall 2x2x2 into each 0.1 cell, never on a boundary
  {
    OccupancyGrid g = makeGrid(0.0);
    std::map<long, int> count;
    bool on_boundary = false;
    for (int i = -40; i < 40; ++i)
      for (int j = -40; j < 40; ++j)
        for (int k = 10; k < 60; ++k) // z 0.5 .. 3.0
        {
          const Eigen::Vector3d c((i + 0.5) * 0.05, (j + 0.5) * 0.05, (k + 0.5) * 0.05);
          for (int a = 0; a < 3; ++a)
          {
            const double f = (c(a) - g.params().origin(a)) / g.params().resolution;
            if (std::fabs(f - std::round(f)) < 1e-6)
              on_boundary = true;
          }
          const Eigen::Vector3i n = g.posToIndex(c);
          count[(static_cast<long>(n.z()) * 1000 + n.y()) * 1000 + n.x()]++;
        }
    bool all8 = true;
    for (const auto &kv : count)
      all8 &= kv.second == 8;
    check(!on_boundary, "octomap centres never lie on a 0.1 m cell boundary");
    check(all8, "each 0.1 m cell holds exactly 8 octomap 0.05 m voxels");
  }

  // 2. index convention identical to ego_planner GridMap::posToIndex / indexToPos
  {
    OccupancyGrid g = makeGrid(0.0);
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> ux(-25, 25), uz(0.5, 3.0);
    bool same = true;
    for (int t = 0; t < 100000; ++t)
    {
      const Eigen::Vector3d p(ux(rng), ux(rng), uz(rng));
      const Eigen::Vector3i n = g.posToIndex(p);
      Eigen::Vector3i ego;
      for (int i = 0; i < 3; ++i)
        ego(i) = std::floor((p(i) - g.params().origin(i)) * (1.0 / 0.1));
      same &= n == ego && g.posToIndex(g.indexToPos(n)) == n;
    }
    check(same, "posToIndex == ego floor((p - o) * res_inv); indexToPos round trip");
    check(g.dims() == Eigen::Vector3i(500, 500, 25), "dims 500 x 500 x 25 for 50 x 50 x 2.5 m");
  }

  // 3. ball inflation: free exactly beyond r_infl (cell-centre distance)
  {
    const double r = 0.3;
    OccupancyGrid g = makeGrid(r);
    const Eigen::Vector3d pt(0.025, 0.025, 1.525);
    g.rebuild({pt}, {});
    const Eigen::Vector3i c = g.posToIndex(pt);
    bool ok = true;
    for (int x = -5; x <= 5; ++x)
      for (int y = -5; y <= 5; ++y)
        for (int z = -5; z <= 5; ++z)
        {
          const Eigen::Vector3i n(c.x() + x, c.y() + y, c.z() + z);
          if (!g.inside(n))
            continue;
          const bool expect_occ = std::sqrt(double(x * x + y * y + z * z)) * 0.1 <= r + 1e-9;
          ok &= g.isFree(n) == !expect_occ;
        }
    check(ok, "inflation ball radius r_infl = 0.3 m");
    check(g.numRawOccupied() == 1, "one point -> one raw cell");
  }

  // 4. dynamic-obstacle capsule mask removes points inside, keeps points outside
  {
    OccupancyGrid g = makeGrid(0.0);
    OccupancyGrid::Capsule cap{Eigen::Vector3d(0, 0, 1.5), Eigen::Vector3d(2, 0, 1.5), 0.4};
    const size_t used = g.rebuild({Eigen::Vector3d(1.0, 0.3, 1.5), Eigen::Vector3d(1.0, 0.5, 1.5),
                                   Eigen::Vector3d(-0.3, 0.0, 1.5), Eigen::Vector3d(-0.5, 0.0, 1.5)},
                                  {cap});
    check(used == 2 && g.isRawOccupied(g.posToIndex(Eigen::Vector3d(1.0, 0.5, 1.5))) &&
              g.isRawOccupied(g.posToIndex(Eigen::Vector3d(-0.5, 0.0, 1.5))),
          "capsule mask drops the 2 points inside, keeps the 2 outside");
  }

  // 5. outside the box (below ground_height 0.5, at/above ceiling 3.0, outside x/y) is not free
  {
    OccupancyGrid g = makeGrid(0.0);
    g.rebuild({}, {});
    check(!g.isFree(Eigen::Vector3d(0, 0, 0.45)) && g.isFree(Eigen::Vector3d(0, 0, 0.55)) &&
              g.isFree(Eigen::Vector3d(0, 0, 2.95)) && !g.isFree(Eigen::Vector3d(0, 0, 3.0)) &&
              !g.isFree(Eigen::Vector3d(25.01, 0, 1.5)),
          "floor 0.5 / ceiling 3.0 / x-y box treated as occupied");
  }

  // 6. segmentFree and nearestFree (vs brute force)
  {
    OccupancyGrid g = makeGrid(0.2);
    g.rebuild({Eigen::Vector3d(1.025, 0.025, 1.525)}, {});
    check(!g.segmentFree(Eigen::Vector3d(0, 0, 1.5), Eigen::Vector3d(2, 0, 1.5)) &&
              g.segmentFree(Eigen::Vector3d(0, 0.5, 1.5), Eigen::Vector3d(2, 0.5, 1.5)),
          "segmentFree: blocked through the inflated cell, free 0.5 m beside it");
    std::mt19937 rng(2);
    std::uniform_real_distribution<double> u(-0.3, 0.3);
    bool ok = true;
    for (int t = 0; t < 200; ++t)
    {
      const Eigen::Vector3d q(1.025 + u(rng), 0.025 + u(rng), 1.525 + u(rng));
      Eigen::Vector3i got;
      if (!g.nearestFree(q, 10, got))
      {
        ok = false;
        continue;
      }
      double best = 1e18;
      const Eigen::Vector3i c = g.posToIndex(q);
      for (int x = -10; x <= 10; ++x)
        for (int y = -10; y <= 10; ++y)
          for (int z = -10; z <= 10; ++z)
          {
            const Eigen::Vector3i n(c.x() + x, c.y() + y, c.z() + z);
            if (g.isFree(n))
              best = std::min(best, (g.indexToPos(n) - q).squaredNorm());
          }
      ok &= std::fabs((g.indexToPos(got) - q).squaredNorm() - best) < 1e-12;
    }
    check(ok, "nearestFree returns the Euclidean-nearest free cell (200 random queries)");
  }

  // 7. rebuild time, 50k occupied voxels, r_infl 0.3 m
  {
    OccupancyGrid g = makeGrid(0.3);
    std::mt19937 rng(3);
    std::uniform_int_distribution<int> ux(-500, 499), uz(10, 59);
    std::vector<Eigen::Vector3d> pts;
    for (int t = 0; t < 50000; ++t)
      pts.emplace_back((ux(rng) + 0.5) * 0.05, (ux(rng) + 0.5) * 0.05, (uz(rng) + 0.5) * 0.05);
    const auto t0 = std::chrono::steady_clock::now();
    g.rebuild(pts, {});
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("[INFO] rebuild 50k points, r_infl 0.3: %.1f ms (%zu raw cells)\n", ms, g.numRawOccupied());
  }

  std::printf("%s (%d failures)\n", failures ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", failures);
  return failures ? 1 : 0;
}
