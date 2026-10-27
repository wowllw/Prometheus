// Offline verification of step 2 (subtask3_porting_plan.md section 3, rows 2-a, 2-b, 2-c)
#include <cmath>

#include "jps_vo_planner/local_planner.h"
#include "test_common.h"

using namespace jps_vo;
using test_common::check;

int main()
{
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  auto rv = [&](double s) { return Eigen::Vector3d(u(rng), u(rng), u(rng)) * s; };

  // ---- 2-a: collision cone (12) vs brute-force closest approach
  {
    int agree = 0, n = 0;
    for (int k = 0; k < 100000; ++k)
    {
      const double R = 0.5 + 0.5 * std::fabs(u(rng));
      Eigen::Vector3d x = rv(6.0);
      if (x.norm() <= R + 0.05)
        continue;
      const Eigen::Vector3d w = rv(1.5);
      // brute force: 200001 samples of t on [0, 3 |x| / |w|], beyond any closest approach
      double dmin = x.norm();
      const double T = 3.0 * x.norm() / std::max(w.norm(), 1e-9); // beyond any closest approach
      for (int i = 0; i <= 200000; ++i)
        dmin = std::min(dmin, (x - w * (T * i / 200000.0)).norm());
      if (std::fabs(dmin - R) < 1e-3)
        continue; // epsilon band around the cone surface (sampling resolution)
      ++n;
      if (inCollisionCone(x, w, R) == (dmin <= R))
        ++agree;
      if (n >= 3000)
        break;
    }
    std::printf("[INFO] 2-a: %d / %d samples agree\n", agree, n);
    check(agree == n, "2-a CC membership (12) == brute-force closest approach <= R");
  }

  // ---- 2-b: t_c (14) vs bisection root of ||x - w t|| = R
  {
    double worst = 0.0;
    int n = 0;
    for (int k = 0; k < 200000 && n < 5000; ++k)
    {
      const double R = 0.6;
      const Eigen::Vector3d x = rv(6.0), w = rv(1.5);
      if (x.norm() <= R + 0.01 || !inCollisionCone(x, w, R))
        continue;
      double lo = 0.0, hi = x.dot(w) / w.squaredNorm(); // f(lo) > 0, f(hi) <= 0
      for (int it = 0; it < 200; ++it)
      {
        const double mid = 0.5 * (lo + hi);
        ((x - w * mid).norm() > R ? lo : hi) = mid;
      }
      worst = std::max(worst, std::fabs(timeToCollision(x, w, R) - 0.5 * (lo + hi)));
      ++n;
    }
    std::printf("[INFO] 2-b: %d samples, max |t_c - root| = %.2e s\n", n, worst);
    check(n > 1000 && worst < 1e-6, "2-b t_c (14) equals the numeric first-contact root (|dt| < 1e-6 s)");
  }

  // ---- 2-c: classic VO (16): v* in F and no feasible candidate closer to v_pref
  {
    ModelConstants c;
    c.deriveTaus();
    Switches sw;
    LocalPlanner lp(c, sw);
    int ok = 0, n = 0, fb = 0;
    for (int k = 0; k < 2000; ++k)
    {
      LocalProblem P;
      P.p_ref = Eigen::Vector3d(0, 0, 1.5);
      P.goal = P.p_ref + Eigen::Vector3d(10, 0, 0) + rv(3.0);
      P.v_prev = rv(0.45);
      P.L_s = 10.0;
      P.lambda = 1.0;
      const int no = 1 + k % 3;
      for (int i = 0; i < no; ++i)
      {
        ObstacleState o;
        o.p = P.p_ref + Eigen::Vector3d(2.0 + 4.0 * std::fabs(u(rng)), 3.0 * u(rng), 0.5 * u(rng));
        o.v = rv(0.5);
        o.r = 0.3;
        P.obs.push_back(o);
      }
      const LocalSolution s = lp.classicVO(P);
      ++n;
      if (s.fallback)
      {
        ++fb;
        ++ok; // F empty: (40) applies, optimality over F is vacuous
        continue;
      }
      const Eigen::Vector3d v_pref = c.v_c * (P.goal - P.p_ref).normalized();
      bool good = lp.admissible(P, s.v) && lp.turnOk(P, s.v) && lp.dynamicOk(P, s.v);
      for (const auto &v : lp.candidates(P))
        if (lp.turnOk(P, v) && lp.dynamicOk(P, v) && (v - v_pref).norm() < (s.v - v_pref).norm() - 1e-12)
          good = false;
      ok += good;
    }
    std::printf("[INFO] 2-c: %d scenes, %d with empty F (fallback 40)\n", n, fb);
    check(ok == n, "2-c classic VO: v* in F and minimal ||v - v_pref|| over the feasible candidates");
  }
  return test_common::finish();
}
