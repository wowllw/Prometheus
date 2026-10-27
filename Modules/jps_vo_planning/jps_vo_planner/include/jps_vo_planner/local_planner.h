#ifndef JPS_VO_PLANNER_LOCAL_PLANNER_H
#define JPS_VO_PLANNER_LOCAL_PLANNER_H

// Local problem P_loc (Stage-2 eqs. 36-38, fallback 40) over a sampled candidate set [ENG-4], plus the two
// independent references used by the degeneracy tests: classic VO (eq. 16) and the lambda = 0 closed form
// (section 4.2(a)). ROS-free.

#include <Eigen/Eigen>
#include <vector>

#include "jps_vo_planner/coupling.h"
#include "jps_vo_planner/global_path.h"
#include "jps_vo_planner/occupancy_grid.h"

namespace jps_vo
{
  struct LocalProblem
  {
    Eigen::Vector3d p_ref = Eigen::Vector3d::Zero();  // p_A (reference state, [ENG-1])
    Eigen::Vector3d v_prev = Eigen::Vector3d::Zero(); // v_prev = v_ref
    Eigen::Vector3d goal = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_g = Eigen::Vector3d::Zero();    // eq. (27); zero if there is no global path
    double lambda = 0.0;                              // lambda(xi) of the reference motion
    double L_s = 2.0;
    const GlobalPath *path = nullptr;                 // nullptr: no global path
    const OccupancyGrid *grid = nullptr;
    std::vector<ObstacleState> obs;                   // states at the planning time
  };

  struct LocalSolution
  {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    double s_g = 0.0, J = 0.0, J_A = 0.0, J_B = 0.0, Gamma = 0.0, rho = 0.0;
    bool fallback = false;
    int n_candidates = 0, n_feasible = 0;
  };

  class LocalPlanner
  {
  public:
    LocalPlanner(const ModelConstants &c, const Switches &sw) : c_(c), sw_(sw) {}

    // candidate velocities inside V_adm (C-v)(C-a) incl. the goal-stop limit [ENG-3]; deterministic order
    std::vector<Eigen::Vector3d> candidates(const LocalProblem &P) const;

    // P_loc (37) s.t. (38); infeasible -> fallback (40)
    LocalSolution solve(const LocalProblem &P) const;
    // eq. (16): argmin over F of ||v - v_pref||, v_pref = v_c g_hat, on the same candidates and constraints
    LocalSolution classicVO(const LocalProblem &P) const;
    // section 4.2(a): v = s t_g with the largest s allowed by (C-v)(C-a); false if the ray misses V_adm
    bool rayClosedForm(const LocalProblem &P, Eigen::Vector3d &v) const;

    // constraint pieces of (38)
    double speedLimit(const LocalProblem &P) const; // min(v_c, sqrt(2 a_max d_goal)) [ENG-3]
    bool admissible(const LocalProblem &P, const Eigen::Vector3d &v) const;  // (C-v)(C-a)
    bool turnOk(const LocalProblem &P, const Eigen::Vector3d &v) const;      // (C-theta)
    bool staticOk(const LocalProblem &P, const Eigen::Vector3d &v) const;    // (C-s) static part, always on [ENG-6]
    bool dynamicOk(const LocalProblem &P, const Eigen::Vector3d &v) const;   // (C-s) v not in VO^{tau2} for all o
    double minTtc(const LocalProblem &P, const Eigen::Vector3d &v) const;    // min_o t_c(v - v_o), eq. (14)

    // terms of J~ (36)(37) for one candidate
    LocalSolution evaluate(const LocalProblem &P, const Eigen::Vector3d &v) const;

  private:
    LocalSolution fallback(const LocalProblem &P, const std::vector<Eigen::Vector3d> &cand) const; // eq. (40)
    Eigen::Vector3d goalDir(const LocalProblem &P) const;

    ModelConstants c_;
    Switches sw_;
  };

} // namespace jps_vo

#endif
