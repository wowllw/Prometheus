#ifndef JPS_VO_PLANNER_PLANNER_CORE_H
#define JPS_VO_PLANNER_PLANNER_CORE_H

// One planning cycle in the order of Stage-2 section 4.1:
//   1. xi / rho of the reference motion and of the edges        (19)(20)(29)
//   2. lambda_e, Gamma_e -> w_e; D_k -> global (re)search on G_J  (31)(35)(39)(26), Proposition 1
//   3. guidance tangent t_g                                       (27)
//   4. P_loc -> v*, s_g*                                          (36)-(38), (40)
//   5. kappa_{e_k} = lambda s_g* / pi                             (25)
// Steps 1-2 are globalStep(), steps 3-5 localStep(). cycle() runs both in sequence (offline tests, deterministic);
// the node runs globalStep() in its own thread [ENG-11] so a long G_J search never blocks the Delta_t local cycle.
// The two steps share path / kappa / goal under a mutex. ROS-free.

#include <Eigen/Eigen>
#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "jps_vo_planner/coupling.h"
#include "jps_vo_planner/global_path.h"
#include "jps_vo_planner/jps3d.h"
#include "jps_vo_planner/local_planner.h"
#include "jps_vo_planner/occupancy_grid.h"

namespace jps_vo
{
  struct CoreConfig
  {
    ModelConstants c;
    Switches sw;
    Jps3D::TieRule tie_rule = Jps3D::TieRule::Strict;
    std::string label = "g6";      // group label written to the log ("mode" column)
    double replan_period = 1.0;    // periodic global replan [s], same as ego thresh_replan_time
    double goal_tol = 0.3;         // goal reached: ||p_ref - goal|| <= goal_tol ...
    double goal_v_tol = 0.1;       // ... and ||v_ref|| <= goal_v_tol (run notes S-1)
    int snap_radius = 10;          // start/goal snapping to the nearest free cell [cells] (checklist G-7)
    double switch_threshold = 0.3; // Hausdorff distance that counts as a global path switch [m] (spec M6)
    double search_margin = 5.0;    // [ENG-8] G_J limited to the start/goal bounding box + margin (full z); 0 = map
  };

  // one row of cycles.csv (spec section 8: Subtask-3 fields + 5 extension fields) and diagnostics
  struct CycleRecord
  {
    double t = 0.0;
    std::string mode;
    double plan_ms_global = 0.0, plan_ms_local = 0.0;
    Eigen::Vector3d p_ref = Eigen::Vector3d::Zero(), v_ref = Eigen::Vector3d::Zero(), p_odom = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_star = Eigen::Vector3d::Zero();
    double lambda = 0.0, xi = kInf, s_g_star = 0.0, kappa = 0.0;
    int path_id = -1;
    unsigned long long path_hash = 0;
    int n_jump_pts = 0;
    double min_surface_dist_ref = kInf, min_surface_dist_odom = kInf;
    double turn_angle = 0.0; // [deg], angle(v*, v_prev)
    double lateral_dev_ref_to_Pi = 0.0;
    // extension fields
    double lateral_dev_ref_to_Pi0 = 0.0;
    std::string replan_reason = "none"; // none | goal | map | periodic | risk_Dk | kappa | fail
    bool fallback_active = false;
    std::string nearest_dyn_obs = "-";
    int encounter_flag = 0;
    // diagnostics (not CSV contract)
    bool reached = false;
    bool path_switched = false;
    int n_feasible = 0, n_candidates = 0;
    double J = 0.0;
    Eigen::Vector3d t_g = Eigen::Vector3d::Zero();
  };

  struct GlobalStepResult
  {
    std::string reason = "none"; // why a search ran this step ("none" if it did not)
    double ms = 0.0;             // wall time of the step (D_k check + search)
    bool switched = false;
  };

  struct PathEvent
  {
    double t;
    int path_id;
    unsigned long long hash;
    std::string reason;
    bool is_switch;
    std::vector<Eigen::Vector3d> pts;
  };

  class PlannerCore
  {
  public:
    void configure(const CoreConfig &cfg);
    const CoreConfig &config() const { return cfg_; }

    void setGoal(const Eigen::Vector3d &goal, const Eigen::Vector3d &p_ref);
    bool hasGoal() const;
    void clearGoal();
    Eigen::Vector3d goal() const;
    double Ls() const;
    void notifyMapUpdated() { map_dirty_ = true; }

    // steps 1-2 (thread-safe against localStep)
    GlobalStepResult globalStep(double t, const OccupancyGrid &grid, const Eigen::Vector3d &p_ref,
                                const Eigen::Vector3d &v_ref, const std::vector<ObstacleState> &obs);
    // steps 3-5; `g` is reported in the record (plan_ms_global, replan_reason)
    CycleRecord localStep(double t, const OccupancyGrid &grid, const Eigen::Vector3d &p_ref, const Eigen::Vector3d &v_ref,
                          const Eigen::Vector3d &p_odom, const std::vector<ObstacleState> &obs, const GlobalStepResult &g);
    // synchronous cycle = globalStep + localStep
    CycleRecord cycle(double t, const OccupancyGrid &grid, const Eigen::Vector3d &p_ref, const Eigen::Vector3d &v_ref,
                      const Eigen::Vector3d &p_odom, const std::vector<ObstacleState> &obs);

    GlobalPath path() const;
    GlobalPath pi0() const;
    std::vector<PathEvent> takePathEvents();

    // ---- pieces exposed for the offline tests ----
    // edge weight w_e of eq. (39) for the jump from -> to entered with dir_in, using the current kappa
    double edgeWeight(const OccupancyGrid &grid, const Eigen::Vector3i &from, const Eigen::Vector3i &to, int dir_in,
                      const Eigen::Vector3d &p_ref, const Eigen::Vector3d &v_ref, const std::vector<ObstacleState> &obs) const;
    bool replanGlobal(double t, const OccupancyGrid &grid, const Eigen::Vector3d &p_ref, const Eigen::Vector3d &v_ref,
                      const std::vector<ObstacleState> &obs, const std::string &reason);
    // D_k of eq. (26) on the edges relaxed by the last search: "none", "risk_Dk" or "kappa"
    std::string changedEdges(const OccupancyGrid &grid, const Eigen::Vector3d &p_ref, const Eigen::Vector3d &v_ref,
                             const std::vector<ObstacleState> &obs) const;
    bool dirDependentWeights() const;
    LocalProblem lastLocalProblem() const;

  private:
    struct CachedEdge
    {
      Eigen::Vector3i from, to;
      int dir_in;
      double w;
      double kappa; // kappa part of w when cached
    };
    using KappaMap = std::unordered_map<unsigned long long, double>;

    double weight(const OccupancyGrid &grid, const Eigen::Vector3i &from, const Eigen::Vector3i &to, int dir_in,
                  const Eigen::Vector3d &p_ref, const Eigen::Vector3d &v_ref, const std::vector<ObstacleState> &obs,
                  const KappaMap &kappa, double L_s) const;
    static unsigned long long edgeKey(const OccupancyGrid &grid, const Eigen::Vector3i &from, const Eigen::Vector3i &to,
                                      int dir_in);
    static unsigned long long kappaKey(const OccupancyGrid &grid, const Eigen::Vector3i &from, const Eigen::Vector3i &to);
    bool remainingPathBlocked(const OccupancyGrid &grid, const GlobalPath &path, const Eigen::Vector3d &p_ref) const;

    CoreConfig cfg_;
    mutable std::mutex mtx_;                 // guards everything below except the global-thread-only members
    bool has_goal_ = false, need_goal_plan_ = false, encounter_ = false;
    Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
    double L_s_ = 2.0;
    GlobalPath path_, pi0_;
    int path_id_ = -1;
    KappaMap kappa_;                         // kappa on the tracked edge e_k only (25)
    std::vector<PathEvent> events_;
    LocalProblem last_problem_;
    unsigned long goal_epoch_ = 0;           // bumped by setGoal; a search started before it is discarded
    std::atomic<bool> map_dirty_{false};
    // global-step-only state
    double last_replan_t_ = -1e18;
    std::unordered_map<unsigned long long, CachedEdge> cache_; // edges relaxed by the last search (D_k domain)
  };

} // namespace jps_vo

#endif
