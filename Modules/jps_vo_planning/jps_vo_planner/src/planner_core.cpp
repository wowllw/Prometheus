#include "jps_vo_planner/planner_core.h"

#include <chrono>

namespace jps_vo
{
  namespace
  {
    double msSince(const std::chrono::steady_clock::time_point &t0)
    {
      return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
  } // namespace

  void PlannerCore::configure(const CoreConfig &cfg)
  {
    cfg_ = cfg;
    cfg_.c.deriveTaus();
  }

  void PlannerCore::setGoal(const Eigen::Vector3d &goal, const Eigen::Vector3d &p_ref)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    goal_ = goal;
    L_s_ = std::max((goal - p_ref).norm(), cfg_.c.L_min); // eq. (21)
    has_goal_ = true;
    need_goal_plan_ = true;
    encounter_ = false;
    path_ = GlobalPath();
    pi0_ = GlobalPath();
    kappa_.clear();
    ++goal_epoch_;
  }

  bool PlannerCore::hasGoal() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return has_goal_;
  }

  void PlannerCore::clearGoal()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    has_goal_ = false;
    ++goal_epoch_;
  }

  Eigen::Vector3d PlannerCore::goal() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return goal_;
  }

  double PlannerCore::Ls() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return L_s_;
  }

  GlobalPath PlannerCore::path() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return path_;
  }

  GlobalPath PlannerCore::pi0() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return pi0_;
  }

  LocalProblem PlannerCore::lastLocalProblem() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return last_problem_;
  }

  std::vector<PathEvent> PlannerCore::takePathEvents()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<PathEvent> out;
    out.swap(events_);
    return out;
  }

  unsigned long long PlannerCore::edgeKey(const OccupancyGrid &grid, const Eigen::Vector3i &from, const Eigen::Vector3i &to,
                                          int dir_in)
  {
    const Eigen::Vector3i d = grid.dims();
    auto ck = [&](const Eigen::Vector3i &n) {
      return (static_cast<unsigned long long>(n.z()) * d.y() + n.y()) * d.x() + n.x();
    };
    const unsigned long long n = static_cast<unsigned long long>(d.x()) * d.y() * d.z();
    return (ck(from) * n + ck(to)) * 27ull + static_cast<unsigned long long>(dir_in);
  }

  unsigned long long PlannerCore::kappaKey(const OccupancyGrid &grid, const Eigen::Vector3i &from, const Eigen::Vector3i &to)
  {
    return edgeKey(grid, from, to, 0); // kappa belongs to the edge e_k, whatever direction it was entered from
  }

  bool PlannerCore::dirDependentWeights() const
  {
    // Gamma_e (35) depends on the incoming direction; it can be non-zero only with C2 + Gamma and lambda_e in (0,1)
    const auto m = cfg_.sw.lambda_mode;
    return cfg_.sw.enable_c2 && cfg_.sw.enable_gamma &&
           (m == Switches::LambdaMode::Adaptive || m == Switches::LambdaMode::Const);
  }

  double PlannerCore::weight(const OccupancyGrid &grid, const Eigen::Vector3i &from, const Eigen::Vector3i &to, int dir_in,
                             const Eigen::Vector3d &p_ref, const Eigen::Vector3d &v_ref,
                             const std::vector<ObstacleState> &obs, const KappaMap &kappa, double L_s) const
  {
    const ModelConstants &c = cfg_.c;
    const Eigen::Vector3d a = grid.indexToPos(from), b = grid.indexToPos(to);
    const double ell = (b - a).norm();
    const double base = ell / L_s; // J_A(sigma_e) = ell / L_s (21)
    if (!cfg_.sw.enable_c2)
      return base; // A4 / A10: w_e = ell_e / L_s, kappa = 0

    Segment sg;
    sg.q = a;
    sg.d = (b - a) / ell;
    sg.ell = ell;
    sg.t_rel = (a - p_ref).norm() / c.v_c; // t_hat(q) = ||q - p_A|| / v_c (section 2.1)
    double rho = 0.0, xi = kInf;
    segmentRiskLite(sg, obs, c, rho, xi);          // (19)(20)(29)
    const double lam_e = lambdaOf(xi, c, cfg_.sw); // (31)

    double gam = 0.0;
    if (cfg_.sw.enable_gamma)
    {
      Eigen::Vector3d t_prev = Eigen::Vector3d::Zero();
      if (dir_in != Jps3D::kNoDir)
        t_prev = Jps3D::dirVec(dir_in).cast<double>().normalized();
      else if (v_ref.norm() >= c.v_min)
        t_prev = v_ref.normalized(); // first edge: current heading
      gam = gammaE(t_prev, sg.d, lam_e, c); // (35)
    }
    auto it = kappa.find(kappaKey(grid, from, to));
    const double kap = it != kappa.end() ? it->second : 0.0;
    // (39): w_e = (1 - lambda_e) ell/L_s + lambda_e (ell/L_s + rho_e) + Gamma_e + kappa_e
    return (1.0 - lam_e) * base + lam_e * (base + rho) + gam + kap;
  }

  double PlannerCore::edgeWeight(const OccupancyGrid &grid, const Eigen::Vector3i &from, const Eigen::Vector3i &to,
                                 int dir_in, const Eigen::Vector3d &p_ref, const Eigen::Vector3d &v_ref,
                                 const std::vector<ObstacleState> &obs) const
  {
    KappaMap kappa;
    double L_s;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      kappa = kappa_;
      L_s = L_s_;
    }
    return weight(grid, from, to, dir_in, p_ref, v_ref, obs, kappa, L_s);
  }

  bool PlannerCore::replanGlobal(double t, const OccupancyGrid &grid, const Eigen::Vector3d &p_ref,
                                 const Eigen::Vector3d &v_ref, const std::vector<ObstacleState> &obs,
                                 const std::string &reason)
  {
    last_replan_t_ = t;
    // snapshot of the shared state; the search itself runs without the lock
    Eigen::Vector3d goal;
    double L_s;
    KappaMap kappa;
    GlobalPath old;
    unsigned long epoch;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      goal = goal_;
      L_s = L_s_;
      kappa = kappa_;
      old = path_;
      epoch = goal_epoch_;
    }
    Eigen::Vector3i s, g;
    if (!grid.nearestFree(p_ref, cfg_.snap_radius, s) || !grid.nearestFree(goal, cfg_.snap_radius, g))
      return false;

    std::unordered_map<unsigned long long, CachedEdge> cache;
    Jps3D jps(&grid, cfg_.tie_rule);
    if (cfg_.search_margin > 0.0)
    {
      const int m = static_cast<int>(std::ceil(cfg_.search_margin / grid.params().resolution));
      jps.setSearchBox(Eigen::Vector3i(std::min(s.x(), g.x()) - m, std::min(s.y(), g.y()) - m, 0),
                       Eigen::Vector3i(std::max(s.x(), g.x()) + m, std::max(s.y(), g.y()) + m, grid.dims().z() - 1));
    }
    auto w = [&](const Eigen::Vector3i &from, const Eigen::Vector3i &to, int din, int) {
      const double we = weight(grid, from, to, din, p_ref, v_ref, obs, kappa, L_s);
      auto kit = kappa.find(kappaKey(grid, from, to));
      cache[edgeKey(grid, from, to, din)] = CachedEdge{from, to, din, we, kit != kappa.end() ? kit->second : 0.0};
      return we;
    };
    const Jps3D::Result r = jps.search(s, g, w, 1.0 / L_s, dirDependentWeights()); // h = h_D / L_s (39)
    if (!r.ok || r.cells.size() < 2)
      return false;

    GlobalPath np;
    for (size_t i = 0; i + 1 < r.cells.size(); ++i)
    {
      PathEdge e;
      e.from_cell = r.cells[i];
      e.to_cell = r.cells[i + 1];
      e.a = grid.indexToPos(e.from_cell);
      e.b = grid.indexToPos(e.to_cell);
      e.ell = (e.b - e.a).norm();
      e.t = (e.b - e.a) / e.ell;
      e.dir_out = r.dirs[i];
      e.dir_in = i == 0 ? Jps3D::kNoDir : r.dirs[i - 1];
      e.key = edgeKey(grid, e.from_cell, e.to_cell, e.dir_in);
      auto it = cache.find(e.key);
      e.w = it != cache.end() ? it->second.w : weight(grid, e.from_cell, e.to_cell, e.dir_in, p_ref, v_ref, obs, kappa, L_s);
      np.edges.push_back(e);
    }
    np.finalize();

    // a switch is a geometric change of the remaining route (Hausdorff to the old path from the projection on)
    bool is_switch = old.empty();
    if (!old.empty())
    {
      GlobalPath rest;
      const auto pr = old.project(p_ref);
      for (size_t i = pr.edge; i < old.edges.size(); ++i)
      {
        PathEdge e = old.edges[i];
        if (static_cast<int>(i) == pr.edge)
        {
          e.a = pr.foot;
          e.ell = (e.b - e.a).norm();
          if (e.ell < 1e-9)
            continue;
        }
        rest.edges.push_back(e);
      }
      rest.finalize();
      is_switch = GlobalPath::hausdorff(rest, np) > cfg_.switch_threshold;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    if (epoch != goal_epoch_)
      return false; // goal changed while searching
    if (is_switch)
      ++path_id_;
    path_ = np;
    if (pi0_.empty())
      pi0_ = np;
    cache_.swap(cache);
    events_.push_back(PathEvent{t, path_id_, np.hash(), reason, is_switch, np.points()});
    return true;
  }

  std::string PlannerCore::changedEdges(const OccupancyGrid &grid, const Eigen::Vector3d &p_ref,
                                        const Eigen::Vector3d &v_ref, const std::vector<ObstacleState> &obs) const
  {
    KappaMap kappa;
    double L_s;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      kappa = kappa_;
      L_s = L_s_;
    }
    bool risk = false, kap = false;
    for (const auto &kv : cache_)
    {
      const CachedEdge &e = kv.second;
      const double w = weight(grid, e.from, e.to, e.dir_in, p_ref, v_ref, obs, kappa, L_s);
      if (std::fabs(w - e.w) > cfg_.c.eps_w) // eq. (26)
      {
        auto kit = kappa.find(kappaKey(grid, e.from, e.to));
        const double k_now = kit != kappa.end() ? kit->second : 0.0;
        // attribute the change to kappa when removing the kappa difference brings it back within eps_w
        if (std::fabs((w - k_now) - (e.w - e.kappa)) <= cfg_.c.eps_w)
          kap = true;
        else
          risk = true;
      }
    }
    return risk ? "risk_Dk" : (kap ? "kappa" : "none");
  }

  bool PlannerCore::remainingPathBlocked(const OccupancyGrid &grid, const GlobalPath &path, const Eigen::Vector3d &p_ref) const
  {
    if (path.empty())
      return false;
    const auto pr = path.project(p_ref);
    for (size_t i = pr.edge; i < path.edges.size(); ++i)
    {
      const Eigen::Vector3d a = static_cast<int>(i) == pr.edge ? pr.foot : path.edges[i].a;
      if (!grid.segmentFree(a, path.edges[i].b))
        return true;
    }
    return false;
  }

  GlobalStepResult PlannerCore::globalStep(double t, const OccupancyGrid &grid, const Eigen::Vector3d &p_ref,
                                           const Eigen::Vector3d &v_ref, const std::vector<ObstacleState> &obs)
  {
    GlobalStepResult res;
    const auto t0 = std::chrono::steady_clock::now();
    bool need_goal = false, has_goal = false;
    GlobalPath path;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      need_goal = need_goal_plan_;
      has_goal = has_goal_;
      path = path_;
    }
    if (!has_goal)
      return res;

    std::string reason = "none";
    if (need_goal)
      reason = "goal";
    else if (map_dirty_.exchange(false) && remainingPathBlocked(grid, path, p_ref))
      reason = "map";
    else if (cfg_.sw.enable_c2 && !path.empty())
    {
      const std::string dk = changedEdges(grid, p_ref, v_ref, obs); // eq. (26)
      if (dk != "none")
        reason = dk;
    }
    if (reason == "none" && t - last_replan_t_ >= cfg_.replan_period)
      reason = "periodic";
    if (reason != "none")
    {
      {
        std::lock_guard<std::mutex> lk(mtx_);
        need_goal_plan_ = false;
      }
      int old_id;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        old_id = path_id_;
      }
      if (!replanGlobal(t, grid, p_ref, v_ref, obs, reason))
        reason = "fail";
      std::lock_guard<std::mutex> lk(mtx_);
      res.switched = path_id_ != old_id;
    }
    res.reason = reason;
    res.ms = msSince(t0);
    return res;
  }

  CycleRecord PlannerCore::localStep(double t, const OccupancyGrid &grid, const Eigen::Vector3d &p_ref,
                                     const Eigen::Vector3d &v_ref, const Eigen::Vector3d &p_odom,
                                     const std::vector<ObstacleState> &obs, const GlobalStepResult &g)
  {
    const ModelConstants &c = cfg_.c;
    const Switches &sw = cfg_.sw;
    CycleRecord rec;
    rec.t = t;
    rec.mode = cfg_.label;
    rec.p_ref = p_ref;
    rec.v_ref = v_ref;
    rec.p_odom = p_odom;
    rec.replan_reason = g.reason;
    rec.plan_ms_global = g.reason == "none" ? 0.0 : g.ms;
    rec.path_switched = g.switched;

    const auto t_loc = std::chrono::steady_clock::now();
    GlobalPath path, pi0;
    Eigen::Vector3d goal;
    double L_s;
    int path_id;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      path = path_;
      pi0 = pi0_;
      goal = goal_;
      L_s = L_s_;
      path_id = path_id_;
    }

    // ---- 3: guidance tangent ----
    Eigen::Vector3d t_g = Eigen::Vector3d::Zero();
    int e_k = -1;
    if (!path.empty())
    {
      const auto pr = path.project(p_ref);
      e_k = pr.edge;
      t_g = path.guideTangent(pr.s, c.ellW()); // (27)
      rec.lateral_dev_ref_to_Pi = pr.dist;
    }
    if (!pi0.empty())
      rec.lateral_dev_ref_to_Pi0 = pi0.project(p_ref).dist;

    // xi of the reference motion sigma_ref = (p_A, t_g(s_k), v_c tau2) (29)(iv); A12: executed direction
    const Eigen::Vector3d to_goal = (goal - p_ref).norm() > 1e-9 ? Eigen::Vector3d((goal - p_ref).normalized())
                                                                  : Eigen::Vector3d::Zero();
    Segment ref;
    ref.q = p_ref;
    ref.d = t_g.norm() > 1e-9 ? t_g : to_goal;
    if (sw.xi_source == Switches::XiSource::Executed && v_ref.norm() >= c.v_min)
      ref.d = v_ref.normalized();
    ref.ell = c.v_c * c.tau2;
    double rho_ref = 0.0;
    segmentRiskLite(ref, obs, c, rho_ref, rec.xi);
    rec.lambda = lambdaOf(rec.xi, c, sw); // (31)

    // ---- 4: P_loc ----
    LocalProblem P;
    P.p_ref = p_ref;
    P.v_prev = v_ref;
    P.goal = goal;
    P.t_g = t_g;
    P.lambda = rec.lambda;
    P.L_s = L_s;
    P.path = path.empty() ? nullptr : &path;
    P.grid = &grid;
    P.obs = obs;
    const LocalSolution sol = LocalPlanner(c, sw).solve(P); // (37)(38), (40)
    rec.v_star = sol.v;
    rec.s_g_star = sol.s_g;
    rec.fallback_active = sol.fallback;
    rec.n_feasible = sol.n_feasible;
    rec.n_candidates = sol.n_candidates;
    rec.J = sol.J;
    rec.t_g = t_g;

    // ---- 5: kappa_{e_k} = lambda s_g* / pi (25); fallback -> 1; only on the tracked edge ----
    {
      std::lock_guard<std::mutex> lk(mtx_);
      P.path = nullptr; // the copy dies with this scope
      last_problem_ = P;
      kappa_.clear();
      if (sw.enable_c2 && e_k >= 0)
      {
        const double kap = sol.fallback ? 1.0 : rec.lambda * sol.s_g / M_PI;
        rec.kappa = kap;
        if (kap > 0.0)
          kappa_[kappaKey(grid, path.edges[e_k].from_cell, path.edges[e_k].to_cell)] = kap;
      }
    }
    rec.plan_ms_local = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_loc).count();

    // ---- logging fields ----
    rec.path_id = path_id;
    rec.path_hash = path.hash();
    rec.n_jump_pts = path.empty() ? 0 : static_cast<int>(path.edges.size()) + 1;
    rec.turn_angle = (sol.v.norm() >= c.v_min && v_ref.norm() >= c.v_min) ? angleBetween(sol.v, v_ref) * 180.0 / M_PI : 0.0;
    bool enc = false;
    for (const auto &o : obs)
    {
      const double dr = (p_ref - o.p).norm() - c.r_q - o.r;
      const double dodom = (p_odom - o.p).norm() - c.r_q - o.r;
      if (dr < rec.min_surface_dist_ref)
      {
        rec.min_surface_dist_ref = dr;
        rec.nearest_dyn_obs = o.name;
      }
      rec.min_surface_dist_odom = std::min(rec.min_surface_dist_odom, dodom);
      // encounter: constant-velocity prediction of reference vs obstacle within tau2 comes closer than R_o
      if (o.v.norm() > 0.05)
      {
        const Eigen::Vector3d x = o.p - p_ref, w = v_ref - o.v;
        const double tc = w.squaredNorm() > 1e-12 ? std::min(c.tau2, std::max(0.0, x.dot(w) / w.squaredNorm())) : 0.0;
        if ((x - w * tc).norm() < c.safetyRadius(o.r))
          enc = true;
      }
    }
    {
      std::lock_guard<std::mutex> lk(mtx_);
      encounter_ = encounter_ || enc;
      rec.encounter_flag = encounter_ ? 1 : 0;
    }
    rec.reached = (goal - p_ref).norm() <= cfg_.goal_tol && v_ref.norm() <= cfg_.goal_v_tol;
    return rec;
  }

  CycleRecord PlannerCore::cycle(double t, const OccupancyGrid &grid, const Eigen::Vector3d &p_ref,
                                 const Eigen::Vector3d &v_ref, const Eigen::Vector3d &p_odom,
                                 const std::vector<ObstacleState> &obs)
  {
    const GlobalStepResult g = globalStep(t, grid, p_ref, v_ref, obs);
    return localStep(t, grid, p_ref, v_ref, p_odom, obs, g);
  }

} // namespace jps_vo
