# Base step 3 (generated from the verified patch of the previous session). Executed with only `rep` and the
# discovered role map `R` in scope; no file name appears here, anchors are matched exactly once.
H = R['core_h']
rep(H, '''#include <unordered_map>
''', '''#include <unordered_map>
#include <unordered_set>
''')
rep(H, '''    double J = 0.0;
    Eigen::Vector3d t_g = Eigen::Vector3d::Zero();
  };
''', '''    double J = 0.0;
    Eigen::Vector3d t_g = Eigen::Vector3d::Zero();
    // [fixes] diagnostics after thesis-frozen-v1 (filled only when a Fixes flag is on; cycles_ext.csv)
    double theta_eff = 0.0, ang_excess = 0.0, reach_turn = 0.0, ang_vprev_tg = 0.0;
    double J_A = 0.0, J_B = 0.0, Gamma = 0.0, cone_term = 0.0, rho_star = 0.0;
    double kappa_on_ek = 0.0, kappa_bound = 0.0;
    int zero_is_opt = 0, ray_feasible = 0, free_mode = 0, e_k = -1, kappa_map_size = 0, budget_active = 0;
  };

  // [fixes] one global search as seen by the R3 trigger and the R4 switch rule (switches_ext.csv)
  struct SwitchDiag
  {
    double t = 0.0;
    std::string reason;
    int is_switch = 0, accepted = 1, forced = 0;
    double C_old = 0.0, C_new = 0.0, dwell_left = 0.0;
    double up = 0.0, down = 0.0; // R3 sums of the last trigger evaluation
    int n_on = 0, n_off = 0;
  };
''')
rep(H, '''    std::vector<PathEvent> takePathEvents();
''', '''    std::vector<PathEvent> takePathEvents();
    std::vector<SwitchDiag> takeSwitchDiags(); // [fixes] filled only when a Fixes flag is on
''')
rep(H, '''    bool dirDependentWeights() const;
    LocalProblem lastLocalProblem() const;
''', '''    bool dirDependentWeights() const;
    LocalProblem lastLocalProblem() const;
    // [fix R3] trigger restricted to the remaining path (increase) and the off-path cached edges (decrease):
    // "none", "risk_Dk" or "kappa"
    std::string changedEdgesRelevant(const OccupancyGrid &grid, const GlobalPath &path, const Eigen::Vector3d &p_ref,
                                     const Eigen::Vector3d &v_ref, const std::vector<ObstacleState> &obs) const;
''')
rep(H, '''    std::unordered_map<unsigned long long, CachedEdge> cache_; // edges relaxed by the last search (D_k domain)
  };
''', '''    std::unordered_map<unsigned long long, CachedEdge> cache_; // edges relaxed by the last search (D_k domain)
    // ---- [fixes] after thesis-frozen-v1 ----
    double kappa_bound_ = 0.0;                         // R2a: kappa <= kappa_bound_ (guarded by mtx_)
    Eigen::Vector3d h_last_ = Eigen::Vector3d::Zero(); // R1c: last heading with |v_ref| >= v_min (mtx_)
    bool free_mode_ = true;                            // R5 (mtx_)
    double t_threat_ = -1e18;                          // R5: last time with lambda > 0 (mtx_)
    double last_switch_t_ = -1e18;                     // R4: time of the last accepted geometric switch (mtx_)
    std::vector<SwitchDiag> switch_diags_;             // (mtx_)
    struct TriggerDiag
    {
      double up = 0.0, down = 0.0;
      int n_on = 0, n_off = 0;
    };
    mutable TriggerDiag last_trigger_; // R3, global step only
  };
''')
C = R['core_cpp']
rep(C, '''    kappa_.clear();
    ++goal_epoch_;
  }
''', '''    kappa_.clear();
    ++goal_epoch_;
    // [fixes] per-goal state
    kappa_bound_ = 0.0;
    h_last_ = Eigen::Vector3d::Zero();
    free_mode_ = true;
    t_threat_ = -1e18;
    last_switch_t_ = -1e18;
  }
''')
rep(C, '''  std::vector<PathEvent> PlannerCore::takePathEvents()
''', '''  std::vector<SwitchDiag> PlannerCore::takeSwitchDiags()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<SwitchDiag> out;
    out.swap(switch_diags_);
    return out;
  }

  std::vector<PathEvent> PlannerCore::takePathEvents()
''')
# replanGlobal: snapshot of last_switch_t_
rep(C, '''    GlobalPath old;
    unsigned long epoch;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      goal = goal_;
      L_s = L_s_;
      kappa = kappa_;
      old = path_;
      epoch = goal_epoch_;
    }
''', '''    GlobalPath old;
    unsigned long epoch;
    double last_switch_t;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      goal = goal_;
      L_s = L_s_;
      kappa = kappa_;
      old = path_;
      epoch = goal_epoch_;
      last_switch_t = last_switch_t_;
    }
''')
rep(C, '''      e.w = it != cache.end() ? it->second.w : weight(grid, e.from_cell, e.to_cell, e.dir_in, p_ref, v_ref, obs, kappa, L_s);
      np.edges.push_back(e);
''', '''      e.w = it != cache.end() ? it->second.w : weight(grid, e.from_cell, e.to_cell, e.dir_in, p_ref, v_ref, obs, kappa, L_s);
      if (it != cache.end()) // [fix R2b] kappa part of w, kept with the edge
        e.kappa = it->second.kappa;
      else
      {
        auto kit = kappa.find(kappaKey(grid, e.from_cell, e.to_cell));
        e.kappa = kit != kappa.end() ? kit->second : 0.0;
      }
      np.edges.push_back(e);
''')
rep(C, '''    bool is_switch = old.empty();
    if (!old.empty())
    {
''', '''    bool is_switch = old.empty();
    bool keep_old = false; // [fix R4]
    SwitchDiag sd;         // [fixes]
    if (!old.empty())
    {
''')
rep(C, '''      rest.finalize();
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
''', '''      rest.finalize();
      is_switch = GlobalPath::hausdorff(rest, np) > cfg_.switch_threshold;
      if (cfg_.sw.fx.r4_switch_hysteresis && is_switch)
      {
        // [fix R4] a voluntary geometric switch needs C(new) <= C(rest of old) - eps_w under the current weights,
        // and T_g since the last accepted switch; goal, map and a blocked old route force the switch
        sd.forced = reason == "goal" || reason == "map" || remainingPathBlocked(grid, old, p_ref);
        for (const auto &e : np.edges)
          sd.C_new += e.w;
        for (const auto &e : rest.edges)
        {
          const double full = (grid.indexToPos(e.to_cell) - grid.indexToPos(e.from_cell)).norm();
          const double w_now = weight(grid, e.from_cell, e.to_cell, e.dir_in, p_ref, v_ref, obs, kappa, L_s);
          sd.C_old += full > 1e-9 ? w_now * std::min(1.0, e.ell / full) : 0.0; // partial first edge
        }
        sd.dwell_left = std::max(0.0, cfg_.c.T_g - (t - last_switch_t));
        keep_old = !sd.forced && !(sd.C_new <= sd.C_old - cfg_.c.eps_w && sd.dwell_left <= 0.0);
      }
    }
    if (cfg_.sw.fx.any())
    {
      sd.t = t;
      sd.reason = reason;
      sd.is_switch = is_switch ? 1 : 0;
      sd.accepted = keep_old ? 0 : 1;
      sd.up = last_trigger_.up;
      sd.down = last_trigger_.down;
      sd.n_on = last_trigger_.n_on;
      sd.n_off = last_trigger_.n_off;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    if (epoch != goal_epoch_)
      return false; // goal changed while searching
    if (keep_old)
    {
      // [fix R4] keep the current route; refresh its weights so that the R3 trigger baseline follows the state
      for (auto &e : path_.edges)
      {
        e.w = weight(grid, e.from_cell, e.to_cell, e.dir_in, p_ref, v_ref, obs, kappa, L_s);
        auto kit = kappa.find(kappaKey(grid, e.from_cell, e.to_cell));
        e.kappa = kit != kappa.end() ? kit->second : 0.0;
      }
      cache_.swap(cache);
      events_.push_back(PathEvent{t, path_id_, path_.hash(), reason, false, path_.points()});
      switch_diags_.push_back(sd);
      return true;
    }
    if (is_switch)
      ++path_id_;
    path_ = np;
    if (pi0_.empty())
      pi0_ = np;
    cache_.swap(cache);
    events_.push_back(PathEvent{t, path_id_, np.hash(), reason, is_switch, np.points()});
    if (cfg_.sw.fx.r4_switch_hysteresis && is_switch)
      last_switch_t_ = t; // [fix R4]
    if (cfg_.sw.fx.any())
      switch_diags_.push_back(sd);
    return true;
  }
''')
rep(C, '''  bool PlannerCore::remainingPathBlocked(''', '''  std::string PlannerCore::changedEdgesRelevant(const OccupancyGrid &grid, const GlobalPath &path,
                                               const Eigen::Vector3d &p_ref, const Eigen::Vector3d &v_ref,
                                               const std::vector<ObstacleState> &obs) const
  {
    // [fix R3] A weight increase off the current path can never make another route better, and a decrease on the
    // path keeps it optimal. Only (i) a net increase of the remaining path cost or (ii) the total decrease over the
    // off-path cached edges can change the decision; each is compared with eps_w.
    KappaMap kappa;
    double L_s;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      kappa = kappa_;
      L_s = L_s_;
    }
    last_trigger_ = TriggerDiag();
    if (path.empty())
      return "none";
    auto kNow = [&](const Eigen::Vector3i &from, const Eigen::Vector3i &to) {
      auto kit = kappa.find(kappaKey(grid, from, to));
      return kit != kappa.end() ? kit->second : 0.0;
    };
    const auto pr = path.project(p_ref);
    std::unordered_set<unsigned long long> on;
    double up = 0.0, up_nk = 0.0;
    for (size_t i = pr.edge; i < path.edges.size(); ++i)
    {
      const PathEdge &e = path.edges[i];
      on.insert(e.key);
      const double w = weight(grid, e.from_cell, e.to_cell, e.dir_in, p_ref, v_ref, obs, kappa, L_s);
      up += w - e.w;
      up_nk += (w - kNow(e.from_cell, e.to_cell)) - (e.w - e.kappa);
      ++last_trigger_.n_on;
    }
    double down = 0.0, down_nk = 0.0;
    for (const auto &kv : cache_)
    {
      if (on.count(kv.first))
        continue;
      const CachedEdge &e = kv.second;
      const double w = weight(grid, e.from, e.to, e.dir_in, p_ref, v_ref, obs, kappa, L_s);
      down += std::max(0.0, e.w - w);
      down_nk += std::max(0.0, (e.w - e.kappa) - (w - kNow(e.from, e.to)));
      ++last_trigger_.n_off;
    }
    last_trigger_.up = up;
    last_trigger_.down = down;
    const double eps = cfg_.c.eps_w;
    if (!(up > eps || down > eps))
      return "none";
    return (up_nk > eps || down_nk > eps) ? "risk_Dk" : "kappa";
  }

  bool PlannerCore::remainingPathBlocked(''')
rep(C, '''      const std::string dk = changedEdges(grid, p_ref, v_ref, obs); // eq. (26)
''', '''      const std::string dk = cfg_.sw.fx.r3_relevant_trigger ? changedEdgesRelevant(grid, path, p_ref, v_ref, obs)
                                                            : changedEdges(grid, p_ref, v_ref, obs); // eq. (26), [fix R3]
''')
# localStep: R1c / R5 state, before P_loc
rep(C, '''    // ---- 4: P_loc ----
    LocalProblem P;
''', '''    // [fixes R1c, R5] local-thread state (under the lock because setGoal resets it)
    Eigen::Vector3d h_prev = Eigen::Vector3d::Zero();
    bool free_mode = false;
    if (sw.fx.any())
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (rec.lambda > 0.0)
      {
        free_mode_ = false;
        t_threat_ = t;
      }
      else if (t - t_threat_ >= c.T_c)
        free_mode_ = true;
      free_mode = free_mode_;
      h_prev = h_last_; // last heading with |v_ref| >= v_min before this cycle
      if (v_ref.norm() >= c.v_min)
        h_last_ = v_ref.normalized();
    }

    // ---- 4: P_loc ----
    LocalProblem P;
''')
rep(C, '''    P.obs = obs;
    const LocalSolution sol = LocalPlanner(c, sw).solve(P); // (37)(38), (40)
''', '''    P.obs = obs;
    if (sw.fx.r1c_heading_memory)
      P.h_prev = h_prev; // [fix R1c]
    if (sw.fx.r5_mode_gating)
      P.free_mode = free_mode; // [fix R5]
    const LocalSolution sol = LocalPlanner(c, sw).solve(P); // (37)(38), (40)
''')
rep(C, '''      last_problem_ = P;
      kappa_.clear();
      if (sw.enable_c2 && e_k >= 0)
      {
        const double kap = sol.fallback ? 1.0 : rec.lambda * sol.s_g / M_PI;
        rec.kappa = kap;
        if (kap > 0.0)
          kappa_[kappaKey(grid, path.edges[e_k].from_cell, path.edges[e_k].to_cell)] = kap;
      }
    }
''', '''      last_problem_ = P;
      double gam = 1.0;
      if (sw.fx.r2a_kappa_decay)
      {
        // [fix R2a] peak-hold with exponential release instead of a per-cycle clear; T_kappa = T_g
        gam = std::exp(-c.dt / c.T_g);
        for (auto it = kappa_.begin(); it != kappa_.end();)
        {
          it->second *= gam;
          if (it->second < 1e-3)
            it = kappa_.erase(it);
          else
            ++it;
        }
        kappa_bound_ = std::max(sol.fallback ? 1.0 : rec.lambda, gam * kappa_bound_);
      }
      else
        kappa_.clear();
      if (sw.enable_c2 && e_k >= 0)
      {
        const double kap = sol.fallback ? 1.0 : rec.lambda * sol.s_g / M_PI;
        rec.kappa = kap;
        const unsigned long long key = kappaKey(grid, path.edges[e_k].from_cell, path.edges[e_k].to_cell);
        if (sw.fx.r2a_kappa_decay)
        {
          if (kap > 0.0)
          {
            double &slot = kappa_[key];
            slot = std::max(slot, kap);
          }
          auto it = kappa_.find(key);
          rec.kappa = it != kappa_.end() ? it->second : 0.0;
        }
        else if (kap > 0.0)
          kappa_[key] = kap;
      }
      if (sw.fx.any())
      {
        rec.kappa_on_ek = rec.kappa;
        rec.kappa_bound = kappa_bound_;
        rec.kappa_map_size = static_cast<int>(kappa_.size());
      }
    }
    if (sw.fx.any())
    {
      // [fixes] diagnostics (cycles_ext.csv)
      rec.theta_eff = sol.theta_eff;
      rec.ang_excess = sol.ang_excess;
      rec.reach_turn = reachTurn(v_ref.norm(), c);
      rec.ang_vprev_tg = angleBetween(v_ref, t_g);
      rec.J_A = sol.J_A;
      rec.J_B = sol.J_B;
      rec.Gamma = sol.Gamma;
      rec.cone_term = c.rho_g * sol.s_g;
      rec.rho_star = sol.rho;
      rec.zero_is_opt = (!sol.fallback && sol.v.norm() < 1e-9) ? 1 : 0;
      rec.free_mode = P.free_mode ? 1 : 0;
      rec.e_k = e_k;
      rec.budget_active = sol.budget_active ? 1 : 0;
      if (t_g.norm() > 1e-9 && v_ref.norm() > 1e-12)
      {
        // R1a's reachable rotation of v_ref towards t_g and whether it satisfies (38)
        const double phi = std::min(rec.reach_turn, angleBetween(v_ref, t_g));
        const Eigen::Vector3d axis = v_ref.cross(t_g);
        const Eigen::Vector3d v_rot =
            axis.norm() > 1e-12 ? Eigen::Vector3d(Eigen::AngleAxisd(phi, axis.normalized()) * v_ref) : v_ref;
        const LocalPlanner lp(c, sw);
        rec.ray_feasible = (lp.admissible(P, v_rot) && lp.turnOk(P, v_rot) && lp.staticOk(P, v_rot) &&
                            (P.lambda <= 0.0 || lp.dynamicOk(P, v_rot))) ? 1 : 0;
      }
    }
''')
