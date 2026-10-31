# Base step 5 (generated from the verified patch of the previous session). Executed with only `rep` and the
# discovered role map `R` in scope; no file name appears here, anchors are matched exactly once.
rep(R['switches_h'], '''    bool r1c_heading_memory = false;       // R1c Gamma turn term measured against the last heading when |v_prev| < v_min
''', '''    bool r1c_heading_memory = false;       // R1c Gamma turn term measured against the last heading when |v_prev| < v_min
    bool r1d_feasible_cone = false;        // R1d cone never narrower than the closest feasible moving candidate
''')
rep(R['coupling_cpp'], '''    return r1a_reachable_cone || r1b_continuous_slack || r1c_heading_memory || r2a_kappa_decay ||''',
    '''    return r1a_reachable_cone || r1b_continuous_slack || r1c_heading_memory || r1d_feasible_cone || r2a_kappa_decay ||''')
rep(R['local_planner_h'], '''    bool free_mode = false;                           // [fix R5] threat-free mode (no lambda > 0 for >= T_c)
  };
''', '''    bool free_mode = false;                           // [fix R5] threat-free mode (no lambda > 0 for >= T_c)
    double theta_floor = 0.0;                         // [fix R1d] smallest angle to t_g of a feasible moving candidate
    bool floor_set = false;                           // [fix R1d] theta_floor computed for this problem
  };
''')
rep(R['local_planner_h'], '''    double coneTheta(const LocalProblem &P, double lambda) const;  // [fix R1a/R5] cone half-angle
''', '''    double coneTheta(const LocalProblem &P, double lambda) const;  // [fix R1a/R1d/R5] cone half-angle
    double feasibleConeFloor(const LocalProblem &P) const;         // [fix R1d]
''')
C = R['local_planner_cpp']
rep(C, '''      if (!(sw_.fx.r1a_reachable_cone || sw_.fx.r1b_continuous_slack || sw_.fx.r5_mode_gating))''',
       '''      if (!(sw_.fx.r1a_reachable_cone || sw_.fx.r1b_continuous_slack || sw_.fx.r1d_feasible_cone || sw_.fx.r5_mode_gating))''')
rep(C, '''      th = std::max(th, angleBetween(P.v_prev, P.t_g) - reachTurn(P.v_prev.norm(), c_));
    return std::min(th, M_PI);
  }
''', '''      th = std::max(th, angleBetween(P.v_prev, P.t_g) - reachTurn(P.v_prev.norm(), c_));
    if (sw_.fx.r1d_feasible_cone && P.floor_set) // [fix R1d] the cone never excludes every feasible moving velocity
      th = std::max(th, P.theta_floor);
    return std::min(th, M_PI);
  }

  double LocalPlanner::feasibleConeFloor(const LocalProblem &P) const
  {
    // smallest angle to t_g over the moving candidates that satisfy (38); 0 when none moves (the cone is then moot)
    const bool dyn_active = P.lambda > 0.0;
    double amin = M_PI;
    bool any = false;
    for (const auto &v : candidates(P))
    {
      if (v.norm() <= 1e-9 || !turnOk(P, v) || !staticOk(P, v) || (dyn_active && !dynamicOk(P, v)))
        continue;
      amin = std::min(amin, angleBetween(v, P.t_g));
      any = true;
    }
    return any ? amin : 0.0;
  }
''')
rep(C, '''  LocalSolution LocalPlanner::solve(const LocalProblem &P) const
  {
    if (sw_.fx.s1_eps_constraint && P.lambda > 0.0)
      return solveBudget(P); // [fix S1]
''', '''  LocalSolution LocalPlanner::solve(const LocalProblem &P) const
  {
    if (sw_.fx.r1d_feasible_cone && !P.floor_set && sw_.enable_c3 && P.t_g.norm() > 1e-9)
    {
      LocalProblem Pf = P; // [fix R1d] widen the cone to the closest feasible moving candidate, then solve as usual
      Pf.theta_floor = feasibleConeFloor(P);
      Pf.floor_set = true;
      return solve(Pf);
    }
    if (sw_.fx.s1_eps_constraint && P.lambda > 0.0)
      return solveBudget(P); // [fix S1]
''')
rep(R['node_cpp'], '''    sw.fx.r1c_heading_memory = param(nh, "fixes/r1c_heading_memory", false);
''', '''    sw.fx.r1c_heading_memory = param(nh, "fixes/r1c_heading_memory", false);
    sw.fx.r1d_feasible_cone = param(nh, "fixes/r1d_feasible_cone", false);
''')
rep(R['node_cpp'], '''      ROS_WARN("[jps_vo] fixes active, this is NOT the thesis-frozen-v1 behaviour: r1a %d r1b %d r1c %d r2a %d r2b %d r3 %d "
               "r4 %d r5 %d s1 %d diag %d",
               sw.fx.r1a_reachable_cone, sw.fx.r1b_continuous_slack, sw.fx.r1c_heading_memory, sw.fx.r2a_kappa_decay,''',
'''      ROS_WARN("[jps_vo] fixes active, this is NOT the thesis-frozen-v1 behaviour: r1a %d r1b %d r1c %d r1d %d r2a %d r2b %d "
               "r3 %d r4 %d r5 %d s1 %d diag %d",
               sw.fx.r1a_reachable_cone, sw.fx.r1b_continuous_slack, sw.fx.r1c_heading_memory, sw.fx.r1d_feasible_cone,
               sw.fx.r2a_kappa_decay,''')
rep(R['node_cpp'], '''        << ", r1c_heading_memory: " << sw.fx.r1c_heading_memory << ", r2a_kappa_decay: " << sw.fx.r2a_kappa_decay''',
'''        << ", r1c_heading_memory: " << sw.fx.r1c_heading_memory << ", r1d_feasible_cone: " << sw.fx.r1d_feasible_cone
        << ", r2a_kappa_decay: " << sw.fx.r2a_kappa_decay''')
# R4 diag: mark the unevaluated cases (goal / map / no old path) as forced so the switch log is unambiguous
rep(R['core_cpp'], '''    if (cfg_.sw.fx.any())
    {
      sd.t = t;
      sd.reason = reason;''', '''    if (cfg_.sw.fx.any())
    {
      if (old.empty() || reason == "goal" || reason == "map")
        sd.forced = 1; // switches the R4 rule never holds back
      sd.t = t;
      sd.reason = reason;''')
