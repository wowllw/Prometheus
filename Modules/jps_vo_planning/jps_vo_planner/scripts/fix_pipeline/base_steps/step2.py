# Base step 2 (generated from the verified patch of the previous session). Executed with only `rep` and the
# discovered role map `R` in scope; no file name appears here, anchors are matched exactly once.
H = R['local_planner_h']
rep(H, '''    std::vector<ObstacleState> obs;                   // states at the planning time
  };
''', '''    std::vector<ObstacleState> obs;                   // states at the planning time
    Eigen::Vector3d h_prev = Eigen::Vector3d::Zero(); // [fix R1c] last heading with |v_ref| >= v_min (zero: none)
    bool free_mode = false;                           // [fix R5] threat-free mode (no lambda > 0 for >= T_c)
  };
''')
rep(H, '''    bool fallback = false;
    int n_candidates = 0, n_feasible = 0;
  };
''', '''    bool fallback = false;
    int n_candidates = 0, n_feasible = 0;
    // [fixes] angle excess over the cone (equals s_g when R1a/R1b/R5 are off), cone half-angle used,
    // weighted-sum value for S1, and whether the S1 risk budget was active
    double ang_excess = 0.0, theta_eff = 0.0, J_ws = 0.0;
    bool budget_active = false;
  };
''')
rep(H, '''    LocalSolution fallback(const LocalProblem &P, const std::vector<Eigen::Vector3d> &cand) const; // eq. (40)
''', '''    LocalSolution fallback(const LocalProblem &P, const std::vector<Eigen::Vector3d> &cand) const; // eq. (40)
    double coneTheta(const LocalProblem &P, double lambda) const;  // [fix R1a/R5] cone half-angle
    LocalSolution solveBudget(const LocalProblem &P) const;        // [fix S1] epsilon-constraint variant of solve()
''')
C = R['local_planner_cpp']
rep(C, '''    // J_A = V_Pi(q(v)) (23)(24); no global path -> straight-line distance to the goal [ENG-9]
    if (lam < 1.0)
      s.J_A = (P.path && !P.path->empty()) ? P.path->value(q, P.L_s) : (P.goal - q).norm() / P.L_s;
''', '''    // J_A = V_Pi(q(v)) (23)(24); no global path -> straight-line distance to the goal [ENG-9]
    if (lam < 1.0)
    {
      if (P.path && !P.path->empty())
        s.J_A = sw_.fx.r2b_kappa_local_excluded ? P.path->valueNoKappa(q, P.L_s) : P.path->value(q, P.L_s); // [fix R2b]
      else
        s.J_A = (P.goal - q).norm() / P.L_s;
    }
''')
rep(C, '''        s.rho = segmentRisk(sg, P.obs, c_).rho;
        s.J_B += s.rho;
''', '''        s.rho = segmentRisk(sg, P.obs, c_).rho;
        if (!(sw_.fx.s1_eps_constraint && lam > 0.0)) // [fix S1] the risk becomes a budget, not a weighted term
          s.J_B += s.rho;
''')
rep(C, '''    s.Gamma = sw_.enable_gamma ? gammaV(v, P.v_prev, lam, c_) : 0.0; // (34)

    // (28): minimal slack, s_g = max(0, angle(v, t_g) - theta_c)
    if (sw_.enable_c3 && P.t_g.norm() > 1e-9 && v.norm() > 1e-12)
      s.s_g = std::max(0.0, angleBetween(v, P.t_g) - thetaC(lam, sw_));

    s.J = (1.0 - lam) * s.J_A + lam * s.J_B + s.Gamma + c_.rho_g * s.s_g; // (36)(37)
    if (sw_.c3_soft_tracking > 0.0)
      s.J += sw_.c3_soft_tracking * (v - c_.v_c * P.t_g).squaredNorm(); // A6 only
    return s;
  }
''', '''    if (sw_.enable_gamma) // (34); [fix R1c] turn term against the last heading through a low-speed phase
      s.Gamma = sw_.fx.r1c_heading_memory ? gammaV(v, P.v_prev, lam, c_, P.h_prev) : gammaV(v, P.v_prev, lam, c_);

    // (28): minimal slack, s_g = max(0, angle(v, t_g) - theta_c)
    if (sw_.enable_c3 && P.t_g.norm() > 1e-9 && v.norm() > 1e-12)
    {
      if (!(sw_.fx.r1a_reachable_cone || sw_.fx.r1b_continuous_slack || sw_.fx.r5_mode_gating))
      {
        s.s_g = std::max(0.0, angleBetween(v, P.t_g) - thetaC(lam, sw_));
        s.ang_excess = s.s_g;
      }
      else
      {
        // [fix R1a/R5] cone half-angle; [fix R1b] slack = distance to the cone in velocity space / v_c
        s.theta_eff = coneTheta(P, lam);
        s.ang_excess = std::max(0.0, angleBetween(v, P.t_g) - s.theta_eff);
        s.s_g = sw_.fx.r1b_continuous_slack ? coneSlack(v, P.t_g, s.theta_eff, c_) : s.ang_excess;
      }
    }

    s.J = (1.0 - lam) * s.J_A + lam * s.J_B + s.Gamma + c_.rho_g * s.s_g; // (36)(37)
    if (sw_.c3_soft_tracking > 0.0)
      s.J += sw_.c3_soft_tracking * (v - c_.v_c * P.t_g).squaredNorm(); // A6 only
    s.J_ws = (sw_.fx.s1_eps_constraint && lam > 0.0) ? s.J + lam * s.rho : s.J; // [fix S1] frozen weighted-sum value
    return s;
  }

  double LocalPlanner::coneTheta(const LocalProblem &P, double lam) const
  {
    double th = thetaC(lam, sw_);
    if (sw_.fx.r5_mode_gating && P.free_mode) // [fix R5] corridor tolerance e_cut over the guidance window ell_W
      th = std::max(th, std::atan(c_.e_cut / c_.ellW()));
    if (sw_.fx.r1a_reachable_cone) // [fix R1a] never narrower than the heading lag that one cycle cannot remove
      th = std::max(th, angleBetween(P.v_prev, P.t_g) - reachTurn(P.v_prev.norm(), c_));
    return std::min(th, M_PI);
  }
''')
rep(C, '''  LocalSolution LocalPlanner::solve(const LocalProblem &P) const
  {
    const auto cand = candidates(P);
''', '''  LocalSolution LocalPlanner::solve(const LocalProblem &P) const
  {
    if (sw_.fx.s1_eps_constraint && P.lambda > 0.0)
      return solveBudget(P); // [fix S1]
    const auto cand = candidates(P);
''')
rep(C, '''    for (int it = 0; found && it < 3 && best.s_g > 0.0; ++it)
    {
      const Eigen::Vector3d axis = best.v.cross(P.t_g);
      Eigen::Vector3d vp;
      if (axis.norm() > 1e-12)
        vp = Eigen::AngleAxisd(best.s_g, axis.normalized()) * best.v;
''', '''    // ang_excess == s_g bit for bit when R1a/R1b/R5 are off (frozen behaviour)
    for (int it = 0; found && it < 3 && best.ang_excess > 0.0; ++it)
    {
      const Eigen::Vector3d axis = best.v.cross(P.t_g);
      Eigen::Vector3d vp;
      if (axis.norm() > 1e-12)
        vp = Eigen::AngleAxisd(best.ang_excess, axis.normalized()) * best.v;
''')
rep(C, '''  LocalSolution LocalPlanner::classicVO(const LocalProblem &P) const
''', '''  LocalSolution LocalPlanner::solveBudget(const LocalProblem &P) const
  {
    // [fix S1] epsilon-constraint form of P_loc: argmin of J~ without the weighted rho over the candidates whose risk
    // stays within rho <= 1 - lambda; when no feasible candidate meets the budget, the frozen weighted sum decides.
    const auto cand = candidates(P);
    const bool dyn_active = P.lambda > 0.0;
    const double budget = 1.0 - P.lambda;
    std::vector<LocalSolution> feas;
    for (const auto &v : cand)
      if (turnOk(P, v) && staticOk(P, v) && (!dyn_active || dynamicOk(P, v)))
        feas.push_back(evaluate(P, v));
    bool any_budget = false;
    for (const auto &s : feas)
      any_budget = any_budget || s.rho <= budget + 1e-12;
    auto obj = [&](const LocalSolution &s) { return any_budget ? s.J : s.J_ws; };
    auto admit = [&](const LocalSolution &s) { return !any_budget || s.rho <= budget + 1e-12; };
    LocalSolution best;
    bool found = false;
    int n_feas = static_cast<int>(feas.size());
    for (const auto &s : feas)
      if (admit(s) && (!found || obj(s) < obj(best) - 1e-12))
      {
        best = s;
        found = true;
      }
    // [ENG-4] refinement onto the cone surface, with the same objective and admission rule
    for (int it = 0; found && it < 3 && best.ang_excess > 0.0; ++it)
    {
      const Eigen::Vector3d axis = best.v.cross(P.t_g);
      Eigen::Vector3d vp;
      if (axis.norm() > 1e-12)
        vp = Eigen::AngleAxisd(best.ang_excess, axis.normalized()) * best.v;
      else
        vp = best.v.norm() * P.t_g.normalized();
      if (!admissible(P, vp) && vp.norm() > 1e-12)
      {
        const Eigen::Vector3d u = vp.normalized();
        const double r = c_.a_max * c_.dt, b = u.dot(P.v_prev);
        const double disc = b * b - P.v_prev.squaredNorm() + r * r;
        if (disc < 0.0)
          break;
        const double lo = std::max(0.0, b - std::sqrt(disc)), hi = std::min(b + std::sqrt(disc), speedLimit(P));
        if (hi < lo)
          break;
        vp = std::min(hi, std::max(lo, best.v.norm())) * u;
      }
      if (!admissible(P, vp) || !turnOk(P, vp) || !staticOk(P, vp) || (dyn_active && !dynamicOk(P, vp)))
        break;
      const LocalSolution s = evaluate(P, vp);
      if (!admit(s) || obj(s) >= obj(best) - 1e-12)
        break;
      best = s;
      ++n_feas;
    }
    if (!found)
      best = fallback(P, cand);
    best.budget_active = any_budget;
    best.n_candidates = static_cast<int>(cand.size());
    best.n_feasible = n_feas;
    return best;
  }

  LocalSolution LocalPlanner::classicVO(const LocalProblem &P) const
''')
