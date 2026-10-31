# Base step 1 (generated from the verified patch of the previous session). Executed with only `rep` and the
# discovered role map `R` in scope; no file name appears here, anchors are matched exactly once.
# ---------------- coupling.h
rep(R['switches_h'],
'''  // Ablation switches (subtask3_porting_plan.md section 5); set from launch parameters only
''',
'''  // Engineering fixes after thesis-frozen-v1 (fixes design note R1-R5, S1). Every flag defaults to false; with all
  // flags false every frozen code path runs unchanged. Set from the launch parameters ~fixes/<name> only.
  struct Fixes
  {
    bool r1a_reachable_cone = false;       // R1a cone widened to the heading reachable in one cycle under (C-a)
    bool r1b_continuous_slack = false;     // R1b cone slack = dist(v, K) / v_c, continuous at v = 0
    bool r1c_heading_memory = false;       // R1c Gamma turn term measured against the last heading when |v_prev| < v_min
    bool r2a_kappa_decay = false;          // R2a kappa peak-hold with exp(-dt / T_g) release instead of a per-cycle clear
    bool r2b_kappa_local_excluded = false; // R2b J_A = V_Pi computed without kappa
    bool r3_relevant_trigger = false;      // R3 replan on remaining-path increase / off-path decrease > eps_w only
    bool r4_switch_hysteresis = false;     // R4 geometric switch only if cheaper by eps_w and T_g after the last switch
    bool r5_mode_gating = false;           // R5 cone base width atan(e_cut / ell_W) while threat-free for >= T_c
    bool s1_eps_constraint = false;        // S1 risk budget rho <= 1 - lambda instead of the weighted rho term
    bool diag_log = false;                 // extended diagnostics only (cycles_ext.csv, switches_ext.csv)
    bool any() const;
  };

  // Ablation switches (subtask3_porting_plan.md section 5); set from launch parameters only
''')
rep(R['switches_h'],
'''    XiSource xi_source = XiSource::Reference;
''',
'''    XiSource xi_source = XiSource::Reference;
    Fixes fx; // engineering fixes after thesis-frozen-v1; all false = frozen behaviour
''')
rep(R['switches_h'],
'''  double gammaE(const Eigen::Vector3d &t_prev, const Eigen::Vector3d &t_e, double lambda_e, const ModelConstants &c);
''',
'''  double gammaE(const Eigen::Vector3d &t_prev, const Eigen::Vector3d &t_e, double lambda_e, const ModelConstants &c);

  // ---- fixes after thesis-frozen-v1 (used only when the matching Fixes flag is on) ----
  // R1a: largest heading change reachable in one cycle from speed s under (C-a): 2 s sin(phi / 2) <= a_max dt
  double reachTurn(double s, const ModelConstants &c);
  // R1b: dist(v, K(theta)) / v_c with K(theta) = {u : angle(u, t_g) <= theta} plus the apex 0
  double coneSlack(const Eigen::Vector3d &v, const Eigen::Vector3d &t_g, double theta, const ModelConstants &c);
  // R1c: eq. (34) whose turn term uses h_prev as the previous heading while |v_prev| < v_min
  double gammaV(const Eigen::Vector3d &v, const Eigen::Vector3d &v_prev, double lambda, const ModelConstants &c,
                const Eigen::Vector3d &h_prev);
''')
# ---------------- coupling.cpp
rep(R['coupling_cpp'],
'''    return (xw - std::sqrt(disc)) / w2; // smaller root of ||x - w t|| = R
  }

} // namespace jps_vo
''',
'''    return (xw - std::sqrt(disc)) / w2; // smaller root of ||x - w t|| = R
  }

  // ---------------------------------------------------------------- fixes after thesis-frozen-v1

  bool Fixes::any() const
  {
    return r1a_reachable_cone || r1b_continuous_slack || r1c_heading_memory || r2a_kappa_decay ||
           r2b_kappa_local_excluded || r3_relevant_trigger || r4_switch_hysteresis || r5_mode_gating ||
           s1_eps_constraint || diag_log;
  }

  double reachTurn(double s, const ModelConstants &c)
  {
    if (!(s > 1e-12))
      return M_PI; // at rest every heading is reachable
    return 2.0 * std::asin(std::min(1.0, c.a_max * c.dt / (2.0 * s)));
  }

  double coneSlack(const Eigen::Vector3d &v, const Eigen::Vector3d &t_g, double theta, const ModelConstants &c)
  {
    const double sp = v.norm();
    if (sp <= 1e-12 || t_g.norm() <= 1e-9)
      return 0.0;
    const double excess = angleBetween(v, t_g) - theta;
    if (excess <= 0.0)
      return 0.0;
    // nearest point of K: on the generator at angle theta when excess <= pi/2, otherwise the apex
    return (excess <= 0.5 * M_PI ? sp * std::sin(excess) : sp) / c.v_c;
  }

  double gammaV(const Eigen::Vector3d &v, const Eigen::Vector3d &v_prev, double lambda, const ModelConstants &c,
                const Eigen::Vector3d &h_prev)
  {
    const double f = lambda * (1.0 - lambda);
    if (f <= 0.0)
      return 0.0;
    Eigen::Vector3d h = Eigen::Vector3d::Zero();
    if (v_prev.norm() >= c.v_min)
      h = v_prev.normalized();
    else if (h_prev.norm() > 1e-9)
      h = h_prev.normalized(); // a turn through a low-speed phase still pays for the heading change
    double psi_theta = 0.0;
    if (v.norm() >= c.v_min && h.squaredNorm() > 0.0)
      psi_theta = (1.0 - v.normalized().dot(h)) / (1.0 - std::cos(c.theta_max));
    Eigen::Vector3d dv = v - v_prev;
    if (v_prev.norm() >= c.v_min)
    {
      const Eigen::Vector3d hp = v_prev.normalized();
      dv -= hp * hp.dot(dv); // P_perp = I - h h^T, as in the frozen eq. (34)
    }
    const double psi_a = dv.squaredNorm() / std::pow(c.a_max * c.dt, 2);
    return f * (c.w_theta * psi_theta + c.w_a * psi_a);
  }

} // namespace jps_vo
''')
# ---------------- global_path
rep(R['global_path_h'],
'''    uint64_t key = 0;     // identity of the G_J state transition (from, to, dir_in)
''',
'''    uint64_t key = 0;     // identity of the G_J state transition (from, to, dir_in)
    double kappa = 0.0;   // [fix R2b] kappa part of w when the path was searched (or refreshed)
''')
rep(R['global_path_h'],
'''    double value(const Eigen::Vector3d &q, double L_s) const;           // V_Pi(q), eq. (23)
''',
'''    double value(const Eigen::Vector3d &q, double L_s) const;           // V_Pi(q), eq. (23)
    double costToGoNoKappa(double s) const;                             // [fix R2b] C_Pi(s) with w_e - kappa_e
    double valueNoKappa(const Eigen::Vector3d &q, double L_s) const;    // [fix R2b] V_Pi(q) with w_e - kappa_e
''')
rep(R['global_path_cpp'],
'''  Eigen::Vector3d GlobalPath::guideTangent(double s, double ell_w) const
''',
'''  double GlobalPath::costToGoNoKappa(double s) const
  {
    double c = 0.0;
    for (size_t i = 0; i < edges.size(); ++i)
    {
      const auto &e = edges[i];
      if (e.ell <= 0.0)
        continue;
      const double lo = std::max(s, cum_[i]), hi = cum_[i + 1];
      if (hi > lo)
        c += (hi - lo) * (e.w - e.kappa) / e.ell;
    }
    return c;
  }

  double GlobalPath::valueNoKappa(const Eigen::Vector3d &q, double L_s) const
  {
    const Projection p = project(q);
    return costToGoNoKappa(p.s) + p.dist / L_s;
  }

  Eigen::Vector3d GlobalPath::guideTangent(double s, double ell_w) const
''')
