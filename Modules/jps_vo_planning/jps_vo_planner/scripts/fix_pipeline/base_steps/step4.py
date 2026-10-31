# Base step 4 (generated from the verified patch of the previous session). Executed with only `rep` and the
# discovered role map `R` in scope; no file name appears here, anchors are matched exactly once.
H = R['node_h']
rep(H, '''    void writePathEvents(const std::vector<PathEvent> &events);
''', '''    void writePathEvents(const std::vector<PathEvent> &events);
    void writeSwitchDiags(const std::vector<SwitchDiag> &diags); // [fixes] switches_ext.csv
''')
rep(H, '''    std::ofstream cycles_csv_, paths_csv_, pi0_csv_, odom_gt_csv_;
''', '''    std::ofstream cycles_csv_, paths_csv_, pi0_csv_, odom_gt_csv_;
    std::ofstream cycles_ext_csv_, switches_ext_csv_; // [fixes] written only when a Fixes flag is on
    bool ext_open_ = false;
''')
C = R['node_cpp']
rep(C, '''    sw.enable_gamma = param(nh, "switches/enable_gamma", true);
''', '''    sw.enable_gamma = param(nh, "switches/enable_gamma", true);
    // engineering fixes after thesis-frozen-v1: every flag defaults to false (= frozen behaviour); config/fixes/*.yaml
    sw.fx.r1a_reachable_cone = param(nh, "fixes/r1a_reachable_cone", false);
    sw.fx.r1b_continuous_slack = param(nh, "fixes/r1b_continuous_slack", false);
    sw.fx.r1c_heading_memory = param(nh, "fixes/r1c_heading_memory", false);
    sw.fx.r2a_kappa_decay = param(nh, "fixes/r2a_kappa_decay", false);
    sw.fx.r2b_kappa_local_excluded = param(nh, "fixes/r2b_kappa_local_excluded", false);
    sw.fx.r3_relevant_trigger = param(nh, "fixes/r3_relevant_trigger", false);
    sw.fx.r4_switch_hysteresis = param(nh, "fixes/r4_switch_hysteresis", false);
    sw.fx.r5_mode_gating = param(nh, "fixes/r5_mode_gating", false);
    sw.fx.s1_eps_constraint = param(nh, "fixes/s1_eps_constraint", false);
    sw.fx.diag_log = param(nh, "fixes/diag_log", false);
    if (sw.fx.any())
      ROS_WARN("[jps_vo] fixes active, this is NOT the thesis-frozen-v1 behaviour: r1a %d r1b %d r1c %d r2a %d r2b %d r3 %d "
               "r4 %d r5 %d s1 %d diag %d",
               sw.fx.r1a_reachable_cone, sw.fx.r1b_continuous_slack, sw.fx.r1c_heading_memory, sw.fx.r2a_kappa_decay,
               sw.fx.r2b_kappa_local_excluded, sw.fx.r3_relevant_trigger, sw.fx.r4_switch_hysteresis,
               sw.fx.r5_mode_gating, sw.fx.s1_eps_constraint, sw.fx.diag_log);
''')
rep(C, '''    const bool inv_ok = rec.lambda >= 0.0 && rec.lambda <= 1.0 && rec.kappa >= 0.0 && rec.kappa <= 1.0 &&
                        (rec.fallback_active || rec.kappa <= rec.lambda + 1e-12);
''', '''    // eq. (25): kappa <= lambda except in fallback; [fix R2a] with memory: kappa <= kappa_bound (decayed max of
    // lambda, or 1 after a fallback)
    const bool inv_ok = rec.lambda >= 0.0 && rec.lambda <= 1.0 && rec.kappa >= 0.0 && rec.kappa <= 1.0 &&
                        (core_cfg_.sw.fx.r2a_kappa_decay ? rec.kappa <= rec.kappa_bound + 1e-12
                                                         : (rec.fallback_active || rec.kappa <= rec.lambda + 1e-12));
''')
rep(C, '''    writePathEvents(core_.takePathEvents());
''', '''    writePathEvents(core_.takePathEvents());
    if (core_cfg_.sw.fx.any())
      writeSwitchDiags(core_.takeSwitchDiags());
''')
rep(C, '''    odom_gt_csv_ << "t,odom_x,odom_y,odom_z,obs_name,obs_x,obs_y,obs_z,obs_vx,obs_vy,obs_vz,obs_r\\n";
    logs_open_ = true;
  }
''', '''    odom_gt_csv_ << "t,odom_x,odom_y,odom_z,obs_name,obs_x,obs_y,obs_z,obs_vx,obs_vy,obs_vz,obs_r\\n";
    logs_open_ = true;
    if (core_cfg_.sw.fx.any())
    {
      // [fixes] extended diagnostics, separate files so that the frozen CSV contract stays byte-identical
      cycles_ext_csv_.open(log_dir_ + "/cycles_ext.csv");
      switches_ext_csv_.open(log_dir_ + "/switches_ext.csv");
      cycles_ext_csv_ << "t,t_g_x,t_g_y,t_g_z,ang_vprev_tg_deg,reach_turn_deg,theta_eff_deg,ang_excess_deg,s_g_star,"
                         "J,J_A,J_B,Gamma,cone_term,rho_star,zero_is_opt,ray_feasible,free_mode,e_k,kappa_on_ek,"
                         "kappa_map_size,kappa_bound,budget_active,fallback_active\\n";
      switches_ext_csv_ << "t,reason,is_switch,accepted,forced,C_old,C_new,dwell_left,up,down,n_on,n_off\\n";
      ext_open_ = cycles_ext_csv_.good() && switches_ext_csv_.good();
    }
  }

  void PlannerNode::writeSwitchDiags(const std::vector<SwitchDiag> &diags)
  {
    if (!ext_open_)
      return;
    for (const auto &d : diags)
      switches_ext_csv_ << tnum(d.t) << "," << d.reason << "," << d.is_switch << "," << d.accepted << "," << d.forced << ","
                        << num(d.C_old) << "," << num(d.C_new) << "," << num(d.dwell_left) << "," << num(d.up) << ","
                        << num(d.down) << "," << d.n_on << "," << d.n_off << "\\n";
    if (!diags.empty())
      switches_ext_csv_.flush();
  }
''')
rep(C, '''      << ", pre_rotate_yaw: " << pre_rotate_ << "}\\n";
  }
''', '''      << ", pre_rotate_yaw: " << pre_rotate_ << "}\\n";
    if (sw.fx.any()) // [fixes] written only when a flag is on, so the frozen meta file stays byte-identical
      f << "fixes: {r1a_reachable_cone: " << sw.fx.r1a_reachable_cone << ", r1b_continuous_slack: " << sw.fx.r1b_continuous_slack
        << ", r1c_heading_memory: " << sw.fx.r1c_heading_memory << ", r2a_kappa_decay: " << sw.fx.r2a_kappa_decay
        << ", r2b_kappa_local_excluded: " << sw.fx.r2b_kappa_local_excluded
        << ", r3_relevant_trigger: " << sw.fx.r3_relevant_trigger << ", r4_switch_hysteresis: " << sw.fx.r4_switch_hysteresis
        << ", r5_mode_gating: " << sw.fx.r5_mode_gating << ", s1_eps_constraint: " << sw.fx.s1_eps_constraint
        << ", diag_log: " << sw.fx.diag_log << "}\\n";
  }
''')
rep(C, '''      << stall_break << "," << stall_hold
      << "\\n";
  }
''', '''      << stall_break << "," << stall_hold
      << "\\n";
    if (ext_open_)
    {
      auto &x = cycles_ext_csv_;
      const double deg = 180.0 / M_PI;
      x << tnum(r.t) << "," << num(r.t_g.x()) << "," << num(r.t_g.y()) << "," << num(r.t_g.z()) << ","
        << num(r.ang_vprev_tg * deg) << "," << num(r.reach_turn * deg) << "," << num(r.theta_eff * deg) << ","
        << num(r.ang_excess * deg) << "," << num(r.s_g_star) << "," << num(r.J) << "," << num(r.J_A) << "," << num(r.J_B)
        << "," << num(r.Gamma) << "," << num(r.cone_term) << "," << num(r.rho_star) << "," << r.zero_is_opt << ","
        << r.ray_feasible << "," << r.free_mode << "," << r.e_k << "," << num(r.kappa_on_ek) << "," << r.kappa_map_size
        << "," << num(r.kappa_bound) << "," << r.budget_active << "," << (r.fallback_active ? 1 : 0) << "\\n";
    }
  }
''')
rep(C, '''    cycles_csv_.flush();
    odom_gt_csv_.flush();
''', '''    cycles_csv_.flush();
    odom_gt_csv_.flush();
    if (ext_open_)
    {
      cycles_ext_csv_.flush();
      switches_ext_csv_.flush();
    }
''')
# ---------------- launch files
L = R['launch_node']
rep(L, '''  <arg name="async_global" default=""/>
''', '''  <arg name="async_global" default=""/>
  <!-- engineering fixes after thesis-frozen-v1: name of a file in config/fixes/ ("" = none = frozen behaviour) -->
  <arg name="fixes_file" default=""/>
''')
rep(L, '''    <rosparam command="load" file="@FIND_CONFIG@/groups/$(arg group).yaml"/>
''', '''    <rosparam command="load" file="@FIND_CONFIG@/groups/$(arg group).yaml"/>
    <rosparam if="$(eval arg('fixes_file') != '')" command="load" file="@FIND_CONFIG@/fixes/$(arg fixes_file).yaml"/>
''')
S = R['launch_sitl']
rep(S, '''  <arg name="pre_rotate_yaw" default="true"/>
''', '''  <arg name="pre_rotate_yaw" default="true"/>
  <arg name="fixes_file" default=""/>  <!-- engineering fixes after thesis-frozen-v1; "" = frozen behaviour -->
''')
rep(S, '''    <arg name="pre_rotate_yaw" value="$(arg pre_rotate_yaw)"/>
''', '''    <arg name="pre_rotate_yaw" value="$(arg pre_rotate_yaw)"/>
    <arg name="fixes_file" value="$(arg fixes_file)"/>
''')
