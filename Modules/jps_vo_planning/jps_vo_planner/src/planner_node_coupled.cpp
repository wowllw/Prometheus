// Coupled mode of jps_vo_planner_node (Subtask 3): 3D-JPS + VO with C1/C2/C3, ablation switches from launch
// parameters, CSV logs of subtask4_ablation_experiment_spec.md section 8 and the runtime verification counters.
#include "jps_vo_planner/planner_node.h"

#include <prometheus_msgs/UAVCommand.h>

#include <chrono>
#include <iomanip>
#include <limits>
#include <sstream>
#include <sys/stat.h>

namespace jps_vo
{
  namespace
  {
    template <typename T>
    T param(ros::NodeHandle &nh, const std::string &name, const T &def)
    {
      T v;
      nh.param<T>(name, v, def);
      return v;
    }

    std::string num(double x)
    {
      if (!std::isfinite(x))
        return x > 0 ? "inf" : "-inf";
      std::ostringstream os;
      os << std::setprecision(9) << x;
      return os.str();
    }

    // wall-clock time stamps: microsecond resolution (9 significant digits would only resolve 10 s)
    std::string tnum(double t)
    {
      std::ostringstream os;
      os << std::fixed << std::setprecision(6) << t;
      return os.str();
    }
  } // namespace

  PlannerNode::~PlannerNode()
  {
    running_ = false;
    if (global_thread_.joinable())
      global_thread_.join();
    if (coupled_ && logs_open_)
      writeChecks();
  }

  void PlannerNode::initCoupled(ros::NodeHandle &nh)
  {
    ModelConstants &c = core_cfg_.c;
    // frozen constants (subtask4_ablation_experiment_spec.md section 2); calibratable ones: v_max, a_max, d_min, tau1, tau2
    c.r_q = param(nh, "model/r_q", c.r_q);
    c.e_cut = param(nh, "model/e_cut", c.e_cut);
    c.v_c = param(nh, "model/v_c", c.v_c);
    c.dt = param(nh, "model/dt", c.dt);
    c.tau_h = param(nh, "model/tau_h", c.tau_h);
    c.m = param(nh, "model/m", c.m);
    c.L_min = param(nh, "model/L_min", c.L_min);
    c.psi_max = param(nh, "model/psi_max_deg", 60.0) * M_PI / 180.0;
    c.theta_max = param(nh, "model/theta_max_deg", 60.0) * M_PI / 180.0;
    c.v_min = param(nh, "model/v_min", c.v_min);
    c.w_theta = param(nh, "model/w_theta", c.w_theta);
    c.w_a = param(nh, "model/w_a", c.w_a);
    c.rho_g = param(nh, "model/rho_g", c.rho_g);
    c.eps_w = param(nh, "model/eps_w", c.eps_w);
    c.T_c = param(nh, "model/T_c", c.T_c);
    c.T_g = param(nh, "model/T_g", c.T_g);
    c.dv_star = param(nh, "model/dv_star", c.dv_star);
    c.d_min = param(nh, "model/d_min", c.d_min);
    c.r_o_max = param(nh, "model/r_o_max", c.r_o_max);
    c.tau1 = param(nh, "model/tau1", -1.0);
    c.tau2 = param(nh, "model/tau2", -1.0);
    c.v_max = max_vel_;
    c.a_max = max_acc_;
    if (c.v_c > c.v_max)
    {
      ROS_WARN("[jps_vo] v_c %.2f > v_max %.2f, set v_c = v_max (D1)", c.v_c, c.v_max);
      c.v_c = c.v_max;
    }

    Switches &sw = core_cfg_.sw;
    bool ok = sw.parseLambdaMode(param<std::string>(nh, "switches/lambda_mode", "adaptive")) &&
              sw.parseLambdaShape(param<std::string>(nh, "switches/lambda_shape", "quintic_log")) &&
              sw.parseThetaCMode(param<std::string>(nh, "switches/theta_c_mode", "linear")) &&
              sw.parseXiSource(param<std::string>(nh, "switches/xi_source", "reference"));
    if (!ok)
      ROS_ERROR("[jps_vo] invalid switch value, check config/groups/*.yaml");
    sw.use_rho_in_pure_vo = param(nh, "switches/use_rho_in_pure_vo", false);
    sw.enable_c2 = param(nh, "switches/enable_c2", true);
    sw.enable_c3 = param(nh, "switches/enable_c3", true);
    sw.c3_soft_tracking = param(nh, "switches/c3_soft_tracking", 0.0);
    sw.enable_gamma = param(nh, "switches/enable_gamma", true);
    // engineering fixes after thesis-frozen-v1: every flag defaults to false (= frozen behaviour); config/fixes/*.yaml
    sw.fx.r1a_reachable_cone = param(nh, "fixes/r1a_reachable_cone", false);
    sw.fx.r1b_continuous_slack = param(nh, "fixes/r1b_continuous_slack", false);
    sw.fx.r1c_heading_memory = param(nh, "fixes/r1c_heading_memory", false);
    sw.fx.r1d_feasible_cone = param(nh, "fixes/r1d_feasible_cone", false);
    sw.fx.r1e_lowspeed_angle = param(nh, "fixes/r1e_lowspeed_angle", false);
    sw.fx.r2a_kappa_decay = param(nh, "fixes/r2a_kappa_decay", false);
    sw.fx.r2b_kappa_local_excluded = param(nh, "fixes/r2b_kappa_local_excluded", false);
    sw.fx.r3_relevant_trigger = param(nh, "fixes/r3_relevant_trigger", false);
    sw.fx.r4_switch_hysteresis = param(nh, "fixes/r4_switch_hysteresis", false);
    sw.fx.r4m_dwell_free_only = param(nh, "fixes/r4m_dwell_free_only", false);
    sw.fx.r4n_leaky_evidence = param(nh, "fixes/r4n_leaky_evidence", false);
    sw.fx.r4s_safety_override = param(nh, "fixes/r4s_safety_override", false);
    sw.fx.r5_mode_gating = param(nh, "fixes/r5_mode_gating", false);
    sw.fx.s1_eps_constraint = param(nh, "fixes/s1_eps_constraint", false);
    sw.fx.diag_log = param(nh, "fixes/diag_log", false);
    if ((sw.fx.r1b_continuous_slack || sw.fx.r1e_lowspeed_angle) && !sw.fx.r1d_feasible_cone)
      ROS_WARN("[jps_vo] fixes: r1b_continuous_slack / r1e_lowspeed_angle without r1d_feasible_cone can stall at "
               "lambda = 0 (zero-width ray blocked); enable r1d_feasible_cone with it");
    if (sw.fx.any())
      ROS_WARN("[jps_vo] fixes active, this is NOT the thesis-frozen-v1 behaviour: r1a %d r1b %d r1c %d r1d %d r1e %d r2a %d r2b %d "
               "r3 %d r4 %d r4m %d r4n %d r4s %d r5 %d s1 %d diag %d",
               sw.fx.r1a_reachable_cone, sw.fx.r1b_continuous_slack, sw.fx.r1c_heading_memory, sw.fx.r1d_feasible_cone,
               sw.fx.r1e_lowspeed_angle, sw.fx.r2a_kappa_decay,
               sw.fx.r2b_kappa_local_excluded, sw.fx.r3_relevant_trigger, sw.fx.r4_switch_hysteresis,
               sw.fx.r4m_dwell_free_only, sw.fx.r4n_leaky_evidence, sw.fx.r4s_safety_override, sw.fx.r5_mode_gating,
               sw.fx.s1_eps_constraint, sw.fx.diag_log);

    const std::string tie = param<std::string>(nh, "jps/tie_rule", "strict");
    core_cfg_.tie_rule = tie == "literal" ? Jps3D::TieRule::Literal : Jps3D::TieRule::Strict;
    core_cfg_.search_margin = param(nh, "jps/search_margin", 5.0);
    core_cfg_.snap_radius = param(nh, "jps/snap_radius", 10);
    core_cfg_.label = param<std::string>(nh, "group", "g6");
    core_cfg_.replan_period = replan_period_ > 0.0 ? replan_period_ : 1.0;
    core_cfg_.goal_tol = param(nh, "goal_reached_pos", 0.3);
    core_cfg_.goal_v_tol = param(nh, "goal_reached_vel", 0.1);
    core_cfg_.switch_threshold = param(nh, "switch_threshold", 0.3);
    core_.configure(core_cfg_);
    core_cfg_ = core_.config(); // with tau1 / tau2 derived from eq. (32)
    async_global_ = param(nh, "async_global", true);
    pre_rotate_ = param(nh, "pre_rotate_yaw", false);
    if (pre_rotate_)
      uav_cmd_pub_ = nh.advertise<prometheus_msgs::UAVCommand>("uav_command", 10);
    log_dir_ = param<std::string>(nh, "log_dir", "");

    MapInput::Params mp;
    mp.grid.origin = Eigen::Vector3d(map_x_min_, map_y_min_, map_z_min_);
    mp.grid.size = Eigen::Vector3d(map_x_max_ - map_x_min_, map_y_max_ - map_y_min_, map_z_max_ - map_z_min_);
    mp.grid.resolution = param(nh, "grid/resolution", 0.1);
    mp.grid.inflation = core_cfg_.c.rInfl(); // eq. (1)
    mp.dyn_prefix = param<std::string>(nh, "dyn/prefix", "dyn_obs_");
    mp.default_radius = param(nh, "dyn/radius", 0.3);
    mp.mask_margin = param(nh, "dyn/mask_margin", 0.15);
    mp.mask_history = param(nh, "dyn/mask_history", 15.0);
    mp.rebuild_rate = param(nh, "grid/rebuild_rate", 5.0);
    mp.obstacle_timeout = param(nh, "dyn/obstacle_timeout", 0.5);
    map_input_.init(nh, mp);

    local_timer_ = nh.createTimer(ros::Duration(core_cfg_.c.dt), &PlannerNode::localCycleCallback, this);
    odom_gt_timer_ = nh.createTimer(ros::Duration(0.05), &PlannerNode::odomGtCallback, this);
    if (async_global_)
    {
      running_ = true;
      global_thread_ = std::thread(&PlannerNode::globalThreadLoop, this);
    }

    const auto &cc = core_cfg_.c;
    ROS_WARN("[jps_vo] coupled mode, group %s: lambda_mode=%s c2=%d c3=%d gamma=%d | v_max %.2f a_max %.2f d_min %.2f "
             "r_infl %.2f tau1 %.3f tau2 %.3f ell_w %.3f | tie_rule %s margin %.1f m | async_global %d | log_dir '%s'",
             core_cfg_.label.c_str(), param<std::string>(nh, "switches/lambda_mode", "adaptive").c_str(), sw.enable_c2,
             sw.enable_c3, sw.enable_gamma, cc.v_max, cc.a_max, cc.d_min, cc.rInfl(), cc.tau1, cc.tau2, cc.ellW(), tie.c_str(),
             core_cfg_.search_margin, async_global_, log_dir_.c_str());
  }

  void PlannerNode::coupledGoal(const Eigen::Vector3d &goal)
  {
    goal_t0_ = ros::Time::now().toSec(); // t0 of P-2: goal publish time (a pre-rotation counts into the run)
    if (pre_rotate_)
    {
      // turn the forward D435i towards the goal before planning: gate off (traj_server stops), yaw command via
      // uav_control at the current position, as ego_replan_fsm::waypointCallback does
      prerot_active_ = true;
      prerot_t0_ = ros::Time::now();
      prerot_pos_ = odom_pos_;
      prerot_yaw_ = std::atan2(goal.y() - odom_pos_.y(), goal.x() - odom_pos_.x());
      pending_goal_ = goal;
      publishGate(false);
      have_traj_ = false;
      ROS_INFO("[jps_vo] pre-rotation to yaw %.1f deg before planning", prerot_yaw_ * 180.0 / M_PI);
      return;
    }
    Eigen::Vector3d p, v;
    referenceState(ros::Time::now(), p, v);
    core_.setGoal(goal, p);
    pi0_written_ = false;
    openLogs();
    writeMeta();
  }

  // ---------------------------------------------------------------- global layer thread [ENG-11]

  void PlannerNode::globalThreadLoop()
  {
    const auto period = std::chrono::duration<double>(core_cfg_.c.dt);
    while (running_)
    {
      const auto t0 = std::chrono::steady_clock::now();
      GlobalSnapshot s;
      {
        std::lock_guard<std::mutex> lk(snap_mtx_);
        s = snap_;
      }
      if (s.active && s.grid)
      {
        const GlobalStepResult g = core_.globalStep(s.t, *s.grid, s.p_ref, s.v_ref, s.obs);
        if (g.reason != "none")
        {
          std::lock_guard<std::mutex> lk(snap_mtx_);
          // reported in the next local cycle; a second search before that keeps the latest reason, summed time
          const double prev_ms = pending_global_.reason == "none" ? 0.0 : pending_global_.ms;
          pending_global_ = g;
          pending_global_.ms += prev_ms;
        }
      }
      std::this_thread::sleep_until(t0 + period);
    }
  }

  // ---------------------------------------------------------------- local cycle

  void PlannerNode::localCycleCallback(const ros::TimerEvent &)
  {
    const ros::Time now = ros::Time::now();
    if (map_input_.update(now))
      core_.notifyMapUpdated();
    if (prerot_active_)
    {
      if (!in_command_ || !have_goal_)
      {
        prerot_active_ = false;
        return;
      }
      const double err = std::fabs(std::remainder(odom_yaw_ - prerot_yaw_, 2.0 * M_PI));
      if (err > 0.15 && (now - prerot_t0_).toSec() < 6.0)
      {
        prometheus_msgs::UAVCommand cmd;
        cmd.header.stamp = now;
        cmd.Agent_CMD = prometheus_msgs::UAVCommand::Move;
        cmd.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
        cmd.position_ref[0] = prerot_pos_.x();
        cmd.position_ref[1] = prerot_pos_.y();
        cmd.position_ref[2] = prerot_pos_.z();
        cmd.yaw_ref = prerot_yaw_;
        cmd.Command_ID = ++uav_cmd_id_;
        uav_cmd_pub_.publish(cmd);
        return;
      }
      ROS_INFO("[jps_vo] pre-rotation done (yaw error %.2f rad after %.1f s)", err, (now - prerot_t0_).toSec());
      prerot_active_ = false;
      core_.setGoal(pending_goal_, odom_pos_);
      pi0_written_ = false;
      openLogs();
      writeMeta();
    }
    if (!in_command_ || !have_goal_ || !core_.hasGoal())
    {
      std::lock_guard<std::mutex> lk(snap_mtx_);
      snap_.active = false;
      return;
    }
    if (!map_input_.haveCloud())
    {
      ++checks_.no_cloud_cycles;
      ROS_WARN_THROTTLE(2.0, "[jps_vo] no octomap cloud yet, holding the reference");
      return;
    }
    if (!last_cycle_time_.isZero())
      checks_.max_cycle_gap = std::max(checks_.max_cycle_gap, (now - last_cycle_time_).toSec());
    last_cycle_time_ = now;

    Eigen::Vector3d p_ref, v_ref;
    referenceState(now, p_ref, v_ref); // [ENG-1]: the planner works on the reference state
    const std::vector<ObstacleState> obs = map_input_.obstacleStates(now);
    const std::shared_ptr<const OccupancyGrid> grid = map_input_.grid();

    CycleRecord rec;
    if (async_global_)
    {
      GlobalStepResult g;
      {
        std::lock_guard<std::mutex> lk(snap_mtx_);
        g = pending_global_;
        pending_global_ = GlobalStepResult();
        snap_.active = true;
        snap_.t = now.toSec();
        snap_.p_ref = p_ref;
        snap_.v_ref = v_ref;
        snap_.obs = obs;
        snap_.grid = grid;
      }
      if (core_.path().empty() && g.reason == "none")
        return; // first global search still running: keep flying the current reference (hover)
      rec = core_.localStep(now.toSec(), *grid, p_ref, v_ref, odom_pos_, obs, g);
    }
    else
      rec = core_.cycle(now.toSec(), *grid, p_ref, v_ref, odom_pos_, obs);

    // ---- runtime checks ----
    ++checks_.cycles;
    if (rec.replan_reason != "none")
    {
      ++checks_.global_searches;
      checks_.global_fails += rec.replan_reason == "fail";
      checks_.max_global_ms = std::max(checks_.max_global_ms, rec.plan_ms_global);
    }
    checks_.max_local_ms = std::max(checks_.max_local_ms, rec.plan_ms_local);
    checks_.cycles_over_dt += rec.plan_ms_local > core_cfg_.c.dt * 1000.0;
    checks_.fallback_cycles += rec.fallback_active;
    // eq. (25): kappa <= lambda except in fallback; [fix R2a] with memory: kappa <= kappa_bound (decayed max of
    // lambda, or 1 after a fallback)
    const bool inv_ok = rec.lambda >= 0.0 && rec.lambda <= 1.0 && rec.kappa >= 0.0 && rec.kappa <= 1.0 &&
                        (core_cfg_.sw.fx.r2a_kappa_decay ? rec.kappa <= rec.kappa_bound + 1e-12
                                                         : (rec.fallback_active || rec.kappa <= rec.lambda + 1e-12));
    checks_.invariant_violations += !inv_ok;

    // ---- stall breaker (Plan B, node layer; the core result is unchanged, only the commanded v* is replaced) ----
    int stall_break = 0;
    const double goal_dist = (core_.goal() - p_ref).norm();
    if (rec.reached || goal_dist <= kStallGoalDist)
    {
      stall_count_ = 0; // goal (nearly) reached: no stall handling, hold cleared
      stall_hold_remaining_ = 0;
    }
    else if (stall_hold_remaining_ > 0)
    {
      // hold: keep commanding the stored escape velocity; the core's v* of this cycle is not used
      rec.v_star = stall_v_esc_;
      --stall_hold_remaining_;
      ++checks_.stall_hold_cycles;
      stall_count_ = 0;
    }
    else
    {
      if (rec.v_star.norm() < 1e-6)
        ++stall_count_;
      else
        stall_count_ = 0;
      if (stall_count_ >= kStallCycles)
      {
        LocalProblem P = core_.lastLocalProblem();
        P.grid = grid.get();
        const Eigen::Vector3d v_esc = stallEscape(P);
        ROS_WARN("[WARN] stall-breaker triggered, force fallback escape: v* (%.3f, %.3f, %.3f) held for %d cycles, after "
                 "%d cycles at (%.2f, %.2f, %.2f), %.2f m from the goal",
                 v_esc.x(), v_esc.y(), v_esc.z(), kStallHoldCycles, stall_count_, p_ref.x(), p_ref.y(), p_ref.z(), goal_dist);
        rec.v_star = v_esc;
        stall_break = 1;
        ++checks_.stall_breaks;
        stall_v_esc_ = v_esc;
        stall_hold_remaining_ = v_esc.norm() > 1e-6 ? kStallHoldCycles - 1 : 0; // this cycle is the first of the hold
        stall_count_ = 0;
      }
    }

    // ---- publish ----
    UniformCubicBspline traj;
    if (rec.reached)
    {
      Eigen::Vector3d goal = core_.goal();
      buildStraightLine(p_ref, v_ref, goal, max_vel_, max_acc_, ctrl_pt_dt_, traj); // settle exactly on the goal
      ROS_INFO("[jps_vo] goal reached at t = %.2f s after the goal", now.toSec() - goal_t0_);
      have_goal_ = false;
      core_.clearGoal();
    }
    else
      traj = buildVelocityBridge(p_ref, v_ref, rec.v_star, max_acc_, core_cfg_.c.tau_h, ctrl_pt_dt_); // [ENG-2]

    Eigen::Vector3d p0, v0;
    traj.evaluate(0.0, p0, v0);
    const double jump = (p0 - p_ref).norm(); // checklist B-3: start == previous reference (< 0.01 m)
    checks_.max_start_jump = std::max(checks_.max_start_jump, jump);
    checks_.continuity_violations += jump >= 0.01;
    checks_.format_violations += !formatOk(traj);
    publishTrajectory(traj, now);

    // ---- logs ----
    writeCycleRow(rec, stall_break, stall_hold_remaining_);
    writePathEvents(core_.takePathEvents());
    if (core_cfg_.sw.fx.any())
      writeSwitchDiags(core_.takeSwitchDiags());
    if (!pi0_written_ && logs_open_)
    {
      const GlobalPath pi0 = core_.pi0();
      if (!pi0.empty())
      {
        pi0_csv_ << "x,y,z\n";
        for (const auto &q : pi0.points())
          pi0_csv_ << num(q.x()) << "," << num(q.y()) << "," << num(q.z()) << "\n";
        pi0_csv_.flush();
        pi0_written_ = true;
      }
    }
    if (rec.reached)
      writeChecks();
  }

  Eigen::Vector3d PlannerNode::stallEscape(const LocalProblem &P) const
  {
    // Fallback (40) via the core's public API: argmax over V_adm of min_o t_c, tie-break ||v - v_pref||,
    // restricted to statically free candidates [ENG-6] and to MOVING candidates (v = 0 is what stalled).
    const LocalPlanner lp(core_cfg_.c, core_cfg_.sw);
    const auto cand = lp.candidates(P);
    const Eigen::Vector3d to_goal = P.goal - P.p_ref;
    const Eigen::Vector3d v_pref =
        to_goal.norm() > 1e-9 ? Eigen::Vector3d(core_cfg_.c.v_c * to_goal.normalized()) : Eigen::Vector3d::Zero();
    bool found = false;
    double best_t = -1.0, best_tie = 0.0;
    Eigen::Vector3d best = Eigen::Vector3d::Zero();
    for (const auto &v : cand)
    {
      if (v.norm() < 1e-3 || !lp.staticOk(P, v))
        continue;
      const double t = lp.minTtc(P, v), tie = (v - v_pref).norm();
      const bool better = !found || t > best_t + 1e-9 || ((std::fabs(t - best_t) <= 1e-9 || (std::isinf(t) && std::isinf(best_t))) && tie < best_tie - 1e-12);
      if (better)
      {
        best = v;
        best_t = t;
        best_tie = tie;
        found = true;
      }
    }
    if (found)
      return best;
    // no moving candidate is statically free: take the moving candidate with the longest free run along its
    // direction (sampled up to 1 m at delta / 2)
    double best_free = -1.0;
    for (const auto &v : cand)
    {
      if (v.norm() < 1e-3 || !P.grid)
        continue;
      const Eigen::Vector3d u = v.normalized();
      double free_len = 0.0;
      for (double l = 0.05; l <= 1.0; l += 0.05)
      {
        if (!P.grid->isFree(Eigen::Vector3d(P.p_ref + u * l)))
          break;
        free_len = l;
      }
      if (free_len > best_free)
      {
        best_free = free_len;
        best = v;
      }
    }
    return best;
  }

  bool PlannerNode::formatOk(const UniformCubicBspline &traj) const
  {
    const size_t m = traj.ctrl_pts.size();
    if (m < 4 || traj.knots().size() != m + 4) // B-1
      return false;
    if ((traj.ctrl_pts[m - 1] - traj.ctrl_pts[m - 3]).norm() > 1e-9) // B-4
      return false;
    const double dt = traj.dt;
    for (size_t i = 0; i + 1 < m; ++i) // B-5 (velocity, 1 % tolerance for the straight-line settle)
      if ((traj.ctrl_pts[i + 1] - traj.ctrl_pts[i]).norm() / dt > max_vel_ * 1.01 + 1e-9)
        return false;
    return true;
  }

  // ---------------------------------------------------------------- logging

  void PlannerNode::openLogs()
  {
    if (logs_open_ || log_dir_.empty())
      return;
    ::mkdir(log_dir_.c_str(), 0775);
    cycles_csv_.open(log_dir_ + "/cycles.csv");
    paths_csv_.open(log_dir_ + "/paths.csv");
    pi0_csv_.open(log_dir_ + "/pi0.csv");
    odom_gt_csv_.open(log_dir_ + "/odom_gt.csv");
    if (!cycles_csv_ || !paths_csv_ || !pi0_csv_ || !odom_gt_csv_)
    {
      ROS_ERROR("[jps_vo] cannot open log files in '%s'", log_dir_.c_str());
      return;
    }
    // spec section 8: Subtask-3 fields, then the 5 extension fields
    cycles_csv_ << "t,mode,plan_ms_global,plan_ms_local,"
                   "p_ref_x,p_ref_y,p_ref_z,v_ref_x,v_ref_y,v_ref_z,"
                   "p_odom_x,p_odom_y,p_odom_z,"
                   "v_star_x,v_star_y,v_star_z,"
                   "lambda,xi,s_g_star,kappa,"
                   "path_id,path_hash,n_jump_pts,"
                   "min_surface_dist_ref,min_surface_dist_odom,"
                   "turn_angle,lateral_dev_ref_to_Pi,"
                   "lateral_dev_ref_to_Pi0,replan_reason,fallback_active,nearest_dyn_obs,encounter_flag,"
                   "stall_break_triggered,stall_hold_remaining\n";
    paths_csv_ << "t,path_id,path_hash,replan_reason,is_switch,idx,x,y,z\n";
    odom_gt_csv_ << "t,odom_x,odom_y,odom_z,obs_name,obs_x,obs_y,obs_z,obs_vx,obs_vy,obs_vz,obs_r\n";
    logs_open_ = true;
    if (core_cfg_.sw.fx.any())
    {
      // [fixes] extended diagnostics, separate files so that the frozen CSV contract stays byte-identical
      cycles_ext_csv_.open(log_dir_ + "/cycles_ext.csv");
      switches_ext_csv_.open(log_dir_ + "/switches_ext.csv");
      cycles_ext_csv_ << "t,t_g_x,t_g_y,t_g_z,ang_vprev_tg_deg,reach_turn_deg,theta_eff_deg,ang_excess_deg,s_g_star,"
                         "J,J_A,J_B,Gamma,cone_term,rho_star,zero_is_opt,ray_feasible,free_mode,e_k,kappa_on_ek,"
                         "kappa_map_size,kappa_bound,budget_active,fallback_active\n";
      switches_ext_csv_ << "t,reason,is_switch,accepted,forced,C_old,C_new,dwell_left,up,down,n_on,n_off,dwell_applied,evidence,safety_override,d_old_pred,d_new_pred\n";
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
                        << num(d.down) << "," << d.n_on << "," << d.n_off << "," << d.dwell_applied << ","
                        << num(d.evidence) << "," << d.safety_override << "," << num(d.d_old_pred) << "," << num(d.d_new_pred)
                        << "\n";
    if (!diags.empty())
      switches_ext_csv_.flush();
  }

  void PlannerNode::writeMeta()
  {
    if (log_dir_.empty())
      return;
    std::ofstream f(log_dir_ + "/planner_meta.yaml");
    const auto &c = core_cfg_.c;
    const auto &sw = core_cfg_.sw;
    const Eigen::Vector3d g = core_.goal();
    f << std::setprecision(9);
    f << "group: " << core_cfg_.label << "\n";
    f << "goal: [" << g.x() << ", " << g.y() << ", " << g.z() << "]\n";
    f << "t0: " << tnum(goal_t0_) << "\nL_s: " << core_.Ls() << "\n";
    f << "constants: {r_q: " << c.r_q << ", e_cut: " << c.e_cut << ", v_c: " << c.v_c << ", dt: " << c.dt
      << ", tau_h: " << c.tau_h << ", m: " << c.m << ", L_min: " << c.L_min << ", psi_max_deg: " << c.psi_max * 180 / M_PI
      << ", theta_max_deg: " << c.theta_max * 180 / M_PI << ", v_min: " << c.v_min << ", w_theta: " << c.w_theta
      << ", w_a: " << c.w_a << ", rho_g: " << c.rho_g << ", eps_w: " << c.eps_w << ", T_c: " << c.T_c << ", T_g: " << c.T_g
      << ", dv_star: " << c.dv_star << ", v_max: " << c.v_max << ", a_max: " << c.a_max << ", d_min: " << c.d_min
      << ", r_o_max: " << c.r_o_max << ", tau1: " << c.tau1 << ", tau2: " << c.tau2 << ", r_infl: " << c.rInfl()
      << ", ell_w: " << c.ellW() << "}\n";
    f << "switches: {lambda_mode: " << static_cast<int>(sw.lambda_mode) << ", lambda_const: " << sw.lambda_const
      << ", use_rho_in_pure_vo: " << sw.use_rho_in_pure_vo << ", enable_c2: " << sw.enable_c2
      << ", enable_c3: " << sw.enable_c3 << ", c3_soft_tracking: " << sw.c3_soft_tracking
      << ", enable_gamma: " << sw.enable_gamma << ", lambda_shape: " << static_cast<int>(sw.lambda_shape)
      << ", theta_c_mode: " << static_cast<int>(sw.theta_c_mode) << ", xi_source: " << static_cast<int>(sw.xi_source) << "}\n";
    f << "engineering: {tie_rule: " << (core_cfg_.tie_rule == Jps3D::TieRule::Strict ? "strict" : "literal")
      << ", search_margin: " << core_cfg_.search_margin << ", async_global: " << async_global_
      << ", replan_period: " << core_cfg_.replan_period << ", switch_threshold: " << core_cfg_.switch_threshold
      << ", pre_rotate_yaw: " << pre_rotate_ << "}\n";
    if (sw.fx.any()) // [fixes] written only when a flag is on, so the frozen meta file stays byte-identical
      f << "fixes: {r1a_reachable_cone: " << sw.fx.r1a_reachable_cone << ", r1b_continuous_slack: " << sw.fx.r1b_continuous_slack
        << ", r1c_heading_memory: " << sw.fx.r1c_heading_memory << ", r1d_feasible_cone: " << sw.fx.r1d_feasible_cone
        << ", r1e_lowspeed_angle: " << sw.fx.r1e_lowspeed_angle
        << ", r2a_kappa_decay: " << sw.fx.r2a_kappa_decay
        << ", r2b_kappa_local_excluded: " << sw.fx.r2b_kappa_local_excluded
        << ", r3_relevant_trigger: " << sw.fx.r3_relevant_trigger << ", r4_switch_hysteresis: " << sw.fx.r4_switch_hysteresis
        << ", r4m_dwell_free_only: " << sw.fx.r4m_dwell_free_only << ", r4n_leaky_evidence: " << sw.fx.r4n_leaky_evidence
        << ", r4s_safety_override: " << sw.fx.r4s_safety_override << ", r5_mode_gating: " << sw.fx.r5_mode_gating << ", s1_eps_constraint: " << sw.fx.s1_eps_constraint
        << ", diag_log: " << sw.fx.diag_log << "}\n";
  }

  void PlannerNode::writeCycleRow(const CycleRecord &r, int stall_break, int stall_hold)
  {
    if (!logs_open_)
      return;
    auto &f = cycles_csv_;
    f << tnum(r.t) << "," << r.mode << "," << num(r.plan_ms_global) << "," << num(r.plan_ms_local) << ","
      << num(r.p_ref.x()) << "," << num(r.p_ref.y()) << "," << num(r.p_ref.z()) << "," << num(r.v_ref.x()) << ","
      << num(r.v_ref.y()) << "," << num(r.v_ref.z()) << "," << num(r.p_odom.x()) << "," << num(r.p_odom.y()) << ","
      << num(r.p_odom.z()) << "," << num(r.v_star.x()) << "," << num(r.v_star.y()) << "," << num(r.v_star.z()) << ","
      << num(r.lambda) << "," << num(r.xi) << "," << num(r.s_g_star) << "," << num(r.kappa) << "," << r.path_id << ","
      << r.path_hash << "," << r.n_jump_pts << "," << num(r.min_surface_dist_ref) << "," << num(r.min_surface_dist_odom)
      << "," << num(r.turn_angle) << "," << num(r.lateral_dev_ref_to_Pi) << "," << num(r.lateral_dev_ref_to_Pi0) << ","
      << r.replan_reason << "," << (r.fallback_active ? 1 : 0) << "," << r.nearest_dyn_obs << "," << r.encounter_flag << ","
      << stall_break << "," << stall_hold
      << "\n";
    if (ext_open_)
    {
      auto &x = cycles_ext_csv_;
      const double deg = 180.0 / M_PI;
      x << tnum(r.t) << "," << num(r.t_g.x()) << "," << num(r.t_g.y()) << "," << num(r.t_g.z()) << ","
        << num(r.ang_vprev_tg * deg) << "," << num(r.reach_turn * deg) << "," << num(r.theta_eff * deg) << ","
        << num(r.ang_excess * deg) << "," << num(r.s_g_star) << "," << num(r.J) << "," << num(r.J_A) << "," << num(r.J_B)
        << "," << num(r.Gamma) << "," << num(r.cone_term) << "," << num(r.rho_star) << "," << r.zero_is_opt << ","
        << r.ray_feasible << "," << r.free_mode << "," << r.e_k << "," << num(r.kappa_on_ek) << "," << r.kappa_map_size
        << "," << num(r.kappa_bound) << "," << r.budget_active << "," << (r.fallback_active ? 1 : 0) << "\n";
    }
  }

  void PlannerNode::writePathEvents(const std::vector<PathEvent> &events)
  {
    if (!logs_open_)
      return;
    for (const auto &e : events)
      for (size_t i = 0; i < e.pts.size(); ++i)
        paths_csv_ << tnum(e.t) << "," << e.path_id << "," << e.hash << "," << e.reason << "," << (e.is_switch ? 1 : 0)
                   << "," << i << "," << num(e.pts[i].x()) << "," << num(e.pts[i].y()) << "," << num(e.pts[i].z()) << "\n";
    if (!events.empty())
      paths_csv_.flush();
  }

  void PlannerNode::odomGtCallback(const ros::TimerEvent &)
  {
    if (!logs_open_ || !have_odom_)
      return;
    const ros::Time now = ros::Time::now();
    const auto obs = map_input_.obstacleStates(now);
    const std::string head = tnum(now.toSec()) + "," + num(odom_pos_.x()) + "," + num(odom_pos_.y()) + "," + num(odom_pos_.z());
    if (obs.empty())
      odom_gt_csv_ << head << ",-,,,,,,,\n";
    for (const auto &o : obs)
      odom_gt_csv_ << head << "," << o.name << "," << num(o.p.x()) << "," << num(o.p.y()) << "," << num(o.p.z()) << ","
                   << num(o.v.x()) << "," << num(o.v.y()) << "," << num(o.v.z()) << "," << num(o.r) << "\n";
  }

  void PlannerNode::writeChecks()
  {
    cycles_csv_.flush();
    odom_gt_csv_.flush();
    if (ext_open_)
    {
      cycles_ext_csv_.flush();
      switches_ext_csv_.flush();
    }
    if (log_dir_.empty())
      return;
    std::ofstream f(log_dir_ + "/checks.yaml");
    const auto &k = checks_;
    f << "cycles: " << k.cycles << "\ncontinuity_violations: " << k.continuity_violations
      << "\nmax_start_jump_m: " << k.max_start_jump << "\nformat_violations: " << k.format_violations
      << "\ncycles_over_dt: " << k.cycles_over_dt << "\nmax_local_ms: " << k.max_local_ms
      << "\nmax_global_ms: " << k.max_global_ms << "\nmax_cycle_gap_s: " << k.max_cycle_gap
      << "\nglobal_searches: " << k.global_searches << "\nglobal_fails: " << k.global_fails
      << "\nfallback_cycles: " << k.fallback_cycles << "\nstall_breaks: " << k.stall_breaks << "\nstall_hold_cycles: " << k.stall_hold_cycles << "\ninvariant_violations: " << k.invariant_violations
      << "\nno_cloud_cycles: " << k.no_cloud_cycles << "\nstale_obstacle_drops: " << map_input_.staleDrops()
      << "\ncloud_frame: '" << map_input_.cloudFrame() << "'\n";
  }

} // namespace jps_vo
