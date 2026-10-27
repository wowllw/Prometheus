#include "jps_vo_planner/planner_node.h"

#include <chrono>
#include <cmath>
#include <algorithm>

#include <ros/master.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <traj_utils/Bspline.h>

namespace jps_vo
{
  PlannerNode::PlannerNode(ros::NodeHandle &nh)
  {
    nh.param("uav_id", uav_id_, 1);
    nh.param<std::string>("planner_mode", planner_mode_, "stub");
    // v_max / a_max are the only motion limits calibrated in the simulation stage
    nh.param("max_vel", max_vel_, 0.8);
    nh.param("max_acc", max_acc_, 0.8);
    nh.param("ctrl_pt_dt", ctrl_pt_dt_, 0.1);
    // > 0: replan from the reference state at this period, to exercise trajectory replacement (risk R-3)
    nh.param("replan_period", replan_period_, 1.0);
    nh.param("goal_tolerance", goal_tolerance_, 0.2);
    // same map box as ego_planner (sitl_ego_planner_basic_octomap.launch)
    nh.param("map/x_min", map_x_min_, -25.0);
    nh.param("map/x_max", map_x_max_, 25.0);
    nh.param("map/y_min", map_y_min_, -25.0);
    nh.param("map/y_max", map_y_max_, 25.0);
    nh.param("map/z_min", map_z_min_, 0.5);
    nh.param("map/z_max", map_z_max_, 3.0);

    if (max_vel_ > 1.0)
    {
      // PX4 COMMAND_CONTROL caps horizontal speed at 1.0 m/s (MPC_XY_VEL_MAX set by uav_control)
      ROS_WARN("[jps_vo] max_vel=%.2f exceeds the PX4 COMMAND cap 1.0 m/s, clamped", max_vel_);
      max_vel_ = 1.0;
    }
    if (planner_mode_ != "stub" && planner_mode_ != "coupled")
    {
      ROS_WARN("[jps_vo] unknown planner_mode '%s', falling back to 'stub'", planner_mode_.c_str());
      planner_mode_ = "stub";
    }
    coupled_ = planner_mode_ == "coupled";

    odom_sub_ = nh.subscribe("odom", 10, &PlannerNode::odomCallback, this, ros::TransportHints().tcpNoDelay());
    cloud_sub_ = nh.subscribe("cloud", 1, &PlannerNode::cloudCallback, this);
    goal_sub_ = nh.subscribe("goal", 1, &PlannerNode::goalCallback, this);
    goal_alt_sub_ = nh.subscribe("goal_alt", 1, &PlannerNode::goalAltCallback, this);
    control_state_sub_ = nh.subscribe("control_state", 1, &PlannerNode::controlStateCallback, this);

    bspline_pub_ = nh.advertise<traj_utils::Bspline>("bspline", 10);
    gate_pub_ = nh.advertise<std_msgs::Bool>("gate", 10);
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 1, true);

    if (coupled_)
      initCoupled(nh); // Subtask 3: 3D-JPS + VO coupled planner (planner_node_coupled.cpp)
    else if (replan_period_ > 0.0)
      replan_timer_ = nh.createTimer(ros::Duration(replan_period_), &PlannerNode::replanTimerCallback, this);
    gate_timer_ = nh.createTimer(ros::Duration(0.1), &PlannerNode::gateTimerCallback, this);
    diag_timer_ = nh.createTimer(ros::Duration(2.0), &PlannerNode::diagTimerCallback, this);

    ROS_WARN("[jps_vo] ready: mode=%s v_max=%.2f a_max=%.2f dt=%.2f replan_period=%.2f",
             planner_mode_.c_str(), max_vel_, max_acc_, ctrl_pt_dt_, replan_period_);
    bool sim_time = false;
    if (ros::param::get("/use_sim_time", sim_time) && sim_time)
      ROS_WARN("[jps_vo] /use_sim_time is true: the Prometheus stack and all time stamps here assume wall clock");
    checkConflicts();
  }

  // ---------------------------------------------------------------- callbacks

  void PlannerNode::odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_vel_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    const auto &q = msg->pose.pose.orientation;
    odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_frame_ = msg->header.frame_id;
    last_odom_stamp_ = msg->header.stamp;
    ++odom_count_;
    have_odom_ = true;
  }

  void PlannerNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    // stub: only check that the map input arrives and in which frame; the 0.1 m grid comes in Subtask 3
    cloud_frame_ = msg->header.frame_id;
    last_cloud_stamp_ = msg->header.stamp;
    cloud_points_ = static_cast<size_t>(msg->width) * msg->height;
    ++cloud_count_;
  }

  void PlannerNode::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    // same convention as ego_planner: z < -0.1 is ignored
    if (msg->pose.position.z < -0.1)
      return;
    if (!have_odom_ || !in_command_)
    {
      ROS_WARN("[jps_vo] goal rejected: have_odom=%d in_command=%d (run arm_and_command.sh and wait for takeoff)",
               have_odom_, in_command_);
      return;
    }

    Eigen::Vector3d g(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (g.x() < map_x_min_ || g.x() > map_x_max_ || g.y() < map_y_min_ || g.y() > map_y_max_)
    {
      ROS_WARN("[jps_vo] goal (%.2f, %.2f) outside the map box, rejected", g.x(), g.y());
      return;
    }
    const double z = std::min(std::max(g.z(), map_z_min_), map_z_max_);
    if (z != g.z())
      ROS_WARN("[jps_vo] goal z %.2f clamped to %.2f", g.z(), z);
    g.z() = z;

    goal_ = g;
    have_goal_ = true;
    ROS_INFO("[jps_vo] new goal [%.2f, %.2f, %.2f]", goal_.x(), goal_.y(), goal_.z());
    if (coupled_)
      coupledGoal(g);
    else
      planAndPublish("new goal");
  }

  void PlannerNode::goalAltCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    geometry_msgs::PoseStamped g = *msg;
    if (g.pose.position.z < 0.1)
      g.pose.position.z = odom_pos_.z(); // 2D goal: keep the current flight height
    goalCallback(geometry_msgs::PoseStampedConstPtr(new geometry_msgs::PoseStamped(g)));
  }

  bool PlannerNode::checkConflicts()
  {
    ros::V_string nodes;
    if (!ros::master::getNodes(nodes))
      return conflict_;
    bool found = false;
    for (const auto &n : nodes)
      if (n.find("ego_planner_node") != std::string::npos)
      {
        found = true;
        ROS_ERROR_THROTTLE(5.0, "[jps_vo] node %s is running: it publishes the same /uav1/planning/bspline and gate "
                                "topics. jps_vo will not publish until it is stopped.", n.c_str());
      }
    if (conflict_ && !found)
      ROS_WARN("[jps_vo] node conflict cleared");
    conflict_ = found;
    return conflict_;
  }

  void PlannerNode::controlStateCallback(const prometheus_msgs::UAVControlStateConstPtr &msg)
  {
    const int state = msg->control_state;
    if (state == last_control_state_)
      return;

    if (state == prometheus_msgs::UAVControlState::COMMAND_CONTROL)
    {
      in_command_ = true;
      if (first_command_entry_)
      {
        // first entry: uav_control is taking off (Init_Pos_Hover). Publishing anything here would make
        // traj_server override the takeoff, so wait for the first goal (same as ego_replan_fsm).
        first_command_entry_ = false;
        ROS_INFO("[jps_vo] entered COMMAND_CONTROL (first time), waiting for goal");
      }
      else
      {
        // re-entry: hold the current position instead of resuming an old trajectory
        have_goal_ = false;
        if (coupled_)
          core_.clearGoal();
        publishTrajectory(buildHover(odom_pos_, ctrl_pt_dt_), ros::Time::now());
        ROS_INFO("[jps_vo] re-entered COMMAND_CONTROL, holding [%.2f, %.2f, %.2f]", odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
      }
    }
    else if (last_control_state_ == prometheus_msgs::UAVControlState::COMMAND_CONTROL)
    {
      in_command_ = false;
      have_goal_ = false;
      have_traj_ = false;
      if (coupled_)
        core_.clearGoal();
      publishGate(false);
      ROS_WARN("[jps_vo] left COMMAND_CONTROL (state=%d), gate off", state);
    }
    last_control_state_ = state;
  }

  void PlannerNode::replanTimerCallback(const ros::TimerEvent &)
  {
    if (!in_command_ || !have_goal_ || !have_traj_)
      return;

    Eigen::Vector3d p, v;
    referenceState(ros::Time::now(), p, v);
    if ((goal_ - p).norm() < goal_tolerance_)
    {
      have_goal_ = false;
      ROS_INFO("[jps_vo] goal reached");
      return;
    }
    planAndPublish("periodic");
  }

  void PlannerNode::gateTimerCallback(const ros::TimerEvent &)
  {
    // traj_server only forwards commands while the gate is true; keep re-sending it while a trajectory
    // exists so one lost message cannot freeze the vehicle
    if (in_command_ && have_traj_)
      publishGate(true);
  }

  void PlannerNode::diagTimerCallback(const ros::TimerEvent &)
  {
    const ros::Time now = ros::Time::now();
    checkConflicts();
    ROS_INFO("[jps_vo][diag] odom: n=%zu frame='%s' age=%.3fs | cloud: n=%zu pts=%zu frame='%s' age=%.3fs | "
             "command=%d gate=%d traj=%d | mode=%s group=%s conflict=%d",
             odom_count_, odom_frame_.c_str(), have_odom_ ? (now - last_odom_stamp_).toSec() : -1.0,
             cloud_count_, cloud_points_, cloud_frame_.c_str(), cloud_count_ ? (now - last_cloud_stamp_).toSec() : -1.0,
             in_command_, gate_on_, have_traj_, planner_mode_.c_str(), coupled_ ? core_cfg_.label.c_str() : "-", conflict_);
  }

  // ---------------------------------------------------------------- planning

  void PlannerNode::referenceState(const ros::Time &t, Eigen::Vector3d &pos, Eigen::Vector3d &vel) const
  {
    if (have_traj_ && traj_.valid())
    {
      // after the end, traj_server holds the last point with zero velocity
      traj_.evaluate((t - traj_start_time_).toSec(), pos, vel);
      if ((t - traj_start_time_).toSec() >= traj_.duration())
        vel.setZero();
      return;
    }
    pos = odom_pos_;
    vel.setZero();
  }

  bool PlannerNode::planAndPublish(const char *reason)
  {
    const ros::Time now = ros::Time::now();
    Eigen::Vector3d p0, v0;
    referenceState(now, p0, v0);

    const auto t0 = std::chrono::steady_clock::now();
    UniformCubicBspline traj;
    const bool ok = buildStraightLine(p0, v0, goal_, max_vel_, max_acc_, ctrl_pt_dt_, traj);
    const double plan_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    if (!ok)
    {
      ROS_ERROR("[jps_vo] straight-line generation failed (%s)", reason);
      return false;
    }

    // continuity check: the new trajectory must start where the reference is now
    Eigen::Vector3d np, nv;
    traj.evaluate(0.0, np, nv);
    ROS_INFO("[jps_vo] plan (%s): plan_time_ms=%.3f duration=%.2fs ctrl_pts=%zu | start-ref jump: pos=%.4fm vel=%.4fm/s | "
             "ref-odom=%.3fm",
             reason, plan_ms, traj.duration(), traj.ctrl_pts.size(), (np - p0).norm(), (nv - v0).norm(),
             (p0 - odom_pos_).norm());

    publishTrajectory(traj, now);
    return true;
  }

  void PlannerNode::publishTrajectory(const UniformCubicBspline &traj, const ros::Time &start_time)
  {
    if (conflict_)
    {
      ROS_ERROR_THROTTLE(2.0, "[jps_vo] trajectory NOT published: ego_planner_node conflict");
      return;
    }
    // start_time must not be in the future: traj_server evaluates t = now - start_time and, for t < 0,
    // commands position (0, 0, 0)
    const ros::Time now = ros::Time::now();
    const ros::Time stamp = start_time > now ? now : start_time;

    traj_utils::Bspline msg;
    msg.drone_id = uav_id_;
    msg.order = 3;
    msg.traj_id = ++traj_id_;
    msg.start_time = stamp;
    msg.knots = traj.knots();
    msg.pos_pts.reserve(traj.ctrl_pts.size());
    for (const auto &q : traj.ctrl_pts)
    {
      geometry_msgs::Point pt;
      pt.x = q.x();
      pt.y = q.y();
      pt.z = q.z();
      msg.pos_pts.push_back(pt);
    }
    bspline_pub_.publish(msg);

    traj_ = traj;
    traj_start_time_ = stamp;
    have_traj_ = true;

    // gate is switched on only after the first trajectory has been sent, so traj_server never
    // replays a stale one; only the off -> on transition is sent here, the steady 10 Hz comes from gateTimerCallback
    if (in_command_ && !gate_on_)
      publishGate(true);

    nav_msgs::Path path;
    path.header.stamp = now;
    path.header.frame_id = "world";
    for (double t = 0.0; t <= traj.duration() + 1e-6; t += 0.1)
    {
      Eigen::Vector3d p, v;
      traj.evaluate(t, p, v);
      geometry_msgs::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_.publish(path);
  }

  void PlannerNode::publishGate(bool on)
  {
    if (on && conflict_)
      return; // never open the gate while another planner may feed traj_server
    std_msgs::Bool b;
    b.data = on;
    gate_pub_.publish(b);
    gate_on_ = on;
  }

} // namespace jps_vo
