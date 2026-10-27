#include <plan_manage/new_test_dasc_manager.h>

namespace ego_planner
{

  // SECTION setup

  void DascManager::init(ros::NodeHandle &nh, double nominal_max_vel)
  {
    nominal_max_vel_ = nominal_max_vel;

    // toggles (runtime). The compile-time switches USE_DASC / USE_DASC_FALLBACK decide
    // whether this class is called at all; these decide what it does when called.
    nh.param("dasc/enable_drift_adapt", enable_drift_adapt_, false);
    nh.param("dasc/enable_vel_adapt", enable_vel_adapt_, false);

    // base margin = the original fixed inflation distance
    nh.param("grid_map/obstacles_inflation", base_margin_, 0.2);

    nh.param("dasc/rho_min", rho_min_, base_margin_);
    nh.param("dasc/rho_max", rho_max_, 3.0 * base_margin_);
    nh.param("dasc/kappa_min", kappa_min_, 0.3);
    nh.param("dasc/conf_factor", conf_factor_, 3.0);
    nh.param("dasc/cov_growth", cov_growth_, 1.0);
    nh.param("dasc/cov_decay", cov_decay_, 1.0);
    nh.param("dasc/speed_law_gain", speed_law_gain_, 1.0);
    nh.param("dasc/min_tail_duration", min_tail_duration_, 0.5);

    odom_pos_cov_.setZero();
    odom_vel_cov_.setZero();
    drift_cov_.setZero();
    have_cov_ = false;
    fallback_count_ = 0;

    ROS_INFO("[DASC] init: drift_adapt=%d, vel_adapt=%d, base_margin=%.3f, nominal_max_vel=%.3f",
             (int)enable_drift_adapt_, (int)enable_vel_adapt_, base_margin_, nominal_max_vel_);
  }

  // !SECTION

  // SECTION odometry interface

  void DascManager::feedOdometry(const nav_msgs::OdometryConstPtr &msg)
  {
    // nav_msgs/Odometry: pose.covariance and twist.covariance are 6x6 row-major
    // (x, y, z, rot_x, rot_y, rot_z). Only the translational 3x3 blocks are used.
    Eigen::Matrix3d pos_cov, vel_cov;
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        pos_cov(r, c) = msg->pose.covariance[r * 6 + c];
        vel_cov(r, c) = msg->twist.covariance[r * 6 + c];
      }
    }

    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_pos_cov_ = pos_cov;
    odom_vel_cov_ = vel_cov;
    odom_stamp_ = msg->header.stamp;
    have_cov_ = true;
  }

  // !SECTION

  // SECTION main interface

  bool DascManager::compute(const ros::Time &now, DascOutput &out)
  {
    // defaults reproduce the original behavior
    out.rho = base_margin_;
    out.kappa = 1.0;
    out.v_dyn = nominal_max_vel_;
    out.valid = false;

    Eigen::Matrix3d pos_cov;
    ros::Time stamp;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!have_cov_)
        return false;
      pos_cov = odom_pos_cov_;
      stamp = odom_stamp_;
    }

    if (enable_drift_adapt_)
    {
      drift_cov_update(pos_cov, stamp, out.rho);
    }

    if (enable_vel_adapt_)
    {
      speed_adapt_law(out.rho, out.kappa);
      out.v_dyn = out.kappa * nominal_max_vel_;
    }

    out.valid = true;
    return true;
  }

  // !SECTION

  // SECTION DASC functions

  void DascManager::drift_cov_update(const Eigen::Matrix3d &pos_cov, const ros::Time &stamp, double &rho)
  {
    // Inputs : pos_cov (3x3 position covariance of the odometry), stamp (sample time)
    // Output : rho (buffer radius, [rho_min_, rho_max_], never below base_margin_)
    // State  : drift_cov_ (stored drift estimate), last_update_stamp_

    /*TODO: Fill in using the corresponding formula from the paper*/
    // (1) fuse pos_cov with drift_cov_ using cov_growth_ / cov_decay_ and the elapsed time
    //     (stamp - last_update_stamp_)
    /*TODO: Fill in using the corresponding formula from the paper*/
    // (2) reduce the fused covariance to a scalar position uncertainty
    /*TODO: Fill in using the corresponding formula from the paper*/
    // (3) scale by conf_factor_ and clamp to [rho_min_, rho_max_]

    drift_cov_ = pos_cov; // placeholder state update
    last_update_stamp_ = stamp;
    rho = base_margin_;   // placeholder output: identical to the original fixed inflation
  }

  void DascManager::speed_adapt_law(const double rho, double &kappa) const
  {
    // Input  : rho (buffer radius)
    // Output : kappa in [kappa_min_, 1]; kappa = 1 when rho == base_margin_

    /*TODO: Fill in using the corresponding formula from the paper*/
    // kappa = f(rho, base_margin_, speed_law_gain_), then clamp to [kappa_min_, 1]

    (void)rho;
    kappa = 1.0; // placeholder output: no speed reduction
  }

  // !SECTION

  // SECTION fallback

  bool DascManager::buildFallbackTail(const LocalTrajData &prev_traj, const ros::Time &now, traj_utils::Bspline &tail)
  {
    // The previous trajectory stays in local_data_ because updateTrajInfo() is only
    // called on success, so after a failed optimization local_data_ is still the last good one.
    if (prev_traj.traj_id_ <= 0 || prev_traj.duration_ <= 0.0)
      return false; // no previous trajectory: original failure handling runs

    double t_elapsed = (now - prev_traj.start_time_).toSec();
    double t_remain = prev_traj.duration_ - t_elapsed;
    if (t_remain < min_tail_duration_)
      return false; // tail exhausted: original failure handling runs

    tail.order = 3;
    tail.start_time = prev_traj.start_time_;
    tail.traj_id = prev_traj.traj_id_;

    Eigen::MatrixXd pos_pts = prev_traj.position_traj_.getControlPoint();
    tail.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      tail.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = prev_traj.position_traj_.getKnot();
    tail.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      tail.knots.push_back(knots(i));
    }

    /*TODO: Fill in using the corresponding formula from the paper*/
    // Optional: cut the control points and knots so that the message starts at t_elapsed.
    // As written, the message is the unmodified previous trajectory with its original
    // start_time, so the part from the current time to the end is the tail that is followed.

    fallback_count_++;
    ROS_WARN("[DASC] optimization failed, following tail of previous trajectory (remaining %.2f s, count %d)",
             t_remain, fallback_count_);
    return true;
  }

  // !SECTION

} // namespace ego_planner
