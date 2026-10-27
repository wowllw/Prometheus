#ifndef _NEW_TEST_DASC_MANAGER_H_
#define _NEW_TEST_DASC_MANAGER_H_

// DASC: Drift-Adaptive Safety Corridor (primary innovation)
//  - drift_cov_update : odometry covariance -> dynamic safety margin (buffer radius rho)
//  - speed_adapt_law  : rho -> speed scaling factor kappa and dynamic max velocity
//  - buildFallbackTail: previous trajectory tail, used when the optimization fails
//
// All hooks that use this class are wrapped in USE_DASC / USE_DASC_FALLBACK
// (disabled by default). This file has no effect unless a hook calls it.

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/plan_container.hpp>

namespace ego_planner
{

  // Result of one DASC evaluation (one planning cycle)
  struct DascOutput
  {
    double rho;     // dynamic safety margin / buffer radius [m]
    double kappa;   // speed scaling factor in [kappa_min, 1]
    double v_dyn;   // dynamic maximum velocity = kappa * nominal max velocity [m/s]
    bool valid;     // false if no odometry covariance has been received yet
  };

  class DascManager
  {
  public:
    DascManager() {}
    ~DascManager() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* setup: reads parameters under "dasc/" and "grid_map/obstacles_inflation" */
    void init(ros::NodeHandle &nh, double nominal_max_vel);

    /* odometry interface: called from both odometry callbacks of EGOReplanFSM */
    void feedOdometry(const nav_msgs::OdometryConstPtr &msg);

    /* main interface: evaluates drift_cov_update and speed_adapt_law for this cycle */
    bool compute(const ros::Time &now, DascOutput &out);

    /* DASC functions */
    // Inputs : 3x3 position covariance (from odometry), time stamp of the sample
    // Output : buffer radius rho [m]; internal drift estimate is updated
    void drift_cov_update(const Eigen::Matrix3d &pos_cov, const ros::Time &stamp, double &rho);

    // Inputs : buffer radius rho [m]
    // Output : speed scaling factor kappa
    void speed_adapt_law(const double rho, double &kappa) const;

    /* fallback: tail of the previous trajectory when the optimizer fails */
    // Inputs : previous trajectory (planner_manager_->local_data_), current time
    // Output : bspline message to publish; returns false if there is no usable tail
    bool buildFallbackTail(const LocalTrajData &prev_traj, const ros::Time &now, traj_utils::Bspline &tail);

    /* statistics (for logging and the feasibility success rate) */
    inline int getFallbackCount(void) { return fallback_count_; }
    inline double getNominalMaxVel(void) { return nominal_max_vel_; }
    inline double getBaseMargin(void) { return base_margin_; }

  private:
    /* parameters */
    bool enable_drift_adapt_;    // toggle: covariance-driven margin
    bool enable_vel_adapt_;      // toggle: speed scaling
    double base_margin_;         // original fixed inflation distance [m]
    double nominal_max_vel_;     // original max velocity [m/s]
    double rho_min_, rho_max_;   // clamp of rho [m]
    double kappa_min_;           // lower bound of kappa
    double conf_factor_;         // confidence factor
    double cov_growth_, cov_decay_;
    double speed_law_gain_;
    double min_tail_duration_;   // minimum usable remainder of the previous trajectory [s]

    /* odometry data (written by the odometry callbacks, read by compute) */
    std::mutex odom_mutex_;
    Eigen::Matrix3d odom_pos_cov_;
    Eigen::Matrix3d odom_vel_cov_;
    ros::Time odom_stamp_;
    bool have_cov_{false};

    /* internal state */
    Eigen::Matrix3d drift_cov_;  // stored drift covariance estimate
    ros::Time last_update_stamp_;
    int fallback_count_{0};

  public:
    typedef std::unique_ptr<DascManager> Ptr;
  };

} // namespace ego_planner

#endif
