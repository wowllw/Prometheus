#ifndef JPS_VO_PLANNER_PLANNER_NODE_H
#define JPS_VO_PLANNER_PLANNER_NODE_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <prometheus_msgs/UAVControlState.h>

#include "jps_vo_planner/bspline_utils.h"
#include "jps_vo_planner/map_input.h"
#include "jps_vo_planner/planner_core.h"

namespace jps_vo
{
  class PlannerNode
  {
  public:
    explicit PlannerNode(ros::NodeHandle &nh);
    ~PlannerNode();

  private:
    // ---- callbacks ----
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    // RViz default "2D Nav Goal" (/move_base_simple/goal): z < 0.1 -> keep the current height
    void goalAltCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    // true while another node (uav1_ego_planner_node) could publish /uav1/planning/bspline; publishing is refused
    bool checkConflicts();
    void controlStateCallback(const prometheus_msgs::UAVControlStateConstPtr &msg);
    void replanTimerCallback(const ros::TimerEvent &e);
    void gateTimerCallback(const ros::TimerEvent &e);
    void diagTimerCallback(const ros::TimerEvent &e);

    // ---- planning ----
    // Reference state at time t: the trajectory currently executed by traj_server, or odom (vel = 0)
    // when no trajectory is active. New trajectories always start from this state (risk R-3).
    void referenceState(const ros::Time &t, Eigen::Vector3d &pos, Eigen::Vector3d &vel) const;
    bool planAndPublish(const char *reason);
    void publishTrajectory(const UniformCubicBspline &traj, const ros::Time &start_time);
    void publishGate(bool on);

    // ---- params ----
    int uav_id_;
    std::string planner_mode_;
    double max_vel_, max_acc_, ctrl_pt_dt_, replan_period_, goal_tolerance_;
    double map_x_min_, map_x_max_, map_y_min_, map_y_max_, map_z_min_, map_z_max_;

    // ---- state ----
    bool have_odom_ = false, have_goal_ = false, have_traj_ = false;
    bool in_command_ = false, first_command_entry_ = true, gate_on_ = false;
    int last_control_state_ = -1;
    Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero(), odom_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
    std::string odom_frame_, cloud_frame_;
    ros::Time last_odom_stamp_, last_cloud_stamp_;
    size_t odom_count_ = 0, cloud_count_ = 0, cloud_points_ = 0;

    UniformCubicBspline traj_;
    ros::Time traj_start_time_;
    long traj_id_ = 0;

    // ---- coupled mode (Subtask 3, planner_node_coupled.cpp) ----
    void initCoupled(ros::NodeHandle &nh);
    void coupledGoal(const Eigen::Vector3d &goal);
    void localCycleCallback(const ros::TimerEvent &e);
    void odomGtCallback(const ros::TimerEvent &e);
    void globalThreadLoop();
    void openLogs();
    void writeMeta();
    void writeCycleRow(const CycleRecord &r, int stall_break, int stall_hold);
    // stall breaker (node layer, always on for every group): v* == 0 for >= kStallCycles cycles while the
    // reference is > kStallGoalDist from the goal -> force the fallback (40) escape for this cycle
    Eigen::Vector3d stallEscape(const LocalProblem &P) const;
    static constexpr int kStallCycles = 12;         // 1.2 s at 10 Hz
    static constexpr double kStallGoalDist = 0.5;   // [m]
    int stall_count_ = 0;
    // hold: after a trigger the escape velocity is commanded for kStallHoldCycles cycles (0.6 s at 10 Hz) instead of
    // the core's v*, so the next core cycle cannot cancel it immediately
    static constexpr int kStallHoldCycles = 6;
    int stall_hold_remaining_ = 0;
    Eigen::Vector3d stall_v_esc_ = Eigen::Vector3d::Zero();
    void writePathEvents(const std::vector<PathEvent> &events);
    void writeChecks();
    bool formatOk(const UniformCubicBspline &traj) const; // checklist B-1 ... B-5

    bool coupled_ = false, async_global_ = true;
    // optional yaw pre-rotation before the first plan, as ego_planner does (param pre_rotate_yaw, default off)
    bool pre_rotate_ = false, prerot_active_ = false;
    double prerot_yaw_ = 0.0, odom_yaw_ = 0.0;
    ros::Time prerot_t0_;
    Eigen::Vector3d prerot_pos_ = Eigen::Vector3d::Zero(), pending_goal_ = Eigen::Vector3d::Zero();
    int uav_cmd_id_ = 0;
    ros::Publisher uav_cmd_pub_;
    CoreConfig core_cfg_;
    PlannerCore core_;
    MapInput map_input_;
    ros::Timer local_timer_, odom_gt_timer_;
    std::string log_dir_;
    std::ofstream cycles_csv_, paths_csv_, pi0_csv_, odom_gt_csv_;
    bool logs_open_ = false, pi0_written_ = false;
    double goal_t0_ = 0.0;
    ros::Time last_cycle_time_;

    // snapshot handed to the global thread, and its result handed back to the next local cycle [ENG-11]
    struct GlobalSnapshot
    {
      bool active = false;
      double t = 0.0;
      Eigen::Vector3d p_ref = Eigen::Vector3d::Zero(), v_ref = Eigen::Vector3d::Zero();
      std::vector<ObstacleState> obs;
      std::shared_ptr<const OccupancyGrid> grid;
    };
    std::mutex snap_mtx_;
    GlobalSnapshot snap_;
    GlobalStepResult pending_global_;
    std::thread global_thread_;
    std::atomic<bool> running_{false};

    // runtime verification counters (checks.yaml)
    struct Checks
    {
      size_t cycles = 0, continuity_violations = 0, format_violations = 0, cycles_over_dt = 0, fallback_cycles = 0;
      size_t invariant_violations = 0, no_cloud_cycles = 0, global_searches = 0, global_fails = 0, stall_breaks = 0;
      size_t stall_hold_cycles = 0;
      double max_start_jump = 0.0, max_local_ms = 0.0, max_global_ms = 0.0, max_cycle_gap = 0.0;
    } checks_;

    // ---- ROS ----
    ros::Subscriber odom_sub_, cloud_sub_, goal_sub_, goal_alt_sub_, control_state_sub_;
    bool conflict_ = false;
    ros::Publisher bspline_pub_, gate_pub_, path_pub_;
    ros::Timer replan_timer_, gate_timer_, diag_timer_;
  };

} // namespace jps_vo

#endif
