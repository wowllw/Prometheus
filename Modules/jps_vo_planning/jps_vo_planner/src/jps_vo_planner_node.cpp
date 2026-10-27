// [File] jps_vo_planner_node.cpp - entry point of the jps_vo_planner ROS node.
//
// The ROS wrapper around the ROS-free core (coupling / jps3d / global_path / local_planner / planner_core /
// bspline_utils) is split over three translation units:
//   jps_vo_planner_node.cpp   main(): node handle, single-threaded ros::spin (all callbacks and timers serialised)
//   planner_node.cpp          topics, goal / control-state handling, reference state, B-spline + gate publishing
//                             (contract validated in Subtask 2), diagnostics and ego-node conflict detection
//   planner_node_coupled.cpp  planner_mode "coupled": parameter -> CoreConfig mapping, 10 Hz localStep on the
//                             main thread, globalStep in a background thread, CSV logging, runtime checks
//
// Threads and locks:
//   main thread    : every ROS callback and timer (odom, cloud, model_states, goal, control_state, 10 Hz local
//                    cycle, 10 Hz gate, 20 Hz odom/ground-truth log, 0.5 Hz diagnostics)
//   global thread  : loops at 1/Delta_t, calls PlannerCore::globalStep on a snapshot (p_ref, v_ref, obstacles,
//                    grid shared_ptr) taken under snap_mtx_; the result is handed back under the same mutex
//   PlannerCore    : its own mutex guards goal / path / pi0 / kappa / events; the G_J search runs unlocked on
//                    copies and is installed atomically (discarded if the goal changed meanwhile)
//   grid           : MapInput rebuilds into a fresh OccupancyGrid and swaps a shared_ptr, so the global thread
//                    keeps a consistent snapshot while the main thread rebuilds
//   No code path holds two of these mutexes at once, so there is no lock-order deadlock.
//
// Topics (names after the remaps of launch/jps_vo_planner.launch):
//   in : /uav1/mavros/local_position/odom, /uav1/octomap_point_cloud_centers, /gazebo/model_states,
//        /uav1/prometheus/motion_planning/goal (RViz "3D Nav Goal" of the Prometheus config),
//        /move_base_simple/goal (RViz default "2D Nav Goal"; z < 0.1 -> current height),
//        /uav1/prometheus/control_state
//   out: /uav1/planning/bspline (traj_utils/Bspline, order 3, start_time <= now),
//        /uav1/prometheus/command/ego_command_stop_pub (gate, 10 Hz while a trajectory is active),
//        /uav1/prometheus/command (only with pre_rotate_yaw), /uav1/jps_vo/path (RViz)
#include <ros/ros.h>
#include "jps_vo_planner/planner_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jps_vo_planner_node");
  ros::NodeHandle nh("~");

  jps_vo::PlannerNode node(nh);

  ros::spin();
  return 0;
}
