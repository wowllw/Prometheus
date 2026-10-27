#ifndef JPS_VO_PLANNER_MAP_INPUT_H
#define JPS_VO_PLANNER_MAP_INPUT_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/PointCloud2.h>

#include "jps_vo_planner/coupling.h"
#include "jps_vo_planner/occupancy_grid.h"

namespace jps_vo
{
  // Dynamic obstacle o of D2: bounding sphere (radius r_o) with constant, measured velocity v_o
  struct DynObstacle
  {
    std::string name;
    Eigen::Vector3d p, v;
    double r;
    ros::Time stamp;
  };

  // ROS side of the static map + ground-truth obstacle input:
  //  - /uav1/octomap_point_cloud_centers -> OccupancyGrid (lazy rebuild, at most rebuild_rate Hz)
  //  - /gazebo/model_states (names with prefix dyn_prefix) -> DynObstacle list, and the swept-volume masks that
  //    keep the depth-camera returns of moving obstacles (incl. octomap trails) out of the static grid
  class MapInput
  {
  public:
    struct Params
    {
      OccupancyGrid::Params grid;
      std::string world_frame = "world";
      std::string dyn_prefix = "dyn_obs_";
      std::map<std::string, double> dyn_radius; // r_o per model name; default_radius otherwise
      double default_radius = 0.3;
      double mask_margin = 0.15;   // added to r_o for masking (octomap voxel size + estimation error) [m]
      double mask_history = 15.0;  // how long a swept volume stays masked [s]
      double rebuild_rate = 5.0;   // [Hz]
      double obstacle_timeout = 0.5; // model_states older than this -> obstacle list considered stale [s]
    };

    void init(ros::NodeHandle &nh, const Params &p);

    // Rebuilds the grid if a newer cloud arrived and the rate limit allows it. Returns true if rebuilt.
    // The rebuild goes into a fresh grid that replaces the published snapshot (double buffering): readers holding
    // the previous snapshot (e.g. the global search thread) are never disturbed.
    bool update(const ros::Time &now);

    std::shared_ptr<const OccupancyGrid> grid() const;
    // Obstacles at their latest ground-truth state; empty if stale (logged)
    std::vector<DynObstacle> obstacles(const ros::Time &now) const;
    // Same, as model states at `now` (p extrapolated with the constant velocity, D2); counts stale drops
    std::vector<ObstacleState> obstacleStates(const ros::Time &now) const;
    size_t staleDrops() const { return stale_drops_; }
    bool haveCloud() const { return have_cloud_; }
    const std::string &cloudFrame() const { return cloud_frame_; }
    double lastRebuildMs() const { return last_rebuild_ms_; }

  private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg);
    std::vector<OccupancyGrid::Capsule> masks(const ros::Time &now) const;

    Params p_;
    std::shared_ptr<const OccupancyGrid> grid_;
    mutable std::mutex grid_mtx_;
    mutable size_t stale_drops_ = 0;
    ros::Subscriber cloud_sub_, model_sub_;

    sensor_msgs::PointCloud2ConstPtr latest_cloud_;
    bool have_cloud_ = false, cloud_dirty_ = false;
    std::string cloud_frame_;
    ros::Time last_rebuild_;
    double last_rebuild_ms_ = 0.0;

    std::map<std::string, DynObstacle> dyn_;
    std::map<std::string, std::deque<std::pair<ros::Time, Eigen::Vector3d>>> dyn_history_;
  };

} // namespace jps_vo

#endif
