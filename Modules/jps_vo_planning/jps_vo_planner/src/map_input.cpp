#include "jps_vo_planner/map_input.h"

#include <chrono>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace jps_vo
{
  void MapInput::init(ros::NodeHandle &nh, const Params &p)
  {
    p_ = p;
    auto g = std::make_shared<OccupancyGrid>();
    g->init(p_.grid);
    g->rebuild({}, {});
    grid_ = g;
    cloud_sub_ = nh.subscribe("cloud", 1, &MapInput::cloudCallback, this);
    // model_states arrives at the physics rate (~1 kHz): queue 1, cheap callback
    model_sub_ = nh.subscribe("/gazebo/model_states", 1, &MapInput::modelStatesCallback, this,
                              ros::TransportHints().tcpNoDelay());
  }

  void MapInput::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (msg->header.frame_id != p_.world_frame)
      ROS_WARN_THROTTLE(5.0, "[jps_vo] cloud frame '%s' != '%s', points used without transform",
                        msg->header.frame_id.c_str(), p_.world_frame.c_str());
    latest_cloud_ = msg;
    cloud_frame_ = msg->header.frame_id;
    have_cloud_ = cloud_dirty_ = true;
  }

  void MapInput::modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
  {
    // ModelStates has no header: stamp with the receive time (wall clock, use_sim_time=false)
    const ros::Time now = ros::Time::now();
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      const std::string &name = msg->name[i];
      if (name.compare(0, p_.dyn_prefix.size(), p_.dyn_prefix) != 0)
        continue;
      DynObstacle &o = dyn_[name];
      o.name = name;
      const auto &pp = msg->pose[i].position;
      const auto &tw = msg->twist[i].linear;
      o.p << pp.x, pp.y, pp.z;
      o.v << tw.x, tw.y, tw.z;
      auto it = p_.dyn_radius.find(name);
      o.r = it != p_.dyn_radius.end() ? it->second : p_.default_radius;
      o.stamp = now;

      // swept-volume history at 10 Hz is enough for masking
      auto &h = dyn_history_[name];
      if (h.empty() || (now - h.back().first).toSec() >= 0.1)
        h.emplace_back(now, o.p);
      while (!h.empty() && (now - h.front().first).toSec() > p_.mask_history)
        h.pop_front();
    }
  }

  std::vector<DynObstacle> MapInput::obstacles(const ros::Time &now) const
  {
    std::vector<DynObstacle> out;
    for (const auto &kv : dyn_)
    {
      if ((now - kv.second.stamp).toSec() > p_.obstacle_timeout)
      {
        ++stale_drops_;
        ROS_WARN_THROTTLE(2.0, "[jps_vo] obstacle '%s' state is %.2fs old, ignored", kv.first.c_str(),
                          (now - kv.second.stamp).toSec());
        continue;
      }
      out.push_back(kv.second);
    }
    return out;
  }

  std::shared_ptr<const OccupancyGrid> MapInput::grid() const
  {
    std::lock_guard<std::mutex> lk(grid_mtx_);
    return grid_;
  }

  std::vector<ObstacleState> MapInput::obstacleStates(const ros::Time &now) const
  {
    std::vector<ObstacleState> out;
    for (const auto &o : obstacles(now))
    {
      ObstacleState s;
      s.name = o.name;
      s.p = o.p + o.v * (now - o.stamp).toSec();
      s.v = o.v;
      s.r = o.r;
      out.push_back(s);
    }
    return out;
  }

  std::vector<OccupancyGrid::Capsule> MapInput::masks(const ros::Time &now) const
  {
    std::vector<OccupancyGrid::Capsule> caps;
    for (const auto &kv : dyn_history_)
    {
      const auto &h = kv.second;
      if (h.empty())
        continue;
      auto it = p_.dyn_radius.find(kv.first);
      const double r = (it != p_.dyn_radius.end() ? it->second : p_.default_radius) + p_.mask_margin;
      // consecutive history points + the latest state form a polyline of capsules
      for (size_t i = 0; i + 1 < h.size(); ++i)
        caps.push_back({h[i].second, h[i + 1].second, r});
      const auto cur = dyn_.find(kv.first);
      const Eigen::Vector3d last = cur != dyn_.end() ? cur->second.p : h.back().second;
      caps.push_back({h.back().second, last, r});
    }
    (void)now;
    return caps;
  }

  bool MapInput::update(const ros::Time &now)
  {
    if (!cloud_dirty_ || !latest_cloud_)
      return false;
    if (p_.rebuild_rate > 0.0 && !last_rebuild_.isZero() && (now - last_rebuild_).toSec() < 1.0 / p_.rebuild_rate)
      return false;

    const auto t0 = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(static_cast<size_t>(latest_cloud_->width) * latest_cloud_->height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*latest_cloud_, "x"), iy(*latest_cloud_, "y"), iz(*latest_cloud_, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz)
      if (std::isfinite(*ix) && std::isfinite(*iy) && std::isfinite(*iz))
        pts.emplace_back(*ix, *iy, *iz);

    auto g = std::make_shared<OccupancyGrid>();
    g->init(p_.grid);
    g->rebuild(pts, masks(now));
    {
      std::lock_guard<std::mutex> lk(grid_mtx_);
      grid_ = g;
    }
    last_rebuild_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    last_rebuild_ = now;
    cloud_dirty_ = false;
    return true;
  }

} // namespace jps_vo
