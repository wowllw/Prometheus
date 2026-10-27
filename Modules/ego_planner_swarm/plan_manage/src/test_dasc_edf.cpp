// test_dasc_edf.cpp
// Unit tests for DascManager (new_test_dasc_manager.cpp) and FastEDFCorridor (new_test_fast_edf_corridor.cpp).
// No ROS master or node is needed: all inputs are simulated in memory.
//  - ROS_MASTER_URI is forced to an unused port, so every parameter read falls back to the coded
//    default (deterministic even when a roscore is running).
//  - GridMap buffers are filled directly (private access is opened for this test file only).
// Output per test case: Test Case Name | Expected | Actual | PASS/FAIL. Exit code 0 only if all pass.

// ---- headers used by grid_map.h / the DASC header, included before opening private access ----
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <unordered_set>
#include <plan_env/raycast.h>
#include "prometheus_msgs/ParamSettings.h"
#include <boost/make_shared.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <traj_utils/Bspline.h>
#include <traj_utils/plan_container.hpp>
#include <plan_manage/new_test_dasc_manager.h>

#include <memory>
#include <utility>
#include <xmlrpcpp/XmlRpcUtil.h>

// private access for this test only: GridMap buffers (to fill the map) and the FastEDF field (to check its layout)
#define private public
#include <plan_env/grid_map.h>
#include <plan_manage/new_test_fast_edf_corridor.h>
#undef private

using namespace ego_planner;

// ============================== reporting ==============================

static int g_pass = 0, g_fail = 0;

static std::string fmt(double v)
{
  char buf[64];
  if (std::isnan(v))
    return "nan";
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

static void report(const std::string &name, const std::string &expected, const std::string &actual, bool ok)
{
  std::printf("%-58s | %-26s | %-26s | %s\n", name.c_str(), expected.c_str(), actual.c_str(), ok ? "PASS" : "FAIL");
  (ok ? g_pass : g_fail)++;
}

static void checkNear(const std::string &name, double expected, double actual, double tol)
{
  bool ok = std::isfinite(actual) && std::fabs(expected - actual) <= tol;
  report(name, fmt(expected) + " +/- " + fmt(tol), fmt(actual), ok);
}

static void checkTrue(const std::string &name, const std::string &expected, const std::string &actual, bool ok)
{
  report(name, expected, actual, ok);
}

// ============================== GridMap in memory ==============================

// Builds a map with origin (0,0,0). Each obstacle marks one raw voxel and inflates it in the inflated
// buffer like GridMap::cloudCallback (box of +-istep voxels in x/y and +-istep_z in z).
static GridMap::Ptr makeMap(const Eigen::Vector3d &size, double res, const std::vector<Eigen::Vector3d> &obstacles,
                            int istep = 2, int istep_z = 2)
{
  GridMap::Ptr m(new GridMap);
  m->mp_.resolution_ = res;
  m->mp_.resolution_inv_ = 1.0 / res;
  m->mp_.map_origin_ = Eigen::Vector3d::Zero();
  m->mp_.map_size_ = size;
  for (int i = 0; i < 3; ++i)
    m->mp_.map_voxel_num_(i) = (int)std::ceil(size(i) / res);
  m->mp_.map_min_boundary_ = m->mp_.map_origin_;
  m->mp_.map_max_boundary_ = m->mp_.map_origin_ + size;
  m->mp_.clamp_min_log_ = std::log(0.12 / 0.88);
  m->mp_.clamp_max_log_ = std::log(0.97 / 0.03);
  m->mp_.min_occupancy_log_ = std::log(0.80 / 0.20);
  m->mp_.unknown_flag_ = 0.01;
  m->mp_.obstacles_inflation_ = istep * res;
  const long n = (long)m->mp_.map_voxel_num_.prod();
  m->md_.occupancy_buffer_.assign(n, m->mp_.clamp_min_log_ - m->mp_.unknown_flag_);
  m->md_.occupancy_buffer_inflate_.assign(n, 0);

  const Eigen::Vector3i nv = m->mp_.map_voxel_num_;
  for (const auto &o : obstacles)
  {
    Eigen::Vector3i id;
    m->posToIndex(o, id);
    if (!m->isInMap(o))
      continue;
    m->md_.occupancy_buffer_[m->toAddress(id)] = m->mp_.clamp_max_log_;
    for (int dx = -istep; dx <= istep; ++dx)
      for (int dy = -istep; dy <= istep; ++dy)
        for (int dz = -istep_z; dz <= istep_z; ++dz)
        {
          Eigen::Vector3i q = id + Eigen::Vector3i(dx, dy, dz);
          if (q(0) < 0 || q(1) < 0 || q(2) < 0 || q(0) >= nv(0) || q(1) >= nv(1) || q(2) >= nv(2))
            continue;
          m->md_.occupancy_buffer_inflate_[m->toAddress(q)] = 1;
        }
  }
  return m;
}

// ground truth: distance from p to the nearest raw occupied voxel centre (brute force)
static double trueDistance(GridMap::Ptr &m, const Eigen::Vector3d &p)
{
  // occupied centres are collected once per map (full scan), then searched linearly
  static GridMap *cached_map = nullptr;
  static std::vector<Eigen::Vector3d> centres;
  if (cached_map != m.get())
  {
    cached_map = m.get();
    centres.clear();
    const Eigen::Vector3i nv = m->mp_.map_voxel_num_;
    for (int x = 0; x < nv(0); ++x)
      for (int y = 0; y < nv(1); ++y)
        for (int z = 0; z < nv(2); ++z)
        {
          Eigen::Vector3i id(x, y, z);
          if (m->getOccupancy(id) != 1)
            continue;
          Eigen::Vector3d c;
          m->indexToPos(id, c);
          centres.push_back(c);
        }
  }
  double best = std::numeric_limits<double>::infinity();
  for (const auto &c : centres)
    best = std::min(best, (c - p).norm());
  return best;
}

// ============================== B-spline helpers ==============================

// rebuild a trajectory from a Bspline message exactly as traj_server.cpp bsplineCallback does
static UniformBspline fromMsg(const traj_utils::Bspline &msg)
{
  Eigen::MatrixXd pts(3, msg.pos_pts.size());
  for (size_t i = 0; i < msg.pos_pts.size(); ++i)
  {
    pts(0, i) = msg.pos_pts[i].x;
    pts(1, i) = msg.pos_pts[i].y;
    pts(2, i) = msg.pos_pts[i].z;
  }
  Eigen::VectorXd knots(msg.knots.size());
  for (size_t i = 0; i < msg.knots.size(); ++i)
    knots(i) = msg.knots[i];
  UniformBspline traj(pts, msg.order, 0.1);
  traj.setKnot(knots);
  return traj;
}

static LocalTrajData makeTraj(int n_cp, double interval, double t_start, int traj_id, unsigned seed,
                              bool non_uniform = false)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> U(-2.0, 2.0);
  Eigen::MatrixXd cps(3, n_cp);
  for (int i = 0; i < n_cp; ++i)
    cps.col(i) = Eigen::Vector3d(0.3 * i + U(rng) * 0.2, U(rng), 1.5 + 0.2 * U(rng));
  LocalTrajData d;
  d.position_traj_ = UniformBspline(cps, 3, interval);
  if (non_uniform)
  {
    Eigen::VectorXd u = d.position_traj_.getKnot();
    std::uniform_real_distribution<double> S(0.05, 0.25);
    u(0) = -3 * 0.1;
    for (int i = 1; i < u.rows(); ++i)
      u(i) = u(i - 1) + S(rng);
    u = u.array() - u(3); // curve starts at u_p = 0, like EGO
    d.position_traj_.setKnot(u);
  }
  d.velocity_traj_ = d.position_traj_.getDerivative();
  d.acceleration_traj_ = d.velocity_traj_.getDerivative();
  d.duration_ = d.position_traj_.getTimeSum();
  d.start_time_ = ros::Time(t_start);
  d.traj_id_ = traj_id;
  d.start_pos_ = d.position_traj_.evaluateDeBoorT(0.0);
  return d;
}

// max |p_tail(t) - p_orig(t)| (and velocity / acceleration) over absolute times now .. end of the original
static void compareTail(LocalTrajData &orig, const traj_utils::Bspline &msg, const ros::Time &now,
                        double &err_p, double &err_v, double &err_a, double &err_end)
{
  UniformBspline tail = fromMsg(msg);
  UniformBspline tail_v = tail.getDerivative();
  UniformBspline tail_a = tail_v.getDerivative();
  const double t0 = (now - orig.start_time_).toSec();
  const double t_tail0 = (now - msg.start_time).toSec();
  err_p = err_v = err_a = 0.0;
  for (int k = 0; k <= 50; ++k)
  {
    double dt = (orig.duration_ - t0) * k / 50.0;
    err_p = std::max(err_p, (orig.position_traj_.evaluateDeBoorT(t0 + dt) - tail.evaluateDeBoorT(t_tail0 + dt)).norm());
    err_v = std::max(err_v, (orig.velocity_traj_.evaluateDeBoorT(t0 + dt) - tail_v.evaluateDeBoorT(t_tail0 + dt)).norm());
    err_a = std::max(err_a, (orig.acceleration_traj_.evaluateDeBoorT(t0 + dt) - tail_a.evaluateDeBoorT(t_tail0 + dt)).norm());
  }
  err_end = std::fabs((orig.start_time_.toSec() + orig.duration_) - (msg.start_time.toSec() + tail.getTimeSum()));
}

// ============================== DASC tests ==============================

static void initDasc(DascManager &d, ros::NodeHandle &nh) { d.init(nh, 0.8); }

static nav_msgs::OdometryConstPtr odomMsg(const Eigen::Matrix3d &pos_cov, double stamp)
{
  nav_msgs::OdometryPtr msg = boost::make_shared<nav_msgs::Odometry>();
  msg->header.stamp = ros::Time(stamp);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      msg->pose.covariance[r * 6 + c] = pos_cov(r, c);
  return msg;
}

static void testDasc(ros::NodeHandle &nh)
{
  std::printf("\n---- DASC: dynamic safety margin rho (drift_cov_update) ----\n");
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  double rho = -1.0;

  {
    DascManager d;
    initDasc(d, nh);
    checkNear("DASC init: base margin (default inflation)", 0.2, d.getBaseMargin(), 1e-12);
    checkNear("DASC init: nominal max velocity", 0.8, d.getNominalMaxVel(), 1e-12);
    d.drift_cov_update(Eigen::Matrix3d::Zero(), ros::Time(10.0), rho);
    checkNear("rho: zero covariance -> base margin", 0.2, rho, 1e-12);
  }
  {
    DascManager d;
    initDasc(d, nh);
    d.drift_cov_update(0.0016 * I, ros::Time(10.0), rho);
    checkNear("rho: sigma=0.04 m -> base + 3*sigma", 0.2 + 3.0 * 0.04, rho, 1e-9);
  }
  {
    DascManager d;
    initDasc(d, nh);
    d.drift_cov_update(1.0 * I, ros::Time(10.0), rho);
    checkNear("rho: large covariance clipped to rho_max", 0.6, rho, 1e-12);
  }
  {
    DascManager d;
    initDasc(d, nh);
    Eigen::Matrix3d bad = I;
    bad(1, 1) = std::numeric_limits<double>::quiet_NaN();
    d.drift_cov_update(bad, ros::Time(10.0), rho);
    checkNear("rho: NaN covariance (first sample) -> base", 0.2, rho, 1e-12);
    Eigen::Matrix3d inf = I * std::numeric_limits<double>::infinity();
    d.drift_cov_update(inf, ros::Time(11.0), rho);
    checkTrue("rho: Inf covariance -> finite, in [0.2,0.6]", "finite in [0.2,0.6]", fmt(rho),
              std::isfinite(rho) && rho >= 0.2 && rho <= 0.6);
  }
  {
    DascManager d;
    initDasc(d, nh);
    Eigen::Matrix3d np = Eigen::Matrix3d::Zero();
    np(0, 0) = -1.0;
    np(1, 1) = 0.0016;
    d.drift_cov_update(np, ros::Time(10.0), rho);
    checkNear("rho: non-PSD covariance projected (neg. eig -> 0)", 0.32, rho, 1e-9);
  }
  {
    DascManager d;
    initDasc(d, nh);
    d.drift_cov_update(0.01 * I, ros::Time(10.0), rho);
    checkNear("rho: attack, sigma=0.1 -> 0.5", 0.5, rho, 1e-9);
    d.drift_cov_update(0.0001 * I, ros::Time(11.0), rho);
    double expect = 0.2 + 3.0 * std::sqrt(0.0001 + std::exp(-1.0) * (0.01 - 0.0001));
    checkNear("rho: release after 1 s, decay exp(-1)", expect, rho, 1e-9);
    double prev = rho;
    d.drift_cov_update(0.0001 * I, ros::Time(12.0), rho);
    checkTrue("rho: release is monotone non-increasing", "< " + fmt(prev), fmt(rho), rho < prev && rho >= 0.2);
    d.drift_cov_update(0.04 * I, ros::Time(12.5), rho);
    checkNear("rho: re-attack immediate, clipped at rho_max", 0.6, rho, 1e-12);
  }
  {
    DascManager d;
    initDasc(d, nh);
    d.drift_cov_update(0.01 * I, ros::Time(100.0), rho);
    d.drift_cov_update(0.0001 * I, ros::Time(50.0), rho); // clock reset (sim restart)
    checkNear("rho: backward stamp -> dt=0, excess kept", 0.5, rho, 1e-9);
    d.drift_cov_update(0.0001 * I, ros::Time(51.0), rho);
    double expect = 0.2 + 3.0 * std::sqrt(0.0001 + std::exp(-1.0) * (0.01 - 0.0001));
    checkNear("rho: re-sync after clock reset, decays again", expect, rho, 1e-9);
  }

  std::printf("\n---- DASC: speed scaling kappa (speed_adapt_law) ----\n");
  {
    DascManager d;
    initDasc(d, nh);
    double kappa = -1.0;
    d.speed_adapt_law(0.2, kappa);
    checkNear("kappa: rho == base -> 1", 1.0, kappa, 1e-12);
    d.speed_adapt_law(0.4, kappa);
    checkNear("kappa: rho = 2*base -> 1-(1-0.5) = 0.5", 0.5, kappa, 1e-12);
    d.speed_adapt_law(2.0, kappa);
    checkNear("kappa: rho = 10*base -> clipped to kappa_min", 0.3, kappa, 1e-12);
    d.speed_adapt_law(0.0, kappa);
    checkNear("kappa: rho below base -> 1", 1.0, kappa, 1e-12);
    d.speed_adapt_law(std::numeric_limits<double>::quiet_NaN(), kappa);
    checkNear("kappa: NaN rho -> kappa_min", 0.3, kappa, 1e-12);
    bool mono = true, in_range = true;
    double last = 2.0;
    for (int i = 0; i <= 200; ++i)
    {
      double r = 0.1 + 0.01 * i;
      d.speed_adapt_law(r, kappa);
      mono = mono && (kappa <= last + 1e-12);
      in_range = in_range && kappa >= 0.3 - 1e-12 && kappa <= 1.0 + 1e-12;
      last = kappa;
    }
    checkTrue("kappa: monotone non-increasing over rho in [0.1,2.1]", "true", mono ? "true" : "false", mono);
    checkTrue("kappa: always within [kappa_min, 1]", "true", in_range ? "true" : "false", in_range);
  }

  std::printf("\n---- DASC: compute() with default toggles (original behaviour) ----\n");
  {
    DascManager d;
    initDasc(d, nh);
    DascOutput out;
    bool ret = d.compute(ros::Time(10.0), out);
    checkTrue("compute: no odometry yet -> false, valid=false", "false/false",
              std::string(ret ? "true" : "false") + "/" + (out.valid ? "true" : "false"), !ret && !out.valid);
    d.feedOdometry(odomMsg(0.04 * I, 10.0));
    ret = d.compute(ros::Time(10.1), out);
    checkTrue("compute: toggles off -> rho=base, kappa=1, v=nominal", "0.2/1/0.8 valid",
              fmt(out.rho) + "/" + fmt(out.kappa) + "/" + fmt(out.v_dyn) + (out.valid ? " valid" : " invalid"),
              ret && out.valid && out.rho == 0.2 && out.kappa == 1.0 && out.v_dyn == 0.8);
  }
}

// ============================== fallback tests ==============================

static void testFallback(ros::NodeHandle &nh)
{
  std::printf("\n---- DASC fallback: tail of the previous B-spline (buildFallbackTail) ----\n");
  {
    DascManager d;
    initDasc(d, nh);
    LocalTrajData none = makeTraj(20, 0.1, 100.0, 0, 1);
    traj_utils::Bspline msg;
    bool ok = d.buildFallbackTail(none, ros::Time(100.2), msg);
    checkTrue("fallback: no previous trajectory (traj_id=0) -> false", "false", ok ? "true" : "false", !ok);

    LocalTrajData prev = makeTraj(20, 0.1, 100.0, 7, 2);
    traj_utils::Bspline msg2;
    ok = d.buildFallbackTail(prev, ros::Time(100.0 + prev.duration_ - 0.3), msg2);
    checkTrue("fallback: remaining 0.3 s < min_tail 0.5 s -> false", "false", ok ? "true" : "false", !ok);
    checkTrue("fallback: counter unchanged after refusals", "0", std::to_string(d.getFallbackCount()),
              d.getFallbackCount() == 0);
  }
  {
    DascManager d;
    initDasc(d, nh);
    LocalTrajData prev = makeTraj(20, 0.1, 100.0, 7, 3);
    traj_utils::Bspline msg;
    ros::Time now(100.55);
    bool ok = d.buildFallbackTail(prev, now, msg);
    checkTrue("fallback: valid tail at t_elapsed=0.55 s -> true", "true", ok ? "true" : "false", ok);
    // a cubic B-spline with n control points has n + p + 1 = n + 4 knots: 15 cps -> 19 knots
    checkTrue("fallback: 5 flown spans cut (20 -> 15 control points)", "15 cps / 19 knots",
              std::to_string(msg.pos_pts.size()) + " cps / " + std::to_string(msg.knots.size()) + " knots",
              msg.pos_pts.size() == 15 && msg.knots.size() == 19);
    checkNear("fallback: start_time shifted by the cut time 0.5 s", 100.5, msg.start_time.toSec(), 1e-6);
    checkTrue("fallback: traj_id kept", "7", std::to_string(msg.traj_id), msg.traj_id == 7);
    double ep, ev, ea, ee;
    compareTail(prev, msg, now, ep, ev, ea, ee);
    checkNear("fallback: position error, traj_server rebuild [m]", 0.0, ep, 1e-9);
    checkNear("fallback: velocity error [m/s]", 0.0, ev, 1e-8);
    checkNear("fallback: acceleration error [m/s^2]", 0.0, ea, 1e-6);
    checkNear("fallback: end time unchanged [s]", 0.0, ee, 1e-6);
    checkTrue("fallback: counter incremented", "1", std::to_string(d.getFallbackCount()), d.getFallbackCount() == 1);
  }
  {
    DascManager d;
    initDasc(d, nh);
    LocalTrajData prev = makeTraj(20, 0.1, 100.0, 9, 4);
    traj_utils::Bspline msg;
    bool ok = d.buildFallbackTail(prev, ros::Time(100.0), msg);
    checkTrue("fallback: t_elapsed=0 -> no cut, message = previous traj", "true/20/100",
              std::string(ok ? "true" : "false") + "/" + std::to_string(msg.pos_pts.size()) + "/" + fmt(msg.start_time.toSec()),
              ok && msg.pos_pts.size() == 20 && msg.start_time == ros::Time(100.0));
  }
  {
    DascManager d;
    initDasc(d, nh);
    LocalTrajData prev = makeTraj(24, 0.1, 200.0, 11, 5, true);
    double worst_p = 0.0, worst_e = 0.0;
    bool all_ok = true;
    for (int k = 1; k <= 20; ++k)
    {
      double te = (prev.duration_ - 0.6) * k / 20.0;
      ros::Time now(200.0 + te);
      traj_utils::Bspline msg;
      all_ok = all_ok && d.buildFallbackTail(prev, now, msg);
      double ep, ev, ea, ee;
      compareTail(prev, msg, now, ep, ev, ea, ee);
      worst_p = std::max(worst_p, ep);
      worst_e = std::max(worst_e, ee);
    }
    checkTrue("fallback: non-uniform knots, 20 cut times accepted", "true", all_ok ? "true" : "false", all_ok);
    // ros::Time keeps nanoseconds, so start_time + shift is rounded to 1 ns; with spans of 0.05 s the
    // random test spline moves up to ~1e2 m/s, i.e. ~1e-7 m per ns. 1e-6 m bounds that rounding.
    checkNear("fallback: non-uniform knots, max position error [m]", 0.0, worst_p, 1e-6);
    checkNear("fallback: non-uniform knots, max end-time error [s]", 0.0, worst_e, 1e-6);
  }
}

// ============================== FastEDF tests ==============================

static void testEdf(ros::NodeHandle &nh)
{
  std::printf("\n---- FastEDF: distance field (updateField / getDistance / isOccupied) ----\n");
  const double res = 0.1;
  const double hd = 0.5 * std::sqrt(3.0) * res;
  const Eigen::Vector3d obs(3.05, 3.05, 1.55); // centre of voxel (30,30,15)

  // field mode: 60 x 60 x 30 = 108000 voxels (< 1e6 budget)
  GridMap::Ptr m = makeMap(Eigen::Vector3d(6.0, 6.0, 3.0), res, {obs});
  FastEDFCorridor f;
  f.init(m, nh);
  checkTrue("EDF: before updateField -> isOccupied UNAVAILABLE", std::to_string(FAST_EDF_UNAVAILABLE),
            std::to_string(f.isOccupied(Eigen::Vector3d(1, 1, 1), 0.2)), f.isOccupied(Eigen::Vector3d(1, 1, 1), 0.2) == FAST_EDF_UNAVAILABLE);
  checkNear("EDF: before updateField -> getDistance -1", -1.0, f.getDistance(Eigen::Vector3d(1, 1, 1)), 0.0);
  f.updateField();
  checkTrue("EDF: field mode ready, field size = map voxels", "ready, 60x60x30",
            std::string(f.isReady() ? "ready, " : "not ready, ") + std::to_string(f.field_size_(0)) + "x" +
                std::to_string(f.field_size_(1)) + "x" + std::to_string(f.field_size_(2)),
            f.isReady() && f.field_size_ == Eigen::Vector3i(60, 60, 30) && f.field_.size() == 108000u);
  checkNear("EDF field: 1.0 m from obstacle = 0.8-2*hd+0.2", 0.8 - 2.0 * hd + 0.2,
            f.getDistance(obs + Eigen::Vector3d(1.0, 0, 0)), 1e-5);
  checkNear("EDF field: far from obstacles -> truncated d_trunc", 1.2, f.getDistance(Eigen::Vector3d(0.55, 0.55, 1.55)), 1e-9);
  checkNear("EDF field: inside inflated region -> 0", 0.0, f.getDistance(obs + Eigen::Vector3d(0.15, 0, 0)), 0.0);
  checkTrue("EDF: isOccupied inside inflated region -> 1", "1", std::to_string(f.isOccupied(obs, 0.2)), f.isOccupied(obs, 0.2) == 1);
  checkTrue("EDF: isOccupied 1 m away, margin 0.2 -> 0", "0",
            std::to_string(f.isOccupied(obs + Eigen::Vector3d(1.0, 0, 0), 0.2)), f.isOccupied(obs + Eigen::Vector3d(1.0, 0, 0), 0.2) == 0);
  checkTrue("EDF: isOccupied 1 m away, margin 0.9 -> 1", "1",
            std::to_string(f.isOccupied(obs + Eigen::Vector3d(1.0, 0, 0), 0.9)), f.isOccupied(obs + Eigen::Vector3d(1.0, 0, 0), 0.9) == 1);
  checkTrue("EDF: outside map -> isOccupied -1, getDistance -1", "-1/-1",
            std::to_string(f.isOccupied(Eigen::Vector3d(-1, 1, 1), 0.2)) + "/" + fmt(f.getDistance(Eigen::Vector3d(-1, 1, 1))),
            f.isOccupied(Eigen::Vector3d(-1, 1, 1), 0.2) == -1 && f.getDistance(Eigen::Vector3d(-1, 1, 1)) == -1.0);
  checkNear("EDF: NaN query -> -1", -1.0, f.getDistance(Eigen::Vector3d(std::nan(""), 1, 1)), 0.0);

  // conservativeness (C3.3: d~ <= d) and grid consistency, random scene, field mode
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> Ux(0.05, 5.95), Uz(0.05, 2.95);
  std::vector<Eigen::Vector3d> scene;
  for (int i = 0; i < 25; ++i)
    scene.push_back(Eigen::Vector3d(Ux(rng), Ux(rng), Uz(rng)));
  GridMap::Ptr ms = makeMap(Eigen::Vector3d(6.0, 6.0, 3.0), res, scene);
  FastEDFCorridor fs;
  fs.init(ms, nh);
  fs.updateField();
  int nonc = 0, gridc = 0, n_q = 2000;
  double min_slack = 1e9;
  for (int k = 0; k < n_q; ++k)
  {
    Eigen::Vector3d p(Ux(rng), Ux(rng), Uz(rng));
    double d = fs.getDistance(p), t = trueDistance(ms, p);
    if (d > t + 1e-9)
      nonc++;
    min_slack = std::min(min_slack, t - d);
    if (ms->getInflateOccupancy(p) == 1 && fs.isOccupied(p, 0.2) != 1)
      gridc++;
  }
  checkTrue("EDF field: d~ <= true distance (2000 random queries)", "0 violations",
            std::to_string(nonc) + " violations (min slack " + fmt(min_slack) + ")", nonc == 0);
  checkTrue("EDF field: grid-occupied points flagged at margin 0.2", "0 misses", std::to_string(gridc) + " misses", gridc == 0);

  // search mode: 110 x 110 x 90 = 1.089e6 voxels (> 1e6 budget)
  GridMap::Ptr mb = makeMap(Eigen::Vector3d(11.0, 11.0, 9.0), res, {obs});
  FastEDFCorridor fb;
  fb.init(mb, nh);
  fb.updateField();
  checkTrue("EDF: large map -> search mode (empty field, ready)", "ready, empty field",
            std::string(fb.isReady() ? "ready, " : "not ready, ") + (fb.field_.empty() ? "empty field" : "dense field"),
            fb.isReady() && fb.field_.empty());
  checkNear("EDF search: 1.0 m from obstacle = 0.8-hd+0.2", 0.8 - hd + 0.2, fb.getDistance(obs + Eigen::Vector3d(1.0, 0, 0)), 1e-9);
  checkNear("EDF search: far from obstacles -> d_trunc", 1.2, fb.getDistance(Eigen::Vector3d(8.05, 8.05, 5.05)), 1e-9);
  int nonc_b = 0;
  std::uniform_real_distribution<double> Ub(1.5, 4.5);
  for (int k = 0; k < 300; ++k)
  {
    Eigen::Vector3d p(Ub(rng), Ub(rng), Ub(rng) - 1.5);
    if (fb.getDistance(p) > trueDistance(mb, p) + 1e-9)
      nonc_b++;
  }
  checkTrue("EDF search: d~ <= true distance (300 random queries)", "0 violations", std::to_string(nonc_b) + " violations", nonc_b == 0);

  std::printf("\n---- FastEDF: preliminary feasibility margin beta (computeBeta) ----\n");
  checkNear("beta: clearance 1.0, rho 0.2 -> 1.0-0.1-(0.2+0.2)", 0.5, f.computeBeta(1.0, 0.2), 1e-12);
  checkNear("beta: clearance 0.45, rho 0.2 -> -0.05 (reject)", -0.05, f.computeBeta(0.45, 0.2), 1e-12);
  checkNear("beta: negative rho treated as 0", 0.7, f.computeBeta(1.0, -0.5), 1e-12);
  checkNear("beta: unavailable clearance (-1) -> -1e6", -1e6, f.computeBeta(-1.0, 0.2), 0.0);
  checkNear("beta: NaN clearance -> -1e6", -1e6, f.computeBeta(std::nan(""), 0.2), 0.0);
  checkNear("beta: NaN rho -> -1e6", -1e6, f.computeBeta(1.0, std::nan("")), 0.0);

  std::printf("\n---- FastEDF: edge admissibility (ParentEdgeAdmissible) and path check ----\n");
  double mm = 0.0;
  bool adm = f.ParentEdgeAdmissible(Eigen::Vector3d(0.55, 0.55, 1.55), Eigen::Vector3d(0.55, 1.55, 1.55), 0.2, mm);
  checkTrue("edge: free space -> admissible, min margin 0.7", "true / 0.7",
            std::string(adm ? "true" : "false") + " / " + fmt(mm), adm && std::fabs(mm - 0.7) < 1e-9);
  adm = f.ParentEdgeAdmissible(Eigen::Vector3d(1.05, 3.05, 1.55), Eigen::Vector3d(5.05, 3.05, 1.55), 0.2, mm);
  checkTrue("edge: through obstacle -> not admissible, margin < 0", "false / <0",
            std::string(adm ? "true" : "false") + " / " + fmt(mm), !adm && mm < 0.0);
  adm = f.ParentEdgeAdmissible(Eigen::Vector3d(1.05, 3.45, 1.55), Eigen::Vector3d(5.05, 3.45, 1.55), 0.2, mm);
  checkTrue("edge: passes 0.4 m from obstacle -> not admissible", "false", adm ? "true" : "false", !adm);
  adm = f.ParentEdgeAdmissible(Eigen::Vector3d(1.05, 4.45, 1.55), Eigen::Vector3d(5.05, 4.45, 1.55), 0.2, mm);
  checkTrue("edge: passes 1.4 m from obstacle -> admissible", "true", adm ? "true" : "false", adm);
  adm = f.ParentEdgeAdmissible(Eigen::Vector3d(-1.0, 1.0, 1.0), Eigen::Vector3d(1.0, 1.0, 1.0), 0.2, mm);
  checkTrue("edge: leaves the map -> not admissible", "false", adm ? "true" : "false", !adm);
  FastEDFCorridor fn;
  fn.init(m, nh);
  adm = fn.ParentEdgeAdmissible(Eigen::Vector3d(0.55, 0.55, 1.55), Eigen::Vector3d(0.55, 1.55, 1.55), 0.2, mm);
  checkTrue("edge: field not ready -> not admissible", "false", adm ? "true" : "false", !adm);

  // certificate (Prop. 3.1 + (A1)): every accepted edge keeps true clearance >= rho + vehicle radius
  int accepted = 0, violated = 0;
  double worst = 1e9;
  for (int k = 0; k < 400; ++k)
  {
    Eigen::Vector3d a(Ux(rng), Ux(rng), Uz(rng));
    Eigen::Vector3d b = a + Eigen::Vector3d(Ux(rng) - 3.0, Ux(rng) - 3.0, Uz(rng) - 1.5) * 0.5;
    double mmk;
    if (!fs.ParentEdgeAdmissible(a, b, 0.2, mmk))
      continue;
    accepted++;
    for (int s = 0; s <= 100; ++s)
    {
      Eigen::Vector3d p = a + (b - a) * (s / 100.0);
      double slack = trueDistance(ms, p) - 0.4;
      worst = std::min(worst, slack);
      if (slack < -1e-9)
      {
        violated++;
        break;
      }
    }
  }
  checkTrue("edge: accepted edges keep clearance >= rho+radius", "0 violations, >0 accepted",
            std::to_string(violated) + " viol., " + std::to_string(accepted) + " acc. (slack " + fmt(worst) + ")",
            violated == 0 && accepted > 0);

  std::vector<Eigen::Vector3d> path_hit, path_free;
  for (int i = 0; i <= 40; ++i)
  {
    path_hit.push_back(Eigen::Vector3d(1.05 + 0.1 * i, 3.05, 1.55));
    path_free.push_back(Eigen::Vector3d(1.05 + 0.1 * i, 0.55, 1.55));
  }
  std::vector<double> beta;
  std::vector<std::pair<int, int>> segs;
  bool ok = f.FastEDFCorridorCheck(path_hit, 0.2, beta, segs);
  bool covers = segs.size() == 1 && segs[0].first <= 20 && segs[0].second >= 20;
  checkTrue("path check: through obstacle -> 1 segment covering it", "1 segment containing idx 20",
            std::to_string(segs.size()) + " seg" + (segs.empty() ? "" : " [" + std::to_string(segs[0].first) + "," + std::to_string(segs[0].second) + "]"),
            ok && covers);
  ok = f.FastEDFCorridorCheck(path_free, 0.2, beta, segs);
  checkTrue("path check: free path -> no segment, all beta >= 0", "0 segments",
            std::to_string(segs.size()) + " segments", ok && segs.empty() && *std::min_element(beta.begin(), beta.end()) >= 0.0);
}

// ============================== main ==============================

int main(int argc, char **argv)
{
  // deterministic parameters: no master is reachable, so every parameter read returns its default
  setenv("ROS_MASTER_URI", "http://127.0.0.1:1/", 1);
  ros::init(argc, argv, "test_dasc_edf", ros::init_options::NoRosout | ros::init_options::NoSigintHandler |
                                             ros::init_options::AnonymousName);
  ros::master::setRetryTimeout(ros::WallDuration(0.05));
  // hide the expected "master unreachable" retries (roscpp, XmlRpc) and the modules' [DASC] prints
  struct SilentXmlRpc : public XmlRpc::XmlRpcErrorHandler
  {
    void error(const char *) override {}
  };
  static SilentXmlRpc silent;
  XmlRpc::XmlRpcErrorHandler::setErrorHandler(&silent);
  if (ros::console::set_logger_level("ros", ros::console::levels::Fatal))
    ros::console::notifyLoggerLevelsChanged();
  ros::NodeHandle nh("~");

  std::printf("%-58s | %-26s | %-26s | %s\n", "Test Case Name", "Expected", "Actual", "PASS/FAIL");
  std::printf("%s\n", std::string(124, '-').c_str());

  testDasc(nh);
  testFallback(nh);
  testEdf(nh);

  std::printf("\n%s\nSUMMARY: %d passed, %d failed, %d total\n", std::string(124, '=').c_str(), g_pass, g_fail, g_pass + g_fail);
  return g_fail == 0 ? 0 : 1;
}
