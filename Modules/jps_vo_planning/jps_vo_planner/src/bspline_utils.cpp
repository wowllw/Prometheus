#include "jps_vo_planner/bspline_utils.h"

#include <algorithm>
#include <cmath>

namespace jps_vo
{
  std::vector<double> UniformCubicBspline::knots() const
  {
    // m + 1 = n + p + 2 knots, n + 1 = number of control points
    const int num_knots = static_cast<int>(ctrl_pts.size()) + 4;
    std::vector<double> u(num_knots);
    for (int j = 0; j < num_knots; ++j)
      u[j] = (j - 3) * dt;
    return u;
  }

  void UniformCubicBspline::evaluate(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel) const
  {
    pos.setZero();
    vel.setZero();
    if (!valid())
      return;

    t = std::min(std::max(t, 0.0), duration());
    const int num_seg = static_cast<int>(ctrl_pts.size()) - 3;
    int i = std::min(static_cast<int>(std::floor(t / dt)), num_seg - 1);
    const double s = t / dt - i;

    const Eigen::Vector3d &P0 = ctrl_pts[i], &P1 = ctrl_pts[i + 1], &P2 = ctrl_pts[i + 2], &P3 = ctrl_pts[i + 3];
    const double s2 = s * s, s3 = s2 * s;
    pos = ((1 - s) * (1 - s) * (1 - s) * P0 + (3 * s3 - 6 * s2 + 4) * P1 + (-3 * s3 + 3 * s2 + 3 * s + 1) * P2 + s3 * P3) / 6.0;
    vel = (-3 * (1 - s) * (1 - s) * P0 + (9 * s2 - 12 * s) * P1 + (-9 * s2 + 6 * s + 3) * P2 + 3 * s2 * P3) / (6.0 * dt);
  }

  UniformCubicBspline buildHover(const Eigen::Vector3d &p, double dt)
  {
    UniformCubicBspline traj;
    traj.dt = dt;
    traj.ctrl_pts.assign(7, p);
    return traj;
  }

  UniformCubicBspline buildVelocityBridge(const Eigen::Vector3d &p0, const Eigen::Vector3d &v0,
                                          const Eigen::Vector3d &v_target, double a_max, double horizon, double dt)
  {
    UniformCubicBspline traj;
    traj.dt = dt;
    const double dv_max = a_max * dt;
    Eigen::Vector3d p = p0, v = v0;
    traj.ctrl_pts.push_back(p0 - v0 * dt); // Q_{-1}: vel(0) = v0
    traj.ctrl_pts.push_back(p0);
    for (double t = 0.0; t < horizon - 1e-9; t += dt)
    {
      Eigen::Vector3d dv = v_target - v;
      if (dv.norm() > dv_max)
        dv *= dv_max / dv.norm();
      v += dv;
      p += v * dt;
      traj.ctrl_pts.push_back(p);
    }
    while (v.norm() > 1e-9) // stop tail
    {
      const double sp = v.norm();
      v *= std::max(0.0, sp - dv_max) / sp;
      p += v * dt;
      traj.ctrl_pts.push_back(p);
    }
    traj.ctrl_pts.push_back(p);
    traj.ctrl_pts.push_back(p); // last three equal: pos = p, vel = acc = 0
    return traj;
  }

  namespace
  {
    // 1D trapezoidal profile: start speed v0 >= 0, end speed 0, distance L
    struct Trapezoid
    {
      double v0, vp, a_acc, a_dec, t1, t2, T, s1, s2;

      Trapezoid(double L, double v0_in, double v_max, double a_max)
      {
        v0 = std::min(v0_in, v_max);
        a_acc = a_dec = a_max;
        const double d_stop = v0 * v0 / (2 * a_max);
        if (d_stop >= L)
        {
          // cannot stop within L at a_max: decelerate uniformly, which exceeds a_max (only when a new goal
          // is closer than the current stopping distance)
          vp = v0;
          a_dec = v0 * v0 / (2 * std::max(L, 1e-6));
          t1 = t2 = 0.0;
        }
        else
        {
          const double d_full = (v_max * v_max - v0 * v0) / (2 * a_max) + v_max * v_max / (2 * a_max);
          vp = d_full <= L ? v_max : std::sqrt((2 * a_max * L + v0 * v0) / 2);
          t1 = (vp - v0) / a_max;
          const double d_acc = (vp * vp - v0 * v0) / (2 * a_max);
          const double d_dec = vp * vp / (2 * a_max);
          t2 = t1 + (L - d_acc - d_dec) / vp;
        }
        T = t2 + (a_dec > 0 ? vp / a_dec : 0.0);
        s1 = v0 * t1 + 0.5 * a_acc * t1 * t1;
        s2 = s1 + vp * (t2 - t1);
      }

      double s(double t) const
      {
        if (t <= t1)
          return v0 * t + 0.5 * a_acc * t * t;
        if (t <= t2)
          return s1 + vp * (t - t1);
        const double tau = std::min(t, T) - t2;
        return s2 + vp * tau - 0.5 * a_dec * tau * tau;
      }
    };
  } // namespace

  bool buildStraightLine(const Eigen::Vector3d &p0, const Eigen::Vector3d &v0, const Eigen::Vector3d &goal,
                         double v_max, double a_max, double dt, UniformCubicBspline &out)
  {
    if (v_max <= 0.0 || a_max <= 0.0 || dt <= 0.0)
      return false;

    const Eigen::Vector3d d = goal - p0;
    const double L = d.norm();
    if (L < 1e-3)
    {
      out = buildHover(goal, dt);
      return true;
    }

    const Eigen::Vector3d dir = d / L;
    const double s0_dot = std::min(std::max(v0.dot(dir), 0.0), v_max);
    Trapezoid prof(L, s0_dot, v_max, a_max);

    const int N = std::max(1, static_cast<int>(std::ceil(prof.T / dt)));
    out.dt = dt;
    out.ctrl_pts.clear();
    out.ctrl_pts.reserve(N + 4);
    // Q_{-1}, Q_0, Q_1 give pos(0) ~= p0 and vel(0) ~= s0_dot * dir
    out.ctrl_pts.push_back(p0 - s0_dot * dt * dir);
    for (int k = 0; k <= N; ++k)
      out.ctrl_pts.push_back(p0 + std::min(prof.s(k * dt), L) * dir);
    out.ctrl_pts.back() = goal;
    // three identical end points: pos = goal, vel = acc = 0
    out.ctrl_pts.push_back(goal);
    out.ctrl_pts.push_back(goal);
    return true;
  }

} // namespace jps_vo
