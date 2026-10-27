#ifndef JPS_VO_PLANNER_BSPLINE_UTILS_H
#define JPS_VO_PLANNER_BSPLINE_UTILS_H

#include <Eigen/Eigen>
#include <vector>

namespace jps_vo
{
  // Uniform cubic B-spline in the exact convention of ego_planner::UniformBspline and
  // traj_server_for_prometheus: order p = 3, knots u_j = (j - p) * dt, t = 0 at u_p,
  // duration = (num_ctrl_pts - p) * dt.
  struct UniformCubicBspline
  {
    std::vector<Eigen::Vector3d> ctrl_pts;
    double dt = 0.1;

    bool valid() const { return ctrl_pts.size() >= 4 && dt > 0.0; }
    double duration() const { return valid() ? (ctrl_pts.size() - 3) * dt : 0.0; }
    std::vector<double> knots() const;
    // t is clamped to [0, duration]
    void evaluate(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel) const;
  };

  // Straight line p0 -> goal with a trapezoidal speed profile (initial speed = projection of v0 on the
  // line, final speed 0). Control points are samples of the profile, so by the convex-hull property the
  // B-spline velocity stays <= v_max. Returns false only on invalid input.
  bool buildStraightLine(const Eigen::Vector3d &p0, const Eigen::Vector3d &v0, const Eigen::Vector3d &goal,
                         double v_max, double a_max, double dt, UniformCubicBspline &out);

  // Zero-velocity trajectory that holds p
  UniformCubicBspline buildHover(const Eigen::Vector3d &p, double dt);

  // [ENG-2] velocity bridge for the local layer: starts at (p0, v0), reaches v_target with |acc| <= a_max, holds it
  // until `horizon`, then stops at a_max (so a missed cycle ends in a hover). Control points are samples of this
  // profile, so the B-spline speed stays <= max(|v0|, |v_target|).
  UniformCubicBspline buildVelocityBridge(const Eigen::Vector3d &p0, const Eigen::Vector3d &v0,
                                          const Eigen::Vector3d &v_target, double a_max, double horizon, double dt);

} // namespace jps_vo

#endif
