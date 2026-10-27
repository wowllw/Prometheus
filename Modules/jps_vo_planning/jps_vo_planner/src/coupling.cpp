#include "jps_vo_planner/coupling.h"

#include <algorithm>

namespace jps_vo
{
  namespace
  {
    double clip01(double x) { return std::min(1.0, std::max(0.0, x)); }
  }

  void ModelConstants::deriveTaus()
  {
    // eq. (32): tau1 = T_c + sqrt(2 R_max / a_max), tau2 = max{tau1 + T_g, tau1 exp(15 dv* / (8 a_max tau1))}
    const double R_max = safetyRadius(r_o_max);
    if (tau1 <= 0.0)
      tau1 = T_c + std::sqrt(2.0 * R_max / a_max);
    if (tau2 <= 0.0)
      tau2 = std::max(tau1 + T_g, tau1 * std::exp(15.0 * dv_star / (8.0 * a_max * tau1)));
  }

  bool Switches::parseLambdaMode(const std::string &s)
  {
    if (s == "adaptive")
      lambda_mode = LambdaMode::Adaptive;
    else if (s == "zero")
      lambda_mode = LambdaMode::Zero;
    else if (s == "one")
      lambda_mode = LambdaMode::One;
    else if (s.compare(0, 6, "const:") == 0)
    {
      lambda_mode = LambdaMode::Const;
      lambda_const = std::stod(s.substr(6));
    }
    else
      return false;
    return true;
  }

  bool Switches::parseLambdaShape(const std::string &s)
  {
    if (s == "quintic_log")
      lambda_shape = LambdaShape::QuinticLog;
    else if (s == "linear")
      lambda_shape = LambdaShape::Linear;
    else if (s == "logistic")
      lambda_shape = LambdaShape::Logistic;
    else
      return false;
    return true;
  }

  bool Switches::parseThetaCMode(const std::string &s)
  {
    if (s == "linear")
      theta_c_mode = ThetaCMode::Linear;
    else if (s == "sqrt")
      theta_c_mode = ThetaCMode::Sqrt;
    else if (s.compare(0, 6, "fixed:") == 0)
    {
      theta_c_mode = ThetaCMode::Fixed;
      theta_c_fixed = std::stod(s.substr(6)) * M_PI / 180.0;
    }
    else
      return false;
    return true;
  }

  bool Switches::parseXiSource(const std::string &s)
  {
    if (s == "reference")
      xi_source = XiSource::Reference;
    else if (s == "executed")
      xi_source = XiSource::Executed;
    else
      return false;
    return true;
  }

  double smoothStep(double u)
  {
    u = clip01(u);
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
  }

  double urgency(double xi, double tau1, double tau2)
  {
    if (!(xi < kInf))
      return 0.0;
    if (xi <= 0.0)
      return 1.0;
    return clip01(std::log(tau2 / xi) / std::log(tau2 / tau1));
  }

  double lambdaOf(double xi, const ModelConstants &c, const Switches &sw)
  {
    switch (sw.lambda_mode)
    {
    case Switches::LambdaMode::Zero:
      return 0.0;
    case Switches::LambdaMode::One:
      return 1.0;
    case Switches::LambdaMode::Const:
      return clip01(sw.lambda_const);
    case Switches::LambdaMode::Adaptive:
      break;
    }
    switch (sw.lambda_shape)
    {
    case Switches::LambdaShape::Linear: // A8: linear in xi on [tau1, tau2]
      return xi < kInf ? clip01((c.tau2 - xi) / (c.tau2 - c.tau1)) : 0.0;
    case Switches::LambdaShape::Logistic: // A8: logistic centred on the window
    {
      if (!(xi < kInf))
        return 0.0;
      const double mid = 0.5 * (c.tau1 + c.tau2);
      return 1.0 / (1.0 + std::exp(10.0 * (xi - mid) / (c.tau2 - c.tau1)));
    }
    case Switches::LambdaShape::QuinticLog:
      break;
    }
    return smoothStep(urgency(xi, c.tau1, c.tau2)); // eq. (31)
  }

  double thetaC(double lambda, const Switches &sw)
  {
    if (!sw.enable_c3)
      return M_PI; // A5: cone always open
    switch (sw.theta_c_mode)
    {
    case Switches::ThetaCMode::Fixed:
      return sw.theta_c_fixed;
    case Switches::ThetaCMode::Sqrt:
      return M_PI * std::sqrt(clip01(lambda));
    case Switches::ThetaCMode::Linear:
      break;
    }
    return M_PI * clip01(lambda); // eq. (28)
  }

  void segmentRiskLite(const Segment &sg, const std::vector<ObstacleState> &obs, const ModelConstants &c, double &rho,
                       double &xi)
  {
    const double T = sg.ell / c.v_c;
    double keep = 1.0;
    xi = kInf;
    for (const auto &o : obs)
    {
      const Eigen::Vector3d x = o.p + o.v * sg.t_rel - sg.q;
      const Eigen::Vector3d w = c.v_c * sg.d - o.v;
      const double w2 = w.squaredNorm();
      const double t_cpa = w2 > 1e-12 ? std::min(T, std::max(0.0, x.dot(w) / w2)) : 0.0;
      const double d_cpa = (x - w * t_cpa).norm();
      const double g = smoothStep(clip01((c.safetyRadius(o.r) + c.m - d_cpa) / c.m));
      const double tau_o = sg.t_rel + t_cpa;
      keep *= 1.0 - urgency(tau_o, c.tau1, c.tau2) * g;
      xi = std::min(xi, tau_o + (1.0 - g) * c.tau2);
    }
    rho = 1.0 - keep;
  }

  SegmentRisk segmentRisk(const Segment &sg, const std::vector<ObstacleState> &obs, const ModelConstants &c)
  {
    SegmentRisk out;
    const double T = sg.ell / c.v_c; // T_sigma = ell / v_c
    double keep = 1.0;
    for (const auto &o : obs)
    {
      // eq. (19)
      const Eigen::Vector3d x = o.p + o.v * sg.t_rel - sg.q;
      const Eigen::Vector3d w = c.v_c * sg.d - o.v;
      const double w2 = w.squaredNorm();
      const double t_cpa = w2 > 1e-12 ? std::min(T, std::max(0.0, x.dot(w) / w2)) : 0.0;
      const double d_cpa = (x - w * t_cpa).norm();
      // eq. (20)
      const double R = c.safetyRadius(o.r);
      const double g = smoothStep(clip01((R + c.m - d_cpa) / c.m));
      const double tau_o = sg.t_rel + t_cpa;
      const double rho_o = urgency(tau_o, c.tau1, c.tau2) * g;
      keep *= (1.0 - rho_o);
      // eq. (29)
      const double xi_o = tau_o + (1.0 - g) * c.tau2;
      out.xi = std::min(out.xi, xi_o);

      out.d_cpa.push_back(d_cpa);
      out.t_cpa.push_back(t_cpa);
      out.g.push_back(g);
      out.rho_o.push_back(rho_o);
      out.xi_o.push_back(xi_o);
    }
    out.rho = 1.0 - keep;
    return out;
  }

  double gammaV(const Eigen::Vector3d &v, const Eigen::Vector3d &v_prev, double lambda, const ModelConstants &c)
  {
    const double f = lambda * (1.0 - lambda);
    if (f <= 0.0)
      return 0.0;
    double psi_theta = 0.0;
    if (v.norm() >= c.v_min && v_prev.norm() >= c.v_min)
      psi_theta = (1.0 - v.normalized().dot(v_prev.normalized())) / (1.0 - std::cos(c.theta_max));
    Eigen::Vector3d dv = v - v_prev;
    if (v_prev.norm() >= c.v_min)
    {
      const Eigen::Vector3d h = v_prev.normalized();
      dv -= h * h.dot(dv); // P_perp = I - h h^T
    }
    const double psi_a = dv.squaredNorm() / std::pow(c.a_max * c.dt, 2);
    return f * (c.w_theta * psi_theta + c.w_a * psi_a);
  }

  double gammaE(const Eigen::Vector3d &t_prev, const Eigen::Vector3d &t_e, double lambda_e, const ModelConstants &c)
  {
    if (t_prev.squaredNorm() < 1e-12)
      return 0.0;
    return lambda_e * (1.0 - lambda_e) * c.w_theta * (1.0 - t_prev.normalized().dot(t_e.normalized())) /
           (1.0 - std::cos(c.theta_max));
  }

  bool inCollisionCone(const Eigen::Vector3d &x, const Eigen::Vector3d &w, double R)
  {
    const double x2 = x.squaredNorm();
    if (x2 <= R * R || w.squaredNorm() < 1e-18)
      return false;
    return w.dot(x) >= w.norm() * std::sqrt(x2 - R * R);
  }

  double timeToCollision(const Eigen::Vector3d &x, const Eigen::Vector3d &w, double R)
  {
    const double x2 = x.squaredNorm();
    if (x2 <= R * R)
      return w.dot(x) > 0.0 ? 0.0 : kInf;
    if (!inCollisionCone(x, w, R))
      return kInf;
    const double xw = x.dot(w), w2 = w.squaredNorm();
    const double disc = std::max(0.0, xw * xw - w2 * (x2 - R * R));
    return (xw - std::sqrt(disc)) / w2; // smaller root of ||x - w t|| = R
  }

} // namespace jps_vo
