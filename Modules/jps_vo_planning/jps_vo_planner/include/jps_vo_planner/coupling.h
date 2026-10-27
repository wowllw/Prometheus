#ifndef JPS_VO_PLANNER_COUPLING_H
#define JPS_VO_PLANNER_COUPLING_H

// Frozen Stage-2 model functions (eqs. 12, 14, 19, 20, 29-35). ROS-free so the offline tests link them directly.
// Equation numbers refer to 阶段二：3D-JPS 与 VO 强耦合数学模型.

#include <Eigen/Eigen>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace jps_vo
{
  constexpr double kInf = std::numeric_limits<double>::infinity();

  struct ObstacleState
  {
    std::string name;
    Eigen::Vector3d p = Eigen::Vector3d::Zero(); // at the planning time t0
    Eigen::Vector3d v = Eigen::Vector3d::Zero(); // constant velocity v_o (D2)
    double r = 0.3;                              // r_o
  };

  // Constants fixed for the whole experiment (subtask4_ablation_experiment_spec.md section 2).
  // Only v_max, a_max, d_min, tau1, tau2 are calibratable; tau1/tau2 <= 0 means "derive from eq. (32)".
  struct ModelConstants
  {
    double r_q = 0.30, e_cut = 0.10, v_c = 0.80, dt = 0.10, tau_h = 4.0;
    double m = 1.20, L_min = 2.0, psi_max = M_PI / 3.0, theta_max = M_PI / 3.0, v_min = 0.10;
    double w_theta = 1.0, w_a = 1.0, rho_g = 10.0, eps_w = 0.05;
    double T_c = 0.80, T_g = 1.50, dv_star = 1.60;
    double v_max = 0.8, a_max = 0.8, d_min = 0.2;
    double r_o_max = 0.3; // largest obstacle radius of the scenario set, for R_max in eq. (32)
    double tau1 = -1.0, tau2 = -1.0;

    void deriveTaus(); // eq. (32), only for values <= 0
    double rInfl() const { return r_q + d_min + e_cut; }                       // eq. (1)
    double ellW() const { return psi_max * v_c * v_c / a_max; }                 // eq. (27)
    double safetyRadius(double r_o) const { return r_q + r_o + d_min; }        // R_o
  };

  // Ablation switches (subtask3_porting_plan.md section 5); set from launch parameters only
  struct Switches
  {
    enum class LambdaMode { Adaptive, Zero, One, Const };
    enum class LambdaShape { QuinticLog, Linear, Logistic };
    enum class ThetaCMode { Linear, Fixed, Sqrt };
    enum class XiSource { Reference, Executed };

    LambdaMode lambda_mode = LambdaMode::Adaptive;
    double lambda_const = 0.5;
    bool use_rho_in_pure_vo = false;
    bool enable_c2 = true;
    bool enable_c3 = true;
    double c3_soft_tracking = 0.0; // A6 only (not one of the 6 groups)
    bool enable_gamma = true;
    LambdaShape lambda_shape = LambdaShape::QuinticLog;
    ThetaCMode theta_c_mode = ThetaCMode::Linear;
    double theta_c_fixed = M_PI / 2.0;
    XiSource xi_source = XiSource::Reference;

    // parse "adaptive" | "zero" | "one" | "const:<c>" etc.; false on unknown text
    bool parseLambdaMode(const std::string &s);
    bool parseLambdaShape(const std::string &s);
    bool parseThetaCMode(const std::string &s); // "linear" | "sqrt" | "fixed:<deg>"
    bool parseXiSource(const std::string &s);
  };

  // eq. (30): S(u) = 6u^5 - 15u^4 + 10u^3 on [0,1]
  double smoothStep(double u);
  // eq. (31): u(xi) = clip(ln(tau2/xi) / ln(tau2/tau1), 0, 1); u(0) = 1, u(inf) = 0
  double urgency(double xi, double tau1, double tau2);
  // eq. (31) lambda(xi) = S(u(xi)), plus the ablation modes/shapes (A1, A2, A3, A8)
  double lambdaOf(double xi, const ModelConstants &c, const Switches &sw);
  // eq. (28): theta_c(lambda) = pi * lambda (A5: pi; A9: fixed / sqrt)
  double thetaC(double lambda, const Switches &sw);

  // Motion segment sigma = (q, d_hat, ell) flown at v_c, starting t_rel = t_sigma - t0 after now (section 2.1)
  struct Segment
  {
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    Eigen::Vector3d d = Eigen::Vector3d::Zero(); // unit (or zero when ell = 0)
    double ell = 0.0;
    double t_rel = 0.0;
  };

  struct SegmentRisk
  {
    double rho = 0.0;  // eq. (20) rho(sigma) in [0,1]
    double xi = kInf;  // eq. (29) xi(sigma) = min_o xi_o
    std::vector<double> d_cpa, t_cpa, g, rho_o, xi_o;
  };
  // eqs. (19), (20), (29) for all obstacles
  SegmentRisk segmentRisk(const Segment &sg, const std::vector<ObstacleState> &obs, const ModelConstants &c);
  // same rho(sigma) and xi(sigma) without the per-obstacle vectors (hot path: edge weights)
  void segmentRiskLite(const Segment &sg, const std::vector<ObstacleState> &obs, const ModelConstants &c, double &rho,
                       double &xi);

  // eq. (34) Gamma(v; xi) given lambda = lambda(xi); P_perp undefined for |v_prev| < v_min -> identity [ENG]
  double gammaV(const Eigen::Vector3d &v, const Eigen::Vector3d &v_prev, double lambda, const ModelConstants &c);
  // eq. (35) Gamma_e; t_prev = zero vector means "no previous direction" -> 0 [ENG]
  double gammaE(const Eigen::Vector3d &t_prev, const Eigen::Vector3d &t_e, double lambda_e, const ModelConstants &c);

  // eq. (12): w in CC_{A|o} with x = p_o - p_A (requires ||x|| > R)
  bool inCollisionCone(const Eigen::Vector3d &x, const Eigen::Vector3d &w, double R);
  // eq. (14): first contact time for w in CC, +inf otherwise.
  // ||x|| <= R (already inside the safety radius, cone undefined): 0 if approaching (w.x > 0), +inf otherwise [ENG-7]
  double timeToCollision(const Eigen::Vector3d &x, const Eigen::Vector3d &w, double R);

  inline double angleBetween(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
  {
    const double na = a.norm(), nb = b.norm();
    if (na < 1e-12 || nb < 1e-12)
      return 0.0;
    return std::acos(std::max(-1.0, std::min(1.0, a.dot(b) / (na * nb))));
  }

} // namespace jps_vo

#endif
