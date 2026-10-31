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

  // Engineering fixes after thesis-frozen-v1 (fixes design note R1-R5, S1). Every flag defaults to false; with all
  // flags false every frozen code path runs unchanged. Set from the launch parameters ~fixes/<name> only.
  struct Fixes
  {
    bool r1a_reachable_cone = false;       // R1a cone widened to the heading reachable in one cycle under (C-a)
    bool r1b_continuous_slack = false;     // R1b cone slack = dist(v, K) / v_c, continuous at v = 0
    bool r1c_heading_memory = false;       // R1c Gamma turn term measured against the last heading when |v_prev| < v_min
    bool r1d_feasible_cone = false;        // R1d cone never narrower than the closest feasible moving candidate
    bool r1e_lowspeed_angle = false;       // R1e cone slack = min(1, |v| / v_min) * angle excess (replaces R1b)
    bool r2a_kappa_decay = false;          // R2a kappa peak-hold with exp(-dt / T_g) release instead of a per-cycle clear
    bool r2b_kappa_local_excluded = false; // R2b J_A = V_Pi computed without kappa
    bool r3_relevant_trigger = false;      // R3 replan on remaining-path increase / off-path decrease > eps_w only
    bool r4_switch_hysteresis = false;     // R4 geometric switch only if cheaper by eps_w and T_g after the last switch
    bool r4m_dwell_free_only = false;      // R4-m the R4 dwell applies only in threat-free mode; the eps_w margin always
    bool r4n_leaky_evidence = false;       // R4-new switch on leaky CUSUM evidence E = max(0, g E + dC - eps_w) >= eps_w
    bool r4s_safety_override = false;      // R4-s force the switch when the old route is predicted to enter a safety sphere
    bool r5_mode_gating = false;           // R5 cone base width atan(e_cut / ell_W) while threat-free for >= T_c
    bool s1_eps_constraint = false;        // S1 risk budget rho <= 1 - lambda instead of the weighted rho term
    bool diag_log = false;                 // extended diagnostics only (cycles_ext.csv, switches_ext.csv)
    bool any() const;
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
    Fixes fx; // engineering fixes after thesis-frozen-v1; all false = frozen behaviour

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

  // ---- fixes after thesis-frozen-v1 (used only when the matching Fixes flag is on) ----
  // R1a: largest heading change reachable in one cycle from speed s under (C-a): 2 s sin(phi / 2) <= a_max dt
  double reachTurn(double s, const ModelConstants &c);
  // R1b: dist(v, K(theta)) / v_c with K(theta) = {u : angle(u, t_g) <= theta} plus the apex 0
  double coneSlack(const Eigen::Vector3d &v, const Eigen::Vector3d &t_g, double theta, const ModelConstants &c);
  // R1c: eq. (34) whose turn term uses h_prev as the previous heading while |v_prev| < v_min
  double gammaV(const Eigen::Vector3d &v, const Eigen::Vector3d &v_prev, double lambda, const ModelConstants &c,
                const Eigen::Vector3d &h_prev);

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
