#include "jps_vo_planner/local_planner.h"

#include <algorithm>
#include <cmath>

namespace jps_vo
{
  namespace
  {
    // fixed unit directions: 26 grid directions + 48-point Fibonacci sphere (deterministic)
    const std::vector<Eigen::Vector3d> &unitDirections()
    {
      static std::vector<Eigen::Vector3d> dirs = [] {
        std::vector<Eigen::Vector3d> d;
        for (int x = -1; x <= 1; ++x)
          for (int y = -1; y <= 1; ++y)
            for (int z = -1; z <= 1; ++z)
              if (x || y || z)
                d.push_back(Eigen::Vector3d(x, y, z).normalized());
        const int n = 48;
        const double ga = M_PI * (3.0 - std::sqrt(5.0));
        for (int i = 0; i < n; ++i)
        {
          const double z = 1.0 - 2.0 * (i + 0.5) / n, r = std::sqrt(1.0 - z * z);
          d.emplace_back(r * std::cos(ga * i), r * std::sin(ga * i), z);
        }
        return d;
      }();
      return dirs;
    }
  } // namespace

  Eigen::Vector3d LocalPlanner::goalDir(const LocalProblem &P) const
  {
    const Eigen::Vector3d g = P.goal - P.p_ref;
    return g.norm() > 1e-9 ? Eigen::Vector3d(g.normalized()) : Eigen::Vector3d::Zero();
  }

  double LocalPlanner::speedLimit(const LocalProblem &P) const
  {
    return std::min(c_.v_c, std::sqrt(2.0 * c_.a_max * (P.goal - P.p_ref).norm()));
  }

  bool LocalPlanner::admissible(const LocalProblem &P, const Eigen::Vector3d &v) const
  {
    return v.norm() <= speedLimit(P) + 1e-9 && (v - P.v_prev).norm() <= c_.a_max * c_.dt + 1e-9;
  }

  bool LocalPlanner::turnOk(const LocalProblem &P, const Eigen::Vector3d &v) const
  {
    if (v.norm() < c_.v_min || P.v_prev.norm() < c_.v_min)
      return true;
    return v.normalized().dot(P.v_prev.normalized()) >= std::cos(c_.theta_max) - 1e-12;
  }

  bool LocalPlanner::staticOk(const LocalProblem &P, const Eigen::Vector3d &v) const
  {
    return P.grid == nullptr || P.grid->segmentFree(P.p_ref, P.p_ref + c_.tau_h * v);
  }

  double LocalPlanner::minTtc(const LocalProblem &P, const Eigen::Vector3d &v) const
  {
    double t = kInf;
    for (const auto &o : P.obs)
      t = std::min(t, timeToCollision(o.p - P.p_ref, v - o.v, c_.safetyRadius(o.r)));
    return t;
  }

  bool LocalPlanner::dynamicOk(const LocalProblem &P, const Eigen::Vector3d &v) const
  {
    return minTtc(P, v) > c_.tau2; // v not in VO^{tau2}_{A|o}(R_o) for all o
  }

  std::vector<Eigen::Vector3d> LocalPlanner::candidates(const LocalProblem &P) const
  {
    const double r = c_.a_max * c_.dt;
    const double vlim = speedLimit(P);
    std::vector<Eigen::Vector3d> raw;
    raw.push_back(P.v_prev);
    for (double f : {1.0 / 3.0, 2.0 / 3.0, 1.0})
      for (const auto &u : unitDirections())
        raw.push_back(P.v_prev + f * r * u);
    // special targets projected into V_adm
    Eigen::Vector3d ray;
    if (rayClosedForm(P, ray))
      raw.push_back(ray);
    raw.push_back(c_.v_c * goalDir(P));
    raw.push_back(c_.v_c * P.t_g);
    raw.push_back(Eigen::Vector3d::Zero());

    std::vector<Eigen::Vector3d> out;
    for (Eigen::Vector3d v : raw)
    {
      const Eigen::Vector3d dv = v - P.v_prev;
      if (dv.norm() > r)
        v = P.v_prev + dv * (r / dv.norm()); // onto B(v_prev, a_max dt)
      if (v.norm() > vlim)
        v *= vlim / v.norm(); // onto ||v|| <= limit
      if (admissible(P, v))
        out.push_back(v);
    }
    if (out.empty())
    {
      // the goal-stop limit fell below ||v_prev|| - a_max dt (discretisation): brake at a_max [ENG-3]
      const double sp = P.v_prev.norm();
      out.push_back(sp > r ? Eigen::Vector3d(P.v_prev * ((sp - r) / sp)) : Eigen::Vector3d::Zero());
    }
    return out;
  }

  bool LocalPlanner::rayClosedForm(const LocalProblem &P, Eigen::Vector3d &v) const
  {
    if (P.t_g.norm() < 1e-9)
      return false;
    const Eigen::Vector3d t = P.t_g.normalized();
    const double r = c_.a_max * c_.dt;
    // ||s t - v_prev||^2 <= r^2  <=>  s in [b - sqrt(disc), b + sqrt(disc)]
    const double b = t.dot(P.v_prev);
    const double disc = b * b - P.v_prev.squaredNorm() + r * r;
    if (disc < 0.0)
      return false;
    const double lo = std::max(0.0, b - std::sqrt(disc));
    const double s = std::min(b + std::sqrt(disc), speedLimit(P));
    if (s < lo - 1e-12)
      return false;
    v = s * t;
    return true;
  }

  LocalSolution LocalPlanner::evaluate(const LocalProblem &P, const Eigen::Vector3d &v) const
  {
    LocalSolution s;
    s.v = v;
    const double lam = P.lambda;
    const Eigen::Vector3d q = P.p_ref + c_.tau_h * v; // q(v)

    // J_A = V_Pi(q(v)) (23)(24); no global path -> straight-line distance to the goal [ENG-9]
    if (lam < 1.0)
    {
      if (P.path && !P.path->empty())
        s.J_A = sw_.fx.r2b_kappa_local_excluded ? P.path->valueNoKappa(q, P.L_s) : P.path->value(q, P.L_s); // [fix R2b]
      else
        s.J_A = (P.goal - q).norm() / P.L_s;
    }

    // J_B = ||q_pref - q(v)|| / L_s + rho(sigma(v)) (24)
    if (lam > 0.0)
    {
      const Eigen::Vector3d q_pref = P.p_ref + c_.tau_h * c_.v_c * goalDir(P);
      s.J_B = (q_pref - q).norm() / P.L_s;
      const bool drop_rho = sw_.lambda_mode == Switches::LambdaMode::One && !sw_.use_rho_in_pure_vo;
      if (!drop_rho)
      {
        Segment sg;
        sg.q = P.p_ref;
        sg.ell = c_.tau_h * v.norm();
        sg.d = v.norm() > 1e-12 ? Eigen::Vector3d(v.normalized()) : Eigen::Vector3d::Zero();
        sg.t_rel = 0.0;
        s.rho = segmentRisk(sg, P.obs, c_).rho;
        if (!(sw_.fx.s1_eps_constraint && lam > 0.0)) // [fix S1] the risk becomes a budget, not a weighted term
          s.J_B += s.rho;
      }
    }

    if (sw_.enable_gamma) // (34); [fix R1c] turn term against the last heading through a low-speed phase
      s.Gamma = sw_.fx.r1c_heading_memory ? gammaV(v, P.v_prev, lam, c_, P.h_prev) : gammaV(v, P.v_prev, lam, c_);

    // (28): minimal slack, s_g = max(0, angle(v, t_g) - theta_c)
    if (sw_.enable_c3 && P.t_g.norm() > 1e-9 && v.norm() > 1e-12)
    {
      if (!(sw_.fx.r1a_reachable_cone || sw_.fx.r1b_continuous_slack || sw_.fx.r1d_feasible_cone || sw_.fx.r5_mode_gating))
      {
        s.s_g = std::max(0.0, angleBetween(v, P.t_g) - thetaC(lam, sw_));
        s.ang_excess = s.s_g;
      }
      else
      {
        // [fix R1a/R5] cone half-angle; [fix R1b] slack = distance to the cone in velocity space / v_c
        s.theta_eff = coneTheta(P, lam);
        s.ang_excess = std::max(0.0, angleBetween(v, P.t_g) - s.theta_eff);
        s.s_g = sw_.fx.r1b_continuous_slack ? coneSlack(v, P.t_g, s.theta_eff, c_) : s.ang_excess;
      }
    }

    s.J = (1.0 - lam) * s.J_A + lam * s.J_B + s.Gamma + c_.rho_g * s.s_g; // (36)(37)
    if (sw_.c3_soft_tracking > 0.0)
      s.J += sw_.c3_soft_tracking * (v - c_.v_c * P.t_g).squaredNorm(); // A6 only
    s.J_ws = (sw_.fx.s1_eps_constraint && lam > 0.0) ? s.J + lam * s.rho : s.J; // [fix S1] frozen weighted-sum value
    return s;
  }

  double LocalPlanner::coneTheta(const LocalProblem &P, double lam) const
  {
    double th = thetaC(lam, sw_);
    if (sw_.fx.r5_mode_gating && P.free_mode) // [fix R5] corridor tolerance e_cut over the guidance window ell_W
      th = std::max(th, std::atan(c_.e_cut / c_.ellW()));
    if (sw_.fx.r1a_reachable_cone) // [fix R1a] never narrower than the heading lag that one cycle cannot remove
      th = std::max(th, angleBetween(P.v_prev, P.t_g) - reachTurn(P.v_prev.norm(), c_));
    if (sw_.fx.r1d_feasible_cone && P.floor_set) // [fix R1d] the cone never excludes every feasible moving velocity
      th = std::max(th, P.theta_floor);
    return std::min(th, M_PI);
  }

  double LocalPlanner::feasibleConeFloor(const LocalProblem &P) const
  {
    // smallest angle to t_g over the moving candidates that satisfy (38); 0 when none moves (the cone is then moot)
    const bool dyn_active = P.lambda > 0.0;
    double amin = M_PI;
    bool any = false;
    for (const auto &v : candidates(P))
    {
      if (v.norm() <= 1e-9 || !turnOk(P, v) || !staticOk(P, v) || (dyn_active && !dynamicOk(P, v)))
        continue;
      amin = std::min(amin, angleBetween(v, P.t_g));
      any = true;
    }
    return any ? amin : 0.0;
  }

  LocalSolution LocalPlanner::solve(const LocalProblem &P) const
  {
    if (sw_.fx.r1d_feasible_cone && !P.floor_set && sw_.enable_c3 && P.t_g.norm() > 1e-9)
    {
      LocalProblem Pf = P; // [fix R1d] widen the cone to the closest feasible moving candidate, then solve as usual
      Pf.theta_floor = feasibleConeFloor(P);
      Pf.floor_set = true;
      return solve(Pf);
    }
    if (sw_.fx.s1_eps_constraint && P.lambda > 0.0)
      return solveBudget(P); // [fix S1]
    const auto cand = candidates(P);
    const bool dyn_active = P.lambda > 0.0; // (C-s) dynamic part "active iff lambda > 0"
    LocalSolution best;
    bool found = false;
    int n_feas = 0;
    for (const auto &v : cand)
    {
      if (!turnOk(P, v) || !staticOk(P, v) || (dyn_active && !dynamicOk(P, v)))
        continue;
      ++n_feas;
      const LocalSolution s = evaluate(P, v);
      if (!found || s.J < best.J - 1e-12)
      {
        best = s;
        found = true;
      }
    }
    // [ENG-4] refinement: the exact-penalty optimum of the continuous problem lies on the cone surface when the cone
    // meets F; the sampled set may only hold points just outside it. Add the projection of the incumbent onto the
    // cone (same speed, rotated towards t_g by s_g) when it satisfies (38), and keep it if J~ improves.
    // ang_excess == s_g bit for bit when R1a/R1b/R5 are off (frozen behaviour)
    for (int it = 0; found && it < 3 && best.ang_excess > 0.0; ++it)
    {
      const Eigen::Vector3d axis = best.v.cross(P.t_g);
      Eigen::Vector3d vp;
      if (axis.norm() > 1e-12)
        vp = Eigen::AngleAxisd(best.ang_excess, axis.normalized()) * best.v;
      else
        vp = best.v.norm() * P.t_g.normalized(); // antiparallel: rotate fully onto t_g
      if (!admissible(P, vp) && vp.norm() > 1e-12)
      {
        // same cone generator u, speed moved into [b - sqrt(disc), b + sqrt(disc)] ∩ [0, limit] (C-a)(C-v)
        const Eigen::Vector3d u = vp.normalized();
        const double r = c_.a_max * c_.dt, b = u.dot(P.v_prev);
        const double disc = b * b - P.v_prev.squaredNorm() + r * r;
        if (disc < 0.0)
          break;
        const double lo = std::max(0.0, b - std::sqrt(disc)), hi = std::min(b + std::sqrt(disc), speedLimit(P));
        if (hi < lo)
          break;
        vp = std::min(hi, std::max(lo, best.v.norm())) * u;
      }
      if (!admissible(P, vp) || !turnOk(P, vp) || !staticOk(P, vp) || (dyn_active && !dynamicOk(P, vp)))
        break;
      const LocalSolution s = evaluate(P, vp);
      if (s.J >= best.J - 1e-12)
        break;
      best = s;
      ++n_feas;
    }
    if (!found)
      best = fallback(P, cand);
    best.n_candidates = static_cast<int>(cand.size());
    best.n_feasible = n_feas;
    return best;
  }

  LocalSolution LocalPlanner::solveBudget(const LocalProblem &P) const
  {
    // [fix S1] epsilon-constraint form of P_loc: argmin of J~ without the weighted rho over the candidates whose risk
    // stays within rho <= 1 - lambda; when no feasible candidate meets the budget, the frozen weighted sum decides.
    const auto cand = candidates(P);
    const bool dyn_active = P.lambda > 0.0;
    const double budget = 1.0 - P.lambda;
    std::vector<LocalSolution> feas;
    for (const auto &v : cand)
      if (turnOk(P, v) && staticOk(P, v) && (!dyn_active || dynamicOk(P, v)))
        feas.push_back(evaluate(P, v));
    bool any_budget = false;
    for (const auto &s : feas)
      any_budget = any_budget || s.rho <= budget + 1e-12;
    auto obj = [&](const LocalSolution &s) { return any_budget ? s.J : s.J_ws; };
    auto admit = [&](const LocalSolution &s) { return !any_budget || s.rho <= budget + 1e-12; };
    LocalSolution best;
    bool found = false;
    int n_feas = static_cast<int>(feas.size());
    for (const auto &s : feas)
      if (admit(s) && (!found || obj(s) < obj(best) - 1e-12))
      {
        best = s;
        found = true;
      }
    // [ENG-4] refinement onto the cone surface, with the same objective and admission rule
    for (int it = 0; found && it < 3 && best.ang_excess > 0.0; ++it)
    {
      const Eigen::Vector3d axis = best.v.cross(P.t_g);
      Eigen::Vector3d vp;
      if (axis.norm() > 1e-12)
        vp = Eigen::AngleAxisd(best.ang_excess, axis.normalized()) * best.v;
      else
        vp = best.v.norm() * P.t_g.normalized();
      if (!admissible(P, vp) && vp.norm() > 1e-12)
      {
        const Eigen::Vector3d u = vp.normalized();
        const double r = c_.a_max * c_.dt, b = u.dot(P.v_prev);
        const double disc = b * b - P.v_prev.squaredNorm() + r * r;
        if (disc < 0.0)
          break;
        const double lo = std::max(0.0, b - std::sqrt(disc)), hi = std::min(b + std::sqrt(disc), speedLimit(P));
        if (hi < lo)
          break;
        vp = std::min(hi, std::max(lo, best.v.norm())) * u;
      }
      if (!admissible(P, vp) || !turnOk(P, vp) || !staticOk(P, vp) || (dyn_active && !dynamicOk(P, vp)))
        break;
      const LocalSolution s = evaluate(P, vp);
      if (!admit(s) || obj(s) >= obj(best) - 1e-12)
        break;
      best = s;
      ++n_feas;
    }
    if (!found)
      best = fallback(P, cand);
    best.budget_active = any_budget;
    best.n_candidates = static_cast<int>(cand.size());
    best.n_feasible = n_feas;
    return best;
  }

  LocalSolution LocalPlanner::classicVO(const LocalProblem &P) const
  {
    const auto cand = candidates(P);
    const Eigen::Vector3d v_pref = c_.v_c * goalDir(P);
    LocalSolution best;
    bool found = false;
    double best_d = 0.0;
    int n_feas = 0;
    for (const auto &v : cand)
    {
      if (!turnOk(P, v) || !staticOk(P, v) || !dynamicOk(P, v))
        continue;
      ++n_feas;
      const double d = (v - v_pref).norm();
      if (!found || d < best_d - 1e-12)
      {
        best_d = d;
        best.v = v;
        found = true;
      }
    }
    if (!found)
      best = fallback(P, cand);
    best.n_candidates = static_cast<int>(cand.size());
    best.n_feasible = n_feas;
    return best;
  }

  LocalSolution LocalPlanner::fallback(const LocalProblem &P, const std::vector<Eigen::Vector3d> &cand) const
  {
    // eq. (40): argmax over V_adm of min_o t_c, tie-break ||v - v_pref||; restricted to statically free
    // candidates when any exist [ENG-6]; with none free, escape towards the nearest free cell [ENG-10]
    LocalSolution s;
    s.fallback = true;
    std::vector<Eigen::Vector3d> pool;
    for (const auto &v : cand)
      if (staticOk(P, v))
        pool.push_back(v);
    const bool any_free = !pool.empty();
    if (!any_free)
      pool = cand;
    const Eigen::Vector3d v_pref = c_.v_c * goalDir(P);
    // no statically free candidate: the reference is inside the inflated region; braking would keep it there
    // forever, so head for the nearest free cell instead [ENG-10]
    Eigen::Vector3d v_escape = Eigen::Vector3d::Zero();
    Eigen::Vector3i free_cell;
    if (!any_free && P.grid && P.grid->nearestFree(P.p_ref, 20, free_cell))
    {
      const Eigen::Vector3d d = P.grid->indexToPos(free_cell) - P.p_ref;
      if (d.norm() > 1e-9)
        v_escape = speedLimit(P) * d.normalized();
    }
    bool found = false;
    double best_t = -1.0, best_tie = 0.0;
    for (const auto &v : pool)
    {
      const double t = any_free ? minTtc(P, v) : 0.0;
      const double tie = any_free ? (v - v_pref).norm() : (v - v_escape).norm();
      const bool better = !found || t > best_t + 1e-9 || (std::fabs(t - best_t) <= 1e-9 && tie < best_tie - 1e-12) ||
                          (std::isinf(t) && std::isinf(best_t) && tie < best_tie - 1e-12);
      if (better)
      {
        best_t = t;
        best_tie = tie;
        s.v = v;
        found = true;
      }
    }
    if (!found)
      s.v = P.v_prev; // empty V_adm cannot happen (v_prev itself is admissible unless above the goal limit)
    return s;
  }

} // namespace jps_vo
