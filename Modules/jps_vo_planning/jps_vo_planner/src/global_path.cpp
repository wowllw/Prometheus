#include "jps_vo_planner/global_path.h"

#include <algorithm>
#include <cmath>

namespace jps_vo
{
  void GlobalPath::finalize()
  {
    cum_.assign(1, 0.0);
    hash_ = 1469598103934665603ull; // FNV-1a over the jump-point cells
    auto mix = [this](int v) {
      hash_ ^= static_cast<uint64_t>(static_cast<uint32_t>(v));
      hash_ *= 1099511628211ull;
    };
    for (const auto &e : edges)
    {
      cum_.push_back(cum_.back() + e.ell);
      for (int i = 0; i < 3; ++i)
        mix(e.from_cell(i));
    }
    if (!edges.empty())
      for (int i = 0; i < 3; ++i)
        mix(edges.back().to_cell(i));
  }

  std::vector<Eigen::Vector3d> GlobalPath::points() const
  {
    std::vector<Eigen::Vector3d> pts;
    for (const auto &e : edges)
      pts.push_back(e.a);
    if (!edges.empty())
      pts.push_back(edges.back().b);
    return pts;
  }

  GlobalPath::Projection GlobalPath::project(const Eigen::Vector3d &q) const
  {
    Projection best;
    best.dist = 1e18;
    for (size_t i = 0; i < edges.size(); ++i)
    {
      const auto &e = edges[i];
      const double t = e.ell > 0.0 ? std::min(e.ell, std::max(0.0, (q - e.a).dot(e.t))) : 0.0;
      const Eigen::Vector3d foot = e.a + e.t * t;
      const double d = (q - foot).norm();
      if (d < best.dist - 1e-12)
      {
        best.dist = d;
        best.s = cum_[i] + t;
        best.edge = static_cast<int>(i);
        best.foot = foot;
      }
    }
    return best;
  }

  int GlobalPath::edgeAt(double s) const
  {
    if (edges.empty())
      return -1;
    for (size_t i = 0; i < edges.size(); ++i)
      if (s < cum_[i + 1])
        return static_cast<int>(i);
    return static_cast<int>(edges.size()) - 1;
  }

  double GlobalPath::costToGo(double s) const
  {
    double c = 0.0;
    for (size_t i = 0; i < edges.size(); ++i)
    {
      const auto &e = edges[i];
      if (e.ell <= 0.0)
        continue;
      const double lo = std::max(s, cum_[i]), hi = cum_[i + 1];
      if (hi > lo)
        c += (hi - lo) * e.w / e.ell; // integral of omega = w_e / ell_e
    }
    return c;
  }

  double GlobalPath::value(const Eigen::Vector3d &q, double L_s) const
  {
    const Projection p = project(q);
    return costToGo(p.s) + p.dist / L_s;
  }

  double GlobalPath::costToGoNoKappa(double s) const
  {
    double c = 0.0;
    for (size_t i = 0; i < edges.size(); ++i)
    {
      const auto &e = edges[i];
      if (e.ell <= 0.0)
        continue;
      const double lo = std::max(s, cum_[i]), hi = cum_[i + 1];
      if (hi > lo)
        c += (hi - lo) * (e.w - e.kappa) / e.ell;
    }
    return c;
  }

  double GlobalPath::valueNoKappa(const Eigen::Vector3d &q, double L_s) const
  {
    const Projection p = project(q);
    return costToGoNoKappa(p.s) + p.dist / L_s;
  }

  Eigen::Vector3d GlobalPath::guideTangent(double s, double ell_w) const
  {
    if (edges.empty())
      return Eigen::Vector3d::Zero();
    // normalised integral of t_Pi over [s, s + ell_w], truncated at the end of the path
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    const double hi_s = s + ell_w;
    for (size_t i = 0; i < edges.size(); ++i)
    {
      const double lo = std::max(s, cum_[i]), hi = std::min(hi_s, cum_[i + 1]);
      if (hi > lo)
        acc += edges[i].t * (hi - lo);
    }
    if (acc.norm() < 1e-9)
      return edges[std::max(0, edgeAt(std::min(s, length())))].t; // at / beyond the end: last tangent
    return acc.normalized();
  }

  double GlobalPath::hausdorff(const GlobalPath &a, const GlobalPath &b, double step)
  {
    auto sampled = [step](const GlobalPath &p) {
      std::vector<Eigen::Vector3d> pts;
      for (const auto &e : p.edges)
      {
        const int n = std::max(1, static_cast<int>(std::ceil(e.ell / step)));
        for (int k = 0; k < n; ++k)
          pts.push_back(e.a + e.t * (e.ell * k / n));
      }
      if (!p.edges.empty())
        pts.push_back(p.edges.back().b);
      return pts;
    };
    auto directed = [](const std::vector<Eigen::Vector3d> &from, const GlobalPath &to) {
      double h = 0.0;
      for (const auto &q : from)
        h = std::max(h, to.project(q).dist);
      return h;
    };
    if (a.empty() || b.empty())
      return a.empty() && b.empty() ? 0.0 : 1e18;
    return std::max(directed(sampled(a), b), directed(sampled(b), a));
  }

} // namespace jps_vo
