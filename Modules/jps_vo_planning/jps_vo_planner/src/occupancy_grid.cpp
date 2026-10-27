#include "jps_vo_planner/occupancy_grid.h"

#include <algorithm>
#include <cmath>

namespace jps_vo
{
  void OccupancyGrid::init(const Params &p)
  {
    p_ = p;
    for (int i = 0; i < 3; ++i)
      dims_(i) = static_cast<int>(std::lround(p_.size(i) / p_.resolution));
    const size_t n = static_cast<size_t>(dims_.x()) * dims_.y() * dims_.z();
    raw_.assign(n, 0);
    inflated_.assign(n, 0);
    raw_cells_.clear();
    num_raw_ = 0;
    buildKernel();
  }

  void OccupancyGrid::buildKernel()
  {
    // ball, centre-to-centre distance <= r_infl; the discretisation error (<= delta * sqrt(3) / 2) must be
    // covered by d_min, see troubleshooting checklist G-4
    kernel_.clear();
    const int k = static_cast<int>(std::ceil(p_.inflation / p_.resolution));
    const double r2 = (p_.inflation / p_.resolution) * (p_.inflation / p_.resolution) + 1e-9;
    for (int x = -k; x <= k; ++x)
      for (int y = -k; y <= k; ++y)
        for (int z = -k; z <= k; ++z)
          if (x * x + y * y + z * z <= r2)
            kernel_.emplace_back(x, y, z);
  }

  Eigen::Vector3i OccupancyGrid::posToIndex(const Eigen::Vector3d &pos) const
  {
    Eigen::Vector3i n;
    for (int i = 0; i < 3; ++i)
      n(i) = static_cast<int>(std::floor((pos(i) - p_.origin(i)) / p_.resolution));
    return n;
  }

  Eigen::Vector3d OccupancyGrid::indexToPos(const Eigen::Vector3i &n) const
  {
    return p_.origin + (n.cast<double>() + Eigen::Vector3d::Constant(0.5)) * p_.resolution;
  }

  size_t OccupancyGrid::rebuild(const std::vector<Eigen::Vector3d> &points, const std::vector<Capsule> &dyn_masks)
  {
    // clear only what was set last time
    for (size_t c : raw_cells_)
      raw_[c] = 0;
    raw_cells_.clear();
    std::fill(inflated_.begin(), inflated_.end(), 0);

    size_t used = 0;
    for (const auto &pt : points)
    {
      bool masked = false;
      for (const auto &cap : dyn_masks)
      {
        const Eigen::Vector3d ab = cap.b - cap.a;
        const double L2 = ab.squaredNorm();
        const double t = L2 > 0.0 ? std::min(1.0, std::max(0.0, (pt - cap.a).dot(ab) / L2)) : 0.0;
        if ((cap.a + t * ab - pt).norm() <= cap.radius)
        {
          masked = true;
          break;
        }
      }
      if (masked)
        continue;

      const Eigen::Vector3i n = posToIndex(pt);
      if (!inside(n))
        continue;
      const size_t c = flat(n);
      if (!raw_[c])
      {
        raw_[c] = 1;
        raw_cells_.push_back(c);
      }
      ++used;
    }
    num_raw_ = raw_cells_.size();

    for (size_t c : raw_cells_)
    {
      const int x = static_cast<int>(c % dims_.x());
      const int y = static_cast<int>((c / dims_.x()) % dims_.y());
      const int z = static_cast<int>(c / (static_cast<size_t>(dims_.x()) * dims_.y()));
      for (const auto &o : kernel_)
      {
        const Eigen::Vector3i m(x + o.x(), y + o.y(), z + o.z());
        if (inside(m))
          inflated_[flat(m)] = 1;
      }
    }
    return used;
  }

  bool OccupancyGrid::segmentFree(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1) const
  {
    const double L = (p1 - p0).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(L / (0.5 * p_.resolution))));
    for (int i = 0; i <= steps; ++i)
      if (!isFree(Eigen::Vector3d(p0 + (p1 - p0) * (static_cast<double>(i) / steps))))
        return false;
    return true;
  }

  bool OccupancyGrid::nearestFree(const Eigen::Vector3d &pos, int max_radius, Eigen::Vector3i &out) const
  {
    const Eigen::Vector3i c = posToIndex(pos);
    double best = 1e18;
    bool found = false;
    for (int r = 0; r <= max_radius; ++r)
    {
      // a cell in Chebyshev shell r is at least (r - 0.5) * delta away from pos, so once that exceeds the best
      // distance found, no further shell can be closer
      if (found && (r - 0.5) * p_.resolution > std::sqrt(best))
        break;
      for (int x = -r; x <= r; ++x)
        for (int y = -r; y <= r; ++y)
          for (int z = -r; z <= r; ++z)
          {
            if (std::max(std::abs(x), std::max(std::abs(y), std::abs(z))) != r)
              continue; // shell r only
            const Eigen::Vector3i n(c.x() + x, c.y() + y, c.z() + z);
            if (!isFree(n))
              continue;
            const double d = (indexToPos(n) - pos).squaredNorm();
            if (d < best)
            {
              best = d;
              out = n;
              found = true;
            }
          }
    }
    return found;
  }

} // namespace jps_vo
