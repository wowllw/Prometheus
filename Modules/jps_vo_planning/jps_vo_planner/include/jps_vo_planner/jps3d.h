#ifndef JPS_VO_PLANNER_JPS3D_H
#define JPS_VO_PLANNER_JPS3D_H

// 3D Jump Point Search on the static grid V_free (Stage-2 section 1.1, eqs. 2-10), searched in the state space
// "jump point x incoming direction" (Proposition 1, limitation (ii)) so that direction-dependent edge weights
// (Gamma_e, eq. 35) keep A* correct. ROS-free.

#include <Eigen/Eigen>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "jps_vo_planner/occupancy_grid.h"

namespace jps_vo
{
  class Jps3D
  {
  public:
    // direction index: (dx+1)*9 + (dy+1)*3 + (dz+1); 13 is the zero vector and means "no incoming direction"
    static constexpr int kNoDir = 13;
    static Eigen::Vector3i dirVec(int idx);
    static int dirIdx(const Eigen::Vector3i &d);

    // edge weight w(x -> j) for a straight jump from cell `from` to cell `to` along dir_out, entered with dir_in
    using EdgeWeight = std::function<double(const Eigen::Vector3i &from, const Eigen::Vector3i &to, int dir_in, int dir_out)>;

    struct Result
    {
      bool ok = false;
      std::vector<Eigen::Vector3i> cells; // start, jump points ..., goal
      std::vector<int> dirs;              // dirs[i] = direction of segment cells[i] -> cells[i+1]
      double cost = 0.0;                  // sum of edge weights
      size_t expansions = 0;
    };

    // Tie rule of eq. (7) for diagonal arrivals: Literal prunes on "<=" as written; Strict prunes only on "<"
    // (Harabor & Grastien [2]). Offline test 1-a: Strict reproduces A* optimality on all maps, Literal does not.
    enum class TieRule { Strict, Literal };

    explicit Jps3D(const OccupancyGrid *grid, TieRule rule = TieRule::Strict) : grid_(grid), tie_rule_(rule) {}

    // [ENG-8] search region: an index box (e.g. start/goal bounding box + margin, full z). Cells outside count as
    // occupied for jumps and forced-neighbour tests alike, i.e. G_J is built on V_free restricted to the box.
    void setSearchBox(const Eigen::Vector3i &lo, const Eigen::Vector3i &hi)
    {
      box_lo_ = lo;
      box_hi_ = hi;
      use_box_ = true;
    }

    // A* over G_J with edge weights w and heuristic h_scale * h_D (eq. 5, metres). Requires w >= h_scale * length
    // (Proposition 1 (a)). dir_dependent = true: states are (jump point, incoming direction), required when w depends
    // on dir_in (Gamma_e, Proposition 1 limitation (ii)); false: standard JPS closure per cell (exact when w does
    // not depend on dir_in).
    Result search(const Eigen::Vector3i &start, const Eigen::Vector3i &goal, const EdgeWeight &w, double h_scale,
                  bool dir_dependent = true) const;
    // geometric JPS: w = segment length [m], h_scale = 1
    Result searchGeometric(const Eigen::Vector3i &start, const Eigen::Vector3i &goal) const;
    // independent reference: 26-connected A* with costs (2), legality (3), heuristic (5) [m]
    Result astar26(const Eigen::Vector3i &start, const Eigen::Vector3i &goal) const;

    // successors of state (x, dir_in) on G_J: (jump point, outgoing direction). Exposed for tests.
    void successors(const Eigen::Vector3i &x, int dir_in, const Eigen::Vector3i &goal,
                    std::vector<std::pair<Eigen::Vector3i, int>> &out) const;

    double hD(const Eigen::Vector3i &n, const Eigen::Vector3i &goal) const; // eq. (5) [m]
    bool legal(const Eigen::Vector3i &x, const Eigen::Vector3i &d) const;   // eq. (3)
    double cellSize() const { return grid_->params().resolution; }

  private:
    // bitmask over direction indices of the forced neighbours F(x, d) (eq. 7 inside the 3x3x3 window)
    uint32_t forcedMask(const Eigen::Vector3i &x, const Eigen::Vector3i &d) const;
    bool jump(const Eigen::Vector3i &x, const Eigen::Vector3i &d, const Eigen::Vector3i &goal, Eigen::Vector3i &out) const;

    bool inside(const Eigen::Vector3i &c) const { return grid_->inside(c); }
    bool freeCell(const Eigen::Vector3i &c) const
    {
      if (use_box_ && (c.x() < box_lo_.x() || c.y() < box_lo_.y() || c.z() < box_lo_.z() || c.x() > box_hi_.x() ||
                       c.y() > box_hi_.y() || c.z() > box_hi_.z()))
        return false;
      return grid_->isFree(c);
    }

    const OccupancyGrid *grid_;
    TieRule tie_rule_;
    bool use_box_ = false;
    Eigen::Vector3i box_lo_{0, 0, 0}, box_hi_{0, 0, 0};
    mutable std::unordered_map<uint64_t, uint32_t> forced_cache_;
    // jump(x, d) memo for one goal: every cell visited along a scan in direction d reaches the same result
    // (same stop point or none), so the result is stored for all of them. Valid while grid, box and goal are fixed.
    mutable std::unordered_map<uint64_t, int64_t> jump_memo_;
    mutable Eigen::Vector3i memo_goal_{-1, -1, -1};
  };

} // namespace jps_vo

#endif
