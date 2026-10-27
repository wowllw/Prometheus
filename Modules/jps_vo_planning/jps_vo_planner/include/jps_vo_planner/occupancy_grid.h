#ifndef JPS_VO_PLANNER_OCCUPANCY_GRID_H
#define JPS_VO_PLANNER_OCCUPANCY_GRID_H

#include <Eigen/Eigen>
#include <cstdint>
#include <vector>

namespace jps_vo
{
  // Static occupancy grid V_free of Stage-2 eq. (1): p(n) = o + delta * n, inflated by r_infl = r_q + d_min + e_cut.
  //
  // Alignment with the inputs (see doc/subtask3_porting_plan.md, section 2):
  //  - origin o = (-25, -25, ground_height 0.5) and delta = 0.1 are the ego_planner grid_map values, so the JPS
  //    grid and the baseline map share cell boundaries;
  //  - octomap (resolution 0.05, key origin 0) publishes voxel centres at (k + 0.5) * 0.05. With o a multiple of
  //    0.1, (p - o) / delta is always integer + 0.25 or + 0.75: every 0.1 cell contains exactly 2 x 2 x 2 octomap
  //    voxels and floor() never sits on a boundary.
  class OccupancyGrid
  {
  public:
    struct Params
    {
      Eigen::Vector3d origin{-25.0, -25.0, 0.5};
      Eigen::Vector3d size{50.0, 50.0, 2.5}; // z: ground_height 0.5 .. virtual ceiling 3.0
      double resolution = 0.1;               // delta
      double inflation = 0.3;                // r_infl = r_q + d_min + e_cut [m]
    };

    // Sphere swept by a dynamic obstacle; points inside are not static occupancy (D2: dynamic obstacles are
    // handled by VO, the voxel map holds static obstacles only)
    struct Capsule
    {
      Eigen::Vector3d a, b;
      double radius;
    };

    void init(const Params &p);

    // Rebuild from the full set of octomap occupied-voxel centres (octomap_point_cloud_centers publishes the
    // whole occupied set each time, so a full rebuild is exact). Points inside any mask capsule are dropped.
    // Returns the number of points that fell into a cell.
    size_t rebuild(const std::vector<Eigen::Vector3d> &points, const std::vector<Capsule> &dyn_masks);

    const Params &params() const { return p_; }
    const Eigen::Vector3i &dims() const { return dims_; }

    bool inside(const Eigen::Vector3i &n) const
    {
      return n.x() >= 0 && n.y() >= 0 && n.z() >= 0 && n.x() < dims_.x() && n.y() < dims_.y() && n.z() < dims_.z();
    }
    Eigen::Vector3i posToIndex(const Eigen::Vector3d &pos) const;
    Eigen::Vector3d indexToPos(const Eigen::Vector3i &n) const; // cell centre

    // n in V_free: inside the box and not occupied after inflation. Outside the box (below ground_height,
    // above the virtual ceiling, outside x/y) counts as occupied, as in ego_planner.
    bool isFree(const Eigen::Vector3i &n) const { return inside(n) && !inflated_[flat(n)]; }
    bool isFree(const Eigen::Vector3d &pos) const { return isFree(posToIndex(pos)); }
    bool isRawOccupied(const Eigen::Vector3i &n) const { return inside(n) && raw_[flat(n)]; }

    // Static half of (C-s): {p0 + t (p1 - p0), t in [0,1]} subset V_free, sampled at delta / 2
    bool segmentFree(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1) const;

    // nearest free cell centre within max_radius cells (start/goal snapping); false if none
    bool nearestFree(const Eigen::Vector3d &pos, int max_radius, Eigen::Vector3i &out) const;

    size_t numRawOccupied() const { return num_raw_; }

  private:
    size_t flat(const Eigen::Vector3i &n) const
    {
      return (static_cast<size_t>(n.z()) * dims_.y() + n.y()) * dims_.x() + n.x();
    }
    void buildKernel();

    Params p_;
    Eigen::Vector3i dims_{0, 0, 0};
    std::vector<uint8_t> raw_, inflated_;
    std::vector<Eigen::Vector3i> kernel_; // ball of radius r_infl (cell-centre distance)
    std::vector<size_t> raw_cells_;
    size_t num_raw_ = 0;
  };

} // namespace jps_vo

#endif
