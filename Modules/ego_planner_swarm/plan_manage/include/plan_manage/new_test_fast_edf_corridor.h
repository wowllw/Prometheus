#ifndef _NEW_TEST_FAST_EDF_CORRIDOR_H_
#define _NEW_TEST_FAST_EDF_CORRIDOR_H_

// FastEDFCorridor (secondary innovation)
//  Distance-field based safety check that replaces the grid occupancy query used by
//  BsplineOptimizer::initControlPoints (segmentation of the initial trajectory).
//
//  plan_env has no distance field, so this class owns a local one, built from GridMap.
//  Hooks that use this class are wrapped in USE_FAST_EDF_CORRIDOR (disabled by default).

#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <utility>
#include <ros/ros.h>
#include <plan_env/grid_map.h>

namespace ego_planner
{

  // returned by isOccupied() when the field cannot answer; the caller falls back to the grid query
  static const int FAST_EDF_UNAVAILABLE = -2;

  class FastEDFCorridor
  {
  public:
    FastEDFCorridor() {}
    ~FastEDFCorridor() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* setup: reads parameters under "fast_edf/" */
    void init(const GridMap::Ptr &grid_map, ros::NodeHandle &nh);

    /* rebuild the local distance field from the current GridMap occupancy (call once per planning cycle) */
    void updateField(void);

    /* distance to the nearest obstacle [m] at pos; negative value if unavailable */
    double getDistance(const Eigen::Vector3d &pos) const;

    /* drop-in replacement for GridMap::getInflateOccupancy(pos):
       -1 outside the map, 1 occupied (clearance below margin), 0 free, FAST_EDF_UNAVAILABLE if no field */
    int isOccupied(const Eigen::Vector3d &pos, const double margin) const;

    /* FastEDFCorridor: check a sampled initial path against the field.
       Inputs : sampled path points, buffer radius rho, vehicle radius
       Outputs: per-point margin beta, list of violating index segments (same type as initControlPoints returns) */
    bool FastEDFCorridorCheck(const std::vector<Eigen::Vector3d> &path, const double rho,
                              std::vector<double> &beta, std::vector<std::pair<int, int>> &segments) const;

    /* ParentEdgeAdmissible: is the edge between a parent point and a child point obstacle-free?
       Inputs : parent point, child point, buffer radius rho
       Outputs: admissible flag, minimum margin along the edge */
    bool ParentEdgeAdmissible(const Eigen::Vector3d &parent, const Eigen::Vector3d &child, const double rho,
                              double &min_margin) const;

    /* preliminary feasibility check: beta = clearance - required clearance; accepted only if beta >= 0 */
    double computeBeta(const double clearance, const double rho) const;

    inline bool isReady(void) const { return field_ready_; }

  private:
    GridMap::Ptr grid_map_;

    /* parameters */
    double vehicle_radius_;      // [m]
    double edge_sample_step_;    // spacing of samples along an edge [m]

    /* local distance field */
    std::vector<float> field_;   // distance values over the local update range
    Eigen::Vector3d field_origin_;
    Eigen::Vector3i field_size_;
    double resolution_;
    bool field_ready_{false};

  public:
    typedef std::unique_ptr<FastEDFCorridor> Ptr;
  };

} // namespace ego_planner

#endif
