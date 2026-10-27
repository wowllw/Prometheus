#include <plan_manage/new_test_fast_edf_corridor.h>

namespace ego_planner
{

  // SECTION setup

  void FastEDFCorridor::init(const GridMap::Ptr &grid_map, ros::NodeHandle &nh)
  {
    grid_map_ = grid_map;

    nh.param("fast_edf/vehicle_radius", vehicle_radius_, 0.2);
    nh.param("fast_edf/edge_sample_step", edge_sample_step_, 0.1);

    resolution_ = grid_map_->getResolution();
    field_origin_ = grid_map_->getOrigin();
    field_size_.setZero();
    field_.clear();
    field_ready_ = false;

    ROS_INFO("[FastEDFCorridor] init: vehicle_radius=%.3f, edge_sample_step=%.3f", vehicle_radius_, edge_sample_step_);
  }

  // !SECTION

  // SECTION distance field

  void FastEDFCorridor::updateField(void)
  {
    // Build the local distance field from the occupancy information in grid_map_
    // (use grid_map_->getOccupancy(...) on the raw, uninflated occupancy).

    /*TODO: Fill in using the corresponding formula from the paper*/
    // Euclidean distance transform over the local update range; fill field_,
    // field_origin_, field_size_ and set field_ready_ = true when done.

    field_ready_ = false; // placeholder: field is never ready, so queries fall back to the grid
  }

  double FastEDFCorridor::getDistance(const Eigen::Vector3d &pos) const
  {
    if (!field_ready_)
      return -1.0;

    /*TODO: Fill in using the corresponding formula from the paper*/
    // look up (or interpolate) the distance value at pos

    return -1.0; // placeholder
  }

  int FastEDFCorridor::isOccupied(const Eigen::Vector3d &pos, const double margin) const
  {
    if (!grid_map_->isInMap(pos))
      return -1; // same convention as GridMap::getInflateOccupancy

    if (!field_ready_)
      return FAST_EDF_UNAVAILABLE;

    double dist = getDistance(pos);
    if (dist < 0.0)
      return FAST_EDF_UNAVAILABLE;

    return (dist < margin) ? 1 : 0;
  }

  // !SECTION

  // SECTION corridor check

  double FastEDFCorridor::computeBeta(const double clearance, const double rho) const
  {
    /*TODO: Fill in using the corresponding formula from the paper*/
    // beta = clearance - (rho + vehicle_radius_ + effective radius term)

    (void)clearance;
    (void)rho;
    return 0.0; // placeholder
  }

  bool FastEDFCorridor::FastEDFCorridorCheck(const std::vector<Eigen::Vector3d> &path, const double rho,
                                             std::vector<double> &beta,
                                             std::vector<std::pair<int, int>> &segments) const
  {
    beta.assign(path.size(), 0.0);
    segments.clear();

    if (!field_ready_)
      return false;

    int in_id = -1;
    for (size_t i = 0; i < path.size(); ++i)
    {
      beta[i] = computeBeta(getDistance(path[i]), rho);

      // preliminary feasibility check: beta < 0 -> reject immediately
      bool violated = (beta[i] < 0.0);

      if (violated && in_id < 0)
      {
        in_id = (int)i;
      }
      else if (!violated && in_id >= 0)
      {
        segments.push_back(std::pair<int, int>(in_id, (int)i - 1));
        in_id = -1;
      }
    }
    if (in_id >= 0)
    {
      segments.push_back(std::pair<int, int>(in_id, (int)path.size() - 1));
    }

    return true;
  }

  bool FastEDFCorridor::ParentEdgeAdmissible(const Eigen::Vector3d &parent, const Eigen::Vector3d &child,
                                             const double rho, double &min_margin) const
  {
    min_margin = 0.0;

    if (!field_ready_)
      return false;

    double length = (child - parent).norm();
    int n_samples = (length > edge_sample_step_) ? (int)(length / edge_sample_step_) : 1;

    bool admissible = true;
    min_margin = 1e9;
    for (int k = 0; k <= n_samples; ++k)
    {
      double a = (double)k / (double)n_samples;
      Eigen::Vector3d pt = (1.0 - a) * parent + a * child;

      double beta = computeBeta(getDistance(pt), rho);
      if (beta < min_margin)
        min_margin = beta;

      // preliminary feasibility check: beta < 0 -> reject immediately
      if (beta < 0.0)
      {
        admissible = false;
        break;
      }
    }

    return admissible;
  }

  // !SECTION

} // namespace ego_planner
