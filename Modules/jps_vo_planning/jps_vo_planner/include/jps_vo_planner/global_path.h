#ifndef JPS_VO_PLANNER_GLOBAL_PATH_H
#define JPS_VO_PLANNER_GLOBAL_PATH_H

// Global path Pi on the jump-point graph G_J with its edge weights w_e (Stage-2 eqs. 10, 23, 27). ROS-free.

#include <Eigen/Eigen>
#include <cstdint>
#include <vector>

namespace jps_vo
{
  struct PathEdge
  {
    Eigen::Vector3i from_cell, to_cell;
    Eigen::Vector3d a, b; // cell centres
    Eigen::Vector3d t;    // unit tangent t_i (eq. 10)
    double ell = 0.0;     // ell_i
    int dir_in = 13;      // incoming direction index (Jps3D), 13 = none
    int dir_out = 13;
    double w = 0.0;       // edge weight w_e (eq. 39), refreshed every cycle
    uint64_t key = 0;     // identity of the G_J state transition (from, to, dir_in)
    double kappa = 0.0;   // [fix R2b] kappa part of w when the path was searched (or refreshed)
  };

  class GlobalPath
  {
  public:
    std::vector<PathEdge> edges;

    bool empty() const { return edges.empty(); }
    double length() const { return cum_.empty() ? 0.0 : cum_.back(); }
    void finalize(); // recompute cumulative arc lengths and the path hash
    uint64_t hash() const { return hash_; }
    std::vector<Eigen::Vector3d> points() const;

    struct Projection
    {
      double s = 0.0;    // s_pi(q)
      double dist = 0.0; // ||q - Pi(s_pi(q))||
      int edge = 0;
      Eigen::Vector3d foot = Eigen::Vector3d::Zero();
    };
    Projection project(const Eigen::Vector3d &q) const; // global nearest point on the polyline

    double costToGo(double s) const;                                    // C_Pi(s), eq. (23), omega = w_e / ell_e
    double value(const Eigen::Vector3d &q, double L_s) const;           // V_Pi(q), eq. (23)
    double costToGoNoKappa(double s) const;                             // [fix R2b] C_Pi(s) with w_e - kappa_e
    double valueNoKappa(const Eigen::Vector3d &q, double L_s) const;    // [fix R2b] V_Pi(q) with w_e - kappa_e
    Eigen::Vector3d guideTangent(double s, double ell_w) const;         // t_g(s), eq. (27)
    int edgeAt(double s) const;

    // symmetric Hausdorff distance between two polylines, sampled every `step` metres
    static double hausdorff(const GlobalPath &a, const GlobalPath &b, double step = 0.1);

  private:
    std::vector<double> cum_; // cum_[i] = arc length at the start of edge i, cum_.back() = total
    uint64_t hash_ = 0;
  };

} // namespace jps_vo

#endif
