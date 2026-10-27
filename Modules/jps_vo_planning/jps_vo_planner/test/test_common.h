#ifndef JPS_VO_PLANNER_TEST_COMMON_H
#define JPS_VO_PLANNER_TEST_COMMON_H

// Shared helpers for the offline tests (no ROS master needed)
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "jps_vo_planner/occupancy_grid.h"

namespace test_common
{
  static int failures = 0;

  inline void check(bool ok, const std::string &what)
  {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (!ok)
      ++failures;
  }

  inline int finish()
  {
    std::printf("%s (%d failures)\n", failures ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", failures);
    return failures ? 1 : 0;
  }

  struct Cylinder
  {
    double x, y, r, z_min, z_max;
  };

  // config/scenarios/planning_easy_cylinders.yaml, one "{name: .., x: .., y: .., r: .., z_min: .., z_max: ..}" per line
  inline std::vector<Cylinder> loadCylinders(const std::string &path)
  {
    std::vector<Cylinder> out;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line))
    {
      Cylinder c;
      char name[64];
      if (std::sscanf(line.c_str(), " - {name: %63[^,], x: %lf, y: %lf, r: %lf, z_min: %lf, z_max: %lf}", name, &c.x, &c.y,
                      &c.r, &c.z_min, &c.z_max) == 6)
        out.push_back(c);
    }
    return out;
  }

  // octomap-like 0.05 m voxel centres inside the cylinders, clipped to the flight band
  inline std::vector<Eigen::Vector3d> cylinderVoxels(const std::vector<Cylinder> &cyl)
  {
    std::vector<Eigen::Vector3d> pts;
    for (const auto &c : cyl)
      for (int i = static_cast<int>(std::floor((c.x - c.r) / 0.05)); i <= static_cast<int>(std::ceil((c.x + c.r) / 0.05)); ++i)
        for (int j = static_cast<int>(std::floor((c.y - c.r) / 0.05)); j <= static_cast<int>(std::ceil((c.y + c.r) / 0.05)); ++j)
        {
          const double x = (i + 0.5) * 0.05, y = (j + 0.5) * 0.05;
          if ((x - c.x) * (x - c.x) + (y - c.y) * (y - c.y) > c.r * c.r)
            continue;
          for (int k = static_cast<int>(std::floor(std::max(c.z_min, 0.5) / 0.05)); (k + 0.5) * 0.05 < std::min(c.z_max, 3.0); ++k)
            pts.emplace_back(x, y, (k + 0.5) * 0.05);
        }
    return pts;
  }

  inline std::string cylinderFile(int argc, char **argv)
  {
    return argc > 1 ? argv[1] : std::string(JPS_VO_PKG_DIR) + "/config/scenarios/planning_easy_cylinders.yaml";
  }

} // namespace test_common

#endif
