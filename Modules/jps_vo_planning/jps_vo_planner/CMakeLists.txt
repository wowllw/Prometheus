cmake_minimum_required(VERSION 3.0.2)
project(jps_vo_planner)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  gazebo_msgs
  traj_utils
  prometheus_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs gazebo_msgs traj_utils prometheus_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

find_package(Threads REQUIRED)

# ROS-free core (Stage-2 model, 3D-JPS, VO, coupling); shared by the node and the offline tests
add_library(jps_vo_core STATIC
  src/coupling.cpp
  src/jps3d.cpp
  src/occupancy_grid.cpp
  src/global_path.cpp
  src/local_planner.cpp
  src/planner_core.cpp
  src/bspline_utils.cpp
)
target_link_libraries(jps_vo_core Threads::Threads)

# planner node: planner_mode stub (Subtask 2) | coupled (Subtask 3)
add_executable(jps_vo_planner_node
  src/jps_vo_planner_node.cpp
  src/planner_node.cpp
  src/planner_node_coupled.cpp
  src/map_input.cpp
)
target_link_libraries(jps_vo_planner_node jps_vo_core ${catkin_LIBRARIES})
add_dependencies(jps_vo_planner_node ${catkin_EXPORTED_TARGETS})

# offline unit tests (no ROS master needed); run: devel/lib/jps_vo_planner/test_<name>
foreach(t test_occupancy_grid test_jps test_vo test_coupling gen_golden test_fixes)
  add_executable(${t} test/${t}.cpp)
  target_link_libraries(${t} jps_vo_core)
  target_compile_definitions(${t} PRIVATE JPS_VO_PKG_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ---- install (catkin_make install / catkin build --install) ----
install(TARGETS jps_vo_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/dyn_obstacles.py
  scripts/check_scenarios.py
  scripts/send_goal.py
  scripts/verify_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
