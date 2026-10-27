#!/bin/bash
# topics recorded in every experiment bag (used by run_experiment.sh and new_test_ego_planner_p450_d435i.sh)
echo /uav1/planning/bspline /rosout_agg /uav1/mavros/local_position/odom /uav1/exp/odom_drift \
     /uav1/prometheus/motion_planning/goal /uav1/prometheus/state /uav1/octomap_point_cloud_centers /exp/monitor_status
