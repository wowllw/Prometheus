#!/bin/bash
# Terminate every ROS / Gazebo / PX4 process of a simulation session (bag closed first).
if pgrep -f "rosbag record" >/dev/null; then
  pkill -INT -f "rosbag record"; pkill -INT -x record
  for _ in $(seq 20); do pgrep -x record >/dev/null || pgrep -f "rosbag record" >/dev/null || break; sleep 1; done
fi
pkill -f "odom_drift_relay.py|ts_lines.py|exp_monitor.py"   # never the calling driver script
pkill -f gazebo; pkill -f roscore; pkill -f roslaunch
pkill -x px4; pkill -x rosmaster; pkill -f "rostopic (echo|pub|hz)"
sleep 3
pkill -9 -x gzserver; pkill -9 -x gzclient; pkill -9 -x px4; pkill -9 -f roslaunch; pkill -9 -x rosmaster
pkill -9 -f "rosbag record"; pkill -9 -x record
sleep 1
# dynamic-obstacle scripts (jps_vo_planner/scripts/dyn_obstacles.py): stopped only now, after rosmaster and
# Gazebo are gone. Their on_shutdown hook calls /gazebo/delete_model for dyn_obs_*; while any master is up that
# call reaches the CURRENT Gazebo and deletes the same-named obstacle of a newer run.
if pgrep -f "^(/usr/bin/)?python3 [^ ]*dyn_obstacles\.py" >/dev/null; then
  pkill -INT -f "^(/usr/bin/)?python3 [^ ]*dyn_obstacles\.py"
  for _ in $(seq 10); do pgrep -f "^(/usr/bin/)?python3 [^ ]*dyn_obstacles\.py" >/dev/null || break; sleep 1; done
  pkill -9 -f "^(/usr/bin/)?python3 [^ ]*dyn_obstacles\.py"
fi
left=$(pgrep -a -f "gzserver|gzclient|rosmaster|roslaunch|px4 |rosbag record|^(/usr/bin/)?python3 [^ ]*dyn_obstacles\.py" | grep -v kill_sim)
[ -n "$left" ] && echo "still running: $left" || echo "all simulation processes terminated"
