#!/bin/bash
# Pre-run check before launching a simulation run (closed_loop_sim_guide.md, step 1 of every run).
# Exits 0 only if no simulation process and no dyn_obstacles.py script is left over.
# Leftover dyn_obstacles.py scripts are terminated here, but only while no ROS master is running, so their
# on_shutdown delete_model call cannot reach a Gazebo instance. Run kill_sim.sh first if a master is still up.
PAT="^(/usr/bin/)?python3 [^ ]*dyn_obstacles\.py"

if pgrep -x rosmaster >/dev/null || pgrep -x gzserver >/dev/null; then
  echo "PRECHECK FAIL: rosmaster/gzserver still running - run Scripts/experiment/kill_sim.sh first"
  pgrep -a -x "rosmaster|gzserver"
  exit 1
fi

if pgrep -f "$PAT" >/dev/null; then
  echo "precheck: leftover dyn_obstacles.py found (no master running), terminating:"
  pgrep -a -f "$PAT"
  pkill -INT -f "$PAT"
  for _ in $(seq 10); do pgrep -f "$PAT" >/dev/null || break; sleep 1; done
  pkill -9 -f "$PAT"
fi

left=$(pgrep -a -f "gzserver|gzclient|rosmaster|roslaunch|px4 |rosbag record|$PAT" | grep -v precheck_run)
if [ -n "$left" ]; then
  echo "PRECHECK FAIL: still running:"; echo "$left"; exit 1
fi
echo "PRECHECK OK: no simulation processes, no dyn_obstacles.py scripts"
