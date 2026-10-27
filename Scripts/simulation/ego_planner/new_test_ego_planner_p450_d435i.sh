#!/bin/bash
# 脚本描述: new_test version of ego_planner_p450_d435i.sh (same 5 tabs, same order and delays).
# Differences: every tab also writes its output to $LOG_DIR, the planner runs through the experiment
# launch file (defaults identical to sitl_ego_planner_basic_octomap.launch), its stdout is time-stamped
# for planning-time analysis, and an optional drift relay / rosbag can be enabled.
#
# Environment (all optional):
#   LOG_DIR=<dir>        log directory            (default ~/.ros/new_test_logs/<date_time>)
#   DRIFT_LEVEL=0|1|2    odometry drift relay     (default 0 = planner uses /mavros/local_position/odom directly)
#   DASC_ADAPT=true|false runtime DASC toggles    (default true; no effect unless built with -DUSE_DASC=ON)
#   RECORD=1             record a rosbag into $LOG_DIR

UAV_ID=1
ROOT=/home/amov/Prometheus
EXP=$ROOT/Scripts/experiment
LOG_DIR=${LOG_DIR:-$HOME/.ros/new_test_logs/$(date +%Y%m%d_%H%M%S)}
DRIFT_LEVEL=${DRIFT_LEVEL:-0}
DASC_ADAPT=${DASC_ADAPT:-true}
mkdir -p "$LOG_DIR"
echo "logs -> $LOG_DIR"

if [ "$DRIFT_LEVEL" != "0" ]; then ODOM=exp/odom_drift; else ODOM=/mavros/local_position/odom; fi

gnome-terminal --window -e 'bash -c "roscore 2>&1 | tee '$LOG_DIR'/roscore.log; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_p450_d435i.launch uav1_id:='$UAV_ID' uav1_init_x:=-0.0 uav1_init_y:=-0.0 2>&1 | tee '$LOG_DIR'/gazebo.log; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch prometheus_uav_control uav_control_main_outdoor.launch uav_id:='$UAV_ID' 2>&1 | tee '$LOG_DIR'/uav_control.log; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo depth_to_octomap.launch uav_id:='$UAV_ID' 2>&1 | tee '$LOG_DIR'/octomap.log; exec bash"' \
--tab -e 'bash -c "sleep 6; stdbuf -oL -eL roslaunch ego_planner new_test_exp_ego_planner.launch uav_id:='$UAV_ID' odometry_topic:='$ODOM' dasc_drift_adapt:='$DASC_ADAPT' dasc_vel_adapt:='$DASC_ADAPT' 2>&1 | python3 -u '$EXP'/ts_lines.py | tee '$LOG_DIR'/planner.log; exec bash"' \

if [ "$DRIFT_LEVEL" != "0" ]; then
  gnome-terminal --tab -e 'bash -c "sleep 5; python3 '$EXP'/odom_drift_relay.py _drift_level:='$DRIFT_LEVEL' 2>&1 | tee '$LOG_DIR'/drift_relay.log; exec bash"'
fi
if [ "${RECORD:-0}" = "1" ]; then
  gnome-terminal --tab -e 'bash -c "sleep 8; cd '$LOG_DIR' && rosbag record -O run.bag '"$($EXP/bag_topics.sh)"' 2>&1 | tee '$LOG_DIR'/rosbag.log; exec bash"'
fi
