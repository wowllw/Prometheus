#!/bin/bash
# 脚本描述: same 5 tabs / order / delays as new_test_ego_planner_p450_d435i.sh, but tab 5 starts the
# independent jps_vo_planner node (+ unmodified traj_server_for_prometheus) instead of ego_planner.
#
# Environment (all optional):
#   LOG_DIR=<dir>         log directory   (default ~/.ros/new_test_logs/jps_vo_<date_time>)
#   PLANNER_MODE=coupled  planner mode    (stub = Subtask-2 straight line, coupled = 3D-JPS + VO)
#   GROUP=g6              ablation group g1..g6 (config/groups/<GROUP>.yaml)
#   REPLAN_PERIOD=1.0     periodic replan [s]
#   PRE_ROTATE=true       yaw pre-rotation towards the goal before planning (default true since 2026-10-26)
#   RECORD=1              record a rosbag into $LOG_DIR

UAV_ID=1
ROOT=/home/amov/Prometheus
EXP=$ROOT/Scripts/experiment
LOG_DIR=${LOG_DIR:-$HOME/.ros/new_test_logs/jps_vo_$(date +%Y%m%d_%H%M%S)}
PLANNER_MODE=${PLANNER_MODE:-coupled}
GROUP=${GROUP:-g6}
REPLAN_PERIOD=${REPLAN_PERIOD:-1.0}
PRE_ROTATE=${PRE_ROTATE:-true}
FIXES_FILE=${FIXES_FILE:-}            # engineering fixes preset (config/fixes/<name>.yaml); "" = thesis-frozen-v1
FIXES_ARG=""; [ -n "$FIXES_FILE" ] && FIXES_ARG=" fixes_file:=$FIXES_FILE"
mkdir -p "$LOG_DIR"
echo "logs -> $LOG_DIR"

gnome-terminal --window -e 'bash -c "roscore 2>&1 | tee '$LOG_DIR'/roscore.log; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_p450_d435i.launch uav1_id:='$UAV_ID' uav1_init_x:=-0.0 uav1_init_y:=-0.0 2>&1 | tee '$LOG_DIR'/gazebo.log; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch prometheus_uav_control uav_control_main_outdoor.launch uav_id:='$UAV_ID' 2>&1 | tee '$LOG_DIR'/uav_control.log; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo depth_to_octomap.launch uav_id:='$UAV_ID' 2>&1 | tee '$LOG_DIR'/octomap.log; exec bash"' \
--tab -e 'bash -c "sleep 6; stdbuf -oL -eL roslaunch jps_vo_planner sitl_jps_vo_planner.launch uav_id:='$UAV_ID' planner_mode:='$PLANNER_MODE' group:='$GROUP' log_dir:='$LOG_DIR' replan_period:='$REPLAN_PERIOD' pre_rotate_yaw:='$PRE_ROTATE"$FIXES_ARG"' 2>&1 | python3 -u '$EXP'/ts_lines.py | tee '$LOG_DIR'/planner.log; exec bash"' \

if [ "${RECORD:-0}" = "1" ]; then
  gnome-terminal --tab -e 'bash -c "sleep 8; cd '$LOG_DIR' && rosbag record -O run.bag '"$($EXP/bag_topics.sh)"' /uav1/jps_vo/path 2>&1 | tee '$LOG_DIR'/rosbag.log; exec bash"'
fi
