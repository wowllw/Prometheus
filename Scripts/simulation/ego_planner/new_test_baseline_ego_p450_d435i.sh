#!/bin/bash
# 脚本描述: clean ego-planner baseline. Same 5 tabs / order / delays as new_test_jps_vo_p450_d435i.sh;
# tab 5 overlays the git worktree ~/Prometheus_baseline (ego_planner_swarm at 66baf852^, i.e. upstream ego
# without DASC/FastEDF) and starts its sitl_ego_planner_basic_octomap.launch (+ traj_server_for_prometheus).
# Build once:
#   cd ~/Prometheus_baseline && source ~/Prometheus/devel/setup.bash && \
#   catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm -DCATKIN_DEVEL_PREFIX=$HOME/Prometheus_baseline/devel
#
# Environment (all optional):
#   LOG_DIR=<dir>   log directory (default ~/.ros/new_test_logs/baseline_<date_time>)
#   RECORD=1        record a rosbag into $LOG_DIR

UAV_ID=1
ROOT=/home/amov/Prometheus
BASELINE=$HOME/Prometheus_baseline
EXP=$ROOT/Scripts/experiment
LOG_DIR=${LOG_DIR:-$HOME/.ros/new_test_logs/baseline_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOG_DIR"
echo "logs -> $LOG_DIR"

if [ ! -x "$BASELINE/devel/lib/ego_planner/ego_planner_node" ]; then
  echo "baseline not built: $BASELINE/devel/lib/ego_planner/ego_planner_node missing"; exit 1
fi

gnome-terminal --window -e 'bash -c "roscore 2>&1 | tee '$LOG_DIR'/roscore.log; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_p450_d435i.launch uav1_id:='$UAV_ID' uav1_init_x:=-0.0 uav1_init_y:=-0.0 2>&1 | tee '$LOG_DIR'/gazebo.log; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch prometheus_uav_control uav_control_main_outdoor.launch uav_id:='$UAV_ID' 2>&1 | tee '$LOG_DIR'/uav_control.log; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo depth_to_octomap.launch uav_id:='$UAV_ID' 2>&1 | tee '$LOG_DIR'/octomap.log; exec bash"' \
--tab -e 'bash -c "sleep 6; source '$BASELINE'/devel/setup.bash; echo ego_planner from: \$(rospack find ego_planner); stdbuf -oL -eL roslaunch ego_planner sitl_ego_planner_basic_octomap.launch uav_id:='$UAV_ID' 2>&1 | python3 -u '$EXP'/ts_lines.py | tee '$LOG_DIR'/planner.log; exec bash"' \

if [ "${RECORD:-0}" = "1" ]; then
  gnome-terminal --tab -e 'bash -c "sleep 8; cd '$LOG_DIR' && rosbag record -O run.bag '"$($EXP/bag_topics.sh)"' 2>&1 | tee '$LOG_DIR'/rosbag.log; exec bash"'
fi
