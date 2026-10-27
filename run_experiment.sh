#!/bin/bash
# run_experiment.sh - batch experiments for DASC / FastEDFCorridor (Chapter 5)
#
# Usage: ./run_experiment.sh <scenario_id> <drift_level> <group> <repetitions> [options]
#   scenario_id   0 | 1 | 2      goal in planning_easy.world (see Scripts/experiment/exp_monitor.py)
#   drift_level   0 | 1 | 2      odometry drift injected by Scripts/experiment/odom_drift_relay.py
#   group         B1   : all macros OFF (original EGO-Planner)              (mode A)
#                 DASC : -DUSE_DASC=ON -DUSE_DASC_FALLBACK=ON              (mode B)
#                 EDF  : -DUSE_FAST_EDF_CORRIDOR=ON                         (mode C)
#                 FULL : all three ON                                       (mode D)
#                 A1   : ablation, -DUSE_FAST_EDF_CORRIDOR=ON only
#                 A2   : ablation, DASC ON, FastEDF ON, dasc_vel_adapt=false (velocity adaptation disabled)
#   repetitions   number of runs (each run restarts ROS/Gazebo from scratch)
# Options:
#   --no-build            reuse the current build (skip the clean build)
#   --mission-timeout S   seconds allowed after the goal is sent (default 120)
#   --out DIR             results root (default /home/amov/Prometheus/exp_results)
#   --rviz                also start RViz (off by default in batch mode)
#
# Output: <out>/S<scenario>_D<drift>_<group>/rep_<k>/{run.bag, planner.log, *.log, result.json, meta.json}
# Analysis: python3 parse_exp_log.py <out>

set -u
ROOT=/home/amov/Prometheus
EXP=$ROOT/Scripts/experiment
UAV_ID=1
ARM_SCRIPT=$ROOT/Scripts/simulation/NO_RC/arm_and_command.sh

usage() { sed -n '2,21p' "$0"; exit 1; }
[ $# -lt 4 ] && usage
SCENARIO=$1; DRIFT=$2; GROUP=$3; REPS=$4; shift 4
DO_BUILD=1; MISSION_TIMEOUT=120; OUT=$ROOT/exp_results; RVIZ=false
while [ $# -gt 0 ]; do
  case $1 in
    --no-build) DO_BUILD=0 ;;
    --mission-timeout) MISSION_TIMEOUT=$2; shift ;;
    --out) OUT=$2; shift ;;
    --rviz) RVIZ=true ;;
    *) echo "unknown option $1"; usage ;;
  esac
  shift
done

case $SCENARIO in 0|1|2) ;; *) echo "scenario_id must be 0,1,2"; exit 1 ;; esac
case $DRIFT in 0|1|2) ;; *) echo "drift_level must be 0,1,2"; exit 1 ;; esac
EXTRA_CMAKE=""
LAUNCH_DASC_VEL_ADAPT=""
case $GROUP in
  B1)   M_DASC=OFF; M_FB=OFF; M_EDF=OFF; ADAPT=false; LAUNCH_DASC_VEL_ADAPT="$ADAPT" ;;
  DASC) M_DASC=ON;  M_FB=ON;  M_EDF=OFF; ADAPT=true; LAUNCH_DASC_VEL_ADAPT="$ADAPT" ;;
  EDF)  M_DASC=OFF; M_FB=OFF; M_EDF=ON;  ADAPT=false; LAUNCH_DASC_VEL_ADAPT="$ADAPT" ;;
  FULL) M_DASC=ON;  M_FB=ON;  M_EDF=ON;  ADAPT=true; LAUNCH_DASC_VEL_ADAPT="$ADAPT" ;;
  A1)   M_DASC=OFF; M_FB=OFF; M_EDF=ON;  ADAPT=false; LAUNCH_DASC_VEL_ADAPT="$ADAPT" ;;
  A2)   M_DASC=ON;  M_FB=ON;  M_EDF=ON;  ADAPT=true; LAUNCH_DASC_VEL_ADAPT="false" ;;
  *) echo "group must be B1, DASC, EDF, FULL, A1 or A2"; exit 1 ;;
esac
[[ "$REPS" =~ ^[0-9]+$ ]] && [ "$REPS" -ge 1 ] || { echo "repetitions must be >= 1"; exit 1; }

CASE_DIR=$OUT/S${SCENARIO}_D${DRIFT}_${GROUP}
mkdir -p "$CASE_DIR"
log() { echo "[run_experiment $(date +%H:%M:%S)] $*" | tee -a "$CASE_DIR/batch.log"; }

# ---------------------------------------------------------------- environment (fix 2)
# Same environment as ~/.bashrc. Only ever called inside a subshell: catkin's setup.bash files reset
# ROS_PACKAGE_PATH, so the PX4-SITL paths are appended last and the parent shell is never modified.
setup_env() {
  set +u   # ROS setup files reference unset variables
  source /opt/ros/noetic/setup.bash
  source ~/prometheus_mavros/devel/setup.bash
  [ -f ~/spirecv-ros/devel/setup.bash ] && source ~/spirecv-ros/devel/setup.bash
  source $ROOT/devel/setup.bash
  export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH:-}:$ROOT/devel/lib
  local d
  for d in uav_models ugv_models sensor_models scene_models texture; do
    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH:-}:$ROOT/Simulator/gazebo_simulator/gazebo_models/$d
  done
  source ~/prometheus_px4/Tools/setup_gazebo.bash ~/prometheus_px4 ~/prometheus_px4/build/amovlab_sitl_default > /dev/null
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4:~/prometheus_px4/Tools/sitl_gazebo
  set -u
}

# ---------------------------------------------------------------- process cleanup (fix 1)
# Terminates gazebo, roscore, px4, mavros, ego_planner and everything else a run starts, escalating
# SIGINT -> SIGTERM -> SIGKILL until nothing is left, then sleeps so sockets and shared memory are freed.
SIM_PATTERNS="gzserver|gzclient|gazebo|bin/px4|px4 |mavros_node|lib/ego_planner/|uav_controller|uav_estimator|octomap_server|roslaunch|rosmaster|roscore|rosout|rosbag record|odom_drift_relay.py|ts_lines.py|exp_monitor.py"
sim_alive() { { pgrep -f "$SIM_PATTERNS"; pgrep -x px4; pgrep -x record; } | grep -v -x -e "$$" -e "${BASHPID:-$$}" | sort -u; }
cleanup() {
  # close the bag first so it is not left as run.bag.active
  if pgrep -f "rosbag record" >/dev/null; then
    pkill -INT -f "rosbag record"; pkill -INT -x record
    for _ in $(seq 20); do pgrep -f "rosbag record" >/dev/null || pgrep -x record >/dev/null || break; sleep 1; done
  fi
  local sig
  for sig in INT TERM KILL; do
    pkill -$sig -f "$SIM_PATTERNS" 2>/dev/null
    pkill -$sig -x px4 2>/dev/null; pkill -$sig -x gzserver 2>/dev/null; pkill -$sig -x gzclient 2>/dev/null
    for _ in $(seq 10); do [ -z "$(sim_alive)" ] && break; sleep 1; done
    [ -z "$(sim_alive)" ] && break
  done
  local left; left=$(sim_alive | xargs -r ps -o pid=,comm= -p 2>/dev/null)
  [ -n "$left" ] && log "WARNING: processes still alive after cleanup: $left"
  sleep 5   # let ports (11311, 14540/14557 ...) and Gazebo shared memory be released
}
trap 'log "interrupted"; cleanup; exit 130' INT TERM

# ---------------------------------------------------------------- polling helpers (fix 3)
wait_for() { # wait_for <timeout_s> <command...>   (polls once per second)
  local t_end=$((SECONDS + $1)); shift
  while [ $SECONDS -lt $t_end ]; do "$@" >/dev/null 2>&1 && return 0; sleep 1; done
  return 1
}
topic_has() { # topic_has <topic> <text>   one message within 3 s that contains <text>
  timeout 3 rostopic echo -n 1 "$1" 2>/dev/null | grep -q "$2"
}

# ---------------------------------------------------------------- build with the group's macros
MACROS="-DUSE_DASC=$M_DASC -DUSE_DASC_FALLBACK=$M_FB -DUSE_FAST_EDF_CORRIDOR=$M_EDF${EXTRA_CMAKE:+ $EXTRA_CMAKE}"
if [ $DO_BUILD -eq 1 ]; then
  log "clean build: $MACROS"
  cleanup
  if ! ( setup_env
         cd $ROOT && rm -rf build/ego_planner_swarm &&
         catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm $MACROS ) > "$CASE_DIR/build.log" 2>&1; then
    log "BUILD FAILED, see $CASE_DIR/build.log"; tail -30 "$CASE_DIR/build.log"; exit 2
  fi
  log "build ok"
fi
# record the macros the binaries were actually built with (from the CMake cache)
BUILT=$(grep -E "^USE_(DASC|DASC_FALLBACK|FAST_EDF_CORRIDOR):" $ROOT/build/ego_planner_swarm/CMakeCache.txt 2>/dev/null | tr '\n' ' ')
log "binaries built with: ${BUILT:-unknown}"

if [ "$DRIFT" != "0" ]; then ODOM=exp/odom_drift; else ODOM=/mavros/local_position/odom; fi
TOPICS=$($EXP/bag_topics.sh)

# ---------------------------------------------------------------- one run, in its own subshell (fix 2)
run_once() { # run_once <k> <run_dir>   -- always called as ( run_once ... )
  local k=$1 RUN=$2
  setup_env
  trap 'exit 130' INT TERM

  roscore > "$RUN/roscore.log" 2>&1 &
  wait_for 30 rosparam get /rosdistro || { log "roscore did not start"; return 3; }
  roslaunch prometheus_gazebo sitl_p450_d435i.launch uav1_id:=$UAV_ID uav1_init_x:=-0.0 uav1_init_y:=-0.0 \
    rviz_enable:=$RVIZ > "$RUN/gazebo.log" 2>&1 &
  roslaunch prometheus_uav_control uav_control_main_outdoor.launch uav_id:=$UAV_ID > "$RUN/uav_control.log" 2>&1 &
  roslaunch prometheus_gazebo depth_to_octomap.launch uav_id:=$UAV_ID > "$RUN/octomap.log" 2>&1 &
  python3 $EXP/odom_drift_relay.py _drift_level:=$DRIFT _seed:=$k > "$RUN/drift_relay.log" 2>&1 &
  (stdbuf -oL -eL roslaunch ego_planner new_test_exp_ego_planner.launch uav_id:=$UAV_ID odometry_topic:=$ODOM \
     dasc_drift_adapt:=$ADAPT dasc_vel_adapt:=${LAUNCH_DASC_VEL_ADAPT} 2>&1 | python3 -u $EXP/ts_lines.py > "$RUN/planner.log") &

  # PX4 <-> MAVROS link: poll /mavros/state instead of fixed delays
  if ! wait_for 180 topic_has /uav$UAV_ID/mavros/state "connected: True"; then
    log "PX4 not connected on /uav$UAV_ID/mavros/state after 180 s, skipping rep"; return 4
  fi
  log "PX4 connected"
  wait_for 60 topic_has /uav$UAV_ID/prometheus/state "connected: True" || { log "uav_control not ready, skipping rep"; return 4; }
  wait_for 60 timeout 3 rosnode ping -c 1 /uav${UAV_ID}_ego_planner_node || { log "planner node not responding, skipping rep"; return 4; }
  # PX4 EKF: wait until local position estimates are published (replaces the fixed settle delay)
  wait_for 60 topic_has /uav$UAV_ID/mavros/local_position/odom "position" || { log "no local position estimate, skipping rep"; return 4; }

  (cd "$RUN" && rosbag record -O run.bag $TOPICS > "$RUN/rosbag.log" 2>&1) &
  wait_for 15 test -e "$RUN/run.bag.active" || log "warning: rosbag did not start"

  # arm + COMMAND_CONTROL with arm_and_command.sh; re-issue if the arm request was lost
  local tries=0 armed=0
  while [ $tries -lt 3 ] && [ $armed -eq 0 ]; do
    tries=$((tries + 1))
    if [ -n "${DISPLAY:-}" ] && command -v gnome-terminal >/dev/null; then
      bash $ARM_SCRIPT >> "$RUN/arm_and_command.log" 2>&1
    else # headless: publish the same two UAVSetup messages arm_and_command.sh publishes
      timeout 10 rostopic pub -1 /uav$UAV_ID/prometheus/setup prometheus_msgs/UAVSetup \
        "{cmd: 0, arming: true, px4_mode: '', control_state: ''}" >> "$RUN/arm_and_command.log" 2>&1
      sleep 3
      timeout 10 rostopic pub -1 /uav$UAV_ID/prometheus/setup prometheus_msgs/UAVSetup \
        "{cmd: 3, arming: true, px4_mode: '', control_state: 'COMMAND_CONTROL'}" >> "$RUN/arm_and_command.log" 2>&1
    fi
    wait_for 15 topic_has /uav$UAV_ID/mavros/state "armed: True" && armed=1
    log "arm attempt $tries: armed=$armed"
  done
  [ $armed -eq 1 ] || { log "vehicle did not arm, skipping rep"; return 5; }

  python3 $EXP/exp_monitor.py --scenario $SCENARIO --out "$RUN/result.json" \
    --mission_timeout $MISSION_TIMEOUT --takeoff_z 1.2 > "$RUN/monitor.log" 2>&1
  log "result: $(python3 -c "import json;r=json.load(open('$RUN/result.json'));print(r['outcome'], 'flight_time=%.1f' % r.get('flight_time', -1))" 2>/dev/null || echo 'no result.json')"
  return 0
}

# ---------------------------------------------------------------- repetitions
for ((k = 1; k <= REPS; k++)); do
  RUN=$CASE_DIR/rep_$k
  rm -rf "$RUN"; mkdir -p "$RUN"
  log "=== S$SCENARIO D$DRIFT $GROUP rep $k/$REPS -> $RUN"
  cleanup
  cat > "$RUN/meta.json" <<META
{"scenario": $SCENARIO, "drift_level": $DRIFT, "group": "$GROUP", "rep": $k,
 "macros": "$MACROS", "built_with": "$BUILT", "dasc_runtime_adapt": $ADAPT,
 "commit": "$(git -C $ROOT rev-parse --short HEAD)", "dirty": $( [ -n "$(git -C $ROOT status --porcelain)" ] && echo true || echo false ),
 "start": "$(date -Iseconds)", "mission_timeout": $MISSION_TIMEOUT}
META
  ( run_once $k "$RUN" )
  rc=$?
  [ $rc -ne 0 ] && log "rep $k ended with code $rc"
  cleanup
  ls -la "$RUN" >> "$CASE_DIR/batch.log"
done
log "done: $CASE_DIR"
