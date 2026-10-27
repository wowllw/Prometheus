#!/bin/bash
# manual_mode_session.sh <A|B|C|D> [goal_x goal_y goal_z] [observe_seconds]
# One Step-5 simulation set, using the same scripts as the manual workflow:
#   clean build with the mode's macros -> new_test_ego_planner_p450_d435i.sh (GUI terminals, logs, bag)
#   -> wait for nodes -> NO_RC/arm_and_command.sh -> goal on the RViz 2D-goal topic -> observe -> pkill all.
ROOT=/home/amov/Prometheus
EXP=$ROOT/Scripts/experiment
MODE=$1; GX=${2:-10.0}; GY=${3:-0.0}; GZ=${4:-1.5}; OBS=${5:-120}
case $MODE in
  A) MACROS="-DUSE_DASC=OFF -DUSE_DASC_FALLBACK=OFF -DUSE_FAST_EDF_CORRIDOR=OFF" ;;
  B) MACROS="-DUSE_DASC=ON -DUSE_DASC_FALLBACK=ON -DUSE_FAST_EDF_CORRIDOR=OFF" ;;
  C) MACROS="-DUSE_DASC=OFF -DUSE_DASC_FALLBACK=OFF -DUSE_FAST_EDF_CORRIDOR=ON" ;;
  D) MACROS="-DUSE_DASC=ON -DUSE_DASC_FALLBACK=ON -DUSE_FAST_EDF_CORRIDOR=ON" ;;
  *) echo "mode must be A|B|C|D"; exit 1 ;;
esac
OUT=$ROOT/exp_results/manual/mode_$MODE
rm -rf "$OUT"; mkdir -p "$OUT/screens"
say() { echo "[mode $MODE $(date +%H:%M:%S)] $*" | tee -a "$OUT/session.log"; }
shot() { gnome-screenshot -f "$OUT/screens/$1.png" 2>/dev/null; }
echo1() { timeout 3 rostopic echo -n 1 "$@" 2>/dev/null; }   # never blocks on a silent topic

say "macros: $MACROS"; bash $EXP/kill_sim.sh | tee -a "$OUT/session.log"; sleep 2
source /opt/ros/noetic/setup.bash
cd $ROOT && rm -rf build/ego_planner_swarm
if ! catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm $MACROS > "$OUT/build.log" 2>&1; then
  say "BUILD FAILED"; tail -40 "$OUT/build.log"; exit 2
fi
say "build ok: $(grep -E '^USE_(DASC|DASC_FALLBACK|FAST_EDF_CORRIDOR):' build/ego_planner_swarm/CMakeCache.txt | tr '\n' ' ')"
source $EXP/env_setup.sh   # full ~/.bashrc environment (PX4 paths) for the terminals opened below

say "terminal 1: new_test_ego_planner_p450_d435i.sh"
LOG_DIR=$OUT RECORD=1 bash $ROOT/Scripts/simulation/ego_planner/new_test_ego_planner_p450_d435i.sh
T_START=$SECONDS; ok=0
while [ $((SECONDS - T_START)) -lt 150 ]; do
  echo1 /uav1/prometheus/state | grep -q "connected: True" && timeout 3 rosnode ping -c 1 /uav1_ego_planner_node >/dev/null 2>&1 && { ok=1; break; }
  sleep 1
done
[ $ok = 1 ] && say "nodes ready after $((SECONDS - T_START)) s" || say "WARNING: nodes not ready after 150 s"
sleep 10; shot 01_ready
say "terminal 2: arm_and_command.sh"
bash $ROOT/Scripts/simulation/NO_RC/arm_and_command.sh > "$OUT/arm_and_command.log" 2>&1
z=0; T1=$SECONDS
while [ $((SECONDS - T1)) -lt 60 ]; do
  z=$(echo1 /uav1/mavros/local_position/odom/pose/pose/position/z | head -1)
  python3 -c "import sys; sys.exit(0 if float('${z:-0}') > 1.2 else 1)" 2>/dev/null && break
  sleep 1
done
say "altitude after takeoff wait ($((SECONDS - T1)) s): z=${z}"
echo1 /uav1/prometheus/state > "$OUT/state_after_takeoff.txt"
sleep 3; shot 02_takeoff
say "goal ($GX, $GY, $GZ) -> /uav1/prometheus/motion_planning/goal"
timeout 10 rostopic pub -1 /uav1/prometheus/motion_planning/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: $GX, y: $GY, z: $GZ}, orientation: {w: 1.0}}}" > /dev/null 2>&1
T0=$SECONDS; reached=0; next_shot=0
while [ $((SECONDS - T0)) -lt $OBS ]; do
  P=$(echo1 /uav1/mavros/local_position/odom/pose/pose/position | tr '\n' ' ')
  echo "$((SECONDS - T0)) $P" >> "$OUT/trace.txt"
  python3 - "$P" "$GX" "$GY" "$GZ" <<'PY' && { reached=1; break; }
import re, sys, math
v = dict(re.findall(r"(\w): (-?[\d.e+-]+)", sys.argv[1]))
try:
    d = math.dist([float(v['x']), float(v['y']), float(v['z'])], [float(a) for a in sys.argv[2:5]])
except Exception:
    sys.exit(1)
sys.exit(0 if d < 0.5 else 1)
PY
  if [ $((SECONDS - T0)) -ge $next_shot ]; then shot "03_flight_$((SECONDS - T0))s"; next_shot=$((next_shot + 20)); fi
  sleep 1
done
say "goal reached: $reached after $((SECONDS - T0)) s; last position: $(tail -1 $OUT/trace.txt)"
shot 04_end
sleep 2
say "terminating: pkill -f gazebo / roscore / roslaunch (kill_sim.sh)"
bash $EXP/kill_sim.sh | tee -a "$OUT/session.log"
ls -la "$OUT" | tee -a "$OUT/session.log"
