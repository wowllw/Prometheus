#!/bin/bash
# [File name] run_ablation_batch.sh
# [Purpose] Batch driver for the Subtask-4 ablation (jps_vo_planner groups g1..g6). Implements rules B-1 .. B-12 and
#   the 15-step per-run procedure of Modules/jps_vo_planning/jps_vo_planner/doc/experiment_batch_run_notes.md.
#   Every attempt: kill_sim.sh + precheck_run.sh -> launch -> ready / config / hover / gate checks -> obstacles ->
#   goal -> wait for the end -> kill_sim.sh -> verify_run.py -> post_process_ablation.py --check-run -> index line.
#   Invalid attempts (I-x) are rerun with the same seed, at most 3 attempts per cell; valid runs are never rerun.
#   Resumable: cells that are already valid or excluded in the index are skipped.
#
# Usage (environment variables, all optional):
#   SEEDS="1 2 ... 20"   VARIANTS="S1_east S2_north S3_climb_ne head_on crossing_90 M1_three_way"
#   GROUPS_SEL="g1 g2 g3 g4 g5 g6"   OUT=~/.ros/ablation   TAG=formal|smoke   RECORD=0   MAX_ATTEMPTS=3
#   DRY_RUN=1 prints the planned attempts and exits.
#   Example (smoke test, Q-D): SEEDS=1 VARIANTS=S1_east GROUPS_SEL="g1 g2 g3 g4 g5" OUT=~/.ros/ablation_smoke \
#                              TAG=smoke bash Scripts/experiment/run_ablation_batch.sh
# Must run inside a logged-in desktop session (B-9): the start script opens gnome-terminal tabs.
# Groups g1..g6 only; the ego-planner baseline (Q-E) needs its own launcher and is not handled here.

set +u   # ROS setup.bash references unset variables
ROOT=$HOME/Prometheus
PKG=$ROOT/Modules/jps_vo_planning/jps_vo_planner
EXP=$ROOT/Scripts/experiment
SC=$PKG/config/scenarios/scenario_ablation_all.yaml
START=./Scripts/simulation/ego_planner/new_test_jps_vo_p450_d435i.sh
OUT=${OUT:-$HOME/.ros/ablation}
TAG=${TAG:-formal}
RECORD=${RECORD:-0}                       # Q-B: no rosbag in the batch
MAX_ATTEMPTS=${MAX_ATTEMPTS:-3}
FIXES_FILE=${FIXES_FILE:-}                # engineering fixes preset (config/fixes/<name>.yaml); "" = thesis-frozen-v1
SEEDS=${SEEDS:-$(seq -s ' ' 1 20)}
VARIANTS=${VARIANTS:-"S1_east S2_north S3_climb_ne head_on crossing_90 M1_three_way"}
GROUPS_SEL=${GROUPS_SEL:-"g1 g2 g3 g4 g5 g6"}
INDEX=$OUT/batch_index.csv
NODE=/uav1_jps_vo_planner_node
GATE_TOPIC=/uav1/prometheus/command/ego_command_stop_pub
declare -A TMO=([S1_east]=70 [S2_north]=70 [S3_climb_ne]=70 [head_on]=70 [crossing_90]=70 [M1_three_way]=80)
declare -A NOBS=([S1_east]=0 [S2_north]=0 [S3_climb_ne]=0 [head_on]=1 [crossing_90]=1 [M1_three_way]=3)
read -r -a ALL_GROUPS <<< "$GROUPS_SEL"
mkdir -p "$OUT"
LOG=$OUT/batch_$(date +%Y%m%d_%H%M%S).log

say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
die() { say "ABORT: $*"; exit 2; }

# ---------------------------------------------------------------- B-9 display
if [ -z "${DISPLAY:-}" ]; then
  if [ -S /tmp/.X11-unix/X0 ]; then export DISPLAY=:0; say "DISPLAY was empty, using :0"; else die "no DISPLAY and no X server socket (B-9)"; fi
fi
command -v gnome-terminal >/dev/null || die "gnome-terminal not found (B-9)"

for v in $VARIANTS; do [ -n "${TMO[$v]:-}" ] || die "unknown variant $v"; done
for g in "${ALL_GROUPS[@]}"; do [ -f "$PKG/config/groups/$g.yaml" ] || die "unknown group $g"; done

# ---------------------------------------------------------------- B-12 index
HDR="tag,variant,group,seed,attempt,log_dir,start,end,precheck,hover_z,invalid_code,end_reason,gate_hz,notes"
[ -f "$INDEX" ] || echo "$HDR" > "$INDEX"

cell_state() {   # $1 variant $2 group $3 seed -> "valid" | "excluded" | <attempts so far>
  python3 - "$INDEX" "$1" "$2" "$3" "$TAG" "$MAX_ATTEMPTS" <<'EOF'
import csv, sys
idx, v, g, k, tag, mx = sys.argv[1:7]
rows = [r for r in csv.DictReader(open(idx)) if (r['tag'], r['variant'], r['group'], r['seed']) == (tag, v, g, k)]
if any(r['invalid_code'] == '' for r in rows): print('valid')
elif len(rows) >= int(mx): print('excluded')
else: print(len(rows))
EOF
}

ros_py() {   # run a short rospy probe; $1 = python body
  (source "$ROOT/devel/setup.bash"; timeout 30 python3 -c "$1" 2>/dev/null)
}

gate_count() {   # messages on the gate topic in 5 s -> "<n> <n_true>"
  ros_py "
import rospy
from std_msgs.msg import Bool
rospy.init_node('abl_gate', anonymous=True)
n=[0,0]
def cb(m): n[0]+=1; n[1]+=m.data is True
rospy.Subscriber('$GATE_TOPIC', Bool, cb); rospy.sleep(5.0); print(n[0], n[1])"
}

hover_probe() {   # 5 s of odom -> "<z_mean> <z_min> <z_max> <v_mean> <v_max>"
  ros_py "
import rospy, math
from nav_msgs.msg import Odometry
rospy.init_node('abl_hover', anonymous=True)
z=[]; v=[]
def cb(m):
    z.append(m.pose.pose.position.z); t=m.twist.twist.linear; v.append(math.sqrt(t.x*t.x+t.y*t.y+t.z*t.z))
rospy.Subscriber('/uav1/mavros/local_position/odom', Odometry, cb); rospy.sleep(5.0)
print('%.3f %.3f %.3f %.3f %.3f' % (sum(z)/len(z), min(z), max(z), sum(v)/len(v), max(v)) if z else 'nan nan nan nan nan')"
}

pub_setup() {   # $1 cmd  $2 control_state; prometheus_msgs/UAVSetup on /uav1/prometheus/setup (blocks ~3 s)
  timeout 10 rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup \
    "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, cmd: $1, arming: true, px4_mode: '', control_state: '$2'}" \
    >/dev/null 2>&1
}

meta_set() {   # append "key: value" to attempt.yaml
  echo "$1: $2" >> "$LOG_DIR/attempt.yaml"
}

# ---------------------------------------------------------------- one attempt
# sets globals: CODE (invalid code or empty), PRE, HOVER, GATE, NOTES
run_attempt() {
  local V=$1 G=$2 K=$3 A=$4
  CODE=""; PRE=fail; HOVER=""; GATE=""; NOTES=""
  LOG_DIR=$OUT/$V/$G/seed${K}_a${A}
  [ -e "$LOG_DIR" ] && { CODE=I-8; NOTES="log dir exists already"; return; }     # B-3: never reuse

  # step 1: clean environment (B-1); kill_sim.sh is its own process
  bash "$EXP/kill_sim.sh" >> "$LOG" 2>&1
  if ! bash "$EXP/precheck_run.sh" 2>&1 | tee -a "$LOG" | grep -q "PRECHECK OK"; then CODE=I-0; return; fi
  PRE=ok

  # step 2: attempt record
  mkdir -p "$LOG_DIR"
  cat > "$LOG_DIR/attempt.yaml" <<EOF
tag: $TAG
variant: $V
group: $G
seed: $K
attempt: $A
start_wall: $(date +%s.%N)
scenario_file: $SC
record: $RECORD
driver: run_ablation_batch.sh
driver_commit: '$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null)'
EOF
  [ -n "$FIXES_FILE" ] && meta_set fixes_file "$FIXES_FILE"

  # step 3: launch (B-4, B-5: GROUP is the only group-dependent variable)
  (cd "$ROOT" && LOG_DIR=$LOG_DIR GROUP=$G FIXES_FILE=$FIXES_FILE PLANNER_MODE=coupled PRE_ROTATE=true RECORD=$RECORD REPLAN_PERIOD=1.0 \
     bash -ic "$START" >/dev/null 2>&1)

  # step 4: ready
  local i ok=0
  for i in $(seq 1 45); do grep -qa "coupled mode" "$LOG_DIR/planner.log" 2>/dev/null && { ok=1; break; }; sleep 2; done
  [ $ok = 1 ] || { CODE=I-7; NOTES="planner not ready in 90 s"; return; }
  source "$ROOT/devel/setup.bash"
  ok=0
  for i in $(seq 1 30); do rostopic echo -n1 /uav1/prometheus/state 2>/dev/null | grep -q "odom_valid: True" && { ok=1; break; }; sleep 2; done
  [ $ok = 1 ] || { CODE=I-7; NOTES="odom_valid not True"; return; }
  rostopic echo -n1 /uav1/prometheus/state | grep -q "connected: True" || { CODE=I-7; NOTES="PX4 not connected"; return; }
  timeout 10 rostopic echo -n1 /uav1/mavros/home_position/home >/dev/null 2>&1 || { CODE=I-7; NOTES="no home position"; return; }

  # step 5: configuration (B-7) and single planner / publisher (B-8)
  local zmin prerot grp
  zmin=$(rosparam get $NODE/map/z_min 2>/dev/null); prerot=$(rosparam get $NODE/pre_rotate_yaw 2>/dev/null)
  grp=$(sed 's/\x1b\[[0-9;]*m//g' "$LOG_DIR/planner.log" | grep -ao "coupled mode, group [a-z0-9]*" | head -1 | awk '{print $4}')
  meta_set runtime_map_z_min "$zmin"; meta_set runtime_pre_rotate_yaw "$prerot"; meta_set runtime_group "$grp"
  if [ "$zmin" != "0.8" ] || [ "$prerot" != "true" ] || [ "$grp" != "$G" ]; then
    CODE=I-10; NOTES="z_min=$zmin pre_rotate=$prerot group=$grp"; return
  fi
  local nplan npub
  nplan=$(rosnode list 2>/dev/null | grep -c -E "planner_node")
  npub=$(rostopic info /uav1/planning/bspline 2>/dev/null | sed -n '/Publishers/,/Subscribers/p' | grep -c '\*')
  meta_set runtime_planner_nodes "$nplan"; meta_set runtime_bspline_publishers "$npub"
  [ "$nplan" = 1 ] && [ "$npub" = 1 ] || { CODE=I-11; NOTES="planner nodes $nplan, bspline publishers $npub"; return; }

  # step 6: arm + COMMAND. Same two UAVSetup messages as Scripts/simulation/NO_RC/arm_and_command.sh, published
  # here directly and each one confirmed, because the script's second message (sent from a new gnome-terminal
  # window) was lost in smoke attempt g1/seed1_a1 (armed, never switched to COMMAND_CONTROL).
  sleep 6
  local try_arm try_cmd
  for try_arm in 1 2 3; do
    pub_setup 0 ""
    rostopic echo -n1 /uav1/prometheus/state 2>/dev/null | grep -q "armed: True" && break
  done
  sleep 3
  for try_cmd in 1 2 3; do
    pub_setup 3 COMMAND_CONTROL
    sleep 2
    rostopic echo -n1 /uav1/prometheus/control_state 2>/dev/null | grep -q "control_state: 2" && break
  done
  meta_set arm_tries "$try_arm"; meta_set command_switch_tries "$try_cmd"

  # step 7: hover check (I-3, Q-A)
  local z
  for i in $(seq 1 40); do
    z=$(rostopic echo -n1 /uav1/mavros/local_position/odom/pose/pose/position/z 2>/dev/null | head -1)
    awk "BEGIN{exit !(${z:-0} > 1.35)}" && break; sleep 1
  done
  sleep 10
  local probe zm zlo zhi vmean vmax
  for probe in 1 2 3 4; do   # "wait until" steady: up to 4 x 5 s windows, I-3 only if none qualifies
    # z band on min/max (Q-A); steadiness on the window MEAN speed (instantaneous odom speed peaks ~0.1 m/s in hover)
    read -r zm zlo zhi vmean vmax <<< "$(hover_probe)"
    awk "BEGIN{exit !($zlo >= 1.35 && $zhi <= 1.65 && $vmean < 0.1)}" 2>/dev/null && break
  done
  meta_set hover_probes "$probe"
  HOVER=$zm
  meta_set hover_z_mean "$zm"; meta_set hover_z_min "$zlo"; meta_set hover_z_max "$zhi"; meta_set hover_v_mean "$vmean"; meta_set hover_v_max "$vmax"
  local st; st=$(rostopic echo -n1 /uav1/prometheus/state 2>/dev/null)
  local armed mode cs
  armed=$(echo "$st" | grep -c "armed: True"); mode=$(echo "$st" | grep -c 'mode: "OFFBOARD"')
  cs=$(rostopic echo -n1 /uav1/prometheus/control_state 2>/dev/null | grep -c "control_state: 2")
  meta_set hover_armed "$armed"; meta_set hover_offboard "$mode"; meta_set hover_command_state "$cs"
  if ! awk "BEGIN{exit !($zlo >= 1.35 && $zhi <= 1.65 && $vmean < 0.1)}" 2>/dev/null || [ "$armed$mode$cs" != "111" ]; then
    CODE=I-3; NOTES="hover z $zlo..$zhi |v|mean $vmean max $vmax armed/offboard/command $armed$mode$cs"; return
  fi

  # step 8: gate silent before the goal
  read -r gn gt <<< "$(gate_count)"
  meta_set gate_before_goal "$gn"
  [ "${gn:-x}" = 0 ] || { CODE=I-11; NOTES="gate published $gn msgs before the goal"; return; }

  # step 9: obstacle script (static variants too: empty seed file); never stopped here (B-2)
  (source "$ROOT/devel/setup.bash"; PYTHONUNBUFFERED=1 rosrun jps_vo_planner dyn_obstacles.py _scenario:=$SC \
     _variant:=$V _seed:=$K _log_dir:=$LOG_DIR > "$LOG_DIR/dyn_obstacles.log" 2>&1 &)
  sleep 5
  local nproc nmod
  nproc=$(pgrep -c -f "^(/usr/bin/)?python3 [^ ]*dyn_obstacles\.py")
  nmod=$(rostopic echo -n1 /gazebo/model_states/name 2>/dev/null | grep -c "dyn_obs_")
  meta_set obstacle_scripts "$nproc"; meta_set obstacle_models "$nmod"
  if [ "${NOBS[$V]}" -gt 0 ] && { [ "$nproc" != 1 ] || [ "$nmod" != "${NOBS[$V]}" ]; }; then
    CODE=I-4; NOTES="obstacle scripts $nproc, models $nmod (expected ${NOBS[$V]})"; return
  fi

  # step 10: goal (t0)
  rosrun jps_vo_planner send_goal.py _scenario:=$SC _variant:=$V >> "$LOG_DIR/send_goal.log" 2>&1
  local t_goal; t_goal=$(date +%s)
  meta_set goal_sent_wall "$t_goal"

  # step 11: gate in flight (V-6; recorded, not an invalid code)
  sleep 6
  read -r gn gt <<< "$(gate_count)"
  GATE=$(awk "BEGIN{printf \"%.1f\", ${gn:-0}/5.0}")
  meta_set gate_in_flight_hz "$GATE"; meta_set gate_in_flight_true "$gt"
  [ "${gn:-0}" -ge 45 ] && [ "${gn:-0}" -le 55 ] && [ "$gn" = "$gt" ] || NOTES="V-6 gate $gn msgs/5 s ($gt true)"

  # step 12: wait for the goal (checks.yaml) or t0 + timeout + 5 s; F-4 if the planner node dies
  local t_end=$(( t_goal + ${TMO[$V]} + 5 )) crashed=0
  while [ ! -f "$LOG_DIR/checks.yaml" ] && [ "$(date +%s)" -lt $t_end ]; do
    sleep 2
    rosnode ping -c 1 $NODE >/dev/null 2>&1 || { crashed=1; break; }
  done
  meta_set planner_crashed "$crashed"
  [ $crashed = 1 ] && NOTES="${NOTES:+$NOTES; }planner node died during the run (F-4)"

  # step 13: shut down (node writes checks.yaml on shutdown if the goal was not reached)
  bash "$EXP/kill_sim.sh" >> "$LOG" 2>&1
  meta_set end_wall "$(date +%s.%N)"

  # step 14: evaluation, then file-based invalid checks (single source: post_process_ablation.py)
  python3 "$PKG/scripts/verify_run.py" "$LOG_DIR" --timeout "${TMO[$V]}" > "$LOG_DIR/verify.txt" 2>&1
  CODE=$(python3 "$PKG/scripts/post_process_ablation.py" --check-run "$LOG_DIR" 2>>"$LOG" | tail -1)
  case "$CODE" in valid) CODE="" ;; I-*) ;; *) NOTES="${NOTES:+$NOTES; }check-run output '$CODE'"; CODE=I-8 ;; esac
}

end_reason_of() {
  python3 -c "
import yaml,sys
try: print(yaml.safe_load(open('$1/metrics.yaml'))['metrics']['end_reason'])
except Exception: print('')" 2>/dev/null
}

# ---------------------------------------------------------------- main loop (B-6 order)
say "batch start: tag=$TAG out=$OUT seeds=[$SEEDS] variants=[$VARIANTS] groups=[${ALL_GROUPS[*]}] RECORD=$RECORD"
prev_code=""; same_code_run=0; pre_fail_run=0
declare -A FIRST_RUNS=() FAULTS=()
for K in $SEEDS; do
  for V in $VARIANTS; do
    n=${#ALL_GROUPS[@]}
    for i in $(seq 0 $((n - 1))); do
      G=${ALL_GROUPS[$(( (i + K - 1) % n ))]}             # rotated group order per seed
      state=$(cell_state "$V" "$G" "$K")
      [ "$state" = valid ] || [ "$state" = excluded ] && { say "skip $V $G seed$K ($state)"; continue; }
      A=$(( state + 1 ))
      while [ $A -le "$MAX_ATTEMPTS" ]; do
        if [ "${DRY_RUN:-0}" = 1 ]; then say "plan $V $G seed$K attempt $A"; break; fi
        free_gb=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc 0-9)
        [ "$free_gb" -ge 5 ] || die "disk free ${free_gb} GB < 5 GB (B-11)"
        say "run $V $G seed$K attempt $A"
        t_s=$(date +%s); run_attempt "$V" "$G" "$K" "$A"; t_e=$(date +%s)
        er=$( [ -d "$LOG_DIR" ] && end_reason_of "$LOG_DIR" )
        echo "$TAG,$V,$G,$K,$A,$LOG_DIR,$t_s,$t_e,$PRE,$HOVER,$CODE,\"$er\",$GATE,\"$NOTES\"" >> "$INDEX"
        say "  -> invalid='${CODE}' end_reason='$er' hover=$HOVER gate=$GATE notes='$NOTES'"
        # B-11 abort rules
        if [ "$CODE" = I-0 ]; then pre_fail_run=$((pre_fail_run + 1)); else pre_fail_run=0; fi
        [ $pre_fail_run -ge 3 ] && die "precheck failed 3 times in a row"
        if [ -n "$CODE" ] && [ "$CODE" = "$prev_code" ]; then same_code_run=$((same_code_run + 1)); else same_code_run=1; fi
        prev_code=$CODE; [ -z "$CODE" ] && same_code_run=0
        [ $same_code_run -ge 5 ] && die "$CODE in 5 consecutive attempts"
        if [ -z "$CODE" ]; then
          FIRST_RUNS[$G]=$(( ${FIRST_RUNS[$G]:-0} + 1 ))
          case "$NOTES" in *"F-4"*) FAULTS[$G]=1 ;; esac
          [ "${FAULTS[$G]:-0}" = 1 ] && [ "${FIRST_RUNS[$G]}" -le 3 ] && die "planner fault (F-4) in an early run of $G: build/config fault?"
          break
        fi
        A=$((A + 1))
      done
      [ "${DRY_RUN:-0}" = 1 ] || [ $A -le "$MAX_ATTEMPTS" ] || say "cell $V $G seed$K EXCLUDED after $MAX_ATTEMPTS invalid attempts"
    done
  done
done
bash "$EXP/kill_sim.sh" >> "$LOG" 2>&1
say "batch end"
