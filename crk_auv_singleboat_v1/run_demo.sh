#!/usr/bin/env bash
set -eo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$BASE"
source /opt/ros/jazzy/setup.bash
if [ ! -f "$BASE/install/setup.bash" ]; then
  printf 'First run: bash install_and_build.sh\n' >&2
  exit 1
fi
source "$BASE/install/setup.bash"
# Isolate this Gazebo transport graph and ROS domain from the previous shapes demo.
export GZ_PARTITION=crk_singleboat_v1
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
mkdir -p "$BASE/results"
RUN_DIR="$(mktemp -d "$BASE/results/run_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
SHARE="$(ros2 pkg prefix crk_auv_demo)/share/crk_auv_demo"
printf 'Run directory: %s\n' "$RUN_DIR"
printf 'One command starts Gazebo, clock/odometry/thrust bridges and controller.\n'
GZ_PID=''
BRIDGE_PID=''
CONTROLLER_PID=''
cleanup() {
  trap - EXIT INT TERM
  if [ -n "$CONTROLLER_PID" ] && kill -0 "$CONTROLLER_PID" 2>/dev/null; then
    kill -INT "$CONTROLLER_PID" 2>/dev/null || true
    wait "$CONTROLLER_PID" 2>/dev/null || true
  fi
  for PID in "$BRIDGE_PID" "$GZ_PID"; do
    if [ -n "$PID" ]; then
      kill -INT "$PID" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
gz sim -r -v 3 "$SHARE/worlds/single_auv.sdf" > "$RUN_DIR/gazebo.log" 2>&1 &
GZ_PID=$!
# Launch the executable directly so signals reach the process rather than a ros2 CLI wrapper.
BRIDGE_EXE="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"
"$BRIDGE_EXE" --ros-args -p config_file:="$SHARE/config/bridge.yaml" > "$RUN_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2
CONTROLLER_EXE="$(ros2 pkg prefix crk_auv_demo)/lib/crk_auv_demo/controller"
"$CONTROLLER_EXE" --ros-args -p use_sim_time:=true -p output_dir:="$RUN_DIR" &
CONTROLLER_PID=$!
STATUS=0
wait -n "$GZ_PID" "$BRIDGE_PID" "$CONTROLLER_PID" || STATUS=$?
printf '\nProcess finished (exit %s). Results: %s\n' "$STATUS" "$RUN_DIR"
exit "$STATUS"
