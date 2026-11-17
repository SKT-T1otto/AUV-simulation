#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
OUTPUT_ROOT="${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}"
if [ ! -f /.dockerenv ] || ! mountpoint -q "$OUTPUT_ROOT"; then
  printf 'Use tools/run_ch3_search.sh from the Ubuntu host with mounted results.\n' >&2; exit 1
fi
if [ "$(id -u)" != 0 ] || ! [[ "${CRK_OUTPUT_UID:-}" =~ ^[0-9]+$ && "${CRK_OUTPUT_GID:-}" =~ ^[0-9]+$ ]]; then
  printf 'Container root and desktop UID/GID are required.\n' >&2; exit 1
fi
G6_PYTHON=/opt/crk-g5-cpu-v1/bin/python
if [ ! -x "$G6_PYTHON" ]; then
  printf 'Run tools/run_ch3_mission_check.sh on the host once to install the G5 CPU runtime.\n' >&2; exit 1
fi
sha256sum -c SHA256SUMS
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
"$G6_PYTHON" -B -c 'import rclpy,torch,numpy,scipy; from ros_gz_interfaces.msg import Contacts,WorldStatistics; from ros_gz_interfaces.srv import ControlWorld,SetEntityPose; from crk_ch3.native_source import activate_native; activate_native()'
"$G6_PYTHON" -B -m unittest discover -s tests -v
"$G6_PYTHON" -B -m unittest discover -s tests_g5 -v
"$G6_PYTHON" -B -u -m crk_ch3.search_run "$@"
