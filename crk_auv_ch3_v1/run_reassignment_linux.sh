#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
OUTPUT_ROOT=/home/legion/AUV-simulation
if [ ! -f /.dockerenv ] || ! mountpoint -q "$OUTPUT_ROOT"; then
  printf 'Run tools/run_ch3_reassignment.sh on the Ubuntu host; mounted results required.\n' >&2; exit 1
fi
if [ "$(id -u)" != 0 ] || ! [[ "${CRK_OUTPUT_UID:-}" =~ ^[0-9]+$ && "${CRK_OUTPUT_GID:-}" =~ ^[0-9]+$ ]]; then
  printf 'Container root plus host UID/GID required.\n' >&2; exit 1
fi
DIAG_PYTHON=/opt/crk-g5-cpu-v1/bin/python
if [ ! -x "$DIAG_PYTHON" ]; then printf 'Existing /opt/crk-g5-cpu-v1 is missing; nothing installed.\n' >&2; exit 1; fi
sha256sum -c SHA256SUMS
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
if [ "${1:-}" = --compare ]; then
  if [ "$#" != 3 ]; then printf 'Expected --compare BASELINE_DIR DEFER_DIR\n' >&2; exit 2; fi
  # umask gives the host read access to the unique comparison output.
  umask 022
  "$DIAG_PYTHON" -B -m crk_ch3.reassignment_report --baseline "$2" --defer "$3"
else
  CONDITION="${1:-}"
  case "$CONDITION" in baseline32|defer_search_reassignment32) ;; *) exit 2 ;; esac
  shift
  if [ "$#" -gt 1 ] || { [ "$#" = 1 ] && [ "$1" != --headless ]; }; then
    printf 'Only --headless may follow the condition.\n' >&2; exit 2
  fi
  "$DIAG_PYTHON" -B -c 'import rclpy,torch,numpy,scipy; from ros_gz_interfaces.msg import Contacts,WorldStatistics; from ros_gz_interfaces.srv import ControlWorld,SetEntityPose; from crk_ch3.native_source import activate_native; activate_native()'
  # Pure/core tests only: no hidden mission instances before the actual run.
  "$DIAG_PYTHON" -B -m unittest discover -s tests -v
  "$DIAG_PYTHON" -B -u -m crk_ch3.search_run --mode search --diagnostic-condition "$CONDITION" "$@"
fi
