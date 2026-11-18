#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
OUTPUT_ROOT="${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}"
if [ ! -f /.dockerenv ] || ! mountpoint -q "$OUTPUT_ROOT"; then
  printf 'Use tools/run_ch3_envelope.sh on the Ubuntu host.\n' >&2; exit 1
fi
if ! [[ "${CRK_OUTPUT_UID:-}" =~ ^[0-9]+$ && "${CRK_OUTPUT_GID:-}" =~ ^[0-9]+$ ]]; then
  printf 'Desktop UID/GID required.\n' >&2; exit 1
fi
PYTHON=/opt/crk-g5-cpu-v1/bin/python
test -x "$PYTHON" || { printf 'Existing G5 CPU environment missing.\n' >&2; exit 1; }
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
sha256sum -c SHA256SUMS
"$PYTHON" -B -c 'import rclpy,torch,numpy,scipy; from ros_gz_interfaces.msg import Contacts,WorldStatistics; from ros_gz_interfaces.srv import ControlWorld,SetEntityPose'
"$PYTHON" -B -m unittest discover -s tests -v
"$PYTHON" -B -m unittest discover -s tests_g5 -v
"$PYTHON" -B -u -m crk_ch3.motion_run "$@"
