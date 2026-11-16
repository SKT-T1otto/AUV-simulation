#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  printf 'ROS 2 Jazzy is missing; run tools/run_ch3_motion.sh on the Ubuntu host.\n' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
command -v gz >/dev/null || { printf 'gz unavailable after sourcing Jazzy.\n' >&2; exit 1; }
OUTPUT_ROOT="${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}"
if [ -f /.dockerenv ] && ! mountpoint -q "$OUTPUT_ROOT"; then
  printf 'Host output directory is not mounted.\n' >&2; exit 1
fi
if [ "$(id -u)" = 0 ] && ! [[ "${CRK_OUTPUT_UID:-}" =~ ^[0-9]+$ && "${CRK_OUTPUT_GID:-}" =~ ^[0-9]+$ ]]; then
  printf 'Desktop UID/GID are required; use the host launcher.\n' >&2; exit 1
fi
sha256sum -c SHA256SUMS
python3 -c 'import rclpy, numpy, scipy; from ros_gz_interfaces.srv import ControlWorld; assert hasattr(ControlWorld.Request().world_control, "multi_step")'
python3 -m unittest discover -s tests -v
python3 -u -m crk_ch3.motion_run "$@"
