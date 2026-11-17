#!/usr/bin/env bash
# Ubuntu host entry: reuse the verified container and mounted result directory.
set -eo pipefail
PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$PROJECT/tools/run_linux.sh" --setup-only
OUTPUT_ROOT="$(cd -- "${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}" && pwd -P)"
sudo docker exec -it -u 0 \
  -e "CRK_OUTPUT_ROOT=$OUTPUT_ROOT" \
  -e "CRK_OUTPUT_UID=$(id -u)" -e "CRK_OUTPUT_GID=$(id -g)" \
  "${CRK_CONTAINER:-crk-ros2-home}" \
  bash /root/AUV-simulation/crk_auv_ch3_v1/run_mission_check_linux.sh
