#!/usr/bin/env bash
# Run on the Ubuntu host, using the verified GPU/X11 container and host mount.
set -eo pipefail
PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$PROJECT/tools/run_linux.sh" --setup-only
OUTPUT_ROOT="$(cd -- "${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}" && pwd -P)"
sudo docker exec -it -u 0 \
  -e "CRK_OUTPUT_ROOT=$OUTPUT_ROOT" \
  -e "CRK_OUTPUT_UID=$(id -u)" -e "CRK_OUTPUT_GID=$(id -g)" \
  -e "DISPLAY=$DISPLAY" -e "XAUTHORITY=$OUTPUT_ROOT/.container/xauth" \
  "${CRK_CONTAINER:-crk-ros2-home}" \
  bash /root/AUV-simulation/crk_auv_ch3_v1/run_search_linux.sh "$@"
