#!/usr/bin/env bash
set -eo pipefail
PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Existing container only: no setup, install, recreate or package download.
OUTPUT_ROOT="$(cd -- "${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}" && pwd -P)"
CASE="${1:-x}"
case "$CASE" in x|y|z|track) ;; *) printf 'First argument: x, y, z or track\n' >&2; exit 1;; esac
if [ "$#" -gt 0 ]; then shift; fi
sudo docker exec -i -u 0 -e "CRK_OUTPUT_ROOT=$OUTPUT_ROOT" \
  -e "CRK_OUTPUT_UID=$(id -u)" -e "CRK_OUTPUT_GID=$(id -g)" \
  -e "DISPLAY=${DISPLAY:-:0}" -e "XAUTHORITY=$OUTPUT_ROOT/.container/xauth" \
  "${CRK_CONTAINER:-crk-ros2-home}" \
  bash /root/AUV-simulation/crk_auv_ch3_v1/run_envelope_linux.sh --envelope-case "$CASE" "$@"
