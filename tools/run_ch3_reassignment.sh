#!/usr/bin/env bash
# Ubuntu host only. Reuse the existing container and mounts; never install.
set -eo pipefail
PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
OUTPUT_ROOT=/home/legion/AUV-simulation
CONTAINER=crk-ros2-home
case "${1:-}" in
  baseline32|defer_search_reassignment32|--compare) ;;
  *) printf 'Usage: bash tools/run_ch3_reassignment.sh baseline32|defer_search_reassignment32 [--headless]\n       bash tools/run_ch3_reassignment.sh --compare BASELINE_DIR DEFER_DIR\n' >&2; exit 2 ;;
esac
MOUNTS="$(sudo docker inspect --format '{{range .Mounts}}{{.Type}}|{{.Source}}|{{.Destination}}|{{.RW}}{{println}}{{end}}' "$CONTAINER")"
for EXPECTED in "bind|$OUTPUT_ROOT|$OUTPUT_ROOT|true" "bind|$PROJECT|/root/AUV-simulation|true"; do
  if ! printf '%s\n' "$MOUNTS" | grep -Fxq "$EXPECTED"; then
    printf 'Existing container mount mismatch: %s; no container changed.\n' "$EXPECTED" >&2; exit 1
  fi
done
sudo docker start "$CONTAINER" >/dev/null
if [ "${1:-}" != --compare ] && [ "${2:-}" != --headless ]; then
  if [ -z "${DISPLAY:-}" ]; then printf 'DISPLAY missing; use --headless or run in desktop terminal.\n' >&2; exit 1; fi
  XAUTH_FILE="$OUTPUT_ROOT/.container/xauth"
  touch "$XAUTH_FILE"
  chmod 600 "$XAUTH_FILE"
  XAUTH_ENTRIES="$(xauth nlist "$DISPLAY")"
  if [ -z "$XAUTH_ENTRIES" ]; then printf 'Missing X11 authority.\n' >&2; exit 1; fi
  printf '%s\n' "$XAUTH_ENTRIES" | sed 's/^..../ffff/' | xauth -f "$XAUTH_FILE" nmerge -
fi
sudo docker exec -i -u 0 \
  -e "CRK_OUTPUT_ROOT=$OUTPUT_ROOT" \
  -e "CRK_OUTPUT_UID=$(id -u)" -e "CRK_OUTPUT_GID=$(id -g)" \
  -e "DISPLAY=${DISPLAY:-}" -e "XAUTHORITY=$OUTPUT_ROOT/.container/xauth" \
  "$CONTAINER" bash /root/AUV-simulation/crk_auv_ch3_v1/run_reassignment_linux.sh "$@"
