#!/usr/bin/env bash
# Run on the Ubuntu host. The existing ROS installation is retained in a snapshot.
set -eo pipefail

if [ -f /.dockerenv ] || [ "$(uname -s)" != Linux ]; then
  printf 'Run this script on the Ubuntu host, outside Docker.\n' >&2
  exit 1
fi
if [ "$(id -u)" = 0 ]; then
  printf 'Run as the desktop user (legion), without sudo; the script invokes sudo when needed.\n' >&2
  exit 1
fi
case "${1:-}" in
  ''|--setup-only) ;;
  *) printf 'Usage: bash tools/run_linux.sh [--setup-only]\n' >&2; exit 2 ;;
esac

PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_ROOT="${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}"
SOURCE_CONTAINER="${CRK_SOURCE_CONTAINER:-crk-ros2-dev}"
CONTAINER="${CRK_CONTAINER:-crk-ros2-home}"
WORKSPACE=/root/AUV-simulation/crk_auv_singleboat_v1
for COMMAND in docker xauth tar; do
  command -v "$COMMAND" >/dev/null || { printf 'Missing command: %s\n' "$COMMAND" >&2; exit 1; }
done
if [ -z "${DISPLAY:-}" ] || [ ! -d /tmp/.X11-unix ]; then
  printf 'An X11 desktop session with DISPLAY is required.\n' >&2
  exit 1
fi
mkdir -p "$OUTPUT_ROOT/results" "$OUTPUT_ROOT/reports" "$OUTPUT_ROOT/figures" "$OUTPUT_ROOT/.container"
OUTPUT_ROOT="$(cd -- "$OUTPUT_ROOT" && pwd -P)"
XAUTH_FILE="$OUTPUT_ROOT/.container/xauth"
touch "$XAUTH_FILE"
chmod 600 "$XAUTH_FILE"
XAUTH_ENTRIES="$(xauth nlist "$DISPLAY")"
if [ -z "$XAUTH_ENTRIES" ]; then
  printf 'No X11 authorization found for DISPLAY=%s\n' "$DISPLAY" >&2
  exit 1
fi
printf '%s\n' "$XAUTH_ENTRIES" | sed 's/^..../ffff/' | xauth -f "$XAUTH_FILE" nmerge -
sudo -v

if ! sudo docker container inspect "$CONTAINER" >/dev/null 2>&1; then
  sudo docker container inspect "$SOURCE_CONTAINER" >/dev/null
  sudo docker start "$SOURCE_CONTAINER" >/dev/null
  # Import existing results once. Extraction as the desktop user makes them accessible.
  if sudo docker exec "$SOURCE_CONTAINER" test -d "$WORKSPACE/results"; then
    sudo docker exec "$SOURCE_CONTAINER" tar -C "$WORKSPACE/results" -cf - . \
      | tar --no-same-owner --skip-old-files -xf - -C "$OUTPUT_ROOT/results"
  fi
  IMAGE="crk-ros2-jazzy:host-output-$(date +%Y%m%d_%H%M%S)-$$"
  printf 'Preserving the installed ROS/Gazebo environment from %s...\n' "$SOURCE_CONTAINER"
  sudo docker commit "$SOURCE_CONTAINER" "$IMAGE" >/dev/null
  sudo docker create --name "$CONTAINER" --pull=never \
    --runtime=nvidia --gpus all --network host --shm-size=1g \
    -e "DISPLAY=$DISPLAY" -e "XAUTHORITY=$XAUTH_FILE" \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
    -e QT_X11_NO_MITSHM=1 -e "CRK_OUTPUT_ROOT=$OUTPUT_ROOT" \
    -e "CRK_OUTPUT_UID=$(id -u)" -e "CRK_OUTPUT_GID=$(id -g)" \
    --mount "type=bind,source=$OUTPUT_ROOT,target=$OUTPUT_ROOT" \
    --mount "type=bind,source=$PROJECT,target=/root/AUV-simulation" \
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly \
    -w "$WORKSPACE" "$IMAGE" sleep infinity >/dev/null
fi

# Verify actual Docker mounts, including reused containers. A same-named directory
# inside the container is insufficient to make output accessible on the host.
MOUNTS="$(sudo docker container inspect --format '{{range .Mounts}}{{printf "%s|%s|%s|%t\n" .Type .Source .Destination .RW}}{{end}}' "$CONTAINER")"
for EXPECTED in "bind|$OUTPUT_ROOT|$OUTPUT_ROOT|true" "bind|$PROJECT|/root/AUV-simulation|true"; do
  if ! printf '%s\n' "$MOUNTS" | grep -Fxq "$EXPECTED"; then
    printf 'Container %s has a different mount configuration. Expected: %s\n' "$CONTAINER" "$EXPECTED" >&2
    printf 'Use a new CRK_CONTAINER name to preserve the existing container.\n' >&2
    exit 1
  fi
done
sudo docker start "$CONTAINER" >/dev/null
EXEC_ARGS=(-u 0 -w "$WORKSPACE" -e "DISPLAY=$DISPLAY" -e "XAUTHORITY=$XAUTH_FILE"
  -e "CRK_OUTPUT_ROOT=$OUTPUT_ROOT" -e "CRK_OUTPUT_UID=$(id -u)" -e "CRK_OUTPUT_GID=$(id -g)")
if ! sudo docker exec "$CONTAINER" test -f "$WORKSPACE/install/setup.bash"; then
  sudo docker exec "${EXEC_ARGS[@]}" "$CONTAINER" bash install_and_build.sh
fi
printf '\nHost output directory: %s\n' "$OUTPUT_ROOT"
if [ "${1:-}" = --setup-only ]; then
  printf 'Setup complete. Run this script without --setup-only to start simulation.\n'
  exit 0
fi

for VARIABLE in CRK_POLICY_MODE CRK_POLICY_FILE CRK_EXTERNAL_ZERO CRK_ACTOR_PATH CRK_ACTOR_METADATA_PATH; do
  if [ "${!VARIABLE+x}" = x ]; then EXEC_ARGS+=(-e "$VARIABLE=${!VARIABLE}"); fi
done
sudo docker exec -it "${EXEC_ARGS[@]}" "$CONTAINER" bash run_demo.sh
