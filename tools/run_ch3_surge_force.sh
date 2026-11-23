#!/usr/bin/env bash
# Ubuntu host: two isolated worlds, one figure and one verified ZIP.
set -eo pipefail
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
root="$(cd -- "${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}" && pwd -P)"
sha="$(git -C "$project" rev-parse HEAD)"
manifest="$(sha256sum "$project/crk_auv_ch3_v1/SHA256SUMS" | cut -d' ' -f1)"
exec sudo docker exec -i -u 0 -e "CRK_OUTPUT_ROOT=$root" -e "CRK_OUTPUT_UID=$(id -u)" \
  -e "CRK_OUTPUT_GID=$(id -g)" -e "CRK_SOURCE_SHA=$sha" -e "CRK_MANIFEST=$manifest" \
  "${CRK_CONTAINER:-crk-ros2-home}" bash -c '
    set -eo pipefail
    cd /root/AUV-simulation/crk_auv_ch3_v1
    mountpoint -q "$CRK_OUTPUT_ROOT"
    printf "%s  SHA256SUMS\n" "$CRK_MANIFEST" | sha256sum -c -
    sha256sum --quiet -c SHA256SUMS
    source /opt/ros/jazzy/setup.bash
    export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
    export MPLCONFIGDIR=/tmp/crk-surge-mpl
    exec /opt/crk-g5-cpu-v1/bin/python -B -u -m crk_ch3.surge_batch
  '
