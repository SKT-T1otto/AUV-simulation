#!/usr/bin/env bash
# Run from a terminal on the Ubuntu graphics desktop, as the desktop user.
set -eo pipefail
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
root="$(cd -- "${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}" && pwd -P)"
container="${CRK_CONTAINER:-crk-ros2-home}"
if [ -z "${DISPLAY:-}" ]; then
  echo '请在 Ubuntu 图形桌面的终端中启动，需要 DISPLAY。' >&2
  exit 1
fi
# Start the existing container and check the same bind mounts used for the trials.
mounts="$(sudo docker inspect --type container --format '{{range .Mounts}}{{println .Source .Destination .RW}}{{end}}' "$container")"
grep -Fxq "$project /root/AUV-simulation true" <<< "$mounts" &&
  grep -Fxq "$root $root true" <<< "$mounts" ||
  { echo '现有容器的源码或结果挂载与当前目录不符。' >&2; exit 1; }
mkdir -p "$root/.container"
cookie="$(xauth nlist "$DISPLAY")"
[ -n "$cookie" ] || { echo '当前图形会话缺少 X11 授权。' >&2; exit 1; }
touch "$root/.container/xauth"
chmod 600 "$root/.container/xauth"
printf '%s\n' "$cookie" | sed 's/^..../ffff/' | xauth -f "$root/.container/xauth" nmerge -
sudo docker start "$container" > /dev/null
echo 'Gazebo 水下观测站巡检 | 蓝、绿、紫为搜索艇，橙色为执行艇，黄色为固定观测站。'
echo '世界将自动步进。终端同步显示发现、交接、执行艇距站距离和驻留计数。'
io=(-i)
if [[ -t 0 && -t 1 ]]; then io=(-it); fi
exec sudo docker exec "${io[@]}" -u 0 -e "DISPLAY=$DISPLAY" -e "XAUTHORITY=$root/.container/xauth" \
  -e "CRK_OUTPUT_ROOT=$root" -e "CRK_OUTPUT_UID=$(id -u)" -e "CRK_OUTPUT_GID=$(id -g)" \
  -e "CRK_SOURCE_SHA=$(git -C "$project" rev-parse HEAD)" \
  -e "CRK_MANIFEST=$(sha256sum "$project/crk_auv_ch3_v1/SHA256SUMS" | cut -d' ' -f1)" \
  "$container" bash -c '
    set -eo pipefail
    cd /root/AUV-simulation/crk_auv_ch3_v1
    mountpoint -q "$CRK_OUTPUT_ROOT"
    printf "%s  SHA256SUMS\n" "$CRK_MANIFEST" | sha256sum -c -
    sha256sum --quiet -c SHA256SUMS
    source /opt/ros/jazzy/setup.bash
    export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONIOENCODING=utf-8
    exec /opt/crk-g5-cpu-v1/bin/python -B -u -m crk_ch3.search_run --mode demo
  '
