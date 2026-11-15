#!/usr/bin/env bash
set -eo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$BASE"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  printf 'ROS 2 Jazzy is missing. Run this script inside crk-ros2-dev.\n' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends python3-numpy python3-scipy python3-colcon-common-extensions
python3 src/crk_auv_demo/crk_auv_demo/generate_assets.py src/crk_auv_demo
python3 -m unittest discover -s tests -v
gz sdf -k src/crk_auv_demo/worlds/single_auv.sdf
colcon build --symlink-install --packages-select crk_auv_demo
printf '\nBuild completed. Next: bash run_demo.sh\n'
