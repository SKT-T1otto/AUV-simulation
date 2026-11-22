#!/usr/bin/env bash
# Ubuntu host, one fresh formal-length mission per initial admission condition.
set -eo pipefail
cd -- "$(dirname -- "$0")/../crk_auv_ch3_v1"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
exec python3 -B -m crk_ch3.far_pair "$@"
