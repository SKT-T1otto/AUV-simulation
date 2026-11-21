#!/usr/bin/env bash
# Ubuntu host entry. The coordinator owns both isolated runs and the final evidence archive.
set -eo pipefail
cd -- "$(dirname -- "$0")/../crk_auv_ch3_v1"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
exec python3 -B -m crk_ch3.initial_batch "$@"
