#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
OUTPUT_ROOT="${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}"
if [ ! -f /.dockerenv ] || ! mountpoint -q "$OUTPUT_ROOT"; then
  printf 'Use tools/run_ch3_mission_check.sh on the Ubuntu host; mounted results are required.\n' >&2
  exit 1
fi
if [ "$(id -u)" != 0 ] || ! [[ "${CRK_OUTPUT_UID:-}" =~ ^[0-9]+$ && "${CRK_OUTPUT_GID:-}" =~ ^[0-9]+$ ]]; then
  printf 'Container root and host desktop UID/GID are required.\n' >&2; exit 1
fi
sha256sum -c SHA256SUMS
# A CPU-only environment avoids adding CUDA wheels or changing the ROS Python.
G5_ENV=/opt/crk-g5-cpu-v1
if [ ! -x "$G5_ENV/bin/python" ]; then
  apt-get update
  apt-get install -y python3-venv
  python3 -m venv --system-site-packages "$G5_ENV"
fi
if ! "$G5_ENV/bin/python" -c 'import torch,numpy,scipy; assert torch.__version__.split("+")[0] == "2.6.0"; assert torch.version.cuda is None; assert numpy.__version__ == "1.26.4"' >/dev/null 2>&1; then
  "$G5_ENV/bin/python" -m pip install --no-cache-dir 'numpy==1.26.4'
  "$G5_ENV/bin/python" -m pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu 'torch==2.6.0'
fi
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
"$G5_ENV/bin/python" -B -m unittest discover -s tests_g5 -v
"$G5_ENV/bin/python" -B -u -m crk_ch3.mission_check
