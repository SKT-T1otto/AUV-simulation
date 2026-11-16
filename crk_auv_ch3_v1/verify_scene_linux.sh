#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
case "${1:-}" in ''|--gui) ;; *) printf 'Usage: bash verify_scene_linux.sh [--gui]\n' >&2; exit 2 ;; esac
OUTPUT_ROOT="${CRK_OUTPUT_ROOT:-/home/legion/AUV-simulation}"
if [ -f /.dockerenv ] && ! mountpoint -q "$OUTPUT_ROOT"; then
  printf 'Host result root is not mounted. Use tools/run_ch3_scene_check.sh on the host.\n' >&2
  exit 1
fi
command -v gz >/dev/null || { printf 'Gazebo is missing. Run in the configured container.\n' >&2; exit 1; }
OWNER=''
if [ "$(id -u)" = 0 ]; then
  if ! [[ "${CRK_OUTPUT_UID:-}" =~ ^[0-9]+$ && "${CRK_OUTPUT_GID:-}" =~ ^[0-9]+$ ]]; then
    printf 'Desktop UID/GID are required; use the host launcher.\n' >&2; exit 1
  fi
  OWNER="$CRK_OUTPUT_UID:$CRK_OUTPUT_GID"
fi
sha256sum -c SHA256SUMS
python3 -m unittest discover -s tests -p test_contracts.py -v
python3 -m unittest discover -s tests -p test_scenario.py -v
python3 -m unittest discover -s tests -p test_preview.py -v
RUN_DIR="$(python3 -m crk_ch3.preview --fixture)"
cleanup() {
  if [ -n "$OWNER" ]; then chown -hR "$OWNER" "$RUN_DIR"; fi
}
trap cleanup EXIT
if [ -n "$OWNER" ]; then chown "$OWNER" "$RUN_DIR"; fi
STATUS=0
gz sdf -k "$RUN_DIR/world.sdf" > "$RUN_DIR/sdf_validation.log" 2>&1 || STATUS=$?
python3 - "$RUN_DIR" "$STATUS" <<'PY'
import json
from pathlib import Path
import sys
p = Path(sys.argv[1])/'summary.json'
d = json.loads(p.read_text(encoding='utf-8'))
d.update(linux_sdf_checked=True, linux_sdf_exit_code=int(sys.argv[2]),
         linux_sdf_valid=int(sys.argv[2]) == 0)
p.write_text(json.dumps(d, indent=2)+'\n', encoding='utf-8')
PY
cat "$RUN_DIR/sdf_validation.log"
printf '\nG3 scene-check results: %s\n' "$RUN_DIR"
if [ "$STATUS" != 0 ]; then exit "$STATUS"; fi
printf 'SDF validation passed. This is a static layout preview, not a mission experiment.\n'
export GZ_PARTITION=crk_ch3_scene_check
if [ "${1:-}" = --gui ]; then
  gz sim -v 3 "$RUN_DIR/world.sdf"
else
  printf 'Add --gui to inspect the four static role markers, two obstacles, target and wait point.\n'
fi
