#!/usr/bin/env bash
# Ubuntu host: free sinking, open-loop balance and bidirectional heave.
set -eo pipefail
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_ch3_surge_force.sh" --heave
