#!/usr/bin/env bash
# Detached inside the existing container; publish a status only after supervisor cleanup.
set -eo pipefail
umask 022
condition="$1"
batch="$2"
case "$condition" in
    initial_original32|initial_quadrant32) prefix=g6_initial_candidates_pair_; wall=34m ;;
    initial_original400|initial_quadrant400) prefix=g6_far_search_pair400_; wall=124m ;;
    *) exit 2 ;;
esac
root=/home/legion/AUV-simulation
[ -f /.dockerenv ] && [ "$(id -u)" = 0 ] && mountpoint -q "$root" || exit 2
[[ "$(dirname -- "$(realpath -e -- "$batch")")" == "$root/results/ch3" ]] || exit 2
[[ "$(basename -- "$batch")" == "$prefix"* ]] || exit 2
exec >"$batch/$condition.runner.log" 2>&1
finish() {
    rc=$?
    trap - EXIT
    printf '%s\n' "$rc" >"$batch/$condition.exit.tmp"
    mv -- "$batch/$condition.exit.tmp" "$batch/$condition.exit"
}
trap finish EXIT
[[ "$CRK_OUTPUT_UID" =~ ^[0-9]+$ && "$CRK_OUTPUT_GID" =~ ^[0-9]+$ ]]
cd -- "$(dirname -- "$0")"
source /opt/ros/jazzy/setup.bash
test -x /opt/crk-g5-cpu-v1/bin/python
sha256sum --quiet -c SHA256SUMS
if [ -f "$batch/CANCEL" ]; then exit 130; fi
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
timeout --signal=TERM --kill-after=90s "$wall" /opt/crk-g5-cpu-v1/bin/python -B -u -m crk_ch3.search_run \
  --mode search --headless --initial-condition "$condition" --pair-directory "$batch"
