#!/usr/bin/env bash
# Profile the invariant smoother for several fixed-lag window sizes on the same
# recorded bag, using the deterministic replay of the online pipeline.
#
# Usage: profile_window_sizes.sh BAG [MODE] [WINDOW_SIZES...]
#   MODE: lossless (default; every sample processed, measures execution time)
#         realtime (recorded pace; measures backlog, drops, latency)
# Example:
#   profile_window_sizes.sh ~/bags/go2_walk lossless 1 2 3 5 10 20
#
# Results: one run directory per window size in $SEB_LOG_DIR (default
# ~/.ros/seb_online/window_profile_<date>) and a comparison table.
set -euo pipefail

BAG=${1:?usage: profile_window_sizes.sh BAG [MODE] [WINDOW_SIZES...]}
MODE=${2:-lossless}
shift || true
shift || true
SIZES=("$@")
if [ ${#SIZES[@]} -eq 0 ]; then SIZES=(1 2 3 5 10); fi

SHARE=$(ros2 pkg prefix seb_online)/share/seb_online
LIB=$(ros2 pkg prefix seb_online)/lib/seb_online
CONFIG=${SEB_CONFIG:-$SHARE/config/go2_online.yaml}
LOG_DIR=${SEB_LOG_DIR:-$HOME/.ros/seb_online/window_profile_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOG_DIR"

RUNS=()
for W in "${SIZES[@]}"; do
  NAME="smoother_w${W}_${MODE}"
  echo ">>> window_size=${W} (${MODE})"
  ros2 run seb_online seb_replay --ros-args --params-file "$CONFIG" \
    -p replay.bag_path:="$BAG" -p replay.mode:="$MODE" \
    -p estimator.type:=invariant_smoother -p smoother.window_size:="$W" \
    -p log.directory:="$LOG_DIR" -p log.run_name:="$NAME"
  RUNS+=("$LOG_DIR/$NAME")
done

python3 "$LIB/analyze_run.py" "${RUNS[@]}" --json "$LOG_DIR/summary.json"
