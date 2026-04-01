#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"

DATASET_REL="data/anymalD_grandtour"

usage() {
  cat <<EOF
Usage: ./run_all_estimators.sh [--dataset <relative/path/from/repo-root>]

Examples:
  ./run_all_estimators.sh
  ./run_all_estimators.sh --dataset data/anymalD_grandtour
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset)
      DATASET_REL="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

DATASET_ABS="${REPO_ROOT}/${DATASET_REL}"

if [[ ! -d "${DATASET_ABS}" ]]; then
  echo "Dataset folder not found: ${DATASET_ABS}" >&2
  exit 1
fi

build_module() {
  local module_dir="$1"
  local build_dir="${module_dir}/build"

  cmake -S "${module_dir}" -B "${build_dir}"
  cmake --build "${build_dir}" -j"$(nproc)"
}

echo "[1/5] Building and running data_process tools"
build_module "${REPO_ROOT}/data_process"
"${REPO_ROOT}/data_process/build/inspect_dataset" "${DATASET_ABS}"
"${REPO_ROOT}/data_process/build/precompute_feet_kinematics" \
  "${DATASET_ABS}/sensor_data.csv"

echo "[2/5] Building and running MUSE"
build_module "${REPO_ROOT}/muse"
"${REPO_ROOT}/muse/build/main_muse"

echo "[3/5] Building and running IEKF"
build_module "${REPO_ROOT}/iekf"
"${REPO_ROOT}/iekf/build/main_iekf" "${DATASET_ABS}"

echo "[4/5] Building and running Invariant Smoother"
build_module "${REPO_ROOT}/invariant_smoother"
"${REPO_ROOT}/invariant_smoother/build/main_invariant_smoother" "${DATASET_ABS}"

echo "[5/5] Done"
echo "Generated outputs:"
echo "  - ${DATASET_ABS}/muse/fused_state.csv"
echo "  - ${DATASET_ABS}/iekf/fused_state.csv"
echo "  - ${DATASET_ABS}/invariant_smoother/fused_state.csv"
