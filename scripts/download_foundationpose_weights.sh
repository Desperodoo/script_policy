#!/usr/bin/env bash

set -euo pipefail

PYTHON_BIN="${1:-/home/amax/miniforge-pypy3/envs/script_policy/bin/python}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FP_REPO="${REPO_ROOT}/third_party/FoundationPose"
WEIGHTS_DIR="${FP_REPO}/weights"
FOLDER_URL="https://drive.google.com/drive/folders/1DFezOAD0oD1BblsXVxqDsl8fj0qzB82i?usp=sharing"

mkdir -p "${WEIGHTS_DIR}"
cd "${WEIGHTS_DIR}"

echo "[FoundationPose] downloading weights into ${WEIGHTS_DIR}"
"${PYTHON_BIN}" -m gdown --folder "${FOLDER_URL}" --continue

if [[ -d "${WEIGHTS_DIR}/no_diffusion" ]]; then
  echo "[FoundationPose] normalizing no_diffusion layout"
  for run_dir in "${WEIGHTS_DIR}"/no_diffusion/*; do
    if [[ -d "${run_dir}" ]]; then
      ln -sfn "no_diffusion/$(basename "${run_dir}")" "${WEIGHTS_DIR}/$(basename "${run_dir}")"
    fi
  done
fi

echo "[FoundationPose] done. Expected subdirs include:"
echo "  - 2023-10-28-18-33-37"
echo "  - 2024-01-11-20-02-45"
