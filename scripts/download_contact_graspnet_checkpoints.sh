#!/usr/bin/env bash

set -euo pipefail

PYTHON_BIN="${1:-/home/amax/miniforge-pypy3/envs/script_policy/bin/python}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CGN_REPO="${REPO_ROOT}/third_party/contact_graspnet"
CKPT_DIR="${CGN_REPO}/checkpoints"
FOLDER_URL="https://drive.google.com/drive/folders/1tBHKf60K8DLM5arm-Chyf7jxkzOr5zGl?usp=sharing"

mkdir -p "${CKPT_DIR}"
cd "${CKPT_DIR}"

echo "[Contact-GraspNet] downloading checkpoints into ${CKPT_DIR}"
"${PYTHON_BIN}" -m gdown --folder "${FOLDER_URL}" --continue

if [[ -d "${CKPT_DIR}/contact_graspnet_models" ]]; then
  echo "[Contact-GraspNet] normalizing checkpoint layout"
  for ckpt_subdir in "${CKPT_DIR}"/contact_graspnet_models/*; do
    if [[ -d "${ckpt_subdir}" ]]; then
      ln -sfn "contact_graspnet_models/$(basename "${ckpt_subdir}")" "${CKPT_DIR}/$(basename "${ckpt_subdir}")"
    fi
  done
fi

echo "[Contact-GraspNet] done. Default expected checkpoint dir:"
echo "  - scene_test_2048_bs3_hor_sigma_001"
