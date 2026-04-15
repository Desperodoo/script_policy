#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${1:-script_policy}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOTWIN_DIR="${REPO_ROOT}/third_party/RoboTwin"

if [[ ! -d "${ROBOTWIN_DIR}" ]]; then
  echo "RoboTwin repo not found at ${ROBOTWIN_DIR}" >&2
  exit 1
fi

source "$(conda info --base)/etc/profile.d/conda.sh"

if ! conda env list | awk '{print $1}' | rg -qx "${ENV_NAME}"; then
  echo "[setup_robotwin_env] Creating conda env ${ENV_NAME} with Python 3.10"
  conda create -n "${ENV_NAME}" python=3.10 -y
fi

conda activate "${ENV_NAME}"

echo "[setup_robotwin_env] Installing RoboTwin Python requirements"
pip install -r "${ROBOTWIN_DIR}/script/requirements.txt"

echo "[setup_robotwin_env] Installing setuptools<81 for sapien compatibility"
pip install "setuptools<81"

echo "[setup_robotwin_env] Installing pytorch3d"
pip install "git+https://github.com/facebookresearch/pytorch3d.git@stable" --no-build-isolation

echo "[setup_robotwin_env] Installing curobo"
if [[ ! -d "${ROBOTWIN_DIR}/envs/curobo" ]]; then
  git -C "${ROBOTWIN_DIR}/envs" clone https://github.com/NVlabs/curobo.git
fi
pip install -e "${ROBOTWIN_DIR}/envs/curobo" --no-build-isolation

echo "[setup_robotwin_env] Patching sapien and mplib"
python "${REPO_ROOT}/scripts/patch_robotwin_dependencies.py"

echo "[setup_robotwin_env] Done. Next step: download RoboTwin assets."
