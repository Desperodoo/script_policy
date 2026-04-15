#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${1:-script_policy}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOTWIN_DIR="${REPO_ROOT}/third_party/RoboTwin"

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "${ENV_NAME}"

cd "${ROBOTWIN_DIR}/assets"
python _download.py

unzip -o background_texture.zip
rm -f background_texture.zip

unzip -o embodiments.zip
rm -f embodiments.zip

unzip -o objects.zip
rm -f objects.zip

cd "${ROBOTWIN_DIR}"
python ./script/update_embodiment_config_path.py

echo "[download_robotwin_assets] Assets downloaded and paths updated."
