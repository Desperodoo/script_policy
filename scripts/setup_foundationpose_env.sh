#!/usr/bin/env bash

set -euo pipefail

ENV_NAME="${1:-script_policy_foundationpose}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FP_REPO="${REPO_ROOT}/third_party/FoundationPose"

if [[ ! -d "${FP_REPO}" ]]; then
  echo "FoundationPose repo not found: ${FP_REPO}" >&2
  exit 1
fi

echo "[FoundationPose] creating/updating env: ${ENV_NAME}"
conda create -y -n "${ENV_NAME}" python=3.9 pip
conda install -y -n "${ENV_NAME}" -c conda-forge eigen=3.4.0 cmake ninja
conda install -y -n "${ENV_NAME}" -c nvidia cuda-cudart=11.8 cuda-cudart-dev=11.8 cuda-nvcc=11.8 cuda-libraries=11.8 cuda-libraries-dev=11.8

echo "[FoundationPose] installing repo requirements"
conda run -n "${ENV_NAME}" python -m pip install -r "${FP_REPO}/requirements.txt"

echo "[FoundationPose] installing nvdiffrast"
conda run -n "${ENV_NAME}" bash -lc '
  export CUDA_HOME="${CONDA_PREFIX}"
  export CPATH="${CONDA_PREFIX}/include:/usr/local/cuda/include:${CPATH:-}"
  export CPLUS_INCLUDE_PATH="${CONDA_PREFIX}/include:/usr/local/cuda/include:${CPLUS_INCLUDE_PATH:-}"
  export LIBRARY_PATH="${CONDA_PREFIX}/lib:${LIBRARY_PATH:-}"
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  python -m pip install --quiet --no-build-isolation --no-cache-dir git+https://github.com/NVlabs/nvdiffrast.git
'

echo "[FoundationPose] installing pytorch3d"
conda run -n "${ENV_NAME}" python -m pip install --quiet --no-index --no-cache-dir pytorch3d \
  -f https://dl.fbaipublicfiles.com/pytorch3d/packaging/wheels/py39_cu118_pyt200/download.html

echo "[FoundationPose] building local extensions"
conda run -n "${ENV_NAME}" bash -lc '
  export CUDA_HOME="${CONDA_PREFIX}"
  export CPATH="${CONDA_PREFIX}/include:/usr/local/cuda/include:${CPATH:-}"
  export CPLUS_INCLUDE_PATH="${CONDA_PREFIX}/include:/usr/local/cuda/include:${CPLUS_INCLUDE_PATH:-}"
  export LIBRARY_PATH="${CONDA_PREFIX}/lib:${LIBRARY_PATH:-}"
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  export CMAKE_PREFIX_PATH="${CONDA_PREFIX}:${CONDA_PREFIX}/lib/python3.9/site-packages/pybind11/share/cmake/pybind11:${CMAKE_PREFIX_PATH:-}"
  cd "'"${FP_REPO}"'"
  bash build_all_conda.sh
'

echo "[FoundationPose] env ready: ${ENV_NAME}"
