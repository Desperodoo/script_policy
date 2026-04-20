#!/usr/bin/env bash

set -euo pipefail

ENV_NAME="${1:-script_policy_contact_graspnet}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CGN_REPO="${REPO_ROOT}/third_party/contact_graspnet"
ENV_FILE="${CGN_REPO}/contact_graspnet_env.yml"

if [[ ! -f "${ENV_FILE}" ]]; then
  echo "Contact-GraspNet env file not found: ${ENV_FILE}" >&2
  exit 1
fi

echo "[Contact-GraspNet] creating env: ${ENV_NAME}"
conda env create -n "${ENV_NAME}" -f "${ENV_FILE}"

echo "[Contact-GraspNet] compiling PointNet++ tf ops"
conda run -n "${ENV_NAME}" bash -lc '
  cd "'"${CGN_REPO}"'"
  sh compile_pointnet_tfops.sh
'

echo "[Contact-GraspNet] env ready: ${ENV_NAME}"
