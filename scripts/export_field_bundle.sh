#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_PATH="${1:-${WORKSPACE_ROOT}/spot_field_bundle.tar.gz}"
STAGING_DIR="$(mktemp -d)"
trap 'rm -rf "${STAGING_DIR}"' EXIT

mkdir -p "${STAGING_DIR}/ros2_ws"
rsync -a \
  --exclude '.git/' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '.pytest_cache/' \
  --exclude '__pycache__/' \
  --exclude '.runtime/' \
  --exclude 'config/field.env' \
  --exclude 'spot_field_bundle.tar.gz' \
  "${WORKSPACE_ROOT}/" \
  "${STAGING_DIR}/ros2_ws/"

tar -C "${STAGING_DIR}" -czf "${OUTPUT_PATH}" ros2_ws
printf 'Created field bundle: %s\n' "${OUTPUT_PATH}"
printf 'Credentials in config/field.env were not included.\n'
