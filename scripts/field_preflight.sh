#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-transport}"
WORKSPACE_ROOT="$(
  cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd
)"
CONFIG_FILE="${FIELD_CONFIG:-${WORKSPACE_ROOT}/config/field.env}"

if [[ -f "${CONFIG_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${CONFIG_FILE}"
  set +a
fi

failures=0

check_value() {
  local name="$1"
  if [[ -z "${!name:-}" ]]; then
    printf 'FAIL: %s is not set\n' "${name}"
    failures=$((failures + 1))
  else
    printf 'OK:   %s is set\n' "${name}"
  fi
}

check_file() {
  local label="$1"
  local path="$2"
  if [[ -f "${path}" ]]; then
    printf 'OK:   %s: %s\n' "${label}" "${path}"
  else
    printf 'FAIL: %s not found: %s\n' "${label}" "${path}"
    failures=$((failures + 1))
  fi
}

camera_index="${CAMERA_INDEX:-0}"
if [[ -e "/dev/video${camera_index}" ]]; then
  printf 'OK:   webcam device: /dev/video%s\n' "${camera_index}"
else
  printf 'FAIL: webcam device missing: /dev/video%s\n' "${camera_index}"
  failures=$((failures + 1))
fi

if python3 -c 'import rclpy, laspy, lazrs' >/dev/null 2>&1; then
  printf 'OK:   ROS Python and LAS/LAZ imports\n'
else
  printf 'FAIL: ROS Python or LAS/LAZ import is unavailable\n'
  failures=$((failures + 1))
fi

trimble_scan_directory="${TRIMBLE_SCAN_DIRECTORY:-/tmp/trimble_scans}"
if [[ -d "${trimble_scan_directory}" ]]; then
  printf 'OK:   Trimble scan directory: %s\n' "${trimble_scan_directory}"
else
  printf 'FAIL: Trimble scan directory missing: %s\n' "${trimble_scan_directory}"
  failures=$((failures + 1))
fi

if [[ "${AUTONOMOUS_NAVIGATION_ENABLED:-false}" == "true" &&
      "${AUTONOMOUS_NAVIGATION:-false}" != "true" ]]; then
  printf 'FAIL: AUTONOMOUS_NAVIGATION_ENABLED requires AUTONOMOUS_NAVIGATION=true\n'
  failures=$((failures + 1))
fi  

if [[ "${AUTONOMOUS_NAVIGATION:-false}" == "true" ]]; then
  if python3 -c 'import nav2_msgs' >/dev/null 2>&1; then
    printf 'OK:   Nav2 Python messages are available\n'
  else
    printf 'FAIL: nav2_msgs is unavailable; install ROS Nav2 dependencies\n'
    failures=$((failures + 1))
  fi

  check_file \
    'navigation priority configuration' \
    "${NAVIGATION_PRIORITY_CONFIG:-}"
fi

if [[ "${MODE}" == "full" ]]; then
  check_file 'YOLO model' "${MODEL_PATH:-}"
  check_file 'dataset configuration' "${DATASET_PATH:-}"
  check_file 'camera-LiDAR calibration' "${CALIBRATION_PATH:-}"

  if [[ -f "${CALIBRATION_PATH:-}" ]]; then
    if python3 - "${CALIBRATION_PATH}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    calibration = yaml.safe_load(stream) or {}
if calibration.get('calibrated') is not True:
    raise SystemExit(1)
PY
    then
      printf 'OK:   calibration is marked calibrated\n'
    else
      printf 'FAIL: calibration is not marked calibrated: true\n'
      failures=$((failures + 1))
    fi
  fi

  if [[ -f "${DATASET_PATH:-}" ]]; then
    if python3 - "${DATASET_PATH}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    dataset = yaml.safe_load(stream) or {}
if not dataset.get('names'):
    raise SystemExit(1)
PY
    then
      printf 'OK:   dataset contains class names\n'
    else
      printf 'FAIL: dataset names list is empty\n'
      failures=$((failures + 1))
    fi
  fi
fi

if ((failures > 0)); then
  printf '\nPreflight failed with %d problem(s).\n' "${failures}"
  exit 1
fi

printf '\nPreflight passed for %s mode.\n' "${MODE}"
