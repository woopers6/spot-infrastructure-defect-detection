#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-transport}"
CONFIG_FILE="${FIELD_CONFIG:-config/field.env}"

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

check_value SPOT_IP
check_value BOSDYN_CLIENT_USERNAME
check_value BOSDYN_CLIENT_PASSWORD

camera_index="${CAMERA_INDEX:-0}"
if [[ -e "/dev/video${camera_index}" ]]; then
  printf 'OK:   webcam device: /dev/video%s\n' "${camera_index}"
else
  printf 'FAIL: webcam device missing: /dev/video%s\n' "${camera_index}"
  failures=$((failures + 1))
fi

if python3 -c 'import bosdyn.client, rclpy' >/dev/null 2>&1; then
  printf 'OK:   Spot SDK and ROS Python imports\n'
else
  printf 'FAIL: Spot SDK or ROS Python import is unavailable\n'
  failures=$((failures + 1))
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
fi

if ((failures > 0)); then
  printf '\nPreflight failed with %d problem(s).\n' "${failures}"
  exit 1
fi

printf '\nPreflight passed for %s mode.\n' "${MODE}"
