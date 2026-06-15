#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-transport}"
if [[ "${MODE}" != "transport" && "${MODE}" != "full" ]]; then
  echo "Usage: $0 [transport|full]"
  exit 2
fi

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export FIELD_CONFIG="${FIELD_CONFIG:-${WORKSPACE_ROOT}/config/field.env}"

if [[ ! -f "${FIELD_CONFIG}" ]]; then
  echo "Missing ${FIELD_CONFIG}"
  echo "Copy config/field.env.example to config/field.env and edit it."
  exit 1
fi

set -a
# shellcheck disable=SC1090
source "${FIELD_CONFIG}"
set +a

source /opt/ros/jazzy/setup.bash
if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run: colcon build --symlink-install"
  exit 1
fi
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"

cd "${WORKSPACE_ROOT}"
"${WORKSPACE_ROOT}/scripts/field_preflight.sh" "${MODE}"

detector=false
fusion=false
if [[ "${MODE}" == "full" ]]; then
  detector=true
  fusion=true
fi

exec ros2 launch spot_eap_bridge full_pipeline.launch.xml \
  spot_ip:="${SPOT_IP}" \
  camera_index:="${CAMERA_INDEX:-0}" \
  camera_frame:="${CAMERA_FRAME:-camera_optical_frame}" \
  lidar_frame:="${LIDAR_FRAME:-lidar}" \
  dataset_path:="${DATASET_PATH:-}" \
  model_path:="${MODEL_PATH:-}" \
  calibration_path:="${CALIBRATION_PATH:-}" \
  detector:="${detector}" \
  fusion:="${fusion}" \
  visualization:=true \
  rviz:="${ENABLE_RVIZ:-true}" \
  pointcloud_monitor:=false \
  image_monitor:=false
