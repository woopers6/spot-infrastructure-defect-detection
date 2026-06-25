#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-transport}"
if [[ "${MODE}" != "transport" && "${MODE}" != "full" ]]; then
  echo "Usage: $0 [transport|full]"
  exit 2
fi

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WORKSPACE_ROOT
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

export ROS_LOG_DIR="${ROS_LOG_DIR:-${WORKSPACE_ROOT}/log/field}"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-${WORKSPACE_ROOT}/.runtime/ultralytics}"
mkdir -p "${ROS_LOG_DIR}" "${YOLO_CONFIG_DIR}"

cd "${WORKSPACE_ROOT}"
"${WORKSPACE_ROOT}/scripts/field_preflight.sh" "${MODE}"

detector=false
fusion=false
if [[ "${MODE}" == "full" ]]; then
  detector=true
  fusion=true
fi

exec ros2 launch pointcloud_bridge full_pipeline.launch.xml \
  lidar_input_topic:="${LIDAR_INPUT_TOPIC:-/lidar/raw}" \
  pointcloud_topic:="${POINTCLOUD_TOPIC:-/lidar/points}" \
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
  autonomous_navigation:="${AUTONOMOUS_NAVIGATION:-false}" \
  autonomous_navigation_enabled:="${AUTONOMOUS_NAVIGATION_ENABLED:-false}" \
  navigation_priority_config:="${NAVIGATION_PRIORITY_CONFIG:-}" \
  trimble_scan_watcher:="${TRIMBLE_SCAN_WATCHER:-true}" \
  trimble_scan_directory:="${TRIMBLE_SCAN_DIRECTORY:-/tmp/trimble_scans}" \
  trimble_scan_topic:="${TRIMBLE_SCAN_TOPIC:-/trimble/x7/scan_points}" \
  trimble_scan_frame:="${TRIMBLE_SCAN_FRAME:-map}" \
  trimble_windows_bridge:="${TRIMBLE_WINDOWS_BRIDGE:-false}" \
  trimble_windows_url:="${TRIMBLE_WINDOWS_URL:-http://127.0.0.1:8765}" \
  trimble_reference_scan_on_start:="${TRIMBLE_REFERENCE_SCAN_ON_START:-true}" \
  scan_decision:="${SCAN_DECISION:-true}" \
  scan_confidence_threshold:="${SCAN_CONFIDENCE_THRESHOLD:-0.65}" \
  scan_min_detections:="${SCAN_MIN_DETECTIONS:-1}" \
  scan_cooldown_sec:="${SCAN_COOLDOWN_SEC:-60.0}" \
  digital_twin_map:="${DIGITAL_TWIN_MAP:-true}" \
  frontier_planner:="${FRONTIER_PLANNER:-true}" \
  defect_map:="${DEFECT_MAP:-true}" \
  pointcloud_monitor:=false \
  image_monitor:=false
