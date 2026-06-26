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
  frame_anchor:="${FRAME_ANCHOR:-true}" \
  robot_world_frame:="${ROBOT_WORLD_FRAME:-odom}" \
  anchor_store_path:="${ANCHOR_STORE_PATH:-/tmp/digital_twin_anchor.yaml}" \
  auto_anchor_on_first_scan:="${AUTO_ANCHOR_ON_FIRST_SCAN:-true}" \
  infrastructure_planner:="${INFRASTRUCTURE_PLANNER:-true}" \
  infrastructure_goal_cooldown_sec:="${INFRASTRUCTURE_GOAL_COOLDOWN_SEC:-20.0}" \
  robot_goal_bridge:="${ROBOT_GOAL_BRIDGE:-false}" \
  robot_goal_backend:="${ROBOT_GOAL_BACKEND:-dry_run}" \
  spot_command_url:="${SPOT_COMMAND_URL:-}" \
  spot_ip:="${SPOT_IP:-}" \
  spot_username:="${SPOT_USERNAME:-}" \
  spot_password:="${SPOT_PASSWORD:-}" \
  spot_command_frame:="${SPOT_COMMAND_FRAME:-odom}" \
  spot_goal_duration_sec:="${SPOT_GOAL_DURATION_SEC:-30.0}" \
  spot_arrival_timeout_sec:="${SPOT_ARRIVAL_TIMEOUT_SEC:-45.0}" \
  spot_auto_power_on:="${SPOT_AUTO_POWER_ON:-false}" \
  spot_stand_before_move:="${SPOT_STAND_BEFORE_MOVE:-true}" \
  defect_map:="${DEFECT_MAP:-true}" \
  pointcloud_monitor:=false \
  image_monitor:=false
