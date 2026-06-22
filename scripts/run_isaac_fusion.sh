#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WORKSPACE_ROOT

source /opt/ros/jazzy/setup.bash
if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run: colcon build --symlink-install"
  exit 1
fi
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"

export ROS_LOG_DIR="${ROS_LOG_DIR:-${WORKSPACE_ROOT}/log/isaac_fusion}"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-${WORKSPACE_ROOT}/.runtime/ultralytics}"
mkdir -p "${ROS_LOG_DIR}" "${YOLO_CONFIG_DIR}"

exec ros2 launch spot_eap_bridge full_pipeline.launch.xml \
  use_spot_sdk:=false \
  publish_camera:=false \
  eap_lidar_topic:="${ISAAC_POINTCLOUD_TOPIC:-/eap/lidar/points}" \
  image_topic:="${ISAAC_IMAGE_TOPIC:-/ros2_image}" \
  detections_2d_topic:="${DETECTIONS_2D_TOPIC:-/detections_2d}" \
  pointcloud_topic:="${POINTCLOUD_TOPIC:-/spot/velodyne/points}" \
  detections_3d_topic:="${DETECTIONS_3D_TOPIC:-/detections_3d}" \
  timestamp_mode:="${TIMESTAMP_MODE:-source}" \
  max_cloud_age_sec:="${MAX_CLOUD_AGE_SEC:-2.0}" \
  max_future_offset_sec:="${MAX_FUTURE_OFFSET_SEC:-0.25}" \
  lidar_frame:="${LIDAR_FRAME:-camera_optical_frame}" \
  camera_frame:="${CAMERA_FRAME:-camera_optical_frame}" \
  dataset_path:="${DATASET_PATH:-${WORKSPACE_ROOT}/src/defect_detection/models/dataset.yaml}" \
  model_path:="${MODEL_PATH:-${WORKSPACE_ROOT}/src/defect_detection/models/yolov11m.engine}" \
  calibration_path:="${CALIBRATION_PATH:-${WORKSPACE_ROOT}/src/defect_detection/config/isaac_sim_calibration.yaml}" \
  detector:=true \
  fusion:=true \
  visualization:=true \
  rviz:="${ENABLE_RVIZ:-true}" \
  pointcloud_monitor:=false \
  image_monitor:=false
