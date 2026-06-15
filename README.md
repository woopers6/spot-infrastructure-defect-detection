# Spot Infrastructure Defect Detection

ROS 2 packages for capturing USB-camera images, running YOLO defect detection,
receiving Spot EAP Velodyne point clouds, and associating 2D detections with
LiDAR points.

## Packages

- `defect_detection`: camera publisher, YOLO detector, test subscribers, and
  2D-to-3D fusion.
- `spot_eap_bridge`: Spot SDK point-cloud client and ROS topic normalization.

## Pipeline

```text
USB webcam -> /ros2_image -> YOLO -> /detections_2d
                                             |
Spot EAP LiDAR -> Spot SDK -> /spot/velodyne/points
                                             |
                                             v
                                      /detections_3d
```

## Requirements

- ROS 2 Jazzy
- Python 3.12
- OpenCV and `cv_bridge`
- Ultralytics
- Boston Dynamics Spot SDK

Install the Python dependencies used outside the ROS package index:

```bash
python3 -m pip install --user --break-system-packages \
  bosdyn-api bosdyn-client bosdyn-core ultralytics
```

Build the workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Hardware Test

Connect this computer to Spot's network and set credentials:

```bash
export SPOT_IP=YOUR_SPOT_IP
export BOSDYN_CLIENT_USERNAME=YOUR_USERNAME
export BOSDYN_CLIENT_PASSWORD=YOUR_PASSWORD
```

Run the webcam and EAP LiDAR transport without detection or fusion:

```bash
ros2 launch spot_eap_bridge full_pipeline.launch.xml \
  image_monitor:=true \
  pointcloud_monitor:=true \
  detector:=false \
  fusion:=false
```

Verify publication rates in another sourced terminal:

```bash
ros2 topic hz /ros2_image
ros2 topic hz /spot/velodyne/points
```

The Spot client defaults to the `velodyne-point-cloud` directory service and
automatically selects a source containing `velodyne`. See
[`src/spot_eap_bridge/README.md`](src/spot_eap_bridge/README.md) for overrides
and timestamp details.

## Detection

Place these files in `src/defect_detection/models/` before enabling YOLO:

- `dataset.yaml`
- `yolov11m.engine`

The TensorRT engine must be compatible with the target GPU and TensorRT
version. Then launch with:

```bash
ros2 launch spot_eap_bridge full_pipeline.launch.xml detector:=true
```

## RViz Visualization

The visualization node republishes:

- The complete cloud on `/rviz/pointcloud`
- 3D bounding boxes and labels on `/detection_markers`

Start the full pipeline and the configured RViz window with:

```bash
ros2 launch spot_eap_bridge full_pipeline.launch.xml \
  detector:=true \
  fusion:=true \
  visualization:=true \
  rviz:=true
```

The RViz configuration includes a `PointCloud2` display and a `MarkerArray`
display. Its fixed frame defaults to `lidar`, matching the default bridge
configuration. Change RViz's **Global Options > Fixed Frame** if the cloud uses
a different `header.frame_id`.

## Fusion Status

The fusion algorithm and synthetic tests are implemented, but live fusion
requires real calibration values. The current values in `fusion_node.py` are
placeholders:

- Webcam intrinsic matrix
- LiDAR-to-camera extrinsic transform
- Camera image dimensions

Keep `fusion:=false` until those values are replaced with calibration from the
rigidly mounted webcam and EAP LiDAR.

## Tests

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select defect_detection spot_eap_bridge
colcon test-result --verbose
```
