# Infrastructure Defect Detection

ROS 2 workspace for USB-camera defect detection, generic point-cloud fusion,
Trimble X7 scan ingestion, digital-twin markers, infrastructure inspection
planning, and optional robot navigation backends.

## Pipeline

```text
USB camera -> /ros2_image -> YOLO -> /detections_2d
                                      |
                                      v
                           scan_decision_node
                                      |
                                      v
                         /digital_twin/scan_required

Trimble X7 LAS/LAZ folder -> /trimble/x7/scan_points
                                      |
                                      v
                /digital_twin/map + /digital_twin/defect_markers
                                      |
                                      v
              frame anchor + infrastructure inspection goals
                                      |
                                      v
                   dry-run, Nav2, or external Spot command bridge
```

Generic live or simulated point clouds can still be bridged:

```text
/lidar/raw -> pointcloud_bridge -> /lidar/points -> /detections_3d
```

For the field robot, navigation perception is intended to use a Luxonis
OAK-D Pro W class camera: IR illumination, wide FOV stereo, and OV9782 global
shutter stereo sensors. In that mode:

```text
OAK RGB image -> YOLO -> /detections_2d
OAK depth + camera_info + /detections_2d -> /detections_3d
OAK visual odometry/VSLAM -> /oak/odom -> oak_odom -> body TF
/detections_3d -> digital twin defect markers -> inspection/rescan goals
```

The OAK path is enabled with:

```text
OAK_DEPTH_NAVIGATION=true
OAK_LOCALIZATION=true
IMAGE_TOPIC=/oak/rgb/image_raw
OAK_DEPTH_TOPIC=/oak/rgb/depth
OAK_CAMERA_INFO_TOPIC=/oak/rgb/camera_info
OAK_ODOM_TOPIC=/oak/odom
ROBOT_WORLD_FRAME=oak_odom
NAVIGATION_BASE_FRAME=body
```

The depth image must be aligned to the RGB image used by YOLO. Topic names
depend on the `depthai_ros` launch file, so confirm them with `ros2 topic list`
on the Jetson and update `config/field.env` if needed.

Mission localization intentionally uses OAK odometry/VSLAM rather than Spot
odom. The OAK localization bridge republishes `/oak/odom` as TF, normally
`oak_odom -> body`. The first Trimble reference scan anchors the digital twin
`map` frame to `oak_odom`, so the planner computes goals from OAK-estimated
motion. Spot still uses its internal low-level balance/motor control to walk,
but the high-level inspection localization source is OAK.

For best results, configure the OAK odometry/VSLAM output so its child frame is
the robot body frame, or provide a calibrated static transform from the OAK
camera frame to `body`.

## Requirements

- ROS 2 Jazzy
- Python 3.12
- OpenCV and `cv_bridge`
- Ultralytics for YOLO
- DepthAI ROS publishing OAK RGB/depth/camera_info topics
- `laspy` and `lazrs` for LAS/LAZ scan ingestion

Install field Python dependencies:

```bash
python3 -m pip install --user --break-system-packages -r requirements-field.txt
```

Build:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Field Setup

Create a machine-specific config:

```bash
cp config/field.env.example config/field.env
```

Edit camera, model, calibration, Nav2, and Trimble scan-folder settings. The X7
workflow assumes scans are written as completed `.las` or `.laz` files into the
configured folder.

Run transport/digital-twin bringup:

```bash
./scripts/run_field.sh transport
```

Run full detection/fusion once the YOLO engine and calibration are ready:

```bash
./scripts/run_field.sh full
```

## Efficient X7 Scan Gate

The scan decision node prevents wasting X7 scans when detections are absent,
stale, or low confidence. It publishes:

- `/digital_twin/scan_required` (`std_msgs/Bool`)
- `/digital_twin/scan_reason` (`std_msgs/String`)

Defaults:

```text
scan_confidence_threshold:=0.65
scan_min_detections:=1
scan_cooldown_sec:=60.0
```

The Trimble scan watcher defaults to `trimble_require_scan_request:=true`, so
it only ingests the next completed scan after a high-confidence request.

For the first station/reference scan, enable the Perspective bridge. It requests
a reference scan on startup even when there are no detections:

```bash
ros2 launch pointcloud_bridge full_pipeline.launch.xml \
  trimble_windows_bridge:=true \
  trimble_windows_url:=http://PERSPECTIVE_HOST_IP:8765 \
  trimble_reference_scan_on_start:=true
```

## Digital Twin And Robot Motion

The X7 scan watcher publishes `/trimble/x7/scan_points`. The occupancy builder
turns that into `/digital_twin/map`.

The frame anchor node records the robot pose when the reference scan arrives and
publishes the transform from the digital-twin `map` frame to the robot world
frame. This is the coordinate glue that lets defect markers and scan stations be
converted into robot-relative goals. It persists:

- `/tmp/digital_twin_anchor.yaml`

The defect map node persists AI markers to YAML and republishes them as:

- `/digital_twin/defect_markers`
- `/digital_twin/rescan_goals`

The infrastructure planner prefers defect rescan goals first, then falls back to
map-frontier exploration goals. It publishes:

- `/infrastructure/inspection_goal`
- `/infrastructure/planner_status`

The robot goal bridge subscribes to `/infrastructure/inspection_goal`. It is off
by default so the stack can propose goals without moving hardware. Enable one of
these backends when the field command path is ready:

```text
ROBOT_GOAL_BRIDGE=true
ROBOT_GOAL_BACKEND=dry_run  # no motion, publishes arrival for software tests
ROBOT_GOAL_BACKEND=nav2     # send NavigateToPose goals
ROBOT_GOAL_BACKEND=http     # POST goals to an external Spot SDK command service
ROBOT_GOAL_BACKEND=spot_sdk # command Spot directly with the Boston Dynamics SDK
```

For Spot, the intended first hardware path is to use Spot-native localization and
mobility for walking, while this ROS stack handles inspection goals, scan
coordination, AI markers, and digital-twin updates.

Direct Spot SDK control requires the Jetson to reach Spot on the mission LAN and
the Boston Dynamics Python SDK to be installed from `requirements-field.txt`.
Configure:

```text
ROBOT_GOAL_BRIDGE=true
ROBOT_GOAL_BACKEND=spot_sdk
SPOT_IP=192.168.80.3
SPOT_USERNAME=...
SPOT_PASSWORD=...
SPOT_COMMAND_FRAME=odom
SPOT_AUTO_POWER_ON=false
SPOT_STAND_BEFORE_MOVE=true
```

Leave `SPOT_AUTO_POWER_ON=false` unless the tablet/operator workflow explicitly
allows the payload to power motors. The backend acquires a lease, optionally
commands stand, sends an SE2 trajectory goal, and publishes waypoint arrival
when the SDK trajectory command completes.

## Perspective Control Host

The Trimble X7 side is coordinated by a Perspective control host. This can be a
Windows laptop, or a Samsung/Android tablet if Trimble Perspective is running
there. The Jetson stays responsible for ROS 2, OAK-D perception, AI detections,
robot goals, and digital-twin processing.

```text
Samsung tablet or Windows laptop
  -> Trimble Perspective controls the X7
  -> Perspective bridge app handles Start/Stop/status/scan-file transfer
  -> Jetson runs ROS 2, OAK-D, YOLO, planner, and digital twin
```

The checked-in companion app currently targets Windows/Tkinter:

```powershell
python tools\trimble_perspective_bridge\windows_app.py
```

Press `Start` in the app to SSH into the Jetson, build the ROS workspace, launch
the autonomy/digital-twin stack, and wait for the Jetson to report ready. Press
`Stop + Download Twin` to stop the Jetson ROS launch and copy configured
digital-twin outputs back to the control host.

The app also listens for Jetson scan requests, optionally launches Perspective,
watches the Perspective export folder, and prepares a Jetson-sized `.las` or
`.laz` copy before transfer. Full-resolution raw scans stay on the Perspective
host by default; this keeps Wi-Fi transfer practical.

### Samsung Tablet Control Panel

If Trimble Perspective runs on the Samsung tablet, the tablet can be the field
control panel instead of the Windows laptop. The recommended tablet design is a
small Python web app running under Termux:

```text
Tablet browser -> local Python control app -> Jetson SSH/HTTP
                                      |
                                      v
                         Trimble Perspective export folder
```

On the tablet, install Termux, then:

```bash
pkg update
pkg install python openssh git
python -m pip install flask requests paramiko watchdog
```

The tablet app should expose the same HTTP endpoints used by the Jetson bridge:

```text
POST /scan_request
POST /waypoint_arrived
POST /jetson_ready
GET /health
GET /status
```

The tablet app should also watch the Perspective export folder for completed
`.las`, `.laz`, or `.e57` files, reduce/copy the scan if needed, transfer it to
the Jetson scan folder, and keep the tablet browser updated with mission state.
Android may limit direct control of another app's buttons, so the reliable path
is file/export-folder automation. If Perspective does not expose a supported
API or predictable export folder, use the tablet app as the mission control
panel and let the operator confirm/export scans in Perspective.

Recommended Wi-Fi starting point:

```text
Jetson max points: 500000
Remote twin paths: /tmp/digital_twin_defects.yaml;/tmp/digital_twin_anchor.yaml
```

## Tests

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select defect_detection pointcloud_bridge
colcon test-result --verbose
```
