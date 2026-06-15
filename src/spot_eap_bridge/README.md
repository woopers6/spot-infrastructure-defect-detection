# Spot EAP Bridge

This package gives the defect detection pipeline a stable ROS 2 point-cloud
interface:

```text
EAP/Core I/O publisher or vendor driver
  -> /eap/lidar/points
  -> spot_eap_bridge
  -> /spot/velodyne/points
  -> defect_detection fusion_node
```

The upstream component must publish `sensor_msgs/msg/PointCloud2` with:

- `x`, `y`, and `z` fields
- a valid acquisition timestamp
- the coordinate frame in which the points are expressed

## Timing

Use `timestamp_mode:=source` for normal operation. The bridge preserves the
LiDAR acquisition timestamp and rejects zero, stale, or future timestamps.
The Jetson, Core I/O, and any separate sensor computer must share a synchronized
clock, preferably PTP and otherwise chrony/NTP.

`timestamp_mode:=receive` replaces the LiDAR timestamp when the Jetson receives
the message. It can help bring up an upstream driver with missing timestamps,
but network delay then becomes fusion error and it should not be used for final
calibration or operation.

The fusion launch defaults to a 100 ms matching tolerance:

```bash
ros2 launch spot_eap_bridge full_pipeline.launch.xml \
  sync_slop_sec:=0.10 \
  sync_queue_size:=30
```

Reduce `sync_slop_sec` after measuring real hardware timing. A useful initial
target is 30-50 ms if camera exposure, LiDAR acquisition, and clocks are stable.

Run the bridge:

```bash
ros2 launch spot_eap_bridge spot_eap_bridge.launch.xml \
  input_topic:=/actual/eap/topic \
  lidar_frame:=actual_lidar_frame
```

Run the bridge and defect pipeline together:

```bash
ros2 launch spot_eap_bridge full_pipeline.launch.xml \
  eap_lidar_topic:=/actual/eap/topic \
  lidar_frame:=actual_lidar_frame \
  detector:=true \
  fusion:=true
```

`lidar_frame` only sets the outgoing message frame ID. It does not rotate or
translate point coordinates. Leave it equal to the upstream frame unless the
upstream points are already expressed in the configured frame.

The optional static transform arguments describe the calibrated rigid mounting
between the LiDAR frame and camera optical frame. Do not enable them with zero
values unless the two frames are physically coincident.
