# Point Cloud Bridge

This package gives the defect detection pipeline a stable ROS 2 point-cloud
interface. An upstream source publishes `sensor_msgs/msg/PointCloud2` on
`/lidar/raw`, and the bridge validates and republishes it on `/lidar/points`.

```text
Upstream LiDAR / simulator / scan source
  -> /lidar/raw
  -> pointcloud_bridge
  -> /lidar/points
  -> defect_detection fusion_node
```

Incoming clouds must include:

- `x`, `y`, and `z` fields
- a valid acquisition timestamp when using `timestamp_mode:=source`
- the coordinate frame in which the points are expressed

Run the bridge:

```bash
ros2 launch pointcloud_bridge pointcloud_bridge.launch.xml \
  input_topic:=/actual/lidar/topic \
  output_topic:=/lidar/points \
  lidar_frame:=actual_lidar_frame
```

`timestamp_mode:=receive` replaces the LiDAR timestamp when the Jetson receives
the message. It can help bring up an upstream driver with missing timestamps,
but it should not be used for final calibration or operation.

`lidar_frame` only sets the outgoing message frame ID. It does not rotate or
translate point coordinates. Leave it equal to the upstream frame unless the
upstream points are already expressed in the configured frame.
