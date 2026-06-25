# Isaac Full 3D Fusion

1. Open Isaac Sim with the ROS 2 bridge extension enabled.
2. Open `scripts/isaac/full_3d_fusion_stage.py` in Isaac's Script Editor and run it once.
3. Start the ROS side from WSL:

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ./scripts/run_isaac_fusion.sh
   ```

4. Press Play in Isaac.

On Play, Isaac generates a fresh culvert USD from
`C:\Users\AVARADAR\Downloads\isaac-culvert-sim\scripts\generate_culvert_usd.py`,
references it under `/World/GeneratedCulvert`, creates a camera render product,
and publishes:

- `/ros2_image`
- `/lidar/raw`

The ROS launch bridges `/lidar/raw` to `/lidar/points`, runs
YOLO, fuses `/detections_2d` with the point cloud, and publishes
`/detections_3d` plus RViz markers.

On Stop, Isaac removes `/World/GeneratedCulvert`, removes the ROS action graph,
and deletes the generated temporary USD.
