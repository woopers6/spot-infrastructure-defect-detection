# Trimble Perspective Bridge App

This is a Windows companion app for the Jetson ROS pipeline.

It provides:

- A small interactive Tkinter UI.
- One-button `Start` to SSH into the Jetson, build, and launch ROS.
- One-button `Stop + Download Twin` to stop ROS and copy digital-twin outputs
  back to the Windows computer.
- HTTP endpoints the Jetson can call:
  - `POST /scan_request`
  - `POST /waypoint_arrived`
  - `POST /jetson_ready`
  - `GET /health`
- Optional launch of Trimble Perspective.
- Optional export command hook if Trimble provides a CLI/API later.
- Automatic watch/copy of completed `.las`, `.laz`, or `.e57` files from a
  Perspective export folder to a Jetson-accessible scan folder.
- Wi-Fi-friendly LAS/LAZ reduction before transfer so full raw scans can stay
  on the Windows machine.

## Run On Windows

Install Python 3.12, then run:

```powershell
python tools\trimble_perspective_bridge\windows_app.py
```

Configure:

- `Jetson host/IP`, `Jetson SSH user`, `Jetson workspace`.
- `Windows IP for Jetson`, the IP address the Jetson can use to reach this app.
- `Perspective EXE`: path to Trimble Perspective.
- `Optional export command`: leave blank unless you have an automation command.
- `Perspective export folder`: where Perspective writes completed scans.
- `Reduced scan folder`: where the app writes Jetson-sized LAS/LAZ copies.
- `Jetson max points`: point cap for the transferred scan. Use `500000` to
  start on Wi-Fi.
- `Jetson scan folder`: usually a network share such as
  `\\JETSON\trimble_scans` or a synced folder.
- `Local twin folder`: where the app downloads digital-twin artifacts on stop.

Press `Start` once. The app runs the default remote command:

```text
cd ~/ros2_ws
colcon build --symlink-install --packages-select defect_detection pointcloud_bridge
TRIMBLE_WINDOWS_BRIDGE=true TRIMBLE_WINDOWS_URL=http://WINDOWS_IP:8765 ./scripts/run_field.sh full
```

Press `Stop + Download Twin` to stop ROS and copy compact configured artifacts
such as `/tmp/digital_twin_defects.yaml` back to Windows. Full raw scans should
stay on the Windows/Perspective machine unless you deliberately add them to
`Remote twin paths`.

If Trimble Perspective has no supported automation API, the app still works as
the coordination layer: the Jetson requests a scan, the app shows/logs it,
Perspective exports the scan, and the app automatically transfers the finished
file to the Jetson.

## Jetson URL

Launch the Jetson bridge node with the Windows laptop address:

```bash
ros2 launch pointcloud_bridge full_pipeline.launch.xml \
  trimble_windows_bridge:=true \
  trimble_windows_url:=http://WINDOWS_IP:8765
```
