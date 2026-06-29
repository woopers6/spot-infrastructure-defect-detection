# Trimble Perspective Bridge App

This is the companion control app pattern for the Jetson ROS pipeline. The
checked-in implementation is a Windows/Tkinter app, but the same HTTP contract
can be implemented on a Samsung/Android tablet when Trimble Perspective runs on
the tablet.

It provides:

- A small interactive Tkinter UI.
- One-button `Start` to SSH into the Jetson, build, and launch ROS.
- One-button `Stop + Download Twin` to stop ROS and copy digital-twin outputs
  back to the control host.
- HTTP endpoints the Jetson can call:
  - `POST /scan_request`
  - `POST /waypoint_arrived`
  - `POST /jetson_ready`
  - `GET /health`
- Optional launch of Trimble Perspective where the host OS supports it.
- Optional export command hook if Trimble provides a CLI/API later.
- Automatic watch/copy of completed `.las`, `.laz`, or `.e57` files from a
  Perspective export folder to a Jetson-accessible scan folder.
- Wi-Fi-friendly LAS/LAZ reduction before transfer so full raw scans can stay
  on the Perspective/control host.

## Run On Windows

Install Python 3.12, then run:

```powershell
python tools\trimble_perspective_bridge\windows_app.py
```

Configure:

- `Jetson host/IP`, `Jetson SSH user`, `Jetson workspace`.
- `Windows IP for Jetson` / control host IP, the address the Jetson can use to
  reach this app.
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
TRIMBLE_WINDOWS_BRIDGE=true TRIMBLE_WINDOWS_URL=http://CONTROL_HOST_IP:8765 ./scripts/run_field.sh full
```

Press `Stop + Download Twin` to stop ROS and copy compact configured artifacts
such as `/tmp/digital_twin_defects.yaml` back to Windows. Full raw scans should
stay on the Windows/Perspective machine unless you deliberately add them to
`Remote twin paths`.

If Trimble Perspective has no supported automation API, the app still works as
the coordination layer: the Jetson requests a scan, the app shows/logs it,
Perspective exports the scan, and the app automatically transfers the finished
file to the Jetson.

## Samsung Tablet Control Host

If Trimble Perspective is running on a Samsung tablet, use the tablet as the
control host and keep the Jetson focused on ROS/OAK/navigation. The current
Windows Tkinter file will not run as-is on Android, so implement the same bridge
as a small Termux Python web app.

Install on the tablet:

```bash
pkg update
pkg install python openssh git
python -m pip install flask requests paramiko watchdog
```

The tablet bridge should provide the same endpoints:

```text
POST /scan_request
POST /waypoint_arrived
POST /jetson_ready
GET /health
```

It should:

- SSH or HTTP into the Jetson for Start/Stop.
- Serve a local browser UI on the tablet.
- Watch the Perspective export folder for `.las`, `.laz`, or `.e57` files.
- Transfer the finished scan to the Jetson scan folder.
- Download compact digital-twin artifacts back to the tablet on Stop.

Android may not allow direct automation of Perspective's buttons. Prefer a
supported Perspective export location or API if available; otherwise let the
operator confirm/export scans in Perspective while the tablet bridge handles
mission state and file transfer.

## Jetson URL

Launch the Jetson bridge node with the control host address:

```bash
ros2 launch pointcloud_bridge full_pipeline.launch.xml \
  trimble_windows_bridge:=true \
  trimble_windows_url:=http://CONTROL_HOST_IP:8765
```
