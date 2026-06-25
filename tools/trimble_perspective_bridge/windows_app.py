#!/usr/bin/env python3
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import shutil
import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


SCAN_SUFFIXES = {'.las', '.laz', '.e57'}


def load_json(path, defaults):
    if not path.is_file():
        return defaults.copy()
    with path.open('r', encoding='utf-8') as stream:
        data = json.load(stream)
    config = defaults.copy()
    config.update(data)
    return config


def save_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2, sort_keys=True)


def newest_scan(scan_dir):
    if not scan_dir.is_dir():
        return None
    candidates = [
        path for path in scan_dir.iterdir()
        if path.is_file() and path.suffix.lower() in SCAN_SUFFIXES
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def file_is_stable(path, stable_age_sec):
    first = path.stat()
    if time.time() - first.st_mtime < stable_age_sec:
        return False
    time.sleep(0.5)
    second = path.stat()
    return first.st_size == second.st_size


def windows_http_url(config):
    host = config.get('windows_advertise_host') or config.get('listen_host')
    if host in {'', '0.0.0.0'}:
        host = 'WINDOWS_IP'
    return f'http://{host}:{int(config["listen_port"])}'


class BridgeServer(ThreadingHTTPServer):

    def __init__(self, address, app):
        super().__init__(address, BridgeHandler)
        self.app = app


class BridgeHandler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        self.server.app.log(f'HTTP: {fmt % args}')

    def do_GET(self):
        if self.path == '/health':
            self.send_json(200, self.server.app.status_payload())
            return
        self.send_json(404, {'error': 'not found'})

    def do_POST(self):
        body = self.read_json()
        if self.path == '/scan_request':
            self.server.app.enqueue_scan_request(body)
            self.send_json(202, {'accepted': True})
            return
        if self.path == '/waypoint_arrived':
            self.server.app.enqueue_waypoint_arrival(body)
            self.send_json(202, {'accepted': True})
            return
        if self.path == '/jetson_ready':
            self.server.app.enqueue_jetson_ready(body)
            self.send_json(202, {'accepted': True})
            return
        if self.path == '/process_status':
            self.server.app.enqueue_process_status(body)
            self.send_json(202, {'accepted': True})
            return
        self.send_json(404, {'error': 'not found'})

    def read_json(self):
        length = int(self.headers.get('content-length', '0'))
        if length == 0:
            return {}
        raw = self.rfile.read(length).decode('utf-8')
        return json.loads(raw)

    def send_json(self, status, payload):
        encoded = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('content-type', 'application/json')
        self.send_header('content-length', str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


class TrimblePerspectiveBridgeApp:

    DEFAULTS = {
        'listen_host': '0.0.0.0',
        'windows_advertise_host': '',
        'listen_port': 8765,
        'jetson_host': 'jetson.local',
        'jetson_user': 'avaradar',
        'jetson_workspace': '~/ros2_ws',
        'jetson_mode': 'full',
        'ssh_command': 'ssh',
        'scp_command': 'scp',
        'remote_start_command': '',
        'remote_stop_command': '',
        'remote_digital_twin_paths': (
            '/tmp/digital_twin_defects.yaml'
        ),
        'local_digital_twin_dir': str(Path.home() / 'Documents' / 'DigitalTwin'),
        'perspective_exe': '',
        'export_command': '',
        'export_dir': str(Path.home() / 'Documents' / 'TrimbleExports'),
        'reduced_scan_dir': str(
            Path.home() / 'Documents' / 'TrimbleJetsonScans'
        ),
        'jetson_max_points': 500000,
        'jetson_scan_dir': r'\\JETSON\trimble_scans',
        'stable_age_sec': 5.0,
        'auto_transfer': True,
        'transfer_reduced_scan': True,
        'auto_scan_on_waypoint': True,
    }

    def __init__(self, config_path):
        self.config_path = Path(config_path).expanduser()
        self.config = load_json(self.config_path, self.DEFAULTS)
        self.events = queue.Queue()
        self.stop_event = threading.Event()
        self.last_transferred = None
        self.pending_scan = False
        self.last_scan_request = None
        self.jetson_process = None
        self.jetson_ready = False
        self.process_state = 'Idle'
        self.process_detail = 'Waiting for Start'

        self.root = tk.Tk()
        self.root.title('Trimble Perspective Bridge')
        self.root.geometry('980x760')

        self.variables = {}
        self.log_text = None
        self.status_var = tk.StringVar(value='State: Starting')
        self.detail_var = tk.StringVar(value='Initializing Windows bridge...')
        self.server = None
        self.server_thread = None
        self.watch_thread = None

        self.build_ui()
        self.load_variables()
        self.start_background_services()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(250, self.process_events)

    def build_ui(self):
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill='both', expand=True)

        status = ttk.Label(
            outer,
            textvariable=self.status_var,
            font=('Segoe UI', 11, 'bold'),
        )
        status.pack(anchor='w', pady=(0, 10))
        detail = ttk.Label(
            outer,
            textvariable=self.detail_var,
            font=('Segoe UI', 10),
        )
        detail.pack(anchor='w', pady=(0, 10))

        form = ttk.Frame(outer)
        form.pack(fill='x')

        self.add_entry(form, 'listen_host', 'Listen host')
        self.add_entry(form, 'windows_advertise_host', 'Windows IP for Jetson')
        self.add_entry(form, 'listen_port', 'Listen port')
        self.add_entry(form, 'jetson_host', 'Jetson host/IP')
        self.add_entry(form, 'jetson_user', 'Jetson SSH user')
        self.add_entry(form, 'jetson_workspace', 'Jetson workspace')
        self.add_entry(form, 'jetson_mode', 'Jetson mode')
        self.add_entry(form, 'ssh_command', 'SSH command')
        self.add_entry(form, 'scp_command', 'SCP command')
        self.add_entry(
            form,
            'remote_start_command',
            'Remote start command',
            width=80,
        )
        self.add_entry(
            form,
            'remote_stop_command',
            'Remote stop command',
            width=80,
        )
        self.add_path_entry(form, 'perspective_exe', 'Perspective EXE', file=True)
        self.add_entry(
            form,
            'export_command',
            'Optional export command',
            width=80,
        )
        self.add_path_entry(form, 'export_dir', 'Perspective export folder')
        self.add_path_entry(form, 'reduced_scan_dir', 'Reduced scan folder')
        self.add_entry(form, 'jetson_max_points', 'Jetson max points')
        self.add_path_entry(form, 'jetson_scan_dir', 'Jetson scan folder')
        self.add_entry(
            form,
            'remote_digital_twin_paths',
            'Remote twin paths',
            width=80,
        )
        self.add_path_entry(form, 'local_digital_twin_dir', 'Local twin folder')
        self.add_entry(form, 'stable_age_sec', 'Stable file age sec')
        self.add_check(form, 'auto_transfer', 'Automatically copy scans to Jetson')
        self.add_check(
            form,
            'transfer_reduced_scan',
            'Transfer reduced scan only',
        )
        self.add_check(
            form,
            'auto_scan_on_waypoint',
            'Request scan when Jetson reports waypoint arrival',
        )

        buttons = ttk.Frame(outer)
        buttons.pack(fill='x', pady=10)
        ttk.Button(buttons, text='Start', command=self.start_system).pack(
            side='left',
            padx=(0, 6),
        )
        ttk.Button(buttons, text='Stop + Download Twin', command=self.stop_system).pack(
            side='left',
            padx=6,
        )
        ttk.Button(buttons, text='Save Config', command=self.save_config).pack(
            side='left',
            padx=6,
        )
        ttk.Button(buttons, text='Launch Perspective', command=self.launch_perspective).pack(
            side='left',
            padx=6,
        )
        ttk.Button(buttons, text='Request Scan Now', command=self.request_scan_button).pack(
            side='left',
            padx=6,
        )
        ttk.Button(buttons, text='Transfer Latest Scan', command=self.transfer_latest_scan).pack(
            side='left',
            padx=6,
        )

        self.log_text = tk.Text(outer, height=18, wrap='word')
        self.log_text.pack(fill='both', expand=True)

    def add_entry(self, parent, key, label, width=50):
        row = ttk.Frame(parent)
        row.pack(fill='x', pady=3)
        ttk.Label(row, text=label, width=24).pack(side='left')
        var = tk.StringVar()
        entry = ttk.Entry(row, textvariable=var, width=width)
        entry.pack(side='left', fill='x', expand=True)
        self.variables[key] = var

    def add_path_entry(self, parent, key, label, file=False):
        row = ttk.Frame(parent)
        row.pack(fill='x', pady=3)
        ttk.Label(row, text=label, width=24).pack(side='left')
        var = tk.StringVar()
        entry = ttk.Entry(row, textvariable=var, width=50)
        entry.pack(side='left', fill='x', expand=True)
        command = (
            lambda: self.pick_file(var)
            if file
            else lambda: self.pick_directory(var)
        )
        ttk.Button(row, text='Browse', command=command).pack(side='left', padx=6)
        self.variables[key] = var

    def add_check(self, parent, key, label):
        var = tk.BooleanVar()
        ttk.Checkbutton(parent, text=label, variable=var).pack(anchor='w', pady=3)
        self.variables[key] = var

    def pick_file(self, var):
        value = filedialog.askopenfilename()
        if value:
            var.set(value)

    def pick_directory(self, var):
        value = filedialog.askdirectory()
        if value:
            var.set(value)

    def load_variables(self):
        for key, var in self.variables.items():
            var.set(self.config.get(key, self.DEFAULTS[key]))

    def read_variables(self):
        data = {}
        for key, var in self.variables.items():
            value = var.get()
            if key == 'listen_port':
                value = int(value)
            elif key == 'jetson_max_points':
                value = int(value)
            elif key == 'stable_age_sec':
                value = float(value)
            data[key] = value
        return data

    def save_config(self):
        try:
            self.config = self.read_variables()
            save_json(self.config_path, self.config)
            self.log(f'Saved config: {self.config_path}')
        except Exception as error:
            messagebox.showerror('Config Error', str(error))

    def start_background_services(self):
        self.save_config()
        host = self.config['listen_host']
        port = int(self.config['listen_port'])
        self.server = BridgeServer((host, port), self)
        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            daemon=True,
        )
        self.server_thread.start()
        self.watch_thread = threading.Thread(target=self.watch_exports, daemon=True)
        self.watch_thread.start()
        self.status_var.set(f'Listening on http://{host}:{port}')
        self.set_state('Idle', f'Listening on http://{host}:{port}')
        self.log(f'HTTP server listening on {host}:{port}')

    def watch_exports(self):
        while not self.stop_event.is_set():
            try:
                if self.config.get('auto_transfer', True):
                    scan = newest_scan(Path(self.config['export_dir']))
                    if scan and scan != self.last_transferred:
                        if file_is_stable(scan, float(self.config['stable_age_sec'])):
                            self.transfer_scan(scan)
                            self.pending_scan = False
                time.sleep(2.0)
            except Exception as error:
                self.events.put(('log', f'Watcher error: {error}'))
                time.sleep(5.0)

    def enqueue_scan_request(self, payload):
        self.events.put(('scan_request', payload))

    def enqueue_waypoint_arrival(self, payload):
        self.events.put(('waypoint_arrived', payload))

    def enqueue_jetson_ready(self, payload):
        self.events.put(('jetson_ready', payload))

    def enqueue_process_status(self, payload):
        self.events.put(('process_status', payload))

    def process_events(self):
        while True:
            try:
                event, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if event == 'log':
                self.log(payload)
            elif event == 'scan_request':
                self.handle_scan_request(payload)
            elif event == 'waypoint_arrived':
                self.handle_waypoint_arrival(payload)
            elif event == 'jetson_ready':
                self.handle_jetson_ready(payload)
            elif event == 'process_status':
                self.handle_process_status(payload)
        self.root.after(250, self.process_events)

    def handle_jetson_ready(self, payload):
        self.jetson_ready = True
        self.set_state('Jetson Ready', 'ROS is running; requesting reference scan')
        self.log(f'Jetson ready: {json.dumps(payload)}')

    def handle_process_status(self, payload):
        state = payload.get('state', 'Status')
        detail = payload.get('detail') or payload.get('reason') or ''
        self.set_state(state, detail)
        self.log(f'Process status: {json.dumps(payload)}')

    def handle_waypoint_arrival(self, payload):
        self.set_state('Waypoint Arrived', 'Robot reached a scan viewpoint')
        self.log(f'Jetson waypoint arrival: {json.dumps(payload)}')
        if self.config.get('auto_scan_on_waypoint', True):
            request = payload.copy()
            request.setdefault('scan_type', 'waypoint_rescan')
            self.handle_scan_request(request)

    def handle_scan_request(self, payload):
        self.pending_scan = True
        self.last_scan_request = payload
        reason = payload.get('reason') or payload.get('scan_type') or 'scan requested'
        self.set_state('Scanning', reason)
        self.log(f'Scan requested: {reason}')
        self.request_scan()

    def request_scan_button(self):
        self.handle_scan_request({'scan_type': 'manual', 'reason': 'button'})

    def request_scan(self):
        self.launch_perspective()
        command = self.config.get('export_command', '').strip()
        if not command:
            self.log(
                'No export command configured. Start/export the scan in '
                'Trimble Perspective; this app will transfer the completed file.'
            )
            return

        values = self.config.copy()
        values['timestamp'] = time.strftime('%Y%m%d_%H%M%S')
        try:
            expanded = command.format(**values)
            subprocess.Popen(expanded, shell=True)
            self.log(f'Ran export command: {expanded}')
        except Exception as error:
            self.log(f'Export command failed: {error}')

    def start_system(self):
        try:
            self.save_config()
            self.jetson_ready = False
            self.set_state('Starting Jetson', 'Building and launching ROS over SSH')
            thread = threading.Thread(target=self.run_jetson_start, daemon=True)
            thread.start()
        except Exception as error:
            messagebox.showerror('Start Error', str(error))

    def stop_system(self):
        try:
            self.set_state('Stopping', 'Stopping ROS and downloading digital twin')
            thread = threading.Thread(target=self.run_jetson_stop, daemon=True)
            thread.start()
        except Exception as error:
            messagebox.showerror('Stop Error', str(error))

    def run_jetson_start(self):
        command = self.remote_start_command()
        self.events.put(('log', f'Starting Jetson with: {command}'))
        try:
            self.jetson_process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            assert self.jetson_process.stdout is not None
            for line in self.jetson_process.stdout:
                self.events.put(('log', 'JETSON: ' + line.rstrip()))
                if self.stop_event.is_set():
                    break
            code = self.jetson_process.wait()
            self.events.put(('log', f'Jetson start command exited: {code}'))
        except Exception as error:
            self.events.put(('log', f'Jetson start failed: {error}'))

    def run_jetson_stop(self):
        if self.jetson_process and self.jetson_process.poll() is None:
            self.jetson_process.terminate()

        stop_command = self.remote_stop_command()
        self.events.put(('log', f'Stopping Jetson with: {stop_command}'))
        self.run_command(stop_command)
        self.download_digital_twin()
        self.events.put((
            'process_status',
            {'state': 'Stopped', 'detail': 'Digital twin download complete'},
        ))
        self.events.put(('log', 'Stop/download complete'))

    def remote_start_command(self):
        configured = self.config.get('remote_start_command', '').strip()
        if configured:
            return configured.format(**self.command_values())

        values = self.command_values()
        remote = (
            f'cd {values["jetson_workspace"]} && '
            'set +u; source /opt/ros/jazzy/setup.bash; set -u; '
            'colcon build --symlink-install '
            '--packages-select defect_detection pointcloud_bridge && '
            f'TRIMBLE_WINDOWS_BRIDGE=true '
            f'TRIMBLE_WINDOWS_URL={values["windows_url"]} '
            f'TRIMBLE_REFERENCE_SCAN_ON_START=true '
            f'./scripts/run_field.sh {values["jetson_mode"]}'
        )
        return self.ssh_command(remote)

    def remote_stop_command(self):
        configured = self.config.get('remote_stop_command', '').strip()
        if configured:
            return configured.format(**self.command_values())
        remote = (
            "pkill -f 'ros2 launch pointcloud_bridge full_pipeline.launch.xml' "
            "|| true; "
            "pkill -f 'trimble_windows_bridge' || true"
        )
        return self.ssh_command(remote)

    def command_values(self):
        values = self.config.copy()
        values['windows_url'] = windows_http_url(self.config)
        return values

    def ssh_target(self):
        return f'{self.config["jetson_user"]}@{self.config["jetson_host"]}'

    def ssh_command(self, remote_command):
        return (
            f'{self.config["ssh_command"]} {self.ssh_target()} '
            f'"{remote_command}"'
        )

    def run_command(self, command):
        try:
            completed = subprocess.run(
                command,
                shell=True,
                check=False,
                capture_output=True,
                text=True,
            )
            if completed.stdout:
                self.events.put(('log', completed.stdout.rstrip()))
            if completed.stderr:
                self.events.put(('log', completed.stderr.rstrip()))
            return completed.returncode
        except Exception as error:
            self.events.put(('log', f'Command failed: {error}'))
            return 1

    def download_digital_twin(self):
        self.events.put((
            'process_status',
            {'state': 'Downloading Twin', 'detail': 'Copying artifacts from Jetson'},
        ))
        local_dir = Path(self.config['local_digital_twin_dir'])
        local_dir.mkdir(parents=True, exist_ok=True)
        remote_paths = [
            item.strip()
            for item in self.config['remote_digital_twin_paths'].split(';')
            if item.strip()
        ]
        for remote_path in remote_paths:
            command = (
                f'{self.config["scp_command"]} -r '
                f'{self.ssh_target()}:{remote_path} "{local_dir}"'
            )
            self.events.put(('log', f'Downloading twin artifact: {command}'))
            self.run_command(command)

    def launch_perspective(self):
        exe = self.config.get('perspective_exe', '').strip()
        if not exe:
            self.log('Perspective EXE is not configured')
            return
        try:
            subprocess.Popen([exe], shell=False)
            self.log(f'Launched Perspective: {exe}')
        except Exception as error:
            self.log(f'Could not launch Perspective: {error}')

    def transfer_latest_scan(self):
        scan = newest_scan(Path(self.config['export_dir']))
        if scan is None:
            self.log('No scan file found to transfer')
            return
        self.transfer_scan(scan)

    def transfer_scan(self, scan):
        self.set_state('Preparing Upload', f'Preparing {scan.name} for Jetson')
        transfer_source = scan
        if self.config.get('transfer_reduced_scan', True):
            transfer_source = self.create_reduced_scan(scan)
            if transfer_source is None:
                return

        self.set_state('Uploading Scan', f'Copying {transfer_source.name} to Jetson')
        destination_dir = Path(self.config['jetson_scan_dir'])
        destination_dir.mkdir(parents=True, exist_ok=True)
        destination = destination_dir / transfer_source.name
        shutil.copy2(transfer_source, destination)
        self.last_transferred = scan
        self.pending_scan = False
        self.set_state('Scan Uploaded', f'{transfer_source.name} is ready on Jetson')
        self.log(f'Transferred {transfer_source.name} -> {destination}')

    def create_reduced_scan(self, scan):
        suffix = scan.suffix.lower()
        if suffix not in {'.las', '.laz'}:
            self.log(
                f'Skipping Jetson transfer for {scan.name}: '
                'only LAS/LAZ can be reduced for ROS ingestion'
            )
            return None

        try:
            import laspy
        except ImportError:
            self.log(
                'laspy is not installed on Windows; copying full LAS/LAZ. '
                'Install with: python -m pip install laspy lazrs'
            )
            return scan

        max_points = int(self.config.get('jetson_max_points', 500000))
        if max_points <= 0:
            return scan

        reduced_dir = Path(self.config['reduced_scan_dir'])
        reduced_dir.mkdir(parents=True, exist_ok=True)
        reduced_path = reduced_dir / f'{scan.stem}_jetson{scan.suffix}'

        try:
            las = laspy.read(scan)
            point_count = len(las.points)
            if point_count <= max_points:
                shutil.copy2(scan, reduced_path)
                self.log(
                    f'Prepared full-size Jetson scan {reduced_path.name} '
                    f'({point_count} points)'
                )
                return reduced_path

            stride = max(1, int(point_count / max_points))
            reduced = laspy.LasData(las.header)
            reduced.points = las.points[::stride].copy()
            reduced.write(reduced_path)
            self.log(
                f'Reduced {scan.name}: {point_count} -> '
                f'{len(reduced.points)} points for Jetson'
            )
            return reduced_path
        except Exception as error:
            self.log(f'Could not reduce {scan.name}: {error}')
            return None

    def status_payload(self):
        return {
            'ok': True,
            'pending_scan': self.pending_scan,
            'last_transferred': (
                str(self.last_transferred) if self.last_transferred else None
            ),
            'last_scan_request': self.last_scan_request,
            'process_state': self.process_state,
            'process_detail': self.process_detail,
        }

    def set_state(self, state, detail=''):
        self.process_state = state
        self.process_detail = detail
        self.status_var.set(f'State: {state}')
        self.detail_var.set(detail)

    def log(self, message):
        timestamp = time.strftime('%H:%M:%S')
        line = f'[{timestamp}] {message}\n'
        if self.log_text is None:
            print(line, end='')
            return
        self.log_text.insert('end', line)
        self.log_text.see('end')

    def close(self):
        self.stop_event.set()
        if self.server:
            self.server.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--config',
        default=str(Path.home() / '.trimble_perspective_bridge.json'),
    )
    args = parser.parse_args()
    app = TrimblePerspectiveBridgeApp(args.config)
    app.run()


if __name__ == '__main__':
    main()
