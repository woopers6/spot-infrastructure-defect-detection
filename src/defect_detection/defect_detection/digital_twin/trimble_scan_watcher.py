from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header


SUPPORTED_SUFFIXES = {'.las', '.laz'}


def newest_scan_file(scan_directory):
    candidates = [
        path for path in scan_directory.iterdir()
        if path.is_file() and path.suffix.lower() in SUPPORTED_SUFFIXES
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def file_is_stable(path, stable_age_sec, previous_size):
    stat = path.stat()
    age = time.time() - stat.st_mtime
    if age < stable_age_sec:
        return False, stat.st_size
    return previous_size in {None, stat.st_size}, stat.st_size


def read_las_xyz(path, max_points):
    try:
        import laspy
    except ImportError as error:
        raise RuntimeError(
            'Install laspy to read Trimble LAS/LAZ scans: '
            'python3 -m pip install laspy lazrs'
        ) from error

    las = laspy.read(path)
    point_count = len(las.x)
    if point_count == 0:
        return np.empty((0, 3), dtype=np.float32)

    stride = max(1, int(np.ceil(point_count / max_points)))
    x = np.asarray(las.x[::stride], dtype=np.float32)
    y = np.asarray(las.y[::stride], dtype=np.float32)
    z = np.asarray(las.z[::stride], dtype=np.float32)
    return np.column_stack((x, y, z))


class TrimbleScanWatcher(Node):

    def __init__(self):
        super().__init__('trimble_scan_watcher')

        self.declare_parameter('scan_directory', '/tmp/trimble_scans')
        self.declare_parameter('output_topic', '/trimble/x7/scan_points')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('poll_period_sec', 2.0)
        self.declare_parameter('stable_age_sec', 5.0)
        self.declare_parameter('max_points', 500000)
        self.declare_parameter('require_scan_request', False)
        self.declare_parameter('scan_required_topic', '/digital_twin/scan_required')

        self.scan_directory = Path(
            self.get_parameter(
                'scan_directory'
            ).get_parameter_value().string_value
        ).expanduser()
        output_topic = self.get_parameter(
            'output_topic'
        ).get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id'
        ).get_parameter_value().string_value
        poll_period = self.get_parameter(
            'poll_period_sec'
        ).get_parameter_value().double_value
        self.stable_age = self.get_parameter(
            'stable_age_sec'
        ).get_parameter_value().double_value
        self.max_points = self.get_parameter(
            'max_points'
        ).get_parameter_value().integer_value
        self.require_scan_request = self.get_parameter(
            'require_scan_request'
        ).get_parameter_value().bool_value
        scan_required_topic = self.get_parameter(
            'scan_required_topic'
        ).get_parameter_value().string_value

        if poll_period <= 0.0:
            raise ValueError('poll_period_sec must be greater than zero')
        if self.stable_age < 0.0:
            raise ValueError('stable_age_sec must be non-negative')
        if self.max_points <= 0:
            raise ValueError('max_points must be greater than zero')

        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            qos_profile_sensor_data,
        )
        self.last_published_path = None
        self.last_candidate_path = None
        self.last_candidate_size = None
        self.scan_requested = not self.require_scan_request
        self.request_subscription = self.create_subscription(
            Bool,
            scan_required_topic,
            self.scan_request_callback,
            10,
        )
        self.timer = self.create_timer(poll_period, self.poll)

        self.get_logger().info(
            f'Watching {self.scan_directory} for Trimble LAS/LAZ scans; '
            f'publishing {output_topic}'
        )

    def scan_request_callback(self, message):
        if message.data:
            self.scan_requested = True

    def poll(self):
        if self.require_scan_request and not self.scan_requested:
            return

        if not self.scan_directory.is_dir():
            self.get_logger().warning(
                f'Scan directory does not exist: {self.scan_directory}'
            )
            return

        candidate = newest_scan_file(self.scan_directory)
        if candidate is None or candidate == self.last_published_path:
            return

        if candidate != self.last_candidate_path:
            self.last_candidate_path = candidate
            self.last_candidate_size = None

        stable, size = file_is_stable(
            candidate,
            self.stable_age,
            self.last_candidate_size,
        )
        self.last_candidate_size = size
        if not stable:
            return

        try:
            points = read_las_xyz(candidate, self.max_points)
        except Exception as error:
            self.get_logger().error(f'Could not read {candidate}: {error}')
            return

        if points.size == 0:
            self.get_logger().warning(f'Scan has no points: {candidate}')
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)
        self.last_published_path = candidate
        if self.require_scan_request:
            self.scan_requested = False
        self.get_logger().info(
            f'Published {len(points)} points from {candidate.name}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrimbleScanWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
