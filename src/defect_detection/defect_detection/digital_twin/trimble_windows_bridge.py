import json
import time
from urllib import error as urlerror
from urllib import request as urlrequest

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


def post_json(url, payload, timeout_sec):
    data = json.dumps(payload).encode('utf-8')
    request = urlrequest.Request(
        url,
        data=data,
        headers={'content-type': 'application/json'},
        method='POST',
    )
    with urlrequest.urlopen(request, timeout=timeout_sec) as response:
        return response.status, response.read().decode('utf-8')


class TrimbleWindowsBridge(Node):

    def __init__(self):
        super().__init__('trimble_windows_bridge')

        self.declare_parameter('windows_url', 'http://127.0.0.1:8765')
        self.declare_parameter('scan_required_topic', '/digital_twin/scan_required')
        self.declare_parameter('scan_reason_topic', '/digital_twin/scan_reason')
        self.declare_parameter('waypoint_arrived_topic', '/digital_twin/waypoint_arrived')
        self.declare_parameter('frontier_goal_topic', '/digital_twin/frontier_goal')
        self.declare_parameter('request_reference_scan_on_start', True)
        self.declare_parameter('http_timeout_sec', 3.0)
        self.declare_parameter('request_cooldown_sec', 10.0)

        self.windows_url = self.get_parameter(
            'windows_url'
        ).get_parameter_value().string_value.rstrip('/')
        scan_required_topic = self.get_parameter(
            'scan_required_topic'
        ).get_parameter_value().string_value
        scan_reason_topic = self.get_parameter(
            'scan_reason_topic'
        ).get_parameter_value().string_value
        waypoint_arrived_topic = self.get_parameter(
            'waypoint_arrived_topic'
        ).get_parameter_value().string_value
        frontier_goal_topic = self.get_parameter(
            'frontier_goal_topic'
        ).get_parameter_value().string_value
        self.request_reference_scan_on_start = self.get_parameter(
            'request_reference_scan_on_start'
        ).get_parameter_value().bool_value
        self.http_timeout = self.get_parameter(
            'http_timeout_sec'
        ).get_parameter_value().double_value
        self.request_cooldown = self.get_parameter(
            'request_cooldown_sec'
        ).get_parameter_value().double_value

        self.last_reason = ''
        self.last_scan_request_time = None
        self.last_frontier_goal = None
        self.reference_requested = False
        self.suppress_next_scan_required = False

        self.scan_request_publisher = self.create_publisher(
            Bool,
            scan_required_topic,
            10,
        )

        self.scan_subscription = self.create_subscription(
            Bool,
            scan_required_topic,
            self.scan_required_callback,
            10,
        )
        self.reason_subscription = self.create_subscription(
            String,
            scan_reason_topic,
            self.scan_reason_callback,
            10,
        )
        self.arrival_subscription = self.create_subscription(
            String,
            waypoint_arrived_topic,
            self.waypoint_arrived_callback,
            10,
        )
        self.frontier_subscription = self.create_subscription(
            PoseStamped,
            frontier_goal_topic,
            self.frontier_goal_callback,
            10,
        )
        self.timer = self.create_timer(1.0, self.startup_tick)
        self.get_logger().info(f'Windows Trimble bridge targeting {self.windows_url}')
        self.post(
            '/jetson_ready',
            {
                'status': 'ros_started',
                'node': 'trimble_windows_bridge',
            },
        )
        self.post_status('Jetson Ready', 'ROS stack started')

    def startup_tick(self):
        if self.reference_requested or not self.request_reference_scan_on_start:
            return
        self.reference_requested = True
        self.open_scan_watcher_gate()
        self.post_status('Scanning', 'Requesting initial reference scan')
        self.send_scan_request(
            {
                'scan_type': 'reference',
                'reason': 'initial reference scan at station',
            },
            bypass_cooldown=True,
        )

    def scan_reason_callback(self, message):
        self.last_reason = message.data

    def scan_required_callback(self, message):
        if not message.data:
            return
        if self.suppress_next_scan_required:
            self.suppress_next_scan_required = False
            return
        self.post_status('Scanning', self.last_reason or 'Requesting defect rescan')
        self.send_scan_request(
            {
                'scan_type': 'defect_rescan',
                'reason': self.last_reason or 'high-confidence detection',
            },
        )

    def frontier_goal_callback(self, goal):
        self.last_frontier_goal = {
            'frame_id': goal.header.frame_id,
            'x': goal.pose.position.x,
            'y': goal.pose.position.y,
            'z': goal.pose.position.z,
        }
        self.post_status(
            'Navigating',
            (
                'Nav2 frontier goal: '
                f'x={goal.pose.position.x:.2f}, '
                f'y={goal.pose.position.y:.2f}'
            ),
        )

    def waypoint_arrived_callback(self, message):
        payload = {
            'reason': message.data,
            'frontier_goal': self.last_frontier_goal,
        }
        self.post_status('Waypoint Arrived', message.data)
        self.open_scan_watcher_gate()
        self.post('/waypoint_arrived', payload)

    def open_scan_watcher_gate(self):
        self.suppress_next_scan_required = True
        message = Bool()
        message.data = True
        self.scan_request_publisher.publish(message)

    def send_scan_request(self, payload, bypass_cooldown=False):
        now = time.monotonic()
        if (
            not bypass_cooldown
            and self.last_scan_request_time is not None
            and now - self.last_scan_request_time < self.request_cooldown
        ):
            self.get_logger().debug('Skipping duplicate Windows scan request')
            return
        self.last_scan_request_time = now
        self.post('/scan_request', payload)

    def post(self, path, payload):
        url = self.windows_url + path
        try:
            status, body = post_json(url, payload, self.http_timeout)
            self.get_logger().info(f'POST {path} -> {status}: {body}')
        except (OSError, urlerror.URLError, TimeoutError) as error:
            self.get_logger().warning(f'POST {path} failed: {error}')

    def post_status(self, state, detail=''):
        self.post('/process_status', {'state': state, 'detail': detail})


def main(args=None):
    rclpy.init(args=args)
    node = TrimbleWindowsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
