import json
import math
import os
import time
from urllib import error as urlerror
from urllib import request as urlrequest

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


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


class RobotGoalBridge(Node):

    def __init__(self):
        super().__init__('infrastructure_robot_goal_bridge')

        self.declare_parameter('goal_topic', '/infrastructure/inspection_goal')
        self.declare_parameter('waypoint_arrived_topic', '/digital_twin/waypoint_arrived')
        self.declare_parameter('status_topic', '/infrastructure/navigation_status')
        self.declare_parameter('backend', 'dry_run')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('spot_command_url', '')
        self.declare_parameter('spot_ip', '')
        self.declare_parameter('spot_username', '')
        self.declare_parameter('spot_password', '')
        self.declare_parameter('spot_command_frame', 'odom')
        self.declare_parameter('spot_goal_duration_sec', 30.0)
        self.declare_parameter('spot_arrival_timeout_sec', 45.0)
        self.declare_parameter('spot_auto_power_on', False)
        self.declare_parameter('spot_stand_before_move', True)
        self.declare_parameter('spot_auto_return_lease', True)
        self.declare_parameter('http_timeout_sec', 3.0)
        self.declare_parameter('dry_run_arrival_delay_sec', 3.0)

        goal_topic = self.get_parameter(
            'goal_topic'
        ).get_parameter_value().string_value
        waypoint_arrived_topic = self.get_parameter(
            'waypoint_arrived_topic'
        ).get_parameter_value().string_value
        status_topic = self.get_parameter(
            'status_topic'
        ).get_parameter_value().string_value
        navigate_action = self.get_parameter(
            'navigate_action'
        ).get_parameter_value().string_value
        self.backend = self.get_parameter(
            'backend'
        ).get_parameter_value().string_value
        self.spot_command_url = self.get_parameter(
            'spot_command_url'
        ).get_parameter_value().string_value.rstrip('/')
        self.spot_ip = self.get_parameter(
            'spot_ip'
        ).get_parameter_value().string_value
        self.spot_username = self.get_parameter(
            'spot_username'
        ).get_parameter_value().string_value
        self.spot_password = self.get_parameter(
            'spot_password'
        ).get_parameter_value().string_value
        self.spot_command_frame = self.get_parameter(
            'spot_command_frame'
        ).get_parameter_value().string_value
        self.spot_goal_duration = self.get_parameter(
            'spot_goal_duration_sec'
        ).get_parameter_value().double_value
        self.spot_arrival_timeout = self.get_parameter(
            'spot_arrival_timeout_sec'
        ).get_parameter_value().double_value
        self.spot_auto_power_on = self.get_parameter(
            'spot_auto_power_on'
        ).get_parameter_value().bool_value
        self.spot_stand_before_move = self.get_parameter(
            'spot_stand_before_move'
        ).get_parameter_value().bool_value
        self.spot_auto_return_lease = self.get_parameter(
            'spot_auto_return_lease'
        ).get_parameter_value().bool_value
        self.http_timeout = self.get_parameter(
            'http_timeout_sec'
        ).get_parameter_value().double_value
        self.dry_run_arrival_delay = self.get_parameter(
            'dry_run_arrival_delay_sec'
        ).get_parameter_value().double_value

        self.arrival_publisher = self.create_publisher(
            String,
            waypoint_arrived_topic,
            10,
        )
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigation_client = ActionClient(self, NavigateToPose, navigate_action)
        self.active_goal = None
        self.dry_run_timer = None
        self.spot = None
        self.spot_command_client = None
        self.spot_lease_keepalive = None

        self.get_logger().info(
            f'Robot goal bridge listening on {goal_topic}; backend={self.backend}'
        )

    def goal_callback(self, goal):
        if self.active_goal is not None:
            self.publish_status('busy; ignoring new inspection goal')
            return

        self.active_goal = goal
        self.publish_status(
            f'goal received: {goal.header.frame_id} '
            f'x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}'
        )
        if self.backend == 'nav2':
            self.send_nav2_goal(goal)
        elif self.backend == 'http':
            self.send_http_goal(goal)
        elif self.backend == 'spot_sdk':
            self.send_spot_sdk_goal(goal)
        else:
            self.get_logger().warning(
                'Dry-run navigation backend active; no robot motion command sent'
            )
            self.dry_run_timer = self.create_timer(
                self.dry_run_arrival_delay,
                self.dry_run_arrived,
            )

    def send_nav2_goal(self, goal):
        if not self.navigation_client.server_is_ready():
            self.finish_goal('nav2 unavailable', arrived=False)
            return
        request = NavigateToPose.Goal()
        request.pose = goal
        future = self.navigation_client.send_goal_async(request)
        future.add_done_callback(self.nav2_goal_response)

    def nav2_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_goal(f'nav2 request failed: {error}', arrived=False)
            return
        if not goal_handle.accepted:
            self.finish_goal('nav2 rejected goal', arrived=False)
            return
        goal_handle.get_result_async().add_done_callback(self.nav2_result)

    def nav2_result(self, future):
        try:
            wrapped = future.result()
            self.finish_goal(f'nav2 completed with status {wrapped.status}')
        except Exception as error:
            self.finish_goal(f'nav2 failed: {error}', arrived=False)

    def send_http_goal(self, goal):
        if not self.spot_command_url:
            self.finish_goal('spot_command_url is not configured', arrived=False)
            return
        payload = {
            'frame_id': goal.header.frame_id,
            'position': {
                'x': goal.pose.position.x,
                'y': goal.pose.position.y,
                'z': goal.pose.position.z,
            },
            'orientation': {
                'x': goal.pose.orientation.x,
                'y': goal.pose.orientation.y,
                'z': goal.pose.orientation.z,
                'w': goal.pose.orientation.w,
            },
        }
        try:
            status, body = post_json(
                self.spot_command_url + '/navigate_to_pose',
                payload,
                self.http_timeout,
            )
            self.finish_goal(f'spot http command accepted: {status} {body}')
        except (OSError, urlerror.URLError, TimeoutError) as error:
            self.finish_goal(f'spot http command failed: {error}', arrived=False)

    def send_spot_sdk_goal(self, goal):
        try:
            spot_goal = self.transform_goal_for_spot(goal)
            command_id = self.command_spot_to_goal(spot_goal)
            self.wait_for_spot_goal(command_id)
        except Exception as error:
            self.finish_goal(f'spot sdk command failed: {error}', arrived=False)

    def transform_goal_for_spot(self, goal):
        if goal.header.frame_id == self.spot_command_frame:
            return goal
        try:
            transform = self.tf_buffer.lookup_transform(
                self.spot_command_frame,
                goal.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            transformed_pose = do_transform_pose(goal.pose, transform)
        except TransformException as error:
            raise RuntimeError(
                f'cannot transform {goal.header.frame_id} goal to '
                f'{self.spot_command_frame}: {error}'
            ) from error

        spot_goal = PoseStamped()
        spot_goal.header.frame_id = self.spot_command_frame
        spot_goal.header.stamp = self.get_clock().now().to_msg()
        spot_goal.pose = transformed_pose
        return spot_goal

    def command_spot_to_goal(self, goal):
        self.ensure_spot_connected()
        yaw = yaw_from_quaternion(goal.pose.orientation)

        try:
            from bosdyn.client.robot_command import RobotCommandBuilder
        except ImportError as error:
            raise RuntimeError(
                'bosdyn-client is not installed. Install requirements-field.txt '
                'on the Jetson before using ROBOT_GOAL_BACKEND=spot_sdk.'
            ) from error

        command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=goal.pose.position.x,
            goal_y=goal.pose.position.y,
            goal_heading=yaw,
            frame_name=self.spot_command_frame,
        )
        end_time = time.time() + self.spot_goal_duration
        command_id = self.spot_command_client.robot_command(
            command,
            end_time_secs=end_time,
        )
        self.publish_status(
            'spot command sent: '
            f'{self.spot_command_frame} '
            f'x={goal.pose.position.x:.2f}, '
            f'y={goal.pose.position.y:.2f}, yaw={yaw:.2f}'
        )
        return command_id

    def ensure_spot_connected(self):
        if self.spot_command_client is not None:
            return
        if not self.spot_ip:
            raise RuntimeError('spot_ip is not configured')

        try:
            import bosdyn.client
            from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
            from bosdyn.client.robot_command import (
                RobotCommandClient,
                blocking_stand,
            )
        except ImportError as error:
            raise RuntimeError(
                'bosdyn-client is not installed. Install requirements-field.txt '
                'on the Jetson before using ROBOT_GOAL_BACKEND=spot_sdk.'
            ) from error

        username = self.spot_username or os.environ.get('BOSDYN_CLIENT_USERNAME')
        password = self.spot_password or os.environ.get('BOSDYN_CLIENT_PASSWORD')
        if not username or not password:
            raise RuntimeError(
                'Spot credentials are not configured. Set SPOT_USERNAME and '
                'SPOT_PASSWORD in config/field.env or BOSDYN_CLIENT_USERNAME/'
                'BOSDYN_CLIENT_PASSWORD in the environment.'
            )

        sdk = bosdyn.client.create_standard_sdk('InfrastructureInspectionClient')
        self.spot = sdk.create_robot(self.spot_ip)
        self.spot.authenticate(username, password)
        self.spot.time_sync.wait_for_sync()

        lease_client = self.spot.ensure_client(LeaseClient.default_service_name)
        self.spot_lease_keepalive = LeaseKeepAlive(
            lease_client,
            must_acquire=True,
            return_at_exit=self.spot_auto_return_lease,
        )
        self.spot_command_client = self.spot.ensure_client(
            RobotCommandClient.default_service_name
        )

        if self.spot_auto_power_on:
            self.publish_status('powering on Spot motors')
            self.spot.power_on(timeout_sec=20)
        if self.spot_stand_before_move:
            self.publish_status('commanding Spot to stand')
            blocking_stand(self.spot_command_client, timeout_sec=10)
        self.publish_status(f'connected to Spot at {self.spot_ip}')

    def wait_for_spot_goal(self, command_id):
        try:
            from bosdyn.client.robot_command import block_for_trajectory_cmd
        except ImportError:
            block_for_trajectory_cmd = None

        if block_for_trajectory_cmd is None:
            time.sleep(min(self.spot_goal_duration, self.spot_arrival_timeout))
            self.finish_goal('spot sdk command duration elapsed')
            return

        result = block_for_trajectory_cmd(
            self.spot_command_client,
            command_id,
            timeout_sec=self.spot_arrival_timeout,
            logger=self.get_logger(),
        )
        self.finish_goal(f'spot sdk trajectory completed: {result}')

    def dry_run_arrived(self):
        if self.active_goal is None:
            return
        if self.dry_run_timer is not None:
            self.dry_run_timer.cancel()
            self.dry_run_timer = None
        self.finish_goal('dry-run waypoint arrived')

    def finish_goal(self, detail, arrived=True):
        self.publish_status(detail)
        if arrived:
            message = String()
            message.data = detail
            self.arrival_publisher.publish(message)
        self.active_goal = None

    def publish_status(self, message):
        status = String()
        status.data = message
        self.status_publisher.publish(status)
        self.get_logger().info(message)

    def destroy_node(self):
        if self.spot_lease_keepalive is not None:
            self.spot_lease_keepalive.shutdown()
            self.spot_lease_keepalive = None
        super().destroy_node()


def yaw_from_quaternion(rotation):
    x = rotation.x
    y = rotation.y
    z = rotation.z
    w = rotation.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = RobotGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
