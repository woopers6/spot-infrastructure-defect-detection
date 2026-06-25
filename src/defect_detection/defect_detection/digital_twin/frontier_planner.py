import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


UNKNOWN = -1
FREE = 0


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FrontierPlanner(Node):

    def __init__(self):
        super().__init__('digital_twin_frontier_planner')

        self.declare_parameter('map_topic', '/digital_twin/map')
        self.declare_parameter('goal_topic', '/digital_twin/frontier_goal')
        self.declare_parameter(
            'waypoint_arrived_topic',
            '/digital_twin/waypoint_arrived',
        )
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('send_nav2_goals', False)
        self.declare_parameter('min_frontier_distance_m', 1.0)

        map_topic = self.get_parameter(
            'map_topic'
        ).get_parameter_value().string_value
        goal_topic = self.get_parameter(
            'goal_topic'
        ).get_parameter_value().string_value
        waypoint_arrived_topic = self.get_parameter(
            'waypoint_arrived_topic'
        ).get_parameter_value().string_value
        self.target_frame = self.get_parameter(
            'target_frame'
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value
        navigate_action = self.get_parameter(
            'navigate_action'
        ).get_parameter_value().string_value
        self.send_nav2_goals = self.get_parameter(
            'send_nav2_goals'
        ).get_parameter_value().bool_value
        self.min_frontier_distance = self.get_parameter(
            'min_frontier_distance_m'
        ).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            navigate_action,
        )
        self.goal_publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        self.arrival_publisher = self.create_publisher(
            String,
            waypoint_arrived_topic,
            10,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            1,
        )
        self.active_goal = False
        self.get_logger().info(
            f'Planning frontier goals from {map_topic}; '
            f'send_nav2_goals={self.send_nav2_goals}'
        )

    def map_callback(self, grid):
        robot_x, robot_y = self.robot_position()
        if robot_x is None:
            return

        frontier = self.select_frontier(grid, robot_x, robot_y)
        if frontier is None:
            self.get_logger().warning('No frontier edge found in digital twin map')
            return

        goal = self.make_goal(grid, frontier, robot_x, robot_y)
        self.goal_publisher.publish(goal)
        if self.send_nav2_goals and not self.active_goal:
            self.send_goal(goal)

    def robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Cannot locate {self.base_frame} in {self.target_frame}: {error}'
            )
            return None, None
        translation = transform.transform.translation
        return translation.x, translation.y

    def select_frontier(self, grid, robot_x, robot_y):
        best = None
        best_distance = math.inf
        data = grid.data
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y * width + x] != FREE:
                    continue
                if not self.has_unknown_neighbor(data, width, x, y):
                    continue
                world_x = origin_x + (x + 0.5) * resolution
                world_y = origin_y + (y + 0.5) * resolution
                distance = math.hypot(world_x - robot_x, world_y - robot_y)
                if distance < self.min_frontier_distance:
                    continue
                if distance < best_distance:
                    best_distance = distance
                    best = world_x, world_y
        return best

    @staticmethod
    def has_unknown_neighbor(data, width, x, y):
        offsets = [
            (-1, -1), (0, -1), (1, -1),
            (-1, 0), (1, 0),
            (-1, 1), (0, 1), (1, 1),
        ]
        return any(data[(y + dy) * width + x + dx] == UNKNOWN for dx, dy in offsets)

    def make_goal(self, grid, frontier, robot_x, robot_y):
        x, y = frontier
        yaw = math.atan2(y - robot_y, x - robot_x)
        goal = PoseStamped()
        goal.header.frame_id = grid.header.frame_id or self.target_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        (
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w,
        ) = quaternion_from_yaw(yaw)
        return goal

    def send_goal(self, goal):
        if not self.navigation_client.server_is_ready():
            self.get_logger().warning('Waiting for Nav2 NavigateToPose')
            return
        request = NavigateToPose.Goal()
        request.pose = goal
        future = self.navigation_client.send_goal_async(request)
        future.add_done_callback(self.goal_response)
        self.active_goal = True

    def goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.active_goal = False
            self.get_logger().warning(f'Frontier navigation request failed: {error}')
            return
        if not goal_handle.accepted:
            self.active_goal = False
            self.get_logger().warning('Frontier navigation goal was rejected')
            return
        goal_handle.get_result_async().add_done_callback(self.goal_result)

    def goal_result(self, future):
        self.active_goal = False
        try:
            wrapped = future.result()
            self.get_logger().info(
                f'Frontier navigation completed with status {wrapped.status}'
            )
            message = String()
            message.data = f'frontier navigation status {wrapped.status}'
            self.arrival_publisher.publish(message)
        except Exception as error:
            self.get_logger().warning(f'Frontier navigation failed: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
