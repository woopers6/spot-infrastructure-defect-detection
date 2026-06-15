import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from defect_detection.autonomous_navigation.planning import (
    DefectWaypoint,
    generate_standoff_candidates,
    merge_detection,
    order_candidates_by_distance,
    path_length,
    priority_score,
    select_next_waypoint,
)
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def detection_confidence(detection):
    if not detection.results:
        return 0.0
    return float(detection.results[0].hypothesis.score)


def detection_class_id(detection):
    if not detection.results:
        return 'unknown'
    return detection.results[0].hypothesis.class_id or 'unknown'


def load_priority_config(path):
    config_path = Path(path).expanduser()
    with config_path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}

    priorities = {
        str(class_id): float(priority)
        for class_id, priority in (
            config.get('class_priorities') or {}
        ).items()
    }
    default_priority = float(config.get('default_priority', 1.0))
    return priorities, default_priority


class AutonomousDefectNavigator(Node):

    def __init__(self):
        super().__init__('autonomous_defect_navigator')

        package_share = Path(get_package_share_directory('defect_detection'))
        self.declare_parameter('enabled', False)
        self.declare_parameter('detections_topic', '/detections_3d')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'priority_config_path',
            str(package_share / 'config' / 'navigation_priorities.yaml'),
        )
        self.declare_parameter('minimum_confidence', 0.50)
        self.declare_parameter('merge_radius', 0.75)
        self.declare_parameter('standoff_distance', 1.50)
        self.declare_parameter('candidate_count', 12)
        self.declare_parameter('planning_period_sec', 2.0)
        self.declare_parameter('retry_cooldown_sec', 30.0)
        self.declare_parameter('max_attempts', 3)
        self.declare_parameter('confidence_weight', 2.0)
        self.declare_parameter('size_weight', 0.5)
        self.declare_parameter('observation_weight', 0.25)
        self.declare_parameter(
            'compute_path_action',
            '/compute_path_to_pose',
        )
        self.declare_parameter(
            'navigate_action',
            '/navigate_to_pose',
        )

        self.enabled = self.get_parameter(
            'enabled'
        ).get_parameter_value().bool_value
        detections_topic = self.get_parameter(
            'detections_topic'
        ).get_parameter_value().string_value
        self.target_frame = self.get_parameter(
            'target_frame'
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value
        priority_config_path = self.get_parameter(
            'priority_config_path'
        ).get_parameter_value().string_value
        self.minimum_confidence = self.get_parameter(
            'minimum_confidence'
        ).get_parameter_value().double_value
        self.merge_radius = self.get_parameter(
            'merge_radius'
        ).get_parameter_value().double_value
        self.standoff_distance = self.get_parameter(
            'standoff_distance'
        ).get_parameter_value().double_value
        self.candidate_count = self.get_parameter(
            'candidate_count'
        ).get_parameter_value().integer_value
        planning_period = self.get_parameter(
            'planning_period_sec'
        ).get_parameter_value().double_value
        self.retry_cooldown = self.get_parameter(
            'retry_cooldown_sec'
        ).get_parameter_value().double_value
        self.max_attempts = self.get_parameter(
            'max_attempts'
        ).get_parameter_value().integer_value
        self.confidence_weight = self.get_parameter(
            'confidence_weight'
        ).get_parameter_value().double_value
        self.size_weight = self.get_parameter(
            'size_weight'
        ).get_parameter_value().double_value
        self.observation_weight = self.get_parameter(
            'observation_weight'
        ).get_parameter_value().double_value
        compute_path_action = self.get_parameter(
            'compute_path_action'
        ).get_parameter_value().string_value
        navigate_action = self.get_parameter(
            'navigate_action'
        ).get_parameter_value().string_value

        if self.standoff_distance <= 0.0:
            raise ValueError('standoff_distance must be greater than zero')
        if self.candidate_count < 4:
            raise ValueError('candidate_count must be at least four')
        if planning_period <= 0.0:
            raise ValueError('planning_period_sec must be greater than zero')

        (
            self.class_priorities,
            self.default_class_priority,
        ) = load_priority_config(priority_config_path)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            compute_path_action,
        )
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            navigate_action,
        )

        self.waypoints = {}
        self.next_waypoint_id = 1
        self.active_waypoint = None
        self.planning_candidates = []
        self.best_candidate = None
        self.best_path_length = math.inf
        self.state = 'idle'

        self.selected_goal_publisher = self.create_publisher(
            PoseStamped,
            '/autonomous_navigation/selected_goal',
            10,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/autonomous_navigation/markers',
            10,
        )
        self.detection_subscription = self.create_subscription(
            Detection3DArray,
            detections_topic,
            self.detections_callback,
            10,
        )
        self.timer = self.create_timer(
            planning_period,
            self.planning_tick,
        )

        mode = 'ENABLED' if self.enabled else 'preview-only'
        self.get_logger().info(
            f'Autonomous defect navigation started in {mode} mode'
        )

    def detections_callback(self, detections_msg):
        now_sec = time.monotonic()

        for detection in detections_msg.detections:
            confidence = detection_confidence(detection)
            if confidence < self.minimum_confidence:
                continue

            source_header = detection.header
            if not source_header.frame_id:
                source_header = detections_msg.header

            point = PointStamped()
            point.header = source_header
            point.point = detection.bbox.center.position

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_header.frame_id,
                    rclpy.time.Time.from_msg(source_header.stamp),
                    timeout=Duration(seconds=0.25),
                )
                mapped_point = do_transform_point(point, transform)
            except TransformException as error:
                self.get_logger().warning(
                    f'Cannot map detection into {self.target_frame}: {error}'
                )
                continue

            waypoint = DefectWaypoint(
                waypoint_id=f'defect_{self.next_waypoint_id}',
                class_id=detection_class_id(detection),
                confidence=confidence,
                x=mapped_point.point.x,
                y=mapped_point.point.y,
                z=mapped_point.point.z,
                size_x=detection.bbox.size.x,
                size_y=detection.bbox.size.y,
                size_z=detection.bbox.size.z,
                first_seen_sec=now_sec,
                last_seen_sec=now_sec,
            )
            merged = merge_detection(
                self.waypoints,
                waypoint,
                self.merge_radius,
            )
            if merged is waypoint:
                self.next_waypoint_id += 1

        self.publish_waypoint_markers()

    def planning_tick(self):
        if self.state != 'idle':
            return

        waypoint = select_next_waypoint(
            self.waypoints,
            time.monotonic(),
            self.class_priorities,
            self.default_class_priority,
            self.confidence_weight,
            self.size_weight,
            self.observation_weight,
        )
        if waypoint is None:
            return

        if not self.path_client.server_is_ready():
            self.get_logger().warning('Waiting for Nav2 ComputePathToPose')
            return

        self.active_waypoint = waypoint
        self.planning_candidates = generate_standoff_candidates(
            waypoint,
            self.standoff_distance,
            self.candidate_count,
        )
        try:
            robot_transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
            robot_position = robot_transform.transform.translation
            self.planning_candidates = order_candidates_by_distance(
                self.planning_candidates,
                robot_position.x,
                robot_position.y,
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Cannot locate {self.base_frame} in {self.target_frame}; '
                f'evaluating all viewpoints in ring order: {error}'
            )
        self.best_candidate = None
        self.best_path_length = math.inf
        self.state = 'planning'
        self.plan_next_candidate()

    def make_pose(self, candidate):
        x, y, yaw = candidate
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quaternion_from_yaw(yaw)
        return pose

    def plan_next_candidate(self):
        if not self.planning_candidates:
            self.finish_candidate_planning()
            return

        candidate = self.planning_candidates.pop(0)
        goal = ComputePathToPose.Goal()
        goal.goal = self.make_pose(candidate)
        goal.use_start = False

        future = self.path_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, item=candidate: self.path_goal_response(
                completed,
                item,
            )
        )

    def path_goal_response(self, future, candidate):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().warning(f'Path request failed: {error}')
            self.plan_next_candidate()
            return

        if not goal_handle.accepted:
            self.plan_next_candidate()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, item=candidate: self.path_result(
                completed,
                item,
            )
        )

    def path_result(self, future, candidate):
        try:
            wrapped_result = future.result()
            if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
                path = wrapped_result.result.path
                if path.poses:
                    candidate_length = path_length(path)
                else:
                    candidate_length = math.inf
                if candidate_length < self.best_path_length:
                    self.best_path_length = candidate_length
                    self.best_candidate = candidate
        except Exception as error:
            self.get_logger().warning(f'Path result failed: {error}')

        self.plan_next_candidate()

    def finish_candidate_planning(self):
        if self.best_candidate is None:
            self.fail_active_waypoint('no collision-free standoff pose')
            return

        goal_pose = self.make_pose(self.best_candidate)
        self.active_waypoint.selected_goal = self.best_candidate
        self.selected_goal_publisher.publish(goal_pose)
        self.publish_waypoint_markers()

        score = priority_score(
            self.active_waypoint,
            self.class_priorities,
            self.default_class_priority,
            self.confidence_weight,
            self.size_weight,
            self.observation_weight,
        )
        self.get_logger().info(
            f'Selected {self.active_waypoint.waypoint_id} '
            f'(priority={score:.2f}, path={self.best_path_length:.2f}m)'
        )

        if not self.enabled:
            self.active_waypoint.cooldown_until_sec = (
                time.monotonic() + self.retry_cooldown
            )
            self.reset_state()
            return

        if not self.navigation_client.server_is_ready():
            self.fail_active_waypoint('NavigateToPose is unavailable')
            return

        navigation_goal = NavigateToPose.Goal()
        navigation_goal.pose = goal_pose
        future = self.navigation_client.send_goal_async(navigation_goal)
        future.add_done_callback(self.navigation_goal_response)
        self.state = 'navigating'

    def navigation_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.fail_active_waypoint(f'navigation request failed: {error}')
            return

        if not goal_handle.accepted:
            self.fail_active_waypoint('navigation goal was rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result)

    def navigation_result(self, future):
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.fail_active_waypoint(f'navigation failed: {error}')
            return

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.active_waypoint.completed = True
            self.get_logger().info(
                f'Reached standoff for {self.active_waypoint.waypoint_id}'
            )
            self.publish_waypoint_markers()
            self.reset_state()
            return

        self.fail_active_waypoint(
            f'Nav2 ended with status {wrapped_result.status}'
        )

    def fail_active_waypoint(self, reason):
        if self.active_waypoint is not None:
            self.active_waypoint.attempts += 1
            self.active_waypoint.cooldown_until_sec = (
                time.monotonic() + self.retry_cooldown
            )
            if self.active_waypoint.attempts >= self.max_attempts:
                self.active_waypoint.abandoned = True
            self.get_logger().warning(
                f'{self.active_waypoint.waypoint_id}: {reason}'
            )
        self.publish_waypoint_markers()
        self.reset_state()

    def reset_state(self):
        self.active_waypoint = None
        self.planning_candidates = []
        self.best_candidate = None
        self.best_path_length = math.inf
        self.state = 'idle'

    def publish_waypoint_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for marker_id, waypoint in enumerate(self.waypoints.values()):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'defect_navigation_queue'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.x
            marker.pose.position.y = waypoint.y
            marker.pose.position.z = waypoint.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 0.9
            if waypoint.completed:
                marker.color.g = 1.0
            elif waypoint.abandoned:
                marker.color.r = 0.4
                marker.color.g = 0.4
                marker.color.b = 0.4
            elif waypoint is self.active_waypoint:
                marker.color.r = 1.0
                marker.color.g = 0.7
            else:
                marker.color.r = 1.0
            markers.markers.append(marker)

        self.marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDefectNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
