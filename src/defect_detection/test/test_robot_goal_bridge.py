import math
import os

from geometry_msgs.msg import PoseStamped, TransformStamped
import pytest
import rclpy

from defect_detection.digital_twin.robot_goal_bridge import (
    RobotGoalBridge,
    normalize_angle,
)


@pytest.mark.parametrize(
    ('angle', 'expected'),
    [
        (0.0, 0.0),
        (math.pi, math.pi),
        (-math.pi, -math.pi),
        (3.0 * math.pi, math.pi),
        (-3.0 * math.pi, -math.pi),
        (2.5 * math.pi, 0.5 * math.pi),
    ],
)
def test_normalize_angle(angle, expected):
    assert normalize_angle(angle) == pytest.approx(expected)


def make_transform(x, y, yaw):
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'body'
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    return transform


def make_goal(x, y, yaw):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def test_arrival_error_uses_tf_distance_and_yaw():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = RobotGoalBridge()
    try:
        now = node.get_clock().now().to_msg()
        transform = make_transform(1.0, 2.0, 0.25)
        transform.header.stamp = now
        node.tf_buffer.set_transform(transform, 'test')

        distance_error, yaw_error = node.arrival_error(make_goal(1.1, 2.2, 0.35))

        assert distance_error == pytest.approx(math.hypot(0.1, 0.2))
        assert yaw_error == pytest.approx(0.10)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_arrival_error_identifies_far_pose():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = RobotGoalBridge()
    try:
        transform = make_transform(0.0, 0.0, 0.0)
        transform.header.stamp = node.get_clock().now().to_msg()
        node.tf_buffer.set_transform(transform, 'test')

        distance_error, yaw_error = node.arrival_error(make_goal(2.0, 0.0, 1.0))

        assert distance_error == pytest.approx(2.0)
        assert yaw_error == pytest.approx(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
