import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('reconnect_interval_sec', 2.0)

        self.camera_index = self.get_parameter(
            'camera_index'
        ).get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            'frame_id'
        ).get_parameter_value().string_value
        frame_rate = self.get_parameter(
            'frame_rate'
        ).get_parameter_value().double_value
        self.reconnect_interval = self.get_parameter(
            'reconnect_interval_sec'
        ).get_parameter_value().double_value

        if frame_rate <= 0.0:
            raise ValueError('frame_rate must be greater than zero')
        if self.reconnect_interval <= 0.0:
            raise ValueError(
                'reconnect_interval_sec must be greater than zero'
            )

        self.publisher_ = self.create_publisher(
            Image,
            'ros2_image',
            qos_profile_sensor_data,
        )

        self.bridge = CvBridge()
        self.cap = None
        self.last_open_attempt = None
        self.open_camera()

        self.timer = self.create_timer(1.0 / frame_rate, self.publish_image)

    def open_camera(self):
        self.last_open_attempt = time.monotonic()
        capture = cv2.VideoCapture(self.camera_index)
        if not capture.isOpened():
            capture.release()
            self.get_logger().warning(
                f'Waiting for camera index {self.camera_index}'
            )
            return False

        if self.cap is not None:
            self.cap.release()
        self.cap = capture
        self.get_logger().info(f'Opened camera index {self.camera_index}')
        return True

    def ensure_camera_open(self):
        if self.cap is not None and self.cap.isOpened():
            return True
        if (
            self.last_open_attempt is not None
            and time.monotonic() - self.last_open_attempt
            < self.reconnect_interval
        ):
            return False
        return self.open_camera()

    def publish_image(self):
        if not self.ensure_camera_open():
            return

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning(
                'Camera read failed; waiting to reconnect'
            )
            self.cap.release()
            self.cap = None
            return

        # OpenCV has no reliable hardware timestamp, so stamp immediately.
        stamp = self.get_clock().now().to_msg()
        ros2_image_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8',
        )
        ros2_image_msg.header.stamp = stamp
        ros2_image_msg.header.frame_id = self.frame_id

        self.publisher_.publish(ros2_image_msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ImagePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
