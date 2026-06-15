import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('frame_rate', 30.0)

        camera_index = self.get_parameter(
            'camera_index'
        ).get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            'frame_id'
        ).get_parameter_value().string_value
        frame_rate = self.get_parameter(
            'frame_rate'
        ).get_parameter_value().double_value

        if frame_rate <= 0.0:
            raise ValueError('frame_rate must be greater than zero')

        self.publisher_ = self.create_publisher(
            Image,
            'ros2_image',
            qos_profile_sensor_data,
        )

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(camera_index)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")

        self.timer = self.create_timer(1.0 / frame_rate, self.publish_image)

    def publish_image(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to capture frame.")
            return

        # OpenCV does not expose a reliable hardware timestamp here. Stamp as
        # soon as read() returns to minimize capture-to-stamp uncertainty.
        stamp = self.get_clock().now().to_msg()
        ros2_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ros2_image_msg.header.stamp = stamp
        ros2_image_msg.header.frame_id = self.frame_id

        self.publisher_.publish(ros2_image_msg)

    def destroy_node(self):
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
