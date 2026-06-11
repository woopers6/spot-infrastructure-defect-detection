import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriber(Node):

    def __init__(self):
        super.__init__('image_subscriber')
        self.latest_msg = None

        self.subscription = self.create_subscription(
            Image,
            'ros2_image',
            self.listener_callback,
            10
        )
    def listener_callback(self, msg):
        self.latest_msg = msg
        self.get_logger().info(
            f'Received image!'
        )
    def get_image_data(self):
        return self.latest_msg
    
def main(args=None):
    rclpy.init(args=args)

    node = ImageSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
     main()
    

