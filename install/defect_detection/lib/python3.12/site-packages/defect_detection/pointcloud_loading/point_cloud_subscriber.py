import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudSubscriber(Node):

    def __init__(self):
        super().__init__('point_cloud_subscriber')

        self.latest_msg = None

        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne/points',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.latest_msg = msg
        self.get_logger().info(
            f'Received point cloud: width={msg.width}, height={msg.height}, frame={msg.header.frame_id}'
        )

    def get_pointcloud_data(self):
        return self.latest_msg
    

def main(args=None):
    rclpy.init(args=args)

    node = PointCloudSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()