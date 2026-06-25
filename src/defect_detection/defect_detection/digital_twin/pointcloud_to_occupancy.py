import math

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x = x0
    y = y0

    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x += sx
        if doubled <= dx:
            error += dx
            y += sy


class PointCloudToOccupancy(Node):

    def __init__(self):
        super().__init__('pointcloud_to_occupancy')

        self.declare_parameter('pointcloud_topic', '/trimble/x7/scan_points')
        self.declare_parameter('map_topic', '/digital_twin/map')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('padding_m', 2.0)
        self.declare_parameter('min_z', -0.25)
        self.declare_parameter('max_z', 1.20)
        self.declare_parameter('max_range_m', 80.0)
        self.declare_parameter('scan_origin_x', 0.0)
        self.declare_parameter('scan_origin_y', 0.0)

        pointcloud_topic = self.get_parameter(
            'pointcloud_topic'
        ).get_parameter_value().string_value
        map_topic = self.get_parameter(
            'map_topic'
        ).get_parameter_value().string_value
        self.resolution = self.get_parameter(
            'resolution'
        ).get_parameter_value().double_value
        self.padding = self.get_parameter(
            'padding_m'
        ).get_parameter_value().double_value
        self.min_z = self.get_parameter('min_z').get_parameter_value().double_value
        self.max_z = self.get_parameter('max_z').get_parameter_value().double_value
        self.max_range = self.get_parameter(
            'max_range_m'
        ).get_parameter_value().double_value
        self.scan_origin_x = self.get_parameter(
            'scan_origin_x'
        ).get_parameter_value().double_value
        self.scan_origin_y = self.get_parameter(
            'scan_origin_y'
        ).get_parameter_value().double_value

        if self.resolution <= 0.0:
            raise ValueError('resolution must be greater than zero')
        if self.max_z <= self.min_z:
            raise ValueError('max_z must be greater than min_z')

        self.publisher = self.create_publisher(OccupancyGrid, map_topic, 1)
        self.subscription = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Converting {pointcloud_topic} into occupancy grid {map_topic}'
        )

    def cloud_callback(self, cloud):
        points = point_cloud2.read_points_numpy(
            cloud,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )
        points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        if points.size == 0:
            return

        z_mask = (points[:, 2] >= self.min_z) & (points[:, 2] <= self.max_z)
        range_mask = np.hypot(
            points[:, 0] - self.scan_origin_x,
            points[:, 1] - self.scan_origin_y,
        ) <= self.max_range
        obstacle_points = points[z_mask & range_mask]
        if len(obstacle_points) == 0:
            self.get_logger().warning('No points remained after map filtering')
            return

        min_x = min(np.min(obstacle_points[:, 0]), self.scan_origin_x) - self.padding
        max_x = max(np.max(obstacle_points[:, 0]), self.scan_origin_x) + self.padding
        min_y = min(np.min(obstacle_points[:, 1]), self.scan_origin_y) - self.padding
        max_y = max(np.max(obstacle_points[:, 1]), self.scan_origin_y) + self.padding

        width = max(1, int(math.ceil((max_x - min_x) / self.resolution)))
        height = max(1, int(math.ceil((max_y - min_y) / self.resolution)))
        grid = np.full((height, width), UNKNOWN, dtype=np.int8)

        origin_cell = self.world_to_cell(
            self.scan_origin_x,
            self.scan_origin_y,
            min_x,
            min_y,
        )

        occupied_cells = set()
        for point in obstacle_points:
            end_cell = self.world_to_cell(point[0], point[1], min_x, min_y)
            if not self.cell_in_bounds(end_cell, width, height):
                continue
            for cell in bresenham(
                origin_cell[0],
                origin_cell[1],
                end_cell[0],
                end_cell[1],
            ):
                if not self.cell_in_bounds(cell, width, height):
                    break
                grid[cell[1], cell[0]] = FREE
            occupied_cells.add(end_cell)

        for x, y in occupied_cells:
            grid[y, x] = OCCUPIED

        message = OccupancyGrid()
        message.header = cloud.header
        message.info.resolution = self.resolution
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = float(min_x)
        message.info.origin.position.y = float(min_y)
        message.info.origin.orientation.w = 1.0
        message.data = grid.reshape(-1).astype(int).tolist()
        self.publisher.publish(message)
        self.get_logger().info(
            f'Published occupancy grid {width}x{height} from X7 scan'
        )

    def world_to_cell(self, x, y, origin_x, origin_y):
        return (
            int(math.floor((x - origin_x) / self.resolution)),
            int(math.floor((y - origin_y) / self.resolution)),
        )

    @staticmethod
    def cell_in_bounds(cell, width, height):
        return 0 <= cell[0] < width and 0 <= cell[1] < height


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToOccupancy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
