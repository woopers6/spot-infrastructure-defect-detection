import os

import numpy as np
import rclpy
from bosdyn.api import point_cloud_pb2
import bosdyn.client
from bosdyn.client import util as bosdyn_util
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def select_point_cloud_source(sources, requested_name=''):
    names = [source.name for source in sources]

    if requested_name:
        if requested_name not in names:
            available = ', '.join(names) or '<none>'
            raise ValueError(
                f'Point-cloud source {requested_name!r} was not found. '
                f'Available sources: {available}'
            )
        return requested_name

    velodyne_sources = [
        name for name in names if 'velodyne' in name.lower()
    ]
    if velodyne_sources:
        return velodyne_sources[0]
    if len(names) == 1:
        return names[0]

    available = ', '.join(names) or '<none>'
    raise ValueError(
        'Could not automatically select a Velodyne point-cloud source. '
        f'Available sources: {available}'
    )


def response_point_cloud(response):
    cloud_data = response.WhichOneof('cloud_data')
    if cloud_data == 'point_cloud':
        return response.point_cloud
    if cloud_data == 'lidar_cloud':
        return response.lidar_cloud.point_cloud
    return None


def decode_xyz32(point_cloud):
    if point_cloud.encoding != point_cloud_pb2.PointCloud.ENCODING_XYZ_32F:
        encoding = point_cloud_pb2.PointCloud.Encoding.Name(
            point_cloud.encoding
        )
        raise ValueError(
            f'Unsupported Spot point-cloud encoding: {encoding}'
        )

    expected_bytes = point_cloud.num_points * 3 * np.dtype('<f4').itemsize
    if len(point_cloud.data) != expected_bytes:
        raise ValueError(
            'Point-cloud byte count does not match num_points: '
            f'expected {expected_bytes}, got {len(point_cloud.data)}'
        )

    return np.frombuffer(
        point_cloud.data,
        dtype='<f4',
    ).reshape((-1, 3))


def robot_timestamp_to_ros_time(timestamp, clock_skew_nsec):
    robot_nsec = timestamp.seconds * 1_000_000_000 + timestamp.nanos
    local_nsec = robot_nsec - clock_skew_nsec
    if local_nsec <= 0:
        raise ValueError('Converted point-cloud timestamp is not valid')

    return Time(
        sec=local_nsec // 1_000_000_000,
        nanosec=local_nsec % 1_000_000_000,
    )


class SpotSdkPointCloud(Node):

    def __init__(self):
        super().__init__('spot_sdk_pointcloud')

        self.declare_parameter('hostname', '')
        self.declare_parameter('service_name', 'velodyne-point-cloud')
        self.declare_parameter('source_name', '')
        self.declare_parameter('output_topic', '/eap/lidar/points')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('rpc_timeout_sec', 5.0)
        self.declare_parameter('downsample_rate', 1)

        hostname = self.get_parameter(
            'hostname'
        ).get_parameter_value().string_value
        hostname = hostname or os.environ.get('SPOT_IP', '')
        service_name = self.get_parameter(
            'service_name'
        ).get_parameter_value().string_value
        requested_source = self.get_parameter(
            'source_name'
        ).get_parameter_value().string_value
        output_topic = self.get_parameter(
            'output_topic'
        ).get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id'
        ).get_parameter_value().string_value
        publish_rate = self.get_parameter(
            'publish_rate'
        ).get_parameter_value().double_value
        self.rpc_timeout = self.get_parameter(
            'rpc_timeout_sec'
        ).get_parameter_value().double_value
        self.downsample_rate = self.get_parameter(
            'downsample_rate'
        ).get_parameter_value().integer_value

        if not hostname:
            raise ValueError(
                'Set the hostname parameter or the SPOT_IP environment variable'
            )
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if self.rpc_timeout <= 0.0:
            raise ValueError('rpc_timeout_sec must be greater than zero')
        if self.downsample_rate <= 0:
            raise ValueError('downsample_rate must be greater than zero')

        sdk = bosdyn.client.create_standard_sdk('spot-eap-ros2')
        self.robot = sdk.create_robot(hostname)
        bosdyn_util.authenticate(self.robot)
        self.robot.sync_with_directory()
        self.robot.time_sync.wait_for_sync(timeout_sec=10.0)

        self.client = self.robot.ensure_client(service_name)
        sources = self.client.list_point_cloud_sources(
            timeout=self.rpc_timeout
        )
        self.source_name = select_point_cloud_source(
            sources,
            requested_name=requested_source,
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            qos_profile_sensor_data,
        )
        self.last_acquisition_time = None
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_point_cloud,
        )

        self.get_logger().info(
            f'Reading Spot point-cloud source {self.source_name!r} '
            f'from service {service_name!r} at {hostname} '
            f'and publishing {output_topic}'
        )

    def publish_point_cloud(self):
        request = point_cloud_pb2.PointCloudRequest(
            point_cloud_source_name=self.source_name,
            cloud_type=point_cloud_pb2.PointCloudRequest.CLOUD_TYPE_POINTS,
            downsample_rate=self.downsample_rate,
        )

        try:
            responses = self.client.get_point_cloud(
                [request],
                timeout=self.rpc_timeout,
            )
            if not responses:
                self.get_logger().warning('Spot returned no point-cloud response')
                return

            spot_cloud = response_point_cloud(responses[0])
            if spot_cloud is None:
                return

            acquisition_time = spot_cloud.source.acquisition_time
            acquisition_key = (
                acquisition_time.seconds,
                acquisition_time.nanos,
            )
            if acquisition_key == self.last_acquisition_time:
                return

            points = decode_xyz32(spot_cloud)
            clock_skew = self.robot.time_sync.get_robot_clock_skew()
            stamp = robot_timestamp_to_ros_time(
                acquisition_time,
                clock_skew.seconds * 1_000_000_000 + clock_skew.nanos,
            )
        except Exception as error:
            self.get_logger().error(
                f'Failed to read Spot point cloud: {error}'
            )
            return

        header = Header()
        header.stamp = stamp
        header.frame_id = (
            self.frame_id or spot_cloud.source.frame_name_sensor
        )
        message = point_cloud2.create_cloud_xyz32(header, points)
        self.publisher.publish(message)
        self.last_acquisition_time = acquisition_key

    def destroy_node(self):
        self.robot.time_sync.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpotSdkPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
