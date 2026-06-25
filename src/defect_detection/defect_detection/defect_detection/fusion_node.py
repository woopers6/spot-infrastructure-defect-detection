from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from defect_detection.defect_localization.extract_3d_detections import (
    extract_detections_3d,
)
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D as Detection3DMsg,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
import yaml


def stamp_to_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def timestamp_delta_seconds(first_stamp, second_stamp):
    first = stamp_to_nanoseconds(first_stamp)
    second = stamp_to_nanoseconds(second_stamp)
    if first == 0 or second == 0:
        raise ValueError('Synchronized messages must have non-zero timestamps')
    return abs(first - second) / 1e9


def load_calibration(calibration_path):
    path = Path(calibration_path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f'Calibration file not found: {path}')

    with path.open('r', encoding='utf-8') as file:
        calibration = yaml.safe_load(file) or {}

    if not calibration.get('calibrated', False):
        raise ValueError(
            f'Calibration file is not marked calibrated: {path}'
        )

    intrinsics = np.asarray(
        calibration.get('camera_matrix'),
        dtype=np.float64,
    )
    lidar_to_camera = np.asarray(
        calibration.get('lidar_to_camera'),
        dtype=np.float64,
    )
    image_width = int(calibration.get('image_width', 0))
    image_height = int(calibration.get('image_height', 0))

    if intrinsics.shape != (3, 3):
        raise ValueError('camera_matrix must be a 3x3 matrix')
    if lidar_to_camera.shape != (4, 4):
        raise ValueError('lidar_to_camera must be a 4x4 matrix')
    if image_width <= 0 or image_height <= 0:
        raise ValueError('image_width and image_height must be positive')

    return intrinsics, lidar_to_camera, (image_height, image_width)


class DetectionFusionNode(Node):

    def __init__(self):
        super().__init__('detection_fusion_node')

        package_share = Path(get_package_share_directory('defect_detection'))
        self.declare_parameter(
            'dataset_path',
            str(package_share / 'models' / 'dataset.yaml'),
        )
        self.declare_parameter(
            'calibration_path',
            str(package_share / 'config' / 'site_calibration.yaml'),
        )
        self.declare_parameter('detections_2d_topic', '/detections_2d')
        self.declare_parameter('pointcloud_topic', '/lidar/points')
        self.declare_parameter('detections_3d_topic', '/detections_3d')
        dataset_path = Path(
            self.get_parameter(
                'dataset_path'
            ).get_parameter_value().string_value
        ).expanduser()
        calibration_path = self.get_parameter(
            'calibration_path'
        ).get_parameter_value().string_value
        detections_2d_topic = self.get_parameter(
            'detections_2d_topic'
        ).get_parameter_value().string_value
        pointcloud_topic = self.get_parameter(
            'pointcloud_topic'
        ).get_parameter_value().string_value
        detections_3d_topic = self.get_parameter(
            'detections_3d_topic'
        ).get_parameter_value().string_value

        if not dataset_path.is_file():
            raise FileNotFoundError(
                f'Dataset configuration not found: {dataset_path}'
            )

        with dataset_path.open('r', encoding='utf-8') as file:
            data = yaml.safe_load(file)

        names = data['names']

        if isinstance(names, dict):
            self.class_names = [
                names[index] for index in sorted(names.keys())
            ]
        else:
            self.class_names = list(names)

        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('sync_slop_sec', 0.10)

        sync_queue_size = self.get_parameter(
            'sync_queue_size'
        ).get_parameter_value().integer_value
        self.sync_slop_sec = self.get_parameter(
            'sync_slop_sec'
        ).get_parameter_value().double_value

        if sync_queue_size <= 0:
            raise ValueError('sync_queue_size must be greater than zero')
        if self.sync_slop_sec <= 0.0:
            raise ValueError('sync_slop_sec must be greater than zero')

        (
            self.intrinsics_matrix,
            self.T_lidar_to_camera,
            self.image_shape,
        ) = load_calibration(calibration_path)
        self.get_logger().info(
            f'Loaded camera-LiDAR calibration from {calibration_path}'
        )

        # Output publisher
        self.detections_3d_pub = self.create_publisher(
            Detection3DArray,
            detections_3d_topic,
            10,
        )

        self.detections_sub = message_filters.Subscriber(
            self,
            Detection2DArray,
            detections_2d_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.pointcloud_sub = message_filters.Subscriber(
            self,
            PointCloud2,
            pointcloud_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [
                self.detections_sub,
                self.pointcloud_sub,
            ],
            queue_size=sync_queue_size,
            slop=self.sync_slop_sec,
        )

        self.synchronizer.registerCallback(
            self.synchronized_callback
        )

        self.get_logger().info(
            f'Fusion node started. Publishing on {detections_3d_topic}'
        )

    def synchronized_callback(
        self,
        detections_2d_msg: Detection2DArray,
        pointcloud_msg: PointCloud2,
    ):
        try:
            timestamp_delta = timestamp_delta_seconds(
                detections_2d_msg.header.stamp,
                pointcloud_msg.header.stamp,
            )
        except ValueError as error:
            self.get_logger().warning(f'Dropping synchronized pair: {error}')
            return

        if timestamp_delta > self.sync_slop_sec:
            self.get_logger().warning(
                'Dropping synchronized pair with timestamp delta '
                f'{timestamp_delta:.3f}s'
            )
            return

        self.get_logger().debug(
            'Received synchronized detection and point-cloud pair '
            f'(delta={timestamp_delta * 1000.0:.1f}ms)'
        )

        detections_3d = extract_detections_3d(
            pointcloud_msg=pointcloud_msg,
            intrinsics_3x3=self.intrinsics_matrix,
            T_lidar_to_camera=self.T_lidar_to_camera,
            boxes=detections_2d_msg.detections,
            class_names=self.class_names,
            image_shape=self.image_shape,
            filter_outliers=True,
            max_depth_deviation=0.50,
            depth_cluster_tolerance=0.20,
            min_points=3,
        )

        output_msg = Detection3DArray()
        output_msg.header = pointcloud_msg.header

        for detection_data in detections_3d:
            detection_msg = self.convert_to_ros_detection3d(
                detection_data
            )

            detection_msg.header = pointcloud_msg.header

            output_msg.detections.append(detection_msg)

        self.detections_3d_pub.publish(output_msg)

        self.get_logger().debug(
            f'Published {len(output_msg.detections)} detections '
            f'to /detections_3d'
        )

    def convert_to_ros_detection3d(self, detection_data):
        detection_msg = Detection3DMsg()

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(
            detection_data.class_id
            if detection_data.class_id is not None
            else -1
        )
        hypothesis.hypothesis.score = float(
            detection_data.confidence
        )

        hypothesis.pose.pose.position.x = float(
            detection_data.centroid_lidar[0]
        )
        hypothesis.pose.pose.position.y = float(
            detection_data.centroid_lidar[1]
        )
        hypothesis.pose.pose.position.z = float(
            detection_data.centroid_lidar[2]
        )

        hypothesis.pose.pose.orientation.x = 0.0
        hypothesis.pose.pose.orientation.y = 0.0
        hypothesis.pose.pose.orientation.z = 0.0
        hypothesis.pose.pose.orientation.w = 1.0

        detection_msg.results.append(hypothesis)

        detection_msg.bbox = BoundingBox3D()

        if detection_data.bbox_3d_lidar is not None:
            bbox = detection_data.bbox_3d_lidar

            detection_msg.bbox.center.position.x = float(
                bbox.center[0]
            )
            detection_msg.bbox.center.position.y = float(
                bbox.center[1]
            )
            detection_msg.bbox.center.position.z = float(
                bbox.center[2]
            )

            detection_msg.bbox.center.orientation.x = 0.0
            detection_msg.bbox.center.orientation.y = 0.0
            detection_msg.bbox.center.orientation.z = 0.0
            detection_msg.bbox.center.orientation.w = 1.0

            detection_msg.bbox.size.x = float(bbox.size[0])
            detection_msg.bbox.size.y = float(bbox.size[1])
            detection_msg.bbox.size.z = float(bbox.size[2])

        return detection_msg


def main(args=None):
    rclpy.init(args=args)

    node = DetectionFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
