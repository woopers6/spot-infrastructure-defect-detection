from pathlib import Path
import time

from geometry_msgs.msg import PointStamped, Pose, PoseArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def detection_confidence(detection):
    if not detection.results:
        return 0.0
    return float(detection.results[0].hypothesis.score)


def detection_class_id(detection):
    if not detection.results:
        return 'unknown'
    return detection.results[0].hypothesis.class_id or 'unknown'


class DefectMapNode(Node):

    def __init__(self):
        super().__init__('digital_twin_defect_map')

        self.declare_parameter('detections_topic', '/detections_3d')
        self.declare_parameter('markers_topic', '/digital_twin/defect_markers')
        self.declare_parameter('rescan_goals_topic', '/digital_twin/rescan_goals')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('store_path', '/tmp/digital_twin_defects.yaml')
        self.declare_parameter('merge_radius_m', 0.75)
        self.declare_parameter('minimum_confidence', 0.50)

        detections_topic = self.get_parameter(
            'detections_topic'
        ).get_parameter_value().string_value
        markers_topic = self.get_parameter(
            'markers_topic'
        ).get_parameter_value().string_value
        rescan_goals_topic = self.get_parameter(
            'rescan_goals_topic'
        ).get_parameter_value().string_value
        self.target_frame = self.get_parameter(
            'target_frame'
        ).get_parameter_value().string_value
        self.store_path = Path(
            self.get_parameter('store_path').get_parameter_value().string_value
        ).expanduser()
        self.merge_radius = self.get_parameter(
            'merge_radius_m'
        ).get_parameter_value().double_value
        self.minimum_confidence = self.get_parameter(
            'minimum_confidence'
        ).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.defects = self.load_store()
        self.next_id = 1 + max(
            [int(item['id'].split('_')[-1]) for item in self.defects
             if item['id'].startswith('defect_')]
            or [0]
        )

        self.marker_publisher = self.create_publisher(MarkerArray, markers_topic, 10)
        self.rescan_publisher = self.create_publisher(PoseArray, rescan_goals_topic, 10)
        self.subscription = self.create_subscription(
            Detection3DArray,
            detections_topic,
            self.detections_callback,
            10,
        )
        self.timer = self.create_timer(2.0, self.publish_outputs)
        self.get_logger().info(
            f'Persisting digital twin defect markers to {self.store_path}'
        )

    def load_store(self):
        if not self.store_path.is_file():
            return []
        with self.store_path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        return list(data.get('defects') or [])

    def save_store(self):
        self.store_path.parent.mkdir(parents=True, exist_ok=True)
        with self.store_path.open('w', encoding='utf-8') as stream:
            yaml.safe_dump({'defects': self.defects}, stream, sort_keys=True)

    def detections_callback(self, detections_msg):
        changed = False
        now_sec = time.time()
        for detection in detections_msg.detections:
            confidence = detection_confidence(detection)
            if confidence < self.minimum_confidence:
                continue
            mapped = self.map_detection(detections_msg, detection)
            if mapped is None:
                continue
            x, y, z = mapped
            defect = self.find_nearby(x, y, z)
            if defect is None:
                defect = {
                    'id': f'defect_{self.next_id}',
                    'class_id': detection_class_id(detection),
                    'confidence': confidence,
                    'x': x,
                    'y': y,
                    'z': z,
                    'observations': 1,
                    'first_seen_sec': now_sec,
                    'last_seen_sec': now_sec,
                    'status': 'needs_rescan',
                }
                self.next_id += 1
                self.defects.append(defect)
            else:
                observations = int(defect.get('observations', 1)) + 1
                defect['x'] = (defect['x'] * (observations - 1) + x) / observations
                defect['y'] = (defect['y'] * (observations - 1) + y) / observations
                defect['z'] = (defect['z'] * (observations - 1) + z) / observations
                defect['confidence'] = max(defect.get('confidence', 0.0), confidence)
                defect['observations'] = observations
                defect['last_seen_sec'] = now_sec
            changed = True

        if changed:
            self.save_store()
            self.publish_outputs()

    def map_detection(self, detections_msg, detection):
        header = detection.header
        if not header.frame_id:
            header = detections_msg.header
        if not header.frame_id:
            self.get_logger().warning('Detection has no source frame')
            return None

        point = PointStamped()
        point.header = header
        point.point = detection.bbox.center.position
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                header.frame_id,
                rclpy.time.Time.from_msg(header.stamp),
                timeout=Duration(seconds=0.25),
            )
            mapped = do_transform_point(point, transform)
        except TransformException as error:
            self.get_logger().warning(
                f'Cannot map defect into {self.target_frame}: {error}'
            )
            return None
        return mapped.point.x, mapped.point.y, mapped.point.z

    def find_nearby(self, x, y, z):
        radius_sq = self.merge_radius * self.merge_radius
        for defect in self.defects:
            dx = defect['x'] - x
            dy = defect['y'] - y
            dz = defect['z'] - z
            if dx * dx + dy * dy + dz * dz <= radius_sq:
                return defect
        return None

    def publish_outputs(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        poses = PoseArray()
        poses.header.frame_id = self.target_frame
        poses.header.stamp = now

        for index, defect in enumerate(self.defects):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = now
            marker.ns = 'digital_twin_defects'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(defect['x'])
            marker.pose.position.y = float(defect['y'])
            marker.pose.position.z = float(defect['z'])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.30
            marker.scale.y = 0.30
            marker.scale.z = 0.30
            marker.color.a = 0.95
            if defect.get('status') == 'confirmed':
                marker.color.g = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.3
            markers.markers.append(marker)

            pose = Pose()
            pose.position.x = marker.pose.position.x
            pose.position.y = marker.pose.position.y
            pose.position.z = marker.pose.position.z
            pose.orientation.w = 1.0
            poses.poses.append(pose)

        self.marker_publisher.publish(markers)
        self.rescan_publisher.publish(poses)


def main(args=None):
    rclpy.init(args=args)
    node = DefectMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
