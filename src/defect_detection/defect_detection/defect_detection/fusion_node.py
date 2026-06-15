from defect_detection.defect_detection.defect_localization.extract_3d_detections import CustomDetection3D
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
import numpy as np
import yaml

from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import (
    Detection2DArray,
    Detection3DArray,
    Detection3D as Detection3DMsg,
    BoundingBox3D,
    ObjectHypothesisWithPose,
)

from defect_localization import extract_detections_3d


class DetectionFusionNode(Node):

    def __init__(self):
        super().__init__("detection_fusion_node")

        with open("dataset.yaml", "r") as file:
            data = yaml.safe_load(file)

        names = data["names"]

        if isinstance(names, dict):
            self.class_names = [
                names[index] for index in sorted(names.keys())
            ]
        else:
            self.class_names = list(names)

        self.intrinsics_matrix = np.array([ #TODO: replace with actual intrinsics
            [1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        self.T_lidar_to_camera = np.eye(4, dtype=np.float64) #TODO: replace with actual extrinsic transformation from lidar to camera

        self.image_shape = (720, 1280)

        # Output publisher
        self.detections_3d_pub = self.create_publisher(
            Detection3DArray,
            "/detections_3d",
            10,
        )

        self.detections_sub = message_filters.Subscriber(
            self,
            Detection2DArray,
            "/detections_2d",
            qos_profile=qos_profile_sensor_data,
        )

        self.pointcloud_sub = message_filters.Subscriber(
            self,
            PointCloud2,
            "/velodyne/points",
            qos_profile=qos_profile_sensor_data,
        )

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [
                self.detections_sub,
                self.pointcloud_sub,
            ],
            queue_size=30,
            slop=0.10,
        )

        self.synchronizer.registerCallback(
            self.synchronized_callback
        )

        self.get_logger().info(
            "Fusion node started. Publishing on /detections_3d"
        )

    def synchronized_callback(
        self,
        detections_2d_msg: Detection2DArray,
        pointcloud_msg: PointCloud2,
    ):
        self.get_logger().info(
            "Received synchronized detection and point-cloud pair"
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

        self.get_logger().info(
            f"Published {len(output_msg.detections)} detections "
            f"to /detections_3d"
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


if __name__ == "__main__":
    main()