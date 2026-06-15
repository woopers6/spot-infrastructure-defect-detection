import rclpy
from rclpy.node import Node

import yaml
import numpy as np
from ultralytics import YOLO

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D
from rclpy.qos import qos_profile_sensor_data


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        with open("dataset.yaml", "r") as file:
            data = yaml.safe_load(file)
        names = data["names"]

        if isinstance(names, dict):
            self.class_names = [
                names[index] for index in sorted(names.keys())
            ]
        else:
            self.class_names = list(names)

        self.classes = list(data["names"])
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        self.model = YOLO("yolov11m.engine")
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            "/ros2_image",
            self.image_callback,
            10
        )

        self.detections_pub = self.create_publisher(
            Detection2DArray,
            "/detections_2d",
            qos_profile_sensor_data
        )

    def image_callback(self, image_msg: Image):
        frame = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding="bgr8"
        )

        results = self.model(frame)

        detections_msg = Detection2DArray()

        #IMPORTANT IMPORTANT IMPORTANT preserve original image capture timestamp, time syncing won't work otherwise
        detections_msg.header = image_msg.header

        if len(results) > 0:
            result = results[0]

            for box in result.boxes:
                detection = self.yolo_box_to_detection2d(box)
                detections_msg.detections.append(detection)

        self.detections_pub.publish(detections_msg)

    def yolo_box_to_detection2d(self, box):
        detection = Detection2D()

        xyxy = box.xyxy.detach().cpu().numpy().reshape(-1)
        x1, y1, x2, y2 = xyxy[:4]

        cx = float((x1 + x2) / 2.0)
        cy = float((y1 + y2) / 2.0)
        width = float(x2 - x1)
        height = float(y2 - y1)

        detection.bbox = BoundingBox2D()
        detection.bbox.center.position.x = cx
        detection.bbox.center.position.y = cy
        detection.bbox.size_x = width
        detection.bbox.size_y = height

        hypothesis = ObjectHypothesisWithPose()

        class_id = int(box.cls.detach().cpu().numpy().reshape(-1)[0])
        confidence = float(box.conf.detach().cpu().numpy().reshape(-1)[0])

        hypothesis.hypothesis.class_id = str(class_id)
        hypothesis.hypothesis.score = confidence

        detection.results.append(hypothesis)

        return detection

    def return_classes(self):
        return self.classes


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()